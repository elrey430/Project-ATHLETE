"""Project ATHLETE - training for recovery: random pushes on the athlete, on top of LocoMuJoCo's randomizer.

The tracker learned to follow the reference closely, but fell when it drifted from it: after sudden command
changes and during combined running moves (random-command test, 2026-10-03). Shaping the controller input
didn't help (the motion matcher already smooths commands), so the tracker itself has to learn to recover.
Pushes make training visit the off-reference states it never saw, and recovering from them pays off in the
tracking reward. Football needs that anyway: contact is a push.

PushRandomizer = LocoMuJoCo's DefaultRandomizer (masses, centre of mass, joint damping, gravity, sensor
noise: each only if switched on in the config) plus pushes:
  - every push_interval_s (uniform range), a horizontal force on the pelvis for push_duration_s;
  - its impulse (force x duration) uniform in push_impulse_ns, in a random direction.
Defaults: 0..20 N s, every 2-5 s, lasting 0.1-0.2 s. 20 N s changes an 86 kg athlete's speed by ~0.23 m/s,
about what the hand-built balance controller recovered from in Milestone 3 (~15 N s forward).
"""

from typing import Any, Tuple

import jax
import jax.numpy as jnp
import mujoco
import numpy as np
from flax import struct

from loco_mujoco.core.domain_randomizer.default import DefaultRandomizer, DefaultRandomizerState


@struct.dataclass
class PushRandomizerState(DefaultRandomizerState):
    push_force: Any        # (3,) world-frame force on the pelvis while a push lasts
    push_time_left: float  # seconds of the current push left (<= 0: no push)
    next_push_in: float    # seconds until the next push starts


class PushRandomizer(DefaultRandomizer):

    def __init__(self, env, push_interval_s=(2.0, 5.0), push_duration_s=(0.1, 0.2), push_impulse_ns=(0.0, 20.0),
                 **kwargs):
        self._push_interval_s = tuple(push_interval_s)
        self._push_duration_s = tuple(push_duration_s)
        self._push_impulse_ns = tuple(push_impulse_ns)
        super().__init__(env, **kwargs)
        self._pelvis = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, env._get_all_info_properties()["root_body_name"])

    def init_state(self, env: Any, key: Any, model, data, backend) -> PushRandomizerState:
        base = super().init_state(env, key, model, data, backend)
        fields = {f: getattr(base, f) for f in base.__dataclass_fields__}
        # First push after the shortest interval (episodes start at random moments of the motion anyway).
        return PushRandomizerState(**fields, push_force=backend.zeros(3), push_time_left=0.0,
                                   next_push_in=self._push_interval_s[0])

    def _sample_link_mass_multipliers(self, model, carry, backend):
        # LocoMuJoCo v1.1.0 bug: with randomize_link_mass off it concatenates a (1, 1) and an (n,) array and
        # fails. Off means every multiplier is 1.
        if not self.rand_conf["randomize_link_mass"]:
            return backend.ones(model.nbody - 1), carry
        return super()._sample_link_mass_multipliers(model, carry, backend)

    def reset(self, env: Any, model, data, carry: Any, backend) -> Tuple[Any, Any]:
        data, carry = super().reset(env, model, data, carry, backend)
        state = carry.domain_randomizer_state.replace(push_force=backend.zeros(3), push_time_left=0.0,
                                                      next_push_in=self._push_interval_s[0])
        if backend == jnp:
            data = data.replace(xfrc_applied=data.xfrc_applied.at[self._pelvis].set(jnp.zeros(6)))
        else:
            data.xfrc_applied[self._pelvis] = 0.0
        return data, carry.replace(domain_randomizer_state=state)

    def update(self, env: Any, model, data, carry: Any, backend) -> Tuple[Any, Any, Any]:
        model, data, carry = super().update(env, model, data, carry, backend)
        state = carry.domain_randomizer_state
        if backend == jnp:
            key, subkey = jax.random.split(carry.key)
            u = jax.random.uniform(subkey, (4,))
            carry = carry.replace(key=key)
        else:
            u = np.random.uniform(size=4)
        lo, hi = self._push_duration_s
        duration = lo + u[0] * (hi - lo)
        impulse = self._push_impulse_ns[0] + u[1] * (self._push_impulse_ns[1] - self._push_impulse_ns[0])
        angle = 2.0 * np.pi * u[2]
        force = backend.array([backend.cos(angle), backend.sin(angle), 0.0]) * impulse / duration
        interval = self._push_interval_s[0] + u[3] * (self._push_interval_s[1] - self._push_interval_s[0])

        start = state.next_push_in <= 0.0
        push_force = backend.where(start, force, state.push_force)
        push_time_left = backend.where(start, duration, state.push_time_left) - env.dt
        next_push_in = backend.where(start, interval, state.next_push_in - env.dt)
        applied = backend.where(push_time_left + env.dt > 0.0, push_force, backend.zeros(3))
        wrench = backend.concatenate([applied, backend.zeros(3)])  # MuJoCo xfrc_applied: force, then torque
        if backend == jnp:
            data = data.replace(xfrc_applied=data.xfrc_applied.at[self._pelvis].set(wrench))
        else:
            data.xfrc_applied[self._pelvis] = wrench
        state = state.replace(push_force=push_force, push_time_left=push_time_left, next_push_in=next_push_in)
        return model, data, carry.replace(domain_randomizer_state=state)
