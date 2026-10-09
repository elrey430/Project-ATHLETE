"""Project ATHLETE - a training-only balance assist ("invisible hand"), faded to zero before training ends.

The tracker falls in ~4-5 of 100 random-command episodes, about half of them hard stops from a run, often
turning (2026-10-07/08). A training episode ends at the first stumble, so the policy rarely gets to practise
the recovery. The assist catches the beginning of an avoidable balance loss with a weak, capped push on the
pelvis toward the reference, so the episode goes on and the policy learns what comes next. The trainer
lowers it in stages to zero (_train_chunked.py --assist-schedule), and the tracker is measured, and used in
the game, WITHOUT it: the athlete's body moves only through its own joint torques, as everywhere else.

Rules (the user's, 2026-10-08):
  - weak in ordinary movement: nothing until the pelvis tilts tilt_start_rad from the reference's AND its
    speed is min_speed_error_mps off. Measured with the run 10 tracker (2026-10-08): 0.25 rad and 0.5 m/s fire
    on 2.9% of ordinary steps and 73% of steps 0.5-0.2 s before a fall (tilt > 0.15 alone: 22% / 95%);
  - brief, with a firm maximum: |F| <= max_force_n x scale, at most max_event_s per balance loss, at most
    episode_budget_ns of impulse per episode;
  - real failure stays failure: no help beyond give_up_tilt_rad, or with the pelvis already low, or during a
    push and lockout_s after it (contact must still knock the athlete down);
  - discouraged: MimicHeadingReward's assist_penalty_w charges (|F| / max_force_n)^2 per step.

AssistPushRandomizer = PushRandomizer (random pushes, sensor noise) + this assist. Force on the pelvis:
  F_xy = severity x clip(kd (ref_v_xy - v_xy), max_force_n x scale)
Velocity only: in LocoMuJoCo's training episodes the athlete starts near x = y = 0 while the reference keeps
its clip's absolute position (hundreds of metres away), so a position term would pull toward nowhere.
  severity = clip((tilt_error - tilt_start_rad) / (tilt_full_rad - tilt_start_rad), 0, 1)
where tilt_error is the angle between the athlete's and the reference's pelvis up axes.
"""

from typing import Any, Tuple

import jax.numpy as jnp
import numpy as np
from flax import struct

from .robustness import PushRandomizer, PushRandomizerState


@struct.dataclass
class AssistPushRandomizerState(PushRandomizerState):
    assist_force: Any        # (3,) world force on the pelvis this step (N)
    assist_event_s: float    # seconds the current balance loss has been assisted
    assist_used_ns: float    # impulse spent this episode (N s)
    since_push_s: float      # seconds since the last push ended


class AssistPushRandomizer(PushRandomizer):

    def __init__(self, env, assist_scale=1.0, max_force_n=130.0, kd=1000.0, tilt_start_rad=0.25,
                 tilt_full_rad=0.6, min_speed_error_mps=0.5, give_up_tilt_rad=0.9, low_margin_m=0.1, max_event_s=0.3,
                 episode_budget_ns=40.0, lockout_s=0.5, **kwargs):
        # assist_scale: the trainer's stage multiplier (1 -> 0). Read when the training program is traced,
        # so a change takes effect when the trainer rebuilds the program (one compile per stage).
        self.assist_scale = float(assist_scale)
        self._max_force, self._kd, self._min_speed_error = float(max_force_n), float(kd), float(min_speed_error_mps)
        self._tilt_start, self._tilt_full, self._give_up = float(tilt_start_rad), float(tilt_full_rad), float(give_up_tilt_rad)
        self._max_event, self._budget, self._lockout = float(max_event_s), float(episode_budget_ns), float(lockout_s)
        super().__init__(env, **kwargs)
        self._lowest_healthy = float(env._get_all_info_properties()["root_height_healthy_range"][0]) + float(low_margin_m)

    @property
    def max_force_n(self):
        return self._max_force

    def init_state(self, env: Any, key: Any, model, data, backend) -> AssistPushRandomizerState:
        base = super().init_state(env, key, model, data, backend)
        fields = {f: getattr(base, f) for f in base.__dataclass_fields__}
        return AssistPushRandomizerState(**fields, assist_force=backend.zeros(3), assist_event_s=0.0,
                                         assist_used_ns=0.0, since_push_s=self._lockout)

    def reset(self, env: Any, model, data, carry: Any, backend) -> Tuple[Any, Any]:
        data, carry = super().reset(env, model, data, carry, backend)
        state = carry.domain_randomizer_state.replace(assist_force=backend.zeros(3), assist_event_s=0.0,
                                                      assist_used_ns=0.0, since_push_s=self._lockout)
        return data, carry.replace(domain_randomizer_state=state)

    @staticmethod
    def _up_axis(quat, backend):
        w, x, y, z = quat[0], quat[1], quat[2], quat[3]
        return backend.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])

    def update(self, env: Any, model, data, carry: Any, backend) -> Tuple[Any, Any, Any]:
        model, data, carry = super().update(env, model, data, carry, backend)  # pushes: xfrc_applied set
        if self.assist_scale <= 0.0:
            return model, data, carry  # stage 0: exactly the push trainer
        state = carry.domain_randomizer_state
        dt = env.dt
        push_active = state.push_time_left + dt > 0.0
        since_push = backend.where(push_active, 0.0, state.since_push_s + dt)

        reference = env.th.traj.data.get(carry.traj_state.traj_no, carry.traj_state.subtraj_step_no, backend)
        q, v = data.qpos, data.qvel
        cos_tilt = backend.clip(backend.dot(self._up_axis(q[3:7], backend), self._up_axis(reference.qpos[3:7], backend)), -1.0, 1.0)
        tilt = backend.arccos(cos_tilt)
        severity = backend.clip((tilt - self._tilt_start) / (self._tilt_full - self._tilt_start), 0.0, 1.0)

        recoverable = (tilt < self._give_up) & (q[2] > self._lowest_healthy) & (since_push > self._lockout) \
            & (state.assist_used_ns < self._budget) & (state.assist_event_s < self._max_event)
        speed_error = reference.qvel[:2] - v[:2]
        severity = backend.where(backend.sqrt(backend.sum(speed_error ** 2)) > self._min_speed_error, severity, 0.0)
        on = (severity > 0.0) & recoverable

        wanted = self._kd * speed_error
        cap = self._max_force * self.assist_scale
        norm = backend.sqrt(backend.sum(wanted ** 2)) + 1e-9
        planar = wanted * backend.minimum(1.0, cap / norm) * severity
        force = backend.where(on, backend.concatenate([planar, backend.zeros(1)]), backend.zeros(3))
        magnitude = backend.sqrt(backend.sum(force ** 2))
        # The budget is a hard limit: the last assisted step only spends what is left.
        room = backend.clip((self._budget - state.assist_used_ns) / (magnitude * dt + 1e-9), 0.0, 1.0)
        force, magnitude = force * room, magnitude * room

        # The event clock runs while help is wanted, and restarts once the athlete is upright again.
        event_s = backend.where(severity > 0.0, state.assist_event_s + dt, 0.0)
        used = state.assist_used_ns + magnitude * dt
        wrench = backend.concatenate([force, backend.zeros(3)])
        pelvis = self._pelvis
        if backend == jnp:
            data = data.replace(xfrc_applied=data.xfrc_applied.at[pelvis].add(wrench))
        else:
            data.xfrc_applied[pelvis] = data.xfrc_applied[pelvis] + np.asarray(wrench)
        state = state.replace(assist_force=force, assist_event_s=event_s, assist_used_ns=used, since_push_s=since_push)
        return model, data, carry.replace(domain_randomizer_state=state)
