"""Project ATHLETE - controller input for LocoMuJoCo training: velocity commands that CHANGE during an episode.

Milestone 4 means a controller sends desired movement and the athlete accelerates, brakes and turns to
follow it. The command is the same intent as Unreal's FAthleteMovementIntent: forward and sideways speed
and turning rate, in the athlete's own (root) frame.

GoalVelocityCommand: LocoMuJoCo's GoalRandomRootVelocity picks ONE random command per episode, at reset;
nothing then teaches changing speed or direction. This goal picks a new command every 2-5 s (a "stand
still" command among them), so accelerating, braking and turning all happen within episodes.

VelocityCommandReward: LocoMuJoCo's TargetVelocityGoalReward (exp(-w * squared error) on the root's x/y
speed and on its turning rate), reading this goal. Theirs looks the goal up by its own class name.
"""

from types import ModuleType
from typing import Any, Dict, Tuple, Union

import jax
import jax.numpy as jnp
import mujoco
import numpy as np
from flax import struct
from jax._src.scipy.spatial.transform import Rotation as jnp_R
from scipy.spatial.transform import Rotation as np_R

from loco_mujoco.core.observations.goals import GoalRandomRootVelocity
from loco_mujoco.core.reward.base import Reward
from loco_mujoco.core.utils import mj_jntname2qvelid
from loco_mujoco.core.utils.math import quat_scalarfirst2scalarlast


@struct.dataclass
class GoalVelocityCommandState:
    goal_vel_x: float
    goal_vel_y: float
    goal_vel_yaw: float
    time_left: float  # seconds until the next command


class GoalVelocityCommand(GoalRandomRootVelocity):
    """
    A new velocity command every min_hold_s..max_hold_s. Each command: forward speed uniform in
    [min_forward, max_forward]; with probability p_lateral a sideways speed in +/- max_lateral; with
    probability p_turn a turning rate in +/- max_yaw; with probability p_stand all zero (stand still).
    Ranges cover the motion data (walking ~1.3, running ~2.9 m/s, turning on the spot up to ~2.4 rad/s).
    """

    def __init__(self, info_props: Dict, min_forward: float = -0.5, max_forward: float = 3.0,
                 max_lateral: float = 0.5, max_yaw: float = 1.2, p_lateral: float = 0.3, p_turn: float = 0.5,
                 p_stand: float = 0.15, min_hold_s: float = 2.0, max_hold_s: float = 5.0, **kwargs):
        super().__init__(info_props, max_x_vel=max(abs(min_forward), max_forward), max_y_vel=max_lateral,
                         max_yaw_vel=max_yaw, **kwargs)
        self.min_forward, self.max_forward = min_forward, max_forward
        self.max_lateral, self.max_yaw = max_lateral, max_yaw
        self.p_lateral, self.p_turn, self.p_stand = p_lateral, p_turn, p_stand
        self.min_hold_s, self.max_hold_s = min_hold_s, max_hold_s
        # Controller input: set to (forward, sideways, turning) to command the athlete directly instead of
        # random commands (CPU MuJoCo only: tests, the drive demo). None = random commands (training).
        self.external_command = None

    def _external_state(self):
        vx, vy, yaw = self.external_command
        return GoalVelocityCommandState(float(vx), float(vy), float(yaw), 1e9)

    def _sample(self, key, backend: ModuleType):
        if backend == np:
            u = np.random.uniform(size=7)
        else:
            key, subkey = jax.random.split(key)
            u = jax.random.uniform(subkey, (7,))
        stand = u[5] < self.p_stand
        vx = self.min_forward + u[0] * (self.max_forward - self.min_forward)
        vy = (2.0 * u[1] - 1.0) * self.max_lateral * (u[3] < self.p_lateral)
        yaw = (2.0 * u[2] - 1.0) * self.max_yaw * (u[4] < self.p_turn)
        hold = self.min_hold_s + u[6] * (self.max_hold_s - self.min_hold_s)
        state = GoalVelocityCommandState(backend.where(stand, 0.0, vx), backend.where(stand, 0.0, vy),
                                         backend.where(stand, 0.0, yaw), hold)
        return state, key

    def init_state(self, env: Any, key: jax.random.PRNGKey, model, data, backend: ModuleType) -> GoalVelocityCommandState:
        return GoalVelocityCommandState(0.0, 0.0, 0.0, 0.0)

    def reset_state(self, env: Any, model, data, carry: Any, backend: ModuleType) -> Tuple[Any, Any]:
        state, key = self._sample(carry.key, backend)
        if backend == np and self.external_command is not None:
            state = self._external_state()
        observation_states = carry.observation_states.replace(**{self.name: state})
        return data, carry.replace(key=key, observation_states=observation_states)

    def get_obs_and_update_state(self, env: Any, model, data, carry: Any, backend: ModuleType):
        state = getattr(carry.observation_states, self.name)
        new_state, key = self._sample(carry.key, backend)
        expired = state.time_left <= 0.0
        state = GoalVelocityCommandState(
            backend.where(expired, new_state.goal_vel_x, state.goal_vel_x),
            backend.where(expired, new_state.goal_vel_y, state.goal_vel_y),
            backend.where(expired, new_state.goal_vel_yaw, state.goal_vel_yaw),
            backend.where(expired, new_state.time_left, state.time_left) - env.dt)
        if backend == np:
            key = key if expired else carry.key  # numpy samples don't consume the jax key anyway
            if self.external_command is not None:
                state = self._external_state()
        carry = carry.replace(key=key, observation_states=carry.observation_states.replace(**{self.name: state}))
        goal = backend.array([state.goal_vel_x, state.goal_vel_y, state.goal_vel_yaw])
        if self.visualize_goal:
            visual = backend.array([state.goal_vel_x, state.goal_vel_y, 0.0, 0.0, 0.0, state.goal_vel_yaw])
            carry = self.set_visuals(visual, env, model, data, carry, self._root_body_id, self._free_jnt_qpos_id,
                                     self.visual_geoms_idx, backend)
        return goal, carry


class VelocityCommandReward(Reward):
    """
    Velocity tracking in the root's frame, each of x/y speed and turning rate scored by the average of a
    SHARP and a BROAD kernel: 0.5 exp(-w_sharp e^2) + 0.5 exp(-w_broad e^2).

    LocoMuJoCo's reward has only the sharp kernel (w = 10), made for robots commanded at <= ~1 m/s. For
    our 0-3 m/s commands it is ~0 whenever the error exceeds ~1 m/s (standing when told 1.5 m/s scores
    exp(-11)), so nothing rewards starting to move; the first trained policy (2026-10-01) learned to stand
    and survive instead of following commands. The broad kernel (w = 1) pays for getting closer from far
    away; the sharp one for precision.
    """

    def __init__(self, env: Any, tracking_w_exp_xy: float = 10.0, tracking_w_exp_yaw: float = 10.0,
                 tracking_w_exp_xy_broad: float = 1.0, tracking_w_exp_yaw_broad: float = 1.0,
                 tracking_w_sum_xy: float = 1.0, tracking_w_sum_yaw: float = 1.0, **kwargs):
        super().__init__(env, **kwargs)
        self._free_jnt_name = self._info_props["root_free_joint_xml_name"]
        self._vel_idx = np.array(mj_jntname2qvelid(self._free_jnt_name, env._model))
        self._w_exp_xy, self._w_exp_yaw = tracking_w_exp_xy, tracking_w_exp_yaw
        self._w_broad_xy, self._w_broad_yaw = tracking_w_exp_xy_broad, tracking_w_exp_yaw_broad
        self._w_sum_xy, self._w_sum_yaw = tracking_w_sum_xy, tracking_w_sum_yaw
        assert GoalVelocityCommand.__name__ in env.obs_container, "VelocityCommandReward needs the GoalVelocityCommand goal"

    def __call__(self, state, action, next_state, absorbing: bool, info: Dict[str, Any], env: Any, model, data,
                 carry: Any, backend: ModuleType) -> Tuple[float, Any]:
        R = np_R if backend == np else jnp_R
        goal = getattr(carry.observation_states, GoalVelocityCommand.__name__)
        root = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, self._free_jnt_name)
        adr = model.jnt_qposadr[root]
        root_quat = R.from_quat(quat_scalarfirst2scalarlast(backend.squeeze(data.qpos[adr:adr + 7])[3:7]))
        velocity = backend.squeeze(data.qvel[self._vel_idx])
        local = root_quat.as_matrix().T @ velocity[:3]
        current = backend.concatenate([local[:2], backend.atleast_1d(velocity[5])])
        wanted = backend.array([goal.goal_vel_x, goal.goal_vel_y, goal.goal_vel_yaw])
        xy_error = backend.mean(backend.square(current[:2] - wanted[:2]))
        yaw_error = backend.square(current[2] - wanted[2])
        xy = 0.5 * backend.exp(-self._w_exp_xy * xy_error) + 0.5 * backend.exp(-self._w_broad_xy * xy_error)
        yaw = 0.5 * backend.exp(-self._w_exp_yaw * yaw_error) + 0.5 * backend.exp(-self._w_broad_yaw * yaw_error)
        return self._w_sum_xy * xy + self._w_sum_yaw * yaw, carry
