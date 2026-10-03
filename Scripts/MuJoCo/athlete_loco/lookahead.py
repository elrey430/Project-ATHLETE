"""Project ATHLETE - the tracking policy's view of where the reference motion is going.

LocoMuJoCo's GoalTrajMimic shows the policy only the reference's NEXT frame, so it can only react. Humans
anticipate: they lean and shift weight before a turn or a stop. With the next frame alone, the tracker
learned walking, running and braking, but turning and starting/stopping fell within seconds even after
700M+ training steps (2026-10-01).

GoalTrajMimicLookahead adds the reference LOOKAHEAD_S seconds ahead (0.1, 0.2, 0.3 s). Per future moment,
relative to the current reference frame (so independent of where the athlete is on the field):
  - pelvis displacement in the current heading frame (2) and the change of heading (cos, sin) (2);
  - pelvis height (1) and the direction of gravity in the pelvis frame, i.e. its tilt (3);
  - pelvis velocity in that moment's heading frame (3);
  - every joint angle (37).
The new values are appended after GoalTrajMimic's, so a policy trained without lookahead can be widened
for them (_train_chunked.py --init-from pads its input layer with zeros).

At run time the future must already exist: locomotion_tests.Driver runs motion matching LOOKAHEAD frames
ahead of the simulation, which delays the reaction to a new command by max(LOOKAHEAD_S).
"""

from typing import Any, Dict, Tuple

import numpy as np

from loco_mujoco.core.observations.goals import GoalTrajMimic

LOOKAHEAD_S = (0.1, 0.2, 0.3)


def _heading(quat, backend):
    w, x, y, z = quat[0], quat[1], quat[2], quat[3]
    return backend.arctan2(2 * (x * y + w * z), 1 - 2 * (y * y + z * z))


def future_features(now_qpos, later_qpos, later_qvel, backend):
    """The 11 + n_joints values describing one future reference frame relative to the current one."""
    yaw0 = _heading(now_qpos[3:7], backend)
    yaw = _heading(later_qpos[3:7], backend)
    c0, s0 = backend.cos(yaw0), backend.sin(yaw0)
    d = later_qpos[:2] - now_qpos[:2]
    w, x, y, z = later_qpos[3], later_qpos[4], later_qpos[5], later_qpos[6]
    gravity = -backend.array([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)])  # R^T (0,0,-1)
    c, s = backend.cos(yaw), backend.sin(yaw)
    v = later_qvel[:3]
    return backend.concatenate([
        backend.array([c0 * d[0] + s0 * d[1], -s0 * d[0] + c0 * d[1],
                       backend.cos(yaw - yaw0), backend.sin(yaw - yaw0), later_qpos[2]]),
        gravity,
        backend.array([c * v[0] + s * v[1], -s * v[0] + c * v[1], v[2]]),
        later_qpos[7:],
    ])


class GoalTrajMimicLookahead(GoalTrajMimic):
    """GoalTrajMimic plus the reference LOOKAHEAD_S seconds ahead (see the module docstring)."""

    def __init__(self, info_props: Dict, lookahead_s: Tuple[float, ...] = LOOKAHEAD_S, **kwargs):
        self._lookahead_s = tuple(lookahead_s)
        self._lookahead_steps = None
        self._lookahead_dim = None
        super().__init__(info_props, **kwargs)

    def _init_from_mj(self, env: Any, model, data, current_obs_size: int):
        self._lookahead_steps = np.array([int(round(t / env.dt)) for t in self._lookahead_s])
        self._lookahead_dim = len(self._lookahead_s) * (11 + model.nq - 7)
        super()._init_from_mj(env, model, data, current_obs_size)  # sizes itself from self.dim

    @property
    def lookahead_steps(self):
        return self._lookahead_steps

    def get_obs_and_update_state(self, env: Any, model, data, carry: Any, backend):
        goal, carry = super().get_obs_and_update_state(env, model, data, carry, backend)
        traj_data = env.th.traj.data
        traj_state = carry.traj_state
        last = env.th.len_trajectory(traj_state.traj_no) - 1
        now = traj_data.get(traj_state.traj_no, traj_state.subtraj_step_no, backend)
        future = []
        for steps in self._lookahead_steps:
            # Within the current clip: the last frame repeats near its end.
            index = backend.minimum(traj_state.subtraj_step_no + int(steps), last)
            later = traj_data.get(traj_state.traj_no, index, backend)
            future.append(future_features(now.qpos, later.qpos, later.qvel, backend))
        return backend.concatenate([goal] + future), carry

    @property
    def dim(self) -> int:
        return super().dim + (self._lookahead_dim or 0)
