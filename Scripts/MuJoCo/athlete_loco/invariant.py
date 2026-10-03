"""Project ATHLETE - a tracking observation that doesn't depend on which way the athlete faces.

Measured 2026-10-01: the tracker turned the WRONG way on the spot when it started facing 90 degrees
instead of 0 (same command, same body). LocoMuJoCo's humanoid observation gives the athlete's own pelvis
orientation and velocity, and the reference's, in WORLD axes. The policy must then work out "the
reference faces 30 degrees left of me" from two absolute orientations, differently for every compass
direction. Walking straight mostly survived that; turning didn't. Trackers like DeepMimic, AMP and PHC
express everything relative to the character's own heading, and this module does the same:

Environment observation (invariant_observation_spec):
  pelvis height, gravity direction in the pelvis frame (its tilt), every joint angle, the pelvis's
  angular velocity (MuJoCo gives it in the pelvis frame), every joint velocity.
Goal (GoalTrackInvariant), all relative to the athlete or to the reference itself:
  - the athlete's marker sites relative to its upper body (as GoalTrajMimic);
  - the reference's joint angles and velocities, pelvis height, tilt, linear velocity in its own pelvis
    frame, angular velocity, and its marker sites relative to its upper body;
  - the reference relative to the athlete: heading difference (cos, sin), pelvis position in the
    athlete's heading frame, height difference;
  - the athlete's own pelvis linear velocity in its pelvis frame;
  - the lookahead (GoalTrajMimicLookahead: future reference frames relative to the current one).
Turning the whole scene about the vertical axis leaves every value unchanged (check_invariance.py).
"""

from typing import Any, List

import numpy as np

from loco_mujoco.core import Observation, ObservationType
from loco_mujoco.core.utils.math import calc_site_velocities, calculate_relative_site_quatities

from .lookahead import GoalTrajMimicLookahead, future_features


def invariant_observation_spec(hinges: List[str]) -> List[Observation]:
    return ([ObservationType.EntryFromFreeJointPos(entry_index=2, obs_name="q_root_height", xml_name="root"),
             ObservationType.ProjectedGravityVector("root_gravity", xml_name="root")]
            + [ObservationType.JointPos(f"q_{name}", xml_name=name) for name in hinges]
            + [ObservationType.EntryFromFreeJointVel(entry_index=3 + i, obs_name=f"dq_root_ang_{axis}", xml_name="root")
               for i, axis in enumerate("xyz")]
            + [ObservationType.JointVel(f"dq_{name}", xml_name=name) for name in hinges])


def _heading(quat, backend):
    w, x, y, z = quat[0], quat[1], quat[2], quat[3]
    return backend.arctan2(2 * (x * y + w * z), 1 - 2 * (y * y + z * z))


def _to_body_frame(quat, vector, backend):
    """R^T v for the rotation of a MuJoCo quaternion (w, x, y, z)."""
    w, x, y, z = quat[0], quat[1], quat[2], quat[3]
    r = backend.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                       [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                       [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    return r.T @ vector


def _gravity(quat, backend):
    return _to_body_frame(quat, backend.array([0.0, 0.0, -1.0]), backend)


class GoalTrackInvariant(GoalTrajMimicLookahead):
    """The tracking goal relative to the athlete's own heading (see the module docstring)."""

    def _init_from_mj(self, env: Any, model, data, current_obs_size: int):
        n_sites = len(self._info_props["sites_for_mimic"]) - 1
        sites = (3 + 3 + 6) * n_sites
        reference = (model.nq - 7) + (model.nv - 6) + 1 + 3 + 3 + 3 + sites
        relative = 2 + 2 + 1 + 3
        self._invariant_dim = sites + reference + relative
        super()._init_from_mj(env, model, data, current_obs_size)

    @property
    def dim(self) -> int:
        if getattr(self, "_invariant_dim", None) is None:  # before _init_from_mj
            return super().dim
        return self._invariant_dim + (self._lookahead_dim or 0)

    def get_obs_and_update_state(self, env: Any, model, data, carry: Any, backend):
        traj_state = carry.traj_state
        traj_data = env.th.traj.data
        ref = traj_data.get(traj_state.traj_no, traj_state.subtraj_step_no, backend)
        site_ids = self._rel_site_ids
        body_ids = self._site_bodyid[site_ids]
        sim_sites = list(calculate_relative_site_quatities(data, site_ids, body_ids, self._body_rootid, backend))
        ref_sites = list(calculate_relative_site_quatities(ref, site_ids, body_ids, self._body_rootid, backend))
        # LocoMuJoCo's relative site positions are world-axis differences (calc_rel_positions), and its
        # relative velocities are turned by the upper body's orientation the wrong way round
        # (calculate_relative_velocity_in_local_frame applies site_xmat, i.e. body-to-world, as
        # world-to-body). Both therefore depend on heading. Here both are taken in the upper body's frame
        # (the first site); the relative angles are already relative.
        for sites, source in ((sim_sites, data), (ref_sites, ref)):
            frame = source.site_xmat[site_ids[0]].reshape(3, 3)
            velocity = calc_site_velocities(site_ids, source, body_ids, self._body_rootid[body_ids], backend)
            sites[0] = sites[0] @ frame  # rows: frame^T (site - upper body)
            sites[2] = ((velocity[1:] - velocity[0]).reshape(-1, 2, 3) @ frame).reshape(-1, 6)

        q, v = data.qpos, data.qvel  # the root free joint comes first
        yaw_sim, yaw_ref = _heading(q[3:7], backend), _heading(ref.qpos[3:7], backend)
        c, s = backend.cos(yaw_sim), backend.sin(yaw_sim)
        d = ref.qpos[:2] - q[:2]
        turn = yaw_ref - yaw_sim

        goal = backend.concatenate([
            backend.ravel(sim_sites[0]), backend.ravel(sim_sites[1]), backend.ravel(sim_sites[2]),
            ref.qpos[7:], ref.qvel[6:],
            backend.array([ref.qpos[2]]), _gravity(ref.qpos[3:7], backend),
            _to_body_frame(ref.qpos[3:7], ref.qvel[:3], backend), ref.qvel[3:6],
            backend.ravel(ref_sites[0]), backend.ravel(ref_sites[1]), backend.ravel(ref_sites[2]),
            backend.array([backend.cos(turn), backend.sin(turn), c * d[0] + s * d[1], -s * d[0] + c * d[1],
                           ref.qpos[2] - q[2]]),
            _to_body_frame(q[3:7], v[:3], backend),
        ])

        last = env.th.len_trajectory(traj_state.traj_no) - 1
        future = []
        for steps in self._lookahead_steps:
            index = backend.minimum(traj_state.subtraj_step_no + int(steps), last)
            later = traj_data.get(traj_state.traj_no, index, backend)
            future.append(future_features(ref.qpos, later.qpos, later.qvel, backend))

        if self.visualize_goal:
            carry = self.set_visuals(env, model, data, carry, backend)
        return backend.concatenate([goal] + future), carry
