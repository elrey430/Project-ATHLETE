"""Project ATHLETE - checks that the tracking observation doesn't depend on the athlete's compass heading.

Usage (LocoMuJoCo Python):
    Intermediate/LocoMuJoCoPython/Scripts/python.exe Scripts/MuJoCo/check_invariance.py

Builds the CPU athlete with the heading-invariant observation (invariant_obs, GoalTrackInvariant), moves
it into a mid-motion state, and takes the observation. Then it turns the whole scene (the athlete and the
reference frames the goal reads) about the vertical axis by several angles and takes it again. The physics
is the same in every direction on a flat floor, so the observation must not change. For comparison it
does the same with the earlier observation (GoalTrajMimicLookahead, world-axis pelvis pose).
"""

import dataclasses
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import athlete_loco  # noqa: E402,F401
from loco_mujoco import TaskFactory  # noqa: E402
from motion_matching import to_rotation, to_wxyz, yaw_rotation  # noqa: E402

VERBOSE = "--verbose" in sys.argv
FIELDS = ("qpos", "qvel", "xpos", "xquat", "cvel", "subtree_com", "site_xpos", "site_xmat")


def turn_state(qpos, qvel, angle):
    """The same state turned by `angle` about the vertical axis through the world origin."""
    qpos, qvel = qpos.copy(), qvel.copy()
    c, s = np.cos(angle), np.sin(angle)
    turn = np.array([[c, -s], [s, c]])
    qpos[:2] = turn @ qpos[:2]
    qpos[3:7] = to_wxyz(yaw_rotation(angle) * to_rotation(qpos[3:7]))
    qvel[:2] = turn @ qvel[:2]  # world linear velocity; the angular part is in the pelvis frame
    return qpos, qvel


def observation_change(goal_type, invariant_obs, angles=(0.5, 1.5, np.pi, -2.0)):
    env = TaskFactory.get_factory_cls("ImitationFactory").make(
        env_name="AthleteReference", default_dataset_conf=dict(task=["walkturn"]), goal_type=goal_type,
        goal_params={"visualize_goal": False}, invariant_obs=invariant_obs,
        th_params=dict(random_start=False, fixed_start_conf=(0, 1500)))
    data = env.th.traj.data
    env.th.traj = dataclasses.replace(env.th.traj, data=data.replace(**{f: np.array(getattr(data, f)) for f in FIELDS}))
    obs = env.reset()
    rng = np.random.default_rng(0)
    for _ in range(20):  # off the reference a little, so relative terms aren't zero
        obs, *_ = env.step(np.atleast_2d(rng.uniform(-0.3, 0.3, env.info.action_space.shape[0])))
    model, d = env._model, env._data
    row = int(env._additional_carry.traj_state.subtraj_step_no)
    saved_sim = (d.qpos.copy(), d.qvel.copy())
    traj = env.th.traj.data
    saved_rows = {f: getattr(traj, f).copy() for f in FIELDS}
    reference = mujoco.MjData(model)
    # After a step, MuJoCo's derived quantities (site positions, velocities) still describe the state
    # before it; recompute them, as for the turned states below, so only the turn differs.
    mujoco.mj_forward(model, d)
    base = np.asarray(env._create_observation(model, d, env._additional_carry)[0])
    worst, where = 0.0, ""
    owner = {}
    for name, obs_type in env.obs_container.items():
        for local, index in enumerate(np.atleast_1d(obs_type.obs_ind)):
            owner[int(index)] = f"{name}[{local}]"
    for angle in angles:
        d.qpos[:], d.qvel[:] = turn_state(*saved_sim, angle)
        mujoco.mj_forward(model, d)
        for r in range(row, min(row + 40, len(traj.qpos))):  # the rows the goal reads (now + lookahead)
            reference.qpos[:], reference.qvel[:] = turn_state(saved_rows["qpos"][r], saved_rows["qvel"][r], angle)
            mujoco.mj_kinematics(model, reference)
            mujoco.mj_comPos(model, reference)
            mujoco.mj_comVel(model, reference)
            traj.qpos[r], traj.qvel[r] = reference.qpos, reference.qvel
            traj.xpos[r], traj.xquat[r], traj.cvel[r] = reference.xpos, reference.xquat, reference.cvel
            traj.subtree_com[r], traj.site_xpos[r], traj.site_xmat[r] = reference.subtree_com, reference.site_xpos, reference.site_xmat
        turned = np.asarray(env._create_observation(model, d, env._additional_carry)[0])
        change = np.abs(turned - base)
        if change.max() > worst:
            worst, where = float(change.max()), owner.get(int(change.argmax()), "?")
        if VERBOSE:
            bad = np.flatnonzero(change > 1e-4)
            print(f"  turned {angle:+.2f} rad: {len(bad)} values change, e.g. "
                  + ", ".join(f"{owner.get(int(i), i)} {change[i]:.3f}" for i in bad[np.argsort(-change[bad])][:6]))
        for f in FIELDS:
            getattr(traj, f)[:] = saved_rows[f]
    return base.shape[0], worst, where


def main():
    for goal_type, invariant_obs in (("GoalTrajMimicLookahead", False), ("GoalTrackInvariant", True)):
        size, worst, where = observation_change(goal_type, invariant_obs)
        verdict = "INVARIANT" if worst < 1e-4 else "depends on heading"
        print(f"{goal_type:24s} invariant_obs={invariant_obs!s:5s}: {size} values, largest change when turned {worst:.2e} (at {where}) -> {verdict}")


if __name__ == "__main__":
    main()
