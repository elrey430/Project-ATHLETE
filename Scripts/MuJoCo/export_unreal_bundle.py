"""Project ATHLETE - exports everything Unreal needs to run the learned athlete (AthleteLearned module).

The tracker was trained in MuJoCo, so Unreal runs the athlete in MuJoCo too (the C library, same version
3.5.0); Unreal does input, camera and rendering. This writes a bundle folder:
  model.mjb       the compiled MuJoCo model exactly as tested (contact model, solver, 2 ms step)
  manifest.json   array index (file, dtype, shape) and every constant the C++ side needs
  *.bin           raw little-endian arrays: policy weights and input normalization, the motion-matching
                  database (playback quantities and features), and golden.* (a recorded Python session)
The golden session (Driver: command shaping, motion matching, observation, policy, simulation) lets the
C++ port check itself step by step (AthleteTests: Athlete.Learned.Parity.*).

Usage (the tracker's motion set must match its training: run 9/10 trained with 100STYLE):
    set ATHLETE_MOTION_SET=100style & set ATHLETE_MOTIONS=C:\\Dev\\ProjectAthlete\\Saved\\MuJoCo\\motions_run9
    Intermediate/LocoMuJoCoPython/Scripts/python.exe Scripts/MuJoCo/export_unreal_bundle.py <agent.pkl> [--out DIR]
"""

import argparse
import json
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import locomotion_tests as lt  # noqa: E402
import motion_matching as mm  # noqa: E402
import controller  # noqa: E402
from athlete_loco.env import FOOT_GEOMS, sole_geom_names  # noqa: E402

PROJECT_ROOT = Path(__file__).resolve().parents[2]
# A short session (< the 6 s trajectory buffer, so no wrap-around): stand, walk, turn while walking, run.
GOLDEN_SCHEDULE = [(1.0, (0.0, 0.0, 0.0)), (1.5, (1.5, 0.0, 0.0)), (1.5, (1.5, 0.0, 0.8)), (1.0, (2.5, 0.3, 0.0))]


class Bundle:
    def __init__(self, out):
        self.out, self.arrays, self.constants = Path(out), {}, {}
        self.out.mkdir(parents=True, exist_ok=True)

    def array(self, name, values, dtype):
        values = np.ascontiguousarray(np.asarray(values, dtype=dtype))
        values.tofile(self.out / f"{name}.bin")
        self.arrays[name] = {"file": f"{name}.bin", "dtype": np.dtype(dtype).name, "shape": list(values.shape)}

    def write_manifest(self):
        (self.out / "manifest.json").write_text(json.dumps({"arrays": self.arrays, "constants": self.constants}, indent=1))


def export(agent, out):
    bundle = Bundle(out)
    driver = lt.Driver(agent, horizon_s=10.0, model="full")
    env, model = driver.env, driver.env._model
    mujoco.mj_saveModel(model, str(bundle.out / "model.mjb"), None)

    # Policy: input normalization (frozen running statistics) and the actor MLP (tanh hidden layers).
    params = driver.train_state.params["FullyConnectedNet_0"]
    stats = driver.train_state.run_stats["RunningMeanStd_0"]
    bundle.array("policy_obs_mean", stats["mean"], np.float32)
    bundle.array("policy_obs_var", stats["var"], np.float32)
    for i in range(3):
        bundle.array(f"policy_w{i}", params[f"Dense_{i}"]["kernel"], np.float32)  # (inputs, outputs)
        bundle.array(f"policy_b{i}", params[f"Dense_{i}"]["bias"], np.float32)
    control = env._control_func
    bundle.array("ctrl_mean", control.norm_act_mean, np.float64)
    bundle.array("ctrl_delta", control.norm_act_delta, np.float64)

    # Observation layout.
    goal = env.obs_container["GoalTrackInvariant"]
    site_names = list(goal._info_props["sites_for_mimic"])
    site_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, s) for s in site_names]
    hinges = [j for j in range(model.njnt) if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE]
    sole_geoms = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n) for f in FOOT_GEOMS for n in sole_geom_names(f)]
    low, high = env._get_all_info_properties()["root_height_healthy_range"]
    bundle.constants.update({
        "obs_dim": int(sum(o.dim for o in env.obs_container.values())), "action_dim": int(model.nu),
        "control_dt": float(env.dt), "substeps": int(env._n_substeps), "timestep": float(model.opt.timestep),
        "mimic_site_ids": site_ids, "mimic_site_names": site_names,
        "hinge_qpos_adr": [int(model.jnt_qposadr[j]) for j in hinges], "hinge_dof_adr": [int(model.jnt_dofadr[j]) for j in hinges],
        "lookahead_steps": [int(s) for s in goal.lookahead_steps], "lead": int(driver.lead),
        "sole_geom_ids": sole_geoms, "root_height_healthy_range": [float(low), float(high)],
        "max_reference_yaw_lead_rad": lt.MAX_REFERENCE_YAW_LEAD_RAD,
        "shaper": {"forward_acceleration": controller.FORWARD_ACCELERATION, "forward_deceleration": controller.FORWARD_DECELERATION,
                   "sideways_acceleration": controller.SIDEWAYS_ACCELERATION, "turn_acceleration": controller.TURN_ACCELERATION},
    })

    # Motion-matching database (MotionDatabase for this motion set).
    db = driver.matcher.db
    bundle.array("db_joints", db.joints, np.float32)
    bundle.array("db_joint_velocity", db.joint_velocity, np.float32)
    bundle.array("db_height", db.height, np.float32)
    bundle.array("db_pelvis_velocity", db.pelvis_velocity, np.float32)
    bundle.array("db_tilt_wxyz", mm.to_wxyz(db.tilt), np.float32)
    bundle.array("db_yaw_rate", db.yaw_rate, np.float32)
    bundle.array("db_angular_velocity", db.angular_velocity, np.float32)
    bundle.array("db_lowest", db.lowest, np.float32)
    bundle.array("db_features", db.features, np.float32)
    bundle.array("db_valid", db.valid, np.uint8)
    bundle.array("db_ranges", db.ranges, np.int32)
    bundle.array("db_range_of", db.range_of, np.int32)
    bundle.array("db_feature_mean", db.mean, np.float64)
    bundle.array("db_feature_scale", db.scale, np.float64)
    bundle.constants["matcher"] = {
        "motion_set": mm.MOTION_SET, "clips": list(mm.CLIPS), "n_frames": int(db.n_frames),
        "start_frame": int(db.first_frame(mm.START_CLIP)), "frequency": mm.FREQUENCY,
        "future_frames": list(mm.FUTURE_FRAMES),
        "slices": {k: [v.start, v.stop] for k, v in db.slices.items()},
        "search_interval_frames": mm.SEARCH_INTERVAL_FRAMES, "ignore_surrounding_frames": mm.IGNORE_SURROUNDING_FRAMES,
        "ignore_behind_frames": mm.IGNORE_BEHIND_FRAMES, "jump_margin": mm.JUMP_MARGIN,
        "min_jump_interval_frames": mm.MIN_JUMP_INTERVAL_FRAMES, "command_change_search": mm.COMMAND_CHANGE_SEARCH,
        "blend_halflife_s": mm.BLEND_HALFLIFE_S, "control_halflife_s": mm.CONTROL_HALFLIFE_S,
    }

    # Golden session, recorded from the Python Driver.
    record = {k: [] for k in ("obs", "action", "command", "frame", "jumps", "ref_qpos", "ref_qvel", "qpos", "qvel")}
    act = driver.act

    def recording_act(ts, obs, key):
        action, ts = act(ts, obs, key)
        record["obs"].append(np.asarray(obs).ravel().copy())
        record["action"].append(np.asarray(action).ravel().copy())
        return action, ts
    driver.act = recording_act
    driver.start_offset = 0
    obs = driver.reset(GOLDEN_SCHEDULE[0][1])
    data = env._data
    initial = (data.qpos.copy(), data.qvel.copy())
    rows_at_reset = [driver.rows[r] for r in range(driver.lead + 1)]
    fell = False
    for seconds, command in GOLDEN_SCHEDULE:
        for _ in range(int(round(seconds / env.dt))):
            now = int(env._additional_carry.traj_state.subtraj_step_no)
            obs, fell = driver.step(obs, command)
            row = (now + 1 + driver.lead) % driver.trajectory_length
            record["command"].append(np.asarray(driver.shaper.command).copy())
            record["frame"].append(driver.matcher.frame)
            record["jumps"].append(driver.matcher.jumps)
            record["ref_qpos"].append(driver.rows[row][0])
            record["ref_qvel"].append(driver.rows[row][1])
            record["qpos"].append(data.qpos.copy())
            record["qvel"].append(data.qvel.copy())
            if fell:
                break
        if fell:
            break
    assert not fell, "the golden session fell: choose an easier schedule"
    bundle.array("golden_initial_qpos", initial[0], np.float64)
    bundle.array("golden_initial_qvel", initial[1], np.float64)
    bundle.array("golden_reset_ref_qpos", [r[0] for r in rows_at_reset], np.float64)
    bundle.array("golden_reset_ref_qvel", [r[1] for r in rows_at_reset], np.float64)
    for k, v in record.items():
        bundle.array(f"golden_{k}", v, np.int32 if k in ("frame", "jumps") else np.float64)
    bundle.constants["golden"] = {"schedule": [[s, list(c)] for s, c in GOLDEN_SCHEDULE], "steps": len(record["action"]),
                                  "agent": str(agent)}
    bundle.write_manifest()
    total = sum((bundle.out / a["file"]).stat().st_size for a in bundle.arrays.values()) / 1e6
    print(f"bundle: {bundle.out} ({len(bundle.arrays)} arrays, {total:.0f} MB; db {db.n_frames} frames; "
          f"golden {len(record['action'])} steps)")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("agent")
    parser.add_argument("--out", default=str(PROJECT_ROOT / "Saved" / "MuJoCo" / "unreal"))
    args = parser.parse_args()
    export(args.agent, args.out)


if __name__ == "__main__":
    main()
