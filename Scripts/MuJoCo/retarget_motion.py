"""Project ATHLETE - fits human motion capture to the athlete's body (for LocoMuJoCo imitation learning).

Usage (LocoMuJoCo Python, see Scripts/SetupLocoMuJoCoPython.ps1):
    Intermediate/LocoMuJoCoPython/Scripts/python.exe Scripts/MuJoCo/retarget_motion.py [--source default] [--datasets walk run]

Sources: motion LocoMuJoCo has already fitted to its human skeleton (SkeletonTorque, torque-driven, no helper
forces on the pelvis). Both are for prototyping only, never shipping:
  - "default": LocoMuJoCo's own mocap (walk, run, walkturn, ...). License not stated. Normal gait
    (measured 2026-09-30: walk at 1.27 m/s, knee flexion median 13 deg, symmetric ankles).
  - "lafan1": LAFAN1 (CC BY-NC-ND 4.0, research only). As fitted to SkeletonTorque it walks CROUCHED (knee
    flexion median 46-51 deg, hips 3-8 cm low) with the left ankle ~15-20 deg more dorsiflexed than the right:
    an artifact of that conversion. Don't use it for training without fixing that.

Method, per frame:
  1. Anatomical landmarks on the skeleton: hip, knee and ankle centers, heel, ball of the foot, shoulder,
     elbow and wrist centers, head.
  2. Rebuilt at the athlete's proportions, joint by joint from the hip center: each segment keeps the
     skeleton's direction but gets the athlete's length (pelvis width, thigh, shank, foot, trunk, shoulder
     width, upper arm, forearm, neck). The hip center path scales with leg length, so stride and speed scale too.
  3. Inverse kinematics on the athlete's model (damped least squares, warm-started from the previous frame,
     within the anatomical joint ranges) to put its landmarks there.
  4. Feet on the floor: each frame is shifted vertically so the lowest point of the sole capsules (the
     athlete's contact model, athlete_loco.env.configure_contacts) touches the ground (walking always has a
     foot down).
Joint velocities come from finite differences. The result is extended with body and site positions by
LocoMuJoCo and written where its loaders look for converted data after `loco-mujoco-set-all-caches --path <out>`:
  <out>/DEFAULT/mocap/AthleteReference/<task>.npz     (config: default_dataset_conf: {task: walk})
  <out>/LAFAN1/AthleteReference/<clip>.npz             (config: lafan1_dataset_conf: {dataset_name: ...})
A report with fitting errors, joint-limit use and foot sliding is written next to each.
"""

import argparse
import json
import sys
import time
from dataclasses import replace
from pathlib import Path

import jax.numpy as jnp
import mujoco
import numpy as np
from huggingface_hub import hf_hub_download

sys.path.insert(0, str(Path(__file__).resolve().parent))
import athlete_loco  # noqa: E402  (registers the athlete environments; its contact model places the feet)
from loco_mujoco.datasets.data_generation import ExtendTrajData  # noqa: E402
from loco_mujoco.environments import LocoEnv  # noqa: E402
from loco_mujoco.trajectory import (Trajectory, TrajectoryData, TrajectoryInfo, TrajectoryModel,  # noqa: E402
                                    interpolate_trajectories)

PROJECT_ROOT = Path(__file__).resolve().parents[2]
ENV_NAME = "AthleteReference"

# Landmark -> (skeleton element, athlete element). ("body", name) = the body's origin (a joint center).
LANDMARKS = {
    "hip_l": (("body", "femur_l"), ("body", "ThighLeft")),
    "hip_r": (("body", "femur_r"), ("body", "ThighRight")),
    "knee_l": (("body", "tibia_l"), ("body", "ShankLeft")),
    "knee_r": (("body", "tibia_r"), ("body", "ShankRight")),
    "ankle_l": (("body", "talus_l"), ("body", "FootLeft")),
    "ankle_r": (("body", "talus_r"), ("body", "FootRight")),
    "heel_l": (("body", "calcn_l"), ("site", "left_heel")),
    "heel_r": (("body", "calcn_r"), ("site", "right_heel")),
    "ball_l": (("body", "toes_l"), ("site", "left_ball_of_foot")),
    "ball_r": (("body", "toes_r"), ("site", "right_ball_of_foot")),
    "shoulder_l": (("body", "humerus_l"), ("body", "UpperArmLeft")),
    "shoulder_r": (("body", "humerus_r"), ("body", "UpperArmRight")),
    "elbow_l": (("body", "ulna_l"), ("body", "ForearmLeft")),
    "elbow_r": (("body", "ulna_r"), ("body", "ForearmRight")),
    "wrist_l": (("body", "hand_l"), ("body", "HandLeft")),
    "wrist_r": (("body", "hand_r"), ("body", "HandRight")),
    "head": (("site", "head_mimic"), ("site", "head_mimic")),
}
# How strongly the fit pulls each landmark (joint centers first; the head only steers the neck).
WEIGHTS = {"head": 0.5, "heel_l": 0.7, "heel_r": 0.7, "ball_l": 0.7, "ball_r": 0.7}
# Chains: landmark <- parent landmark ("H" = hip center, "S" = shoulder center: virtual landmarks).
CHAIN = [("hip_l", "H"), ("hip_r", "H"), ("knee_l", "hip_l"), ("knee_r", "hip_r"), ("ankle_l", "knee_l"),
         ("ankle_r", "knee_r"), ("heel_l", "ankle_l"), ("heel_r", "ankle_r"), ("ball_l", "ankle_l"),
         ("ball_r", "ankle_r"), ("S", "H"), ("shoulder_l", "S"), ("shoulder_r", "S"), ("elbow_l", "shoulder_l"),
         ("elbow_r", "shoulder_r"), ("wrist_l", "elbow_l"), ("wrist_r", "elbow_r"), ("head", "S")]


def element_position(model, data, element):
    kind, name = element
    if kind == "body":
        return data.xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)].copy()
    return data.site_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)].copy()


def landmarks(model, data, side):
    """All landmarks plus the virtual hip and shoulder centers, for side 0 (skeleton) or 1 (athlete)."""
    points = {name: element_position(model, data, pair[side]) for name, pair in LANDMARKS.items()}
    points["H"] = 0.5 * (points["hip_l"] + points["hip_r"])
    points["S"] = 0.5 * (points["shoulder_l"] + points["shoulder_r"])
    return points


def segment_ratios(skeleton_points, athlete_points):
    """Athlete segment length / skeleton segment length for every link of CHAIN."""
    return {child: np.linalg.norm(athlete_points[child] - athlete_points[parent])
            / np.linalg.norm(skeleton_points[child] - skeleton_points[parent]) for child, parent in CHAIN}


def rebuild(skeleton_points, ratios, leg_scale):
    """The skeleton's landmarks rebuilt with the athlete's segment lengths."""
    target = {"H": skeleton_points["H"] * np.array([leg_scale, leg_scale, leg_scale])}
    for child, parent in CHAIN:
        target[child] = target[parent] + (skeleton_points[child] - skeleton_points[parent]) * ratios[child]
    return target


def yaw_matrix(direction):
    yaw = np.arctan2(direction[1], direction[0])
    return np.array([[np.cos(yaw), -np.sin(yaw), 0.0], [np.sin(yaw), np.cos(yaw), 0.0], [0.0, 0.0, 1.0]])


def nearest_rotation(matrix):
    u, _, vt = np.linalg.svd(matrix)
    return u @ np.diag([1.0, 1.0, np.linalg.det(u @ vt)]) @ vt


class FootOrientation:
    """
    Heel and ball of the foot follow the skeleton foot's ROTATION, not its landmarks: the two feet are
    built differently (the skeleton's calcaneus marker isn't where the athlete's heel is).

    "Flat" is calibrated per foot on the frames where that foot is planted (low and nearly still): a
    foot bearing weight in walking is flat on the ground. Measured 2026-09-30: this clip's skeleton feet
    are NOT flat in its own neutral (left ankle 31 deg dorsiflexed on average, the feet 9 deg toes-up and
    8 deg toes-down in the first frame; LocoMuJoCo issue #88 reports odd feet on this skeleton), so its
    own angles can't be used directly. The motion between plantings (heel rise, toe-off, swing) is kept.
    """

    def __init__(self, skeleton, skeleton_frames, athlete, athlete_data, frequency):
        self.local = {}
        data = mujoco.MjData(skeleton)
        for side, suffix in (("l", "Left"), ("r", "Right")):
            calcn = mujoco.mj_name2id(skeleton, mujoco.mjtObj.mjOBJ_BODY, f"calcn_{side}")
            toes = mujoco.mj_name2id(skeleton, mujoco.mjtObj.mjOBJ_BODY, f"toes_{side}")
            rotations, toe_positions = [], []
            for qpos in skeleton_frames:
                data.qpos[:] = qpos
                mujoco.mj_kinematics(skeleton, data)
                rotations.append(data.xmat[calcn].reshape(3, 3).copy())
                toe_positions.append(data.xpos[toes].copy())
                toe_positions[-1] = np.append(toe_positions[-1], data.xpos[calcn])  # [toes xyz, calcn xyz]
            rotations, points = np.array(rotations), np.array(toe_positions)
            speed = np.r_[0.0, np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1) * frequency]
            planted = (points[:, 2] < np.percentile(points[:, 2], 25)) & (speed < 0.15)
            # The foot's orientation relative to its own heading, averaged over planted frames = "flat".
            offsets = [yaw_matrix(p[:3] - p[3:]).T @ r for p, r in zip(points[planted], rotations[planted])]
            flat = nearest_rotation(np.mean(offsets, axis=0))
            ankle = athlete_data.xpos[mujoco.mj_name2id(athlete, mujoco.mjtObj.mjOBJ_BODY, f"Foot{suffix}")]
            full = "left" if side == "l" else "right"
            for point, site in ((f"heel_{side}", f"{full}_heel"), (f"ball_{side}", f"{full}_ball_of_foot")):
                offset = athlete_data.site_xpos[mujoco.mj_name2id(athlete, mujoco.mjtObj.mjOBJ_SITE, site)] - ankle
                self.local[point] = (calcn, flat.T @ offset)  # the athlete's foot points along +X in its reference pose
            self.planted_share = float(planted.mean())

    def apply(self, skeleton_data, targets):
        for point, (calcn, local) in self.local.items():
            ankle = targets["ankle_" + point[-1]]
            targets[point] = ankle + skeleton_data.xmat[calcn].reshape(3, 3) @ local
        return targets


class Fitter:
    """Damped least-squares inverse kinematics on the athlete's model, within the joint ranges."""

    def __init__(self, model):
        self.model = model
        self.data = mujoco.MjData(model)
        self.elements = [(name, pair[1]) for name, pair in LANDMARKS.items()]
        self.weights = np.repeat([WEIGHTS.get(name, 1.0) for name, _ in self.elements], 3)
        self.hinges = [j for j in range(model.njnt) if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE]
        self.jacp = np.zeros((3, model.nv))

    def residual_and_jacobian(self, targets):
        m, d = self.model, self.data
        residual = np.zeros(3 * len(self.elements))
        jacobian = np.zeros((3 * len(self.elements), m.nv))
        for i, (name, (kind, element)) in enumerate(self.elements):
            if kind == "body":
                body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, element)
                mujoco.mj_jacBody(m, d, self.jacp, None, body)
                position = d.xpos[body]
            else:
                site = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, element)
                mujoco.mj_jacSite(m, d, self.jacp, None, site)
                position = d.site_xpos[site]
            residual[3 * i:3 * i + 3] = targets[name] - position
            jacobian[3 * i:3 * i + 3] = self.jacp
        return residual, jacobian

    def solve(self, qpos, targets, iterations=30, damping=1e-3, tolerance=1e-6):
        m, d = self.model, self.data
        d.qpos[:] = qpos
        W = self.weights
        for _ in range(iterations):
            mujoco.mj_kinematics(m, d)
            mujoco.mj_comPos(m, d)
            residual, jacobian = self.residual_and_jacobian(targets)
            JW = jacobian * W[:, None]
            step = np.linalg.solve(JW.T @ jacobian + damping * np.eye(m.nv), JW.T @ residual)
            mujoco.mj_integratePos(m, d.qpos, step, 1.0)
            for j in self.hinges:
                address = m.jnt_qposadr[j]
                d.qpos[address] = np.clip(d.qpos[address], *m.jnt_range[j])
            if np.linalg.norm(step) < tolerance:
                break
        mujoco.mj_kinematics(m, d)
        mujoco.mj_comPos(m, d)
        residual, _ = self.residual_and_jacobian(targets)
        return d.qpos.copy(), np.linalg.norm(residual.reshape(-1, 3), axis=1)


def lowest_foot_point(model, data):
    """Lowest point of the feet (world z): the sole capsules the athlete stands on (athlete_loco.env)."""
    return athlete_loco.env.lowest_sole_point(model, data)


SOURCES = {  # source -> (file on LocoMuJoCo's Hugging Face dataset repo, output folder under --out)
    "default": ("DefaultDatasets/mocap/SkeletonTorque/{}.npz", "DEFAULT/mocap"),
    "lafan1": ("Lafan1/mocap/SkeletonTorque/{}.npz", "LAFAN1"),
}


def load_source(source, dataset, skeleton):
    """The clip as qpos rows of the SkeletonTorque model (joints matched by name), and its frequency."""
    traj = Trajectory.load(hf_hub_download(repo_id="robfiras/loco-mujoco-datasets", repo_type="dataset",
                                           filename=SOURCES[source][0].format(dataset)))
    rows = np.asarray(traj.data.qpos)
    qpos = np.tile(skeleton.qpos0, (len(rows), 1))
    column = 0
    for name in traj.info.joint_names:  # some files carry extra foot joints the model doesn't have
        size = 7 if name == "root" else 1
        joint = mujoco.mj_name2id(skeleton, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint >= 0:
            address = skeleton.jnt_qposadr[joint]
            qpos[:, address:address + size] = rows[:, column:column + size]
        column += size
    return qpos, float(traj.info.frequency)


def save_motion(athlete, qpos, qvel, frequency, target, split_points=None):
    """
    Saves athlete motion (qpos/qvel per frame) as a LocoMuJoCo trajectory, extended with body and site
    positions at the environment's control rate. split_points: start frame of each separate clip in the
    arrays, plus the total (default: one clip).
    """
    n_frames = len(qpos)
    joint_names = [mujoco.mj_id2name(athlete, mujoco.mjtObj.mjOBJ_JOINT, j) for j in range(athlete.njnt)]
    info = TrajectoryInfo(joint_names=joint_names, frequency=frequency,
                          model=TrajectoryModel(njnt=athlete.njnt, jnt_type=jnp.array(athlete.jnt_type)))
    split_points = [0, n_frames] if split_points is None else split_points
    traj_data = TrajectoryData(qpos=jnp.array(qpos), qvel=jnp.array(qvel), split_points=jnp.array(split_points))
    traj = Trajectory(info=info, data=traj_data)

    env = LocoEnv.registered_envs[ENV_NAME](th_params=dict(random_start=False, fixed_start_conf=(0, 0)))
    traj_data, traj_info = interpolate_trajectories(traj.data, traj.info, 1.0 / env.dt)
    env.load_trajectory(Trajectory(info=traj_info, data=traj_data), warn=False)
    traj_data, traj_info = env.th.traj.data, env.th.traj.info
    callback = ExtendTrajData(env, model=env._model, n_samples=traj_data.n_samples)
    env.play_trajectory(n_episodes=env.th.n_trajectories, render=False, callback_class=callback)
    traj_data, traj_info = callback.extend_trajectory_data(traj_data, traj_info)
    extended = replace(env.th.traj, data=traj_data, info=traj_info)
    target.parent.mkdir(parents=True, exist_ok=True)
    extended.save(str(target))


def retarget(source, dataset, out_dir, max_seconds=None):
    started = time.time()
    skeleton = LocoEnv.registered_envs["SkeletonTorque"]()._model
    source_qpos, frequency = load_source(source, dataset, skeleton)
    if max_seconds:
        source_qpos = source_qpos[:int(max_seconds * frequency)]

    skeleton_data = mujoco.MjData(skeleton)
    athlete_env = LocoEnv.registered_envs[ENV_NAME]()
    athlete = athlete_env._model

    # Proportions, from the skeleton's first frame and the athlete's reference pose.
    skeleton_data.qpos[:] = source_qpos[0]
    mujoco.mj_kinematics(skeleton, skeleton_data)
    fitter = Fitter(athlete)
    mujoco.mj_kinematics(athlete, fitter.data)
    skeleton_points = landmarks(skeleton, skeleton_data, 0)
    athlete_points = landmarks(athlete, fitter.data, 1)
    ratios = segment_ratios(skeleton_points, athlete_points)
    leg_scale = athlete_points["H"][2] / skeleton_points["H"][2]
    feet = FootOrientation(skeleton, source_qpos, athlete, fitter.data, frequency)

    # Fit every frame, warm-starting from the previous one.
    n_frames = len(source_qpos)
    qpos = np.zeros((n_frames, athlete.nq))
    errors = np.zeros((n_frames, len(LANDMARKS)))
    current = athlete.qpos0.copy()
    current[:3] = rebuild(skeleton_points, ratios, leg_scale)["H"]  # start near the first frame
    for frame in range(n_frames):
        skeleton_data.qpos[:] = source_qpos[frame]
        mujoco.mj_kinematics(skeleton, skeleton_data)
        targets = feet.apply(skeleton_data, rebuild(landmarks(skeleton, skeleton_data, 0), ratios, leg_scale))
        current, errors[frame] = fitter.solve(current, targets, iterations=50 if frame == 0 else 15)
        qpos[frame] = current

    # Feet on the floor.
    data = mujoco.MjData(athlete)
    for frame in range(n_frames):
        data.qpos[:] = qpos[frame]
        mujoco.mj_kinematics(athlete, data)
        qpos[frame, 2] -= lowest_foot_point(athlete, data)

    # Central differences with MuJoCo's own position differencing (the root's angular velocity in the
    # root's frame, as MuJoCo defines free-joint velocities). First and last frames dropped.
    qvel = np.zeros((n_frames - 2, athlete.nv))
    for frame in range(1, n_frames - 1):
        mujoco.mj_differentiatePos(athlete, qvel[frame - 1], 2.0 / frequency, qpos[frame - 1], qpos[frame + 1])
    qpos, errors, n_frames = qpos[1:-1], errors[1:-1], n_frames - 2

    target = out_dir / SOURCES[source][1] / ENV_NAME / f"{dataset}.npz"
    save_motion(athlete, qpos, qvel, frequency, target)

    # Report: how well the body could follow, how much of the joint ranges it used, and foot sliding.
    hinge_ids = fitter.hinges
    at_limit = {}
    for j in hinge_ids:
        q = qpos[:, athlete.jnt_qposadr[j]]
        low, high = athlete.jnt_range[j]
        share = float(np.mean((q <= low + 1e-3) | (q >= high - 1e-3)))
        if share > 0.01:
            at_limit[mujoco.mj_id2name(athlete, mujoco.mjtObj.mjOBJ_JOINT, j)] = round(share, 3)
    slide = []
    for side, geom_name in (("left", "FootLeft"), ("right", "FootRight")):
        site = mujoco.mj_name2id(athlete, mujoco.mjtObj.mjOBJ_SITE, f"{side}_foot_mimic")
        positions, heights = [], []
        for frame in range(n_frames):
            data.qpos[:] = qpos[frame]
            mujoco.mj_kinematics(athlete, data)
            positions.append(data.site_xpos[site].copy())
            heights.append(data.site_xpos[site][2])
        positions, heights = np.array(positions), np.array(heights)
        speed = np.linalg.norm(np.diff(positions[:, :2], axis=0), axis=1) * frequency
        planted = heights[1:] < np.percentile(heights, 30)  # the foot's lowest 30% of frames: stance
        slide.append(float(np.median(speed[planted])))
    report = {
        "dataset": dataset, "source": f"LocoMuJoCo {source} mocap on SkeletonTorque (prototyping only: see script)",
        "frames": n_frames, "frequency_hz": frequency, "duration_s": n_frames / frequency,
        "leg_scale": round(float(leg_scale), 4), "segment_ratios": {k: round(float(v), 4) for k, v in ratios.items()},
        "fit_error_m": {name: {"mean": round(float(errors[:, i].mean()), 4), "p95": round(float(np.percentile(errors[:, i], 95)), 4)}
                        for i, name in enumerate(LANDMARKS)},
        "joints_at_limit_share_of_frames": at_limit,
        "stance_foot_slide_mps_median": {"left": round(slide[0], 4), "right": round(slide[1], 4)},
        "root_speed_mps_median": round(float(np.median(np.linalg.norm(qvel[:, :2], axis=1))), 3),
        "output": str(target), "seconds": round(time.time() - started, 1),
    }
    target.with_name(f"{dataset}_report.json").write_text(json.dumps(report, indent=2))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--source", choices=sorted(SOURCES), default="default")
    parser.add_argument("--datasets", nargs="+", default=["walk"])
    parser.add_argument("--out", default=str(PROJECT_ROOT / "Saved" / "MuJoCo" / "motions"))
    parser.add_argument("--max-seconds", type=float, default=None, help="only the first N seconds (for quick checks)")
    args = parser.parse_args()
    for dataset in args.datasets:
        print(json.dumps(retarget(args.source, dataset, Path(args.out), args.max_seconds), indent=2))


if __name__ == "__main__":
    main()
