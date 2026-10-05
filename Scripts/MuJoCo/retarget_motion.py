"""Project ATHLETE - fits human motion capture to the athlete's body (for LocoMuJoCo imitation learning).

Usage (LocoMuJoCo Python, see Scripts/SetupLocoMuJoCoPython.ps1):
    Intermediate/LocoMuJoCoPython/Scripts/python.exe Scripts/MuJoCo/retarget_motion.py [--source default] [--datasets walk run]
    ... retarget_motion.py --source bvh --files <a.bvh> [<b.bvh> ...] [--prefix 100style_] [--frames START END]

Sources, LocoMuJoCo's (motion it has already fitted to its human skeleton, SkeletonTorque) - both for
prototyping only, never shipping:
  - "default": LocoMuJoCo's own mocap (walk, run, walkturn, ...). License not stated. Normal gait
    (measured 2026-09-30: walk at 1.27 m/s, knee flexion median 13 deg, symmetric ankles).
  - "lafan1": LAFAN1 (CC BY-NC-ND 4.0, research only). As fitted to SkeletonTorque it walks CROUCHED (knee
    flexion median 46-51 deg, hips 3-8 cm low) with the left ankle ~15-20 deg more dorsiflexed than the right:
    an artifact of that conversion. Don't use it for training without fixing that.
and BVH files (bvh.py), any skeleton with the usual joints (BvhSource finds them by name). Licenses are the
files' own: check each library's terms before shipping (Docs/MotionData.md).

Method, per frame:
  1. Anatomical landmarks on the skeleton: hip, knee and ankle centers, heel, ball of the foot, shoulder,
     elbow and wrist centers, head.
  2. Rebuilt at the athlete's proportions, joint by joint from the hip center: each segment keeps the
     skeleton's direction but gets the athlete's length (pelvis width, thigh, shank, foot, trunk, shoulder
     width, upper arm, forearm, neck). The hip center path scales with leg length, so stride and speed scale too.
  3. Inverse kinematics on the athlete's model (damped least squares, warm-started from the previous frame,
     within the anatomical joint ranges) to put its landmarks there.
  4. Feet on the floor, by the lowest point of the sole capsules (the athlete's contact model,
     athlete_loco.env.configure_contacts). Two modes (--floor):
       - "frame": every frame shifted so a foot touches the ground. Right for walking (always a foot down)
         but it REMOVES RUNNING'S FLIGHT PHASES: the source run has both feet > 4 cm up in 40% of frames,
         the fitted one in none (measured 2026-10-04). Kept for LocoMuJoCo clips, so current training data
         doesn't change unasked.
       - "clip": one height for the whole clip, from the stance frames (the median of 0.5 s moving minima
         of the lowest sole point). Keeps flight phases. Default for BVH clips.
Joint velocities come from finite differences. The result is extended with body and site positions by
LocoMuJoCo and written where its loaders look for converted data after `loco-mujoco-set-all-caches --path <out>`:
  <out>/DEFAULT/mocap/AthleteReference/<task>.npz     (config: default_dataset_conf: {task: walk})
  <out>/LAFAN1/AthleteReference/<clip>.npz             (config: lafan1_dataset_conf: {dataset_name: ...})
  BVH clips go with the default ones (DEFAULT/mocap), named <prefix><file name>, lower case.
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
import bvh  # noqa: E402
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


IK_RESTART_ERROR_M = 0.05  # a landmark this far off triggers a fresh fit of the frame (sources with ik_restarts)


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
    ratios = {}
    for child, parent in CHAIN:
        source_length = np.linalg.norm(skeleton_points[child] - skeleton_points[parent])
        # A segment the source doesn't have (BVH: no heel) keeps the source's (zero) length; the feet's
        # heel and ball targets come from FootOrientation anyway.
        ratios[child] = np.linalg.norm(athlete_points[child] - athlete_points[parent]) / source_length \
            if source_length > 1e-6 else 1.0
    return ratios


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

    def __init__(self, source, athlete, athlete_data, frequency):
        self.local = {}
        for side, suffix in (("l", "Left"), ("r", "Right")):
            rotations, toe_positions = [], []
            for frame in range(source.n_frames):
                source.set_frame(frame)
                rotation, toe, back = source.foot(side)
                rotations.append(rotation)
                toe_positions.append(np.append(toe, back))  # [toes xyz, heel-end xyz]
            rotations, points = np.array(rotations), np.array(toe_positions)
            speed = np.r_[0.0, np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1) * frequency]
            planted = (points[:, 2] < np.percentile(points[:, 2], 25)) & (speed < 0.15)
            if source.flat_needs_heel_down:
                # The toe alone is also low and still at push-off, heel already up: calibrating "flat" there
                # tilted 100STYLE feet toes-down, so real flat stance came out toes-up and the ankles sat at
                # their dorsiflexion limit half the time (2026-10-04). Require the back of the foot down too.
                # Forefoot runners rarely put the heel down: widen the thresholds until there are enough frames.
                back_speed = np.r_[0.0, np.linalg.norm(np.diff(points[:, 3:5], axis=0), axis=1) * frequency]
                for share, still in ((25, 0.15), (35, 0.25), (50, 0.4)):
                    flat_frames = planted & (points[:, 5] < np.percentile(points[:, 5], share)) & (back_speed < still)
                    if flat_frames.sum() >= max(20, 0.01 * len(points)):
                        break
                planted = flat_frames if flat_frames.any() else planted
            # The foot's orientation relative to its own heading, averaged over planted frames = "flat".
            offsets = [yaw_matrix(p[:3] - p[3:]).T @ r for p, r in zip(points[planted], rotations[planted])]
            flat = nearest_rotation(np.mean(offsets, axis=0))
            ankle = athlete_data.xpos[mujoco.mj_name2id(athlete, mujoco.mjtObj.mjOBJ_BODY, f"Foot{suffix}")]
            full = "left" if side == "l" else "right"
            for point, site in ((f"heel_{side}", f"{full}_heel"), (f"ball_{side}", f"{full}_ball_of_foot")):
                offset = athlete_data.site_xpos[mujoco.mj_name2id(athlete, mujoco.mjtObj.mjOBJ_SITE, site)] - ankle
                self.local[point] = flat.T @ offset  # the athlete's foot points along +X in its reference pose
            self.planted_share = float(planted.mean())

    def apply(self, source, targets):
        """Heel and ball targets for the source's current frame."""
        for point, local in self.local.items():
            ankle = targets["ankle_" + point[-1]]
            targets[point] = ankle + source.foot(point[-1])[0] @ local
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


class SkeletonSource:
    """LocoMuJoCo's mocap on its SkeletonTorque model. set_frame(i), then points() and foot(side)."""

    def __init__(self, source, dataset):
        self.model = LocoEnv.registered_envs["SkeletonTorque"]()._model
        self.qpos, self.frequency = load_source(source, dataset, self.model)
        self.data = mujoco.MjData(self.model)
        self.name, self.folder = dataset, SOURCES[source][1]
        self.description = f"LocoMuJoCo {source} mocap on SkeletonTorque (prototyping only: see script)"
        self.leg_scale_from_lengths = False  # hip height in the first frame (these clips start upright)
        self.floor = "frame"
        self.flat_needs_heel_down = False  # unchanged for the current clips
        self.ik_restarts = False
        self.feet = {side: (mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, f"calcn_{side}"),
                            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, f"toes_{side}")) for side in "lr"}

    @property
    def n_frames(self):
        return len(self.qpos)

    def trim(self, n_frames):
        self.qpos = self.qpos[:n_frames]

    def set_frame(self, frame):
        self.data.qpos[:] = self.qpos[frame]
        mujoco.mj_kinematics(self.model, self.data)

    def points(self):
        return landmarks(self.model, self.data, 0)

    def foot(self, side):
        """(foot rotation 3x3, toe point, heel-end point) in the world."""
        calcn, toes = self.feet[side]
        return self.data.xmat[calcn].reshape(3, 3).copy(), self.data.xpos[toes].copy(), self.data.xpos[calcn].copy()


# BVH joint names per landmark, first match wins (100STYLE, CMU conversions, Mixamo-style, the self-test's).
# Shoulder, hip and knee are found as parents (of elbow, knee and ankle): "LeftShoulder" is the upper arm in
# some files and the collarbone in others.
BVH_NAMES = {
    "wrist": ("{S}Hand", "{S}Wrist", "{s}_wrist", "Hand{S}"),
    "ankle": ("{S}Foot", "{S}Ankle", "{s}_ankle", "Foot{S}"),
    "ball": ("{S}ToeBase", "{S}Toe", "{S}Toes", "{s}_toe", "Ball{S}", "{S}Foot_end", "{S}Ankle_end"),
    "head": ("Head", "head"),
}


class BvhSource:
    """A BVH file (or a frame range of it). Same interface as SkeletonSource."""

    def __init__(self, path, prefix="", frames=None, scale=None, y_up=True):
        motion = bvh.read(path)
        self.path, self.name, self.folder = Path(path), f"{prefix}{Path(path).stem}".lower(), "DEFAULT/mocap"
        self.description = f"BVH {Path(path).name}"
        self.leg_scale_from_lengths = True
        self.floor = "clip"
        self.flat_needs_heel_down = True
        self.ik_restarts = True
        self.frequency = motion.frequency
        values = motion.values if frames is None else motion.values[frames[0]:frames[1]]
        positions, rotations = bvh.Motion(motion.joints, values, motion.frame_time).forward_kinematics()
        self.ids = self._find(motion)
        if scale is None:  # metres per file unit, from the hip height above the ankle (~0.8 m in people)
            up = 1 if y_up else 2
            hip_height = np.median(positions[:, self.ids["hip_l"], up] - positions[:, self.ids["ankle_l"], up])
            scale = min((0.01, 0.0254, 1.0), key=lambda unit: abs(np.log(hip_height * unit / 0.8)))
        self.scale = scale
        self.positions, self.rotations = bvh.world(positions, scale, y_up), bvh.world_rotation(rotations, y_up)

    def _find(self, motion):
        def first(kind, side):
            for pattern in BVH_NAMES[kind]:
                name = pattern.format(S={"l": "Left", "r": "Right"}[side], s=side)
                if name in motion.index:
                    return motion.index[name]
            raise KeyError(f"{self.path.name}: no {kind} joint for side {side} (tried {BVH_NAMES[kind]})")

        def parent(i):
            return motion.joints[i].parent

        ids = {}
        for side in "lr":
            ids[f"wrist_{side}"] = first("wrist", side)
            ids[f"elbow_{side}"] = parent(ids[f"wrist_{side}"])
            ids[f"shoulder_{side}"] = parent(ids[f"elbow_{side}"])
            ids[f"ankle_{side}"] = first("ankle", side)
            ids[f"knee_{side}"] = parent(ids[f"ankle_{side}"])
            ids[f"hip_{side}"] = parent(ids[f"knee_{side}"])
            ids[f"ball_{side}"] = first("ball", side)
        ids["head"] = first("head", "l")
        # The athlete's head landmark is the head's centre; a BVH "Head" joint sits at the base of the skull,
        # so use the midpoint to its end site (top of the head) when the file has one.
        end = motion.index.get(f"{motion.joints[ids['head']].name}_end")
        self.head_end = end if end is not None and np.linalg.norm(motion.joints[end].offset) > 0 else None
        return ids

    @property
    def n_frames(self):
        return len(self.positions)

    def trim(self, n_frames):
        self.positions, self.rotations = self.positions[:n_frames], self.rotations[:n_frames]

    def set_frame(self, frame):
        self.frame = frame

    def points(self):
        p = self.positions[self.frame]
        points = {name: p[i].copy() for name, i in self.ids.items()}
        if self.head_end is not None:
            points["head"] = 0.5 * (points["head"] + p[self.head_end])
        points["heel_l"], points["heel_r"] = points["ankle_l"].copy(), points["ankle_r"].copy()  # no heel joint
        points["H"] = 0.5 * (points["hip_l"] + points["hip_r"])
        points["S"] = 0.5 * (points["shoulder_l"] + points["shoulder_r"])
        return points

    def foot(self, side):
        """(foot rotation 3x3, toe point, heel-end point) in the world; the ankle stands in for the heel."""
        f = self.frame
        return (self.rotations[f, self.ids[f"ankle_{side}"]].copy(), self.positions[f, self.ids[f"ball_{side}"]].copy(),
                self.positions[f, self.ids[f"ankle_{side}"]].copy())


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


def retarget(source, out_dir, max_seconds=None):
    """Fits a source (SkeletonSource or BvhSource) to the athlete; writes the clip and its report."""
    started = time.time()
    frequency, dataset = source.frequency, source.name
    if max_seconds:
        source.trim(int(max_seconds * frequency))

    athlete_env = LocoEnv.registered_envs[ENV_NAME]()
    athlete = athlete_env._model

    # Proportions, from the skeleton's first frame and the athlete's reference pose.
    source.set_frame(0)
    fitter = Fitter(athlete)
    mujoco.mj_kinematics(athlete, fitter.data)
    skeleton_points = source.points()
    athlete_points = landmarks(athlete, fitter.data, 1)
    ratios = segment_ratios(skeleton_points, athlete_points)
    if source.leg_scale_from_lengths:
        # Pose-independent: thigh + shank. (A clip's first frame can be mid-stride, knees bent: the hip
        # height there scaled a running self-test's path by 1.04 instead of 1.)
        def leg(points):
            return sum(np.linalg.norm(points[f"hip_{s}"] - points[f"knee_{s}"]) +
                       np.linalg.norm(points[f"knee_{s}"] - points[f"ankle_{s}"]) for s in "lr")
        leg_scale = leg(athlete_points) / leg(skeleton_points)
    else:
        leg_scale = athlete_points["H"][2] / skeleton_points["H"][2]
    feet = FootOrientation(source, athlete, fitter.data, frequency)

    # Fit every frame, warm-starting from the previous one.
    n_frames = source.n_frames
    qpos = np.zeros((n_frames, athlete.nq))
    errors = np.zeros((n_frames, len(LANDMARKS)))
    current = athlete.qpos0.copy()
    current[:3] = rebuild(skeleton_points, ratios, leg_scale)["H"]  # start near the first frame
    restarts = 0
    for frame in range(n_frames):
        source.set_frame(frame)
        targets = feet.apply(source, rebuild(source.points(), ratios, leg_scale))
        current, errors[frame] = fitter.solve(current, targets, iterations=50 if frame == 0 else 15)
        if source.ik_restarts and errors[frame].max() > IK_RESTART_ERROR_M:
            # Warm starts can lock a limb into a wrong solution for a whole clip (100STYLE: one arm twisted,
            # shoulder at its limit in up to 100% of frames, wrist 20-35 cm off). Retry from the neutral
            # pose (root kept) and keep whichever fits better.
            fresh = athlete.qpos0.copy()
            fresh[:7] = current[:7]
            retry, retry_errors = fitter.solve(fresh, targets, iterations=50)
            if retry_errors.sum() < errors[frame].sum():
                current, errors[frame] = retry, retry_errors
                restarts += 1
        qpos[frame] = current

    # Feet on the floor (see the module docstring).
    data = mujoco.MjData(athlete)
    lowest = np.zeros(n_frames)
    for frame in range(n_frames):
        data.qpos[:] = qpos[frame]
        mujoco.mj_kinematics(athlete, data)
        lowest[frame] = lowest_foot_point(athlete, data)
    if source.floor == "frame":
        qpos[:, 2] -= lowest
    else:
        window = max(1, min(n_frames, int(0.5 * frequency)))
        moving_min = np.lib.stride_tricks.sliding_window_view(lowest, window).min(axis=1)
        qpos[:, 2] -= np.median(moving_min)
    airborne = float(np.mean(lowest - (lowest if source.floor == "frame" else np.median(moving_min)) > 0.02))

    # Central differences with MuJoCo's own position differencing (the root's angular velocity in the
    # root's frame, as MuJoCo defines free-joint velocities). First and last frames dropped.
    qvel = np.zeros((n_frames - 2, athlete.nv))
    for frame in range(1, n_frames - 1):
        mujoco.mj_differentiatePos(athlete, qvel[frame - 1], 2.0 / frequency, qpos[frame - 1], qpos[frame + 1])
    qpos, errors, n_frames = qpos[1:-1], errors[1:-1], n_frames - 2

    target = out_dir / source.folder / ENV_NAME / f"{dataset}.npz"
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
        "dataset": dataset, "source": source.description,
        "frames": n_frames, "frequency_hz": frequency, "duration_s": n_frames / frequency,
        "leg_scale": round(float(leg_scale), 4), "segment_ratios": {k: round(float(v), 4) for k, v in ratios.items()},
        "fit_error_m": {name: {"mean": round(float(errors[:, i].mean()), 4), "p95": round(float(np.percentile(errors[:, i], 95)), 4)}
                        for i, name in enumerate(LANDMARKS)},
        "joints_at_limit_share_of_frames": at_limit,
        "stance_foot_slide_mps_median": {"left": round(slide[0], 4), "right": round(slide[1], 4)},
        "root_speed_mps_median": round(float(np.median(np.linalg.norm(qvel[:, :2], axis=1))), 3),
        "floor": source.floor, "share_of_frames_both_feet_above_2cm": round(airborne, 3),
        "ik_restarts_kept": restarts,
        "output": str(target), "seconds": round(time.time() - started, 1),
    }
    target.with_name(f"{dataset}_report.json").write_text(json.dumps(report, indent=2))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--source", choices=sorted(SOURCES) + ["bvh"], default="default")
    parser.add_argument("--datasets", nargs="+", default=["walk"])
    parser.add_argument("--files", nargs="+", default=[], help="BVH files (--source bvh)")
    parser.add_argument("--prefix", default="", help="BVH: clip name prefix, e.g. 100style_")
    parser.add_argument("--frames", nargs=2, type=int, default=None, metavar=("START", "END"),
                        help="BVH: only these file frames (e.g. without the T-poses)")
    parser.add_argument("--scale", type=float, default=None, help="BVH: metres per file unit (default: guessed)")
    parser.add_argument("--z-up", action="store_true", help="BVH: the file is Z up (default Y up)")
    parser.add_argument("--floor", choices=["frame", "clip"], default=None,
                        help="foot placement: every frame (removes flight phases) or one height per clip; "
                             "default frame for LocoMuJoCo sources, clip for BVH")
    parser.add_argument("--out", default=str(PROJECT_ROOT / "Saved" / "MuJoCo" / "motions"))
    parser.add_argument("--max-seconds", type=float, default=None, help="only the first N seconds (for quick checks)")
    args = parser.parse_args()
    if args.source == "bvh":
        sources = (BvhSource(f, args.prefix, args.frames, args.scale, not args.z_up) for f in args.files)
    else:
        sources = (SkeletonSource(args.source, dataset) for dataset in args.datasets)
    for source in sources:
        source.floor = args.floor or source.floor
        print(json.dumps(retarget(source, Path(args.out), args.max_seconds), indent=2))


if __name__ == "__main__":
    main()
