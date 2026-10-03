"""Project ATHLETE - motion matching: controller input -> a reference motion from the athlete's mocap.

Usage (LocoMuJoCo Python):
    Intermediate/LocoMuJoCoPython/Scripts/python.exe Scripts/MuJoCo/motion_matching.py check
    Intermediate/LocoMuJoCoPython/Scripts/python.exe Scripts/MuJoCo/motion_matching.py generate [--count 6 --seconds 300]

The kinematic half of DReCon-style control (Bergamin et al. 2019, "DReCon: Data-Driven Responsive Control
of Physics-Based Characters"). For Milestone 4 a controller sends desired movement: forward and sideways
speed and a turning rate, in the athlete's own frame. Motion matching picks the mocap that best does it,
and a trained physics policy (a DeepMimic tracker, PPO) makes the simulated athlete follow that motion with
its own capped joint torques. Motion matching is the standard game technique (Clavet, Ubisoft 2016; Holden
et al. 2020); Unreal's Pose Search plugin is the in-engine equivalent.

Database: the mocap clips fitted to the athlete (retarget_motion.py) at 100 Hz. Each frame gets features in
the character's heading frame (the yaw of the pelvis's forward axis):
  - where the body is now: foot positions (relative to the pelvis), foot velocities, pelvis velocity;
  - where the motion goes next: pelvis position and facing 0.33, 0.67 and 1.0 s ahead.
Each feature group is normalized by its own standard deviation, then weighted (FEATURE_WEIGHTS).

Search: every 0.1 s, at a clip's end, or when the command changes. The matcher compares a query with every
frame. It jumps to the best one only if that beats simply playing on; frames within 0.2 s of the playing
one are skipped, so it can't loop back into the moment it just played. The query is the current frame's
pose features plus the trajectory the controller wants.
  - Inertialization hides the jumps: the difference between the old and new pose decays to zero over
    ~0.1 s (critically damped) instead of cross-fading two animations.
  - The desired trajectory comes from the controller's own velocity and turning rate. These approach the
    command with a first-order lag (CONTROL_HALFLIFE_S) and are integrated over 1 s. Deriving it from the
    animation instead pulled the search back while a jump was still blending (stuck walking at 1.6 m/s
    when 2.5 was commanded).

Root motion: the kinematic character's position and heading integrate the database's own pelvis velocity
and turning rate, so the reference moves exactly as the mocap moves. The pelvis height is corrected so the
lowest foot point stays where the mocap frame has it; blends otherwise put feet ~5 cm into or above the floor.

The tracker trains on clips this matcher generates from random commands ("generate"), next to the mocap
clips (LocoMuJoCo starts episodes in a random FILE, then a random frame: the number of files sets the share). It then sees in training the same kind of reference it follows at run time, jumps included. This is
DReCon's recipe.
"""

import argparse
import json
import sys
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parent))  # athlete_loco (the athlete's contact model)

PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MOTIONS = PROJECT_ROOT / "Saved" / "MuJoCo" / "motions"
FREQUENCY = 100.0  # Hz: the clips' rate and the policy's control rate
DT = 1.0 / FREQUENCY
# The mocap fitted to the athlete, plus left/right mirrored copies of the moving clips (mirror_motion.py):
# turns both ways in equal measure (the turning clip alone turns left on average).
CLIPS = ("walk", "run", "walkturn", "random_walk", "stepinplace1", "stepinplace2",
         "walk_mirror", "run_mirror", "walkturn_mirror", "random_walk_mirror")
START_CLIP = "stepinplace1"
FOOT_SITES = ("left_foot_mimic", "right_foot_mimic")
FUTURE_FRAMES = (33, 67, 100)  # trajectory samples 0.33 / 0.67 / 1.0 s ahead
# Tuned 2026-10-01 on the kinematic check below (Holden's defaults 0.75/1/1/1/1.5 never left the first steps
# of a walk: this data has few starts, and pose terms outweighed where the controller wanted to go).
FEATURE_WEIGHTS = {"foot_position": 0.5, "foot_velocity": 0.5, "pelvis_velocity": 0.75,
                   "future_position": 2.0, "future_facing": 1.5}
SEARCH_INTERVAL_FRAMES = 10
IGNORE_SURROUNDING_FRAMES = 20  # candidates this close ahead of the playing frame (same clip) are skipped,
IGNORE_BEHIND_FRAMES = 100      # and this far behind it: never replay the moment just played (it loops)
JUMP_MARGIN = 2.0               # jump only if the best frame's cost is this much below playing on (fewer, cleaner jumps: 6 -> 1.4/s walking)
BLEND_HALFLIFE_S = 0.1
CONTROL_HALFLIFE_S = 0.25
N_OFFSETS_EXTRA = 1 + 3 + 2 + 1  # height, tilt (rotation vector), pelvis velocity xy, turning rate


def heading(quat_wxyz):
    """Yaw of the pelvis's forward (x) axis, from MuJoCo quaternions (w, x, y, z), shape (..., 4)."""
    w, x, y, z = np.moveaxis(np.asarray(quat_wxyz), -1, 0)
    return np.arctan2(2 * (x * y + w * z), 1 - 2 * (y * y + z * z))


def rotate_2d(vectors, angle):
    c, s = np.cos(angle), np.sin(angle)
    x, y = vectors[..., 0], vectors[..., 1]
    return np.stack([c * x - s * y, s * x + c * y], axis=-1)


def wrap(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def yaw_rotation(yaw):
    yaw = np.asarray(yaw, float)
    return Rotation.from_rotvec(np.stack([np.zeros_like(yaw), np.zeros_like(yaw), yaw], axis=-1))


def to_rotation(quat_wxyz):
    return Rotation.from_quat(np.asarray(quat_wxyz)[..., [1, 2, 3, 0]])


def to_wxyz(rotation):
    return rotation.as_quat()[..., [3, 0, 1, 2]]


def decay_offset(x, v, halflife, dt):
    """Critically damped decay of an offset x (rate v) to zero: inertialization (Holden 2020)."""
    y = 2.0 * np.log(2.0) / (halflife + 1e-5)
    j1 = v + x * y
    e = np.exp(-y * dt)
    return e * (x + j1 * dt), e * (v - j1 * y * dt)


class FootGround:
    """Lowest point of the athlete's soles for a pose (world z), on the contact model used for training."""

    def __init__(self, model_path=None):
        from athlete_loco.env import build_athlete_model, lowest_sole_point  # noqa: E402
        self.model = build_athlete_model(str(model_path) if model_path else None)
        self.data = mujoco.MjData(self.model)
        self._lowest = lowest_sole_point

    def lowest(self, qpos):
        self.data.qpos[:] = qpos
        mujoco.mj_kinematics(self.model, self.data)
        return self._lowest(self.model, self.data)


class MotionDatabase:
    """The fitted mocap clips, with per-frame playback quantities and normalized matching features."""

    def __init__(self, motions_dir=DEFAULT_MOTIONS, clips=CLIPS):
        folder = Path(motions_dir) / "DEFAULT" / "mocap" / "AthleteReference"
        qpos, qvel, feet, self.ranges, self.range_names = [], [], [], [], []
        start = 0
        for clip in clips:
            data = np.load(folder / f"{clip}.npz", allow_pickle=True)
            assert abs(float(data["frequency"]) - FREQUENCY) < 1e-6, f"{clip}: expected {FREQUENCY} Hz"
            site_names = [str(s) for s in data["site_names"]]
            foot_ids = [site_names.index(s) for s in FOOT_SITES]
            qpos.append(np.asarray(data["qpos"], np.float64))
            qvel.append(np.asarray(data["qvel"], np.float64))
            feet.append(np.asarray(data["site_xpos"], np.float64)[:, foot_ids])
            splits = np.asarray(data["split_points"]).tolist()
            for a, b in zip(splits[:-1], splits[1:]):
                self.ranges.append((start + a, start + b))
                self.range_names.append(clip)
            start += len(data["qpos"])
        qpos, qvel, feet = np.concatenate(qpos), np.concatenate(qvel), np.concatenate(feet)
        self.n_frames = len(qpos)
        self.range_of = np.zeros(self.n_frames, int)
        for i, (a, b) in enumerate(self.ranges):
            self.range_of[a:b] = i

        # Playback quantities.
        self.yaw = np.zeros(self.n_frames)
        self.yaw_rate = np.zeros(self.n_frames)
        foot_velocity = np.zeros_like(feet)
        for a, b in self.ranges:
            self.yaw[a:b] = np.unwrap(heading(qpos[a:b, 3:7]))
            self.yaw_rate[a:b] = np.gradient(self.yaw[a:b], DT)
            foot_velocity[a:b] = np.gradient(feet[a:b], DT, axis=0)
        self.tilt = yaw_rotation(self.yaw).inv() * to_rotation(qpos[:, 3:7])  # pelvis vs its heading
        self.pelvis_velocity = np.concatenate([rotate_2d(qvel[:, :2], -self.yaw), qvel[:, 2:3]], axis=1)  # heading frame
        self.height = qpos[:, 2]
        self.angular_velocity = qvel[:, 3:6]  # MuJoCo free joint: in the pelvis's own frame
        self.joints = qpos[:, 7:]
        self.joint_velocity = qvel[:, 6:]
        self.nq, self.nv = qpos.shape[1], qvel.shape[1]
        self.ground = FootGround()
        self.lowest = np.array([self.ground.lowest(q) for q in qpos])  # feet height in the mocap itself

        # Matching features.
        relative_feet = feet - qpos[:, None, :3]
        groups = {
            "foot_position": np.concatenate([rotate_2d(relative_feet[..., :2], -self.yaw[:, None]),
                                             feet[..., 2:]], axis=2).reshape(self.n_frames, 6),
            "foot_velocity": np.concatenate([rotate_2d(foot_velocity[..., :2], -self.yaw[:, None]),
                                             foot_velocity[..., 2:]], axis=2).reshape(self.n_frames, 6),
            "pelvis_velocity": self.pelvis_velocity,
            "future_position": np.zeros((self.n_frames, 2 * len(FUTURE_FRAMES))),
            "future_facing": np.zeros((self.n_frames, 2 * len(FUTURE_FRAMES))),
        }
        self.valid = np.zeros(self.n_frames, bool)
        for a, b in self.ranges:
            frames = np.arange(a, b)
            self.valid[a:b] = frames + max(FUTURE_FRAMES) < b
            for k, ahead in enumerate(FUTURE_FRAMES):
                later = np.minimum(frames + ahead, b - 1)
                groups["future_position"][a:b, 2 * k:2 * k + 2] = rotate_2d(qpos[later, :2] - qpos[frames, :2], -self.yaw[frames])
                turned = self.yaw[later] - self.yaw[frames]
                groups["future_facing"][a:b, 2 * k:2 * k + 2] = np.stack([np.cos(turned), np.sin(turned)], axis=1)
        self.slices, columns, self.mean, self.scale = {}, [], [], []
        start = 0
        for name, values in groups.items():
            self.slices[name] = slice(start, start + values.shape[1])
            start += values.shape[1]
            mean = values[self.valid].mean(axis=0)
            scale = values[self.valid].std(axis=0).mean() / FEATURE_WEIGHTS[name]
            columns.append((values - mean) / scale)
            self.mean.append(mean)
            self.scale.append(np.full(values.shape[1], scale))
        self.features = np.concatenate(columns, axis=1).astype(np.float32)
        self.mean, self.scale = np.concatenate(self.mean), np.concatenate(self.scale)
        self.search_ids = np.flatnonzero(self.valid)
        self.search_features = self.features[self.search_ids]

    def normalize(self, name, values):
        s = self.slices[name]
        return (np.asarray(values) - self.mean[s]) / self.scale[s]

    def cost(self, frame, query):
        return float(np.sum(np.square(self.features[frame] - query.astype(np.float32))))

    def search(self, query, near=None):
        """Best frame for the query, skipping frames of `near`'s clip from IGNORE_BEHIND_FRAMES before it to
        IGNORE_SURROUNDING_FRAMES after it."""
        cost = np.sum(np.square(self.search_features - query.astype(np.float32)), axis=1)
        if near is not None:
            a, b = self.ranges[self.range_of[near]]
            low, high = np.searchsorted(self.search_ids, [max(a, near - IGNORE_BEHIND_FRAMES),
                                                          min(b, near + IGNORE_SURROUNDING_FRAMES + 1)])
            cost[low:high] = np.inf
        best = int(np.argmin(cost))
        return int(self.search_ids[best]), float(cost[best])

    def first_frame(self, clip):
        return next(a for (a, b), name in zip(self.ranges, self.range_names) if name == clip)


class MotionMatcher:
    """The kinematic character: plays the database, jumping where the controller wants to go."""

    def __init__(self, database):
        self.db = database
        self.n_joints = database.joints.shape[1]
        self.reset()

    def reset(self, x=0.0, y=0.0, yaw=0.0, frame=None):
        self.frame = self.db.first_frame(START_CLIP) if frame is None else frame
        self.position, self.facing = np.array([x, y], float), float(yaw)
        self.offset = np.zeros(self.n_joints + N_OFFSETS_EXTRA)
        self.offset_rate = np.zeros_like(self.offset)
        self.command, self.since_search, self.jumps = None, SEARCH_INTERVAL_FRAMES, 0
        # The controller's own movement state (heading frame), approaching the command; the desired trajectory
        # starts from it, not from the animation (which lags mid-blend and would pull the search back).
        self.control_velocity = self.db.pelvis_velocity[self.frame, :2].copy()
        self.control_yaw_rate = float(self.db.yaw_rate[self.frame])
        return self.reference()

    def _pose(self, frame, offset, offset_rate):
        """Playback values of a frame plus inertialization offsets."""
        db, n = self.db, self.n_joints
        return {"joints": db.joints[frame] + offset[:n],
                "joint_velocity": db.joint_velocity[frame] + offset_rate[:n],
                "height": db.height[frame] + offset[n],
                "vertical_velocity": db.pelvis_velocity[frame, 2] + offset_rate[n],
                "tilt": Rotation.from_rotvec(offset[n + 1:n + 4]) * db.tilt[frame],
                "velocity": db.pelvis_velocity[frame, :2] + offset[n + 4:n + 6],
                "yaw_rate": db.yaw_rate[frame] + offset[n + 6]}

    def _query(self, command):
        db = self.db
        velocity, yaw_rate = self.control_velocity.copy(), self.control_yaw_rate
        lag = np.exp(-np.log(2.0) * DT / CONTROL_HALFLIFE_S)
        position, turned, future_position, future_facing = np.zeros(2), 0.0, [], []
        for i in range(1, max(FUTURE_FRAMES) + 1):
            velocity = command[:2] + (velocity - command[:2]) * lag
            yaw_rate = command[2] + (yaw_rate - command[2]) * lag
            turned += yaw_rate * DT
            position += rotate_2d(velocity, turned) * DT
            if i in FUTURE_FRAMES:
                future_position.extend(position)
                future_facing.extend([np.cos(turned), np.sin(turned)])
        query = db.features[self.frame].astype(np.float64).copy()
        query[db.slices["future_position"]] = db.normalize("future_position", future_position)
        query[db.slices["future_facing"]] = db.normalize("future_facing", future_facing)
        return query

    def _jump(self, frame):
        """Start playing `frame`; the current pose's difference from it becomes the decaying offset."""
        db, n = self.db, self.n_joints
        old = self._pose(self.frame, self.offset, self.offset_rate)
        x, v = np.zeros_like(self.offset), np.zeros_like(self.offset)
        x[:n] = old["joints"] - db.joints[frame]
        v[:n] = old["joint_velocity"] - db.joint_velocity[frame]
        x[n] = old["height"] - db.height[frame]
        v[n] = old["vertical_velocity"] - db.pelvis_velocity[frame, 2]
        x[n + 1:n + 4] = (old["tilt"] * db.tilt[frame].inv()).as_rotvec()
        x[n + 4:n + 6] = old["velocity"] - db.pelvis_velocity[frame, :2]
        x[n + 6] = old["yaw_rate"] - db.yaw_rate[frame]
        self.offset, self.offset_rate, self.frame = x, v, frame
        self.jumps += 1

    def step(self, command):
        """Advance one frame toward the command (forward m/s, sideways m/s, turning rad/s, own frame)."""
        db = self.db
        command = np.asarray(command, float)
        changed = self.command is None or np.abs(command - self.command).max() > 1e-6
        self.command = command
        lag = np.exp(-np.log(2.0) * DT / CONTROL_HALFLIFE_S)
        self.control_velocity = command[:2] + (self.control_velocity - command[:2]) * lag
        self.control_yaw_rate = command[2] + (self.control_yaw_rate - command[2]) * lag
        next_frame = self.frame + 1
        at_end = next_frame >= db.ranges[db.range_of[self.frame]][1]
        self.since_search += 1
        if at_end or changed or self.since_search >= SEARCH_INTERVAL_FRAMES:
            self.since_search = 0
            # Jump only if a frame elsewhere beats simply playing on (Holden 2020); never back into the
            # moment just played, which loops (measured: stuck re-starting the same step).
            query = self._query(command)
            best, best_cost = db.search(query, near=None if at_end else next_frame)
            if at_end or not db.valid[next_frame] or best_cost < db.cost(next_frame, query) - JUMP_MARGIN:
                self._jump(best)
                next_frame = best
        self.frame = next_frame
        self.offset, self.offset_rate = decay_offset(self.offset, self.offset_rate, BLEND_HALFLIFE_S, DT)
        pose = self._pose(self.frame, self.offset, self.offset_rate)
        self.facing += pose["yaw_rate"] * DT
        self.position += rotate_2d(pose["velocity"], self.facing) * DT
        return self.reference(pose)

    def reference(self, pose=None):
        """(qpos, qvel) of the reference in MuJoCo's layout for the athlete (free joint first)."""
        pose = pose or self._pose(self.frame, self.offset, self.offset_rate)
        qpos, qvel = np.zeros(self.db.nq), np.zeros(self.db.nv)
        qpos[:2], qpos[2] = self.position, pose["height"]
        qpos[3:7] = to_wxyz(yaw_rotation(self.facing) * pose["tilt"])
        qpos[7:] = pose["joints"]
        # Ground: blending leg angles moves the feet through or above the floor (measured +/-5 cm). Keep the
        # lowest foot point where the mocap frame has it (on the floor, or in the air while running).
        qpos[2] -= self.db.ground.lowest(qpos) - self.db.lowest[self.frame]
        qvel[:2], qvel[2] = rotate_2d(pose["velocity"], self.facing), pose["vertical_velocity"]
        qvel[3:6] = self.db.angular_velocity[self.frame]
        qvel[6:] = pose["joint_velocity"]
        return qpos, qvel

    def follow(self, xy, yaw, max_distance=0.5, max_yaw=0.35):
        """Keep the reference near the simulated athlete (run time): within max_distance m and max_yaw rad."""
        offset = self.position - np.asarray(xy)
        distance = np.linalg.norm(offset)
        if distance > max_distance:
            self.position = np.asarray(xy) + offset * max_distance / distance
        error = wrap(self.facing - yaw)
        if abs(error) > max_yaw:
            self.facing = yaw + np.sign(error) * max_yaw


def random_commands(rng, seconds, turn_heavy=False, speed_heavy=False, run_maneuvers=False):
    """
    Controller input like a player's: held 1-4 s; stand, walk, run, turn, sidestep.
    turn_heavy: for training turns (2026-10-01: the tracker couldn't follow them). 80% of commands turn,
    and a quarter of all commands turn on the spot.
    speed_heavy: for training speed changes (2026-10-02: the tracker lagged the reference speed). Mostly
    straight (10% turn), speeds spread over 0..3 m/s with a third of commands running (2..3 m/s).
    run_maneuvers: for training running turns and sidesteps (2026-10-02: every random-command fall was at
    >= 1.9 m/s while turning or sidestepping, or after a big speed jump). 70% of commands run at 1.8..3 m/s,
    turning (60%, up to 1.2 rad/s) and/or sidestepping (40%, up to 0.5 m/s); the rest are any speed or a
    stand, so the speed jumps between them are large. Held 1.5-3.5 s.
    """
    if run_maneuvers:
        schedule, t = [], 0.0
        while t < seconds:
            hold = rng.uniform(1.5, 3.5)
            if rng.random() < 0.7:
                command = (rng.uniform(1.8, 3.0), rng.uniform(-0.5, 0.5) * (rng.random() < 0.4),
                           rng.uniform(-1.2, 1.2) * (rng.random() < 0.6))
            elif rng.random() < 0.3:
                command = (0.0, 0.0, 0.0)
            else:
                command = (rng.uniform(0.0, 3.0), 0.0, rng.uniform(-1.2, 1.2) * (rng.random() < 0.3))
            schedule.append((hold, command))
            t += hold
        return schedule
    p_turn, p_spot = (0.8, 0.25) if turn_heavy else (0.1, 0.0) if speed_heavy else (0.5, 0.0)
    schedule, t = [], 0.0
    while t < seconds:
        hold = rng.uniform(1.0, 4.0)
        if rng.random() < 0.15:
            command = (0.0, 0.0, 0.0)
        elif rng.random() < p_spot:
            command = (0.0, 0.0, rng.choice([-1.0, 1.0]) * rng.uniform(0.5, 1.5))
        elif speed_heavy and rng.random() < 1 / 3:
            command = (rng.uniform(2.0, 3.0), 0.0, rng.uniform(-0.5, 0.5) * (rng.random() < p_turn))
        else:
            command = (rng.uniform(0.0, 3.0), rng.uniform(-0.4, 0.4) * (rng.random() < 0.2),
                       rng.uniform(-1.5, 1.5) * (rng.random() < p_turn))
        schedule.append((hold, command))
        t += hold
    return schedule


def play(matcher, schedule):
    matcher.reset()
    qpos, qvel, commands = [], [], []
    for hold, command in schedule:
        for _ in range(int(round(hold * FREQUENCY))):
            q, v = matcher.step(command)
            qpos.append(q)
            qvel.append(v)
            commands.append(command)
    return np.array(qpos), np.array(qvel), np.array(commands)


def kinematic_check(database):
    """The Milestone 4 acceptance moves, on the kinematic reference alone (no physics)."""
    matcher = MotionMatcher(database)
    report = {}

    def forward_speed(qpos, qvel):
        yaw = heading(qpos[:, 3:7])
        return qvel[:, 0] * np.cos(yaw) + qvel[:, 1] * np.sin(yaw)

    def smooth(x, n=50):
        return np.convolve(x, np.ones(n) / n, mode="same")

    q, v, _ = play(matcher, [(2.0, (0, 0, 0)), (6.0, (1.5, 0, 0))])
    s = smooth(forward_speed(q, v))
    reach = np.flatnonzero(s[200:] >= 0.9 * 1.5)
    report["accelerate"] = {"time_to_90pct_s": None if len(reach) == 0 else round(reach[0] * DT, 2),
                            "final_speed_mps": round(float(s[-200:].mean()), 2)}
    q, v, _ = play(matcher, [(6.0, (1.5, 0, 0)), (4.0, (0, 0, 0))])
    s = smooth(forward_speed(q, v))
    slow = np.flatnonzero(np.abs(s[600:]) < 0.15)
    report["brake"] = {"speed_before_mps": round(float(s[599]), 2),
                       "time_to_below_0.15_s": None if len(slow) == 0 else round(slow[0] * DT, 2)}
    q, v, _ = play(matcher, [(3.0, (1.0, 0, 0)), (4.0, (1.0, 0, 0.8)), (2.0, (1.0, 0, 0))])
    yaw = np.unwrap(heading(q[:, 3:7]))
    report["turn"] = {"heading_change_rad": round(float(yaw[700] - yaw[300]), 2), "commanded_rad": 3.2}
    q, v, _ = play(matcher, [(2.0, (0, 0, 0)), (4.0, (0, 0, 1.0))])
    yaw = np.unwrap(heading(q[:, 3:7]))
    report["turn_on_spot"] = {"heading_change_rad": round(float(yaw[-1] - yaw[200]), 2), "commanded_rad": 4.0,
                              "drift_m": round(float(np.linalg.norm(q[-1, :2] - q[200, :2])), 2)}
    q, v, _ = play(matcher, [(1.0, (0, 0, 0)), (6.0, (2.5, 0, 0))])
    s = smooth(forward_speed(q, v))
    report["run"] = {"final_speed_mps": round(float(s[-200:].mean()), 2), "commanded_mps": 2.5}
    seconds = 120
    rng = np.random.default_rng(0)
    q, v, c = play(matcher, random_commands(rng, seconds))
    s = smooth(forward_speed(q, v))
    report["random"] = {"seconds": seconds, "forward_speed_error_mps": round(float(np.mean(np.abs(s - c[:, 0]))), 3),
                        "jumps_per_s": round(matcher.jumps / seconds, 2)}
    return report


def generate(database, motions_dir, name, seconds, seed, turn_heavy=False, speed_heavy=False, run_maneuvers=False):
    """
    One continuous clip the matcher makes from random controller input, saved for training next to the
    mocap. (One clip per file: LocoMuJoCo's trajectory extender overran its buffer with several.)
    """
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from retarget_motion import ENV_NAME, save_motion  # noqa: E402  (loads LocoMuJoCo and the athlete)
    from loco_mujoco.environments import LocoEnv  # noqa: E402
    model = LocoEnv.registered_envs[ENV_NAME]()._model
    rng = np.random.default_rng(seed)
    matcher = MotionMatcher(database)
    qpos, _, _ = play(matcher, random_commands(rng, seconds, turn_heavy, speed_heavy, run_maneuvers))
    # Velocities from the positions themselves (central differences, MuJoCo's own differencing), as
    # retarget_motion.py does: consistent with qpos, jumps and blends included. Ends dropped.
    qvel = np.zeros((len(qpos) - 2, model.nv))
    for i in range(1, len(qpos) - 1):
        mujoco.mj_differentiatePos(model, qvel[i - 1], 2.0 * DT, qpos[i - 1], qpos[i + 1])
    target = Path(motions_dir) / "DEFAULT" / "mocap" / ENV_NAME / f"{name}.npz"
    save_motion(model, qpos[1:-1], qvel, FREQUENCY, target)
    return {"output": str(target), "seconds": seconds, "frames": len(qvel),
            "jumps_per_s": round(matcher.jumps / seconds, 2)}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("action", choices=["check", "generate"])
    parser.add_argument("--motions", default=str(DEFAULT_MOTIONS))
    parser.add_argument("--name", default="mm_random")
    parser.add_argument("--count", type=int, default=1, help="clips <name>_<first> .. <name>_<first+count-1> (seed + index)")
    parser.add_argument("--first", type=int, default=0, help="index of the first clip (to generate in parallel processes)")
    parser.add_argument("--seconds", type=float, default=300.0)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--turn-heavy", action="store_true", help="mostly turning commands (random_commands)")
    parser.add_argument("--speed-heavy", action="store_true", help="mostly straight, varied speeds (random_commands)")
    parser.add_argument("--run-maneuvers", action="store_true", help="running turns, sidesteps and speed jumps (random_commands)")
    args = parser.parse_args()
    database = MotionDatabase(args.motions)
    print(f"motion database: {database.n_frames} frames ({database.n_frames * DT / 60:.1f} min), "
          f"{len(database.search_ids)} searchable, {database.features.shape[1]} features", flush=True)
    if args.action == "check":
        print(json.dumps(kinematic_check(database), indent=2))
    else:
        for i in range(args.first, args.first + args.count):
            name = args.name if args.count == 1 and args.first == 0 else f"{args.name}_{i}"
            print(json.dumps(generate(database, args.motions, name, args.seconds, args.seed + i, args.turn_heavy, args.speed_heavy,
                                         args.run_maneuvers)), flush=True)


if __name__ == "__main__":
    main()
