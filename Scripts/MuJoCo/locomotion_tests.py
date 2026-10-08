"""Project ATHLETE - Milestone 4 acceptance: does the athlete follow controller input?

Usage (LocoMuJoCo Python):
    Intermediate/LocoMuJoCoPython/Scripts/python.exe Scripts/MuJoCo/locomotion_tests.py <PPOJax_saved.pkl> [--trials 5 --need 4]

The trained policy drives the athlete on CPU MuJoCo (the shared contact model, athlete_loco.env; --model
full: MuJoCo's default solver, training: the GPU training settings), 2 ms physics / 10 ms control as in
training, deterministic (the policy's mean action, input normalization frozen). The command is the
controller input: forward speed, sideways speed and turning rate in the athlete's own frame. It
passes through the controller layer (controller.CommandShaper: human acceleration limits) as in the game;
--no-shaping sends it unshaped. Measures compare against the command as given, not the shaped one.
  - Tracking policy (PPO, GoalTrajMimic): motion matching turns the command into a reference motion from
    the mocap and the policy follows it (Driver, motion_matching.py).
  - Command policy (AMP, GoalVelocityCommand): the command goes into its observation.
Each test starts standing (in a step-in-place clip) and runs --trials times, each trial starting at a
different moment of the stepping (TRIAL_OFFSETS_FRAMES); it passes if at least --need trials pass. One
deterministic trial of a sensitive physical system can flip on a hair (2026-10-02), so a pass has to
hold across starts.

Tests (measured on the pelvis, heading frame, 0.5 s moving average):
  accelerate     stand 2 s, then 1.5 m/s for 6 s: time to reach 90% of 1.5 m/s; speed error over the last 2 s
  brake          1.5 m/s for 6 s, then stop for 4 s: time until below 0.15 m/s for 1 s; distance after the stop command
  turn           1.0 m/s, then turn at 0.8 rad/s for 4 s: heading change achieved vs 3.2 rad commanded
  turn on spot   stand, then turn at 1.0 rad/s for 4 s with no forward speed: heading change vs 4.0 rad
  run            stand 1 s, then 2.5 m/s for 6 s: speed reached
  random         N x 20 s of random commands (new every 3 s; forward 0..3 m/s, as the controller allows):
                 falls, tracking error
A fall = the pelvis leaves its healthy height range (the training's own definition). Pass criteria are in
CRITERIA below (per trial); they're a first bar to beat, not a final standard.
"""

import argparse
import dataclasses
import json
import sys
from pathlib import Path

import jax
import jax.numpy as jnp
import mujoco
import numpy as np
from omegaconf import OmegaConf

sys.path.insert(0, str(Path(__file__).resolve().parent))
import athlete_loco  # noqa: E402,F401
import motion_matching  # noqa: E402
from controller import CommandShaper  # noqa: E402
from loco_mujoco import TaskFactory, algorithms  # noqa: E402

# Checkpoints from the cloud trainer pickle its discriminator class as __main__.SelectedInputNet (the
# trainer ran as a script): make that name resolvable here before loading one.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Cloud" / "jobs"))
from _train_chunked import SelectedInputNet  # noqa: E402
sys.modules["__main__"].SelectedInputNet = SelectedInputNet

STAND_CLIP_INDEX = 4  # order of default_dataset_conf.task in athlete_locomotion.yaml: ..., stepinplace1 (4), ...
TRACKING_GOALS = ("GoalTrajMimic", "GoalTrajMimicLookahead", "GoalTrackInvariant")
MAX_REFERENCE_YAW_LEAD_RAD = 0.35  # the reference may face this far from the athlete, no further
TRIAL_OFFSETS_FRAMES = (0, 17, 34, 51, 68)  # trial starts, frames into the step-in-place clip (~a step apart)
TRAJECTORY_FIELDS = ("qpos", "qvel", "xpos", "xquat", "cvel", "subtree_com", "site_xpos", "site_xmat")
CRITERIA = {
    "accelerate": "no fall; 90% of 1.5 m/s within 3 s; mean speed error < 0.25 m/s over the last 2 s",
    "brake": "no fall; below 0.15 m/s within 2.5 s of the stop command",
    "turn": "no fall; heading change within 30% of the commanded 3.2 rad",
    "turn_on_spot": "no fall; heading change within 30% of the commanded 4.0 rad",
    "run": "no fall; mean speed over the last 2 s at least 2.0 m/s",
    "random": "at most 1 fall per 10 episodes; mean forward-speed error < 0.4 m/s",
}


class Driver:
    """
    Turns controller input into the athlete's movement, for either kind of trained policy:
      - a TRACKING policy (DeepMimic, goal GoalTrajMimic): motion matching (motion_matching.py) turns the
        command into a reference motion, and the policy makes the athlete follow it (DReCon-style);
      - a COMMAND policy (goal GoalVelocityCommand, e.g. AMP): the command goes straight into its observation.
    reset(command) and step(obs, command) are the controller interface; run(schedule) drives a test.
    shape_commands: the player's command goes through the controller layer (controller.CommandShaper:
    human acceleration limits) before it reaches the athlete, as it would in the game.
    """

    def __init__(self, agent_path, horizon_s, model="full", shape_commands=True):
        self.Algorithm = algorithms.AMPJax if "AMP" in Path(agent_path).name else algorithms.PPOJax
        self.agent_conf, agent_state = self.Algorithm.load_agent(agent_path)
        config = self.agent_conf.config
        params = OmegaConf.to_container(config.experiment.env_params, resolve=True)
        task_params = OmegaConf.to_container(config.experiment.task_factory.params, resolve=True)
        for key in ("use_mjwarp", "nconmax", "njmax", "domain_randomization_params"):
            params.pop(key, None)
        # Tests run the athlete as it is: no training randomization (pushes, sensor noise), which a
        # checkpoint's training config may switch on (2026-10-04: it silently pushed the athlete in tests).
        params["domain_randomization_type"] = "NoDomainRandomization"
        self.tracking = params.get("goal_type") in TRACKING_GOALS
        # Both use the athlete's contact model (athlete_loco.env.configure_contacts). "full": MuJoCo's default
        # solver; "training": the GPU training settings (few solver iterations: MjxAthleteReference) on CPU.
        env_name = {"full": "AthleteReference", "training": "MjxAthleteReference"}[model]
        params.update(env_name=env_name, timestep=0.002, n_substeps=5, headless=True,
                      horizon=int(horizon_s / 0.01) + 10, goal_params={"visualize_goal": False},
                      # A fall = the pelvis leaves its healthy height range (not: leaves the reference).
                      terminal_state_type="HeightBasedTerminalStateHandler")
        if self.tracking:
            # The reference comes from motion matching, written into a one-clip trajectory buffer each step.
            # Its contents don't matter, its length does: where it wraps around the lookahead briefly repeats
            # one frame, so long sessions (the drive demo) use the longest clip (walk, ~14.7 min).
            buffer_clip = motion_matching.START_CLIP if horizon_s <= 50 else "walk"
            task_params = {"default_dataset_conf": {"task": [buffer_clip]}}
            params["th_params"] = dict(random_start=False, fixed_start_conf=(0, 0))
        else:
            params["th_params"] = dict(random_start=False, fixed_start_conf=(STAND_CLIP_INDEX, 0))
        factory = TaskFactory.get_factory_cls(config.experiment.task_factory.name)
        self.env = factory.make(**params, **task_params)
        if self.tracking:
            data = self.env.th.traj.data
            self.env.th.traj = dataclasses.replace(
                self.env.th.traj, data=data.replace(**{f: np.array(getattr(data, f)) for f in TRAJECTORY_FIELDS}))
            self.trajectory_length = int(self.env.th.len_trajectory(0))
            self.matcher = motion_matching.MotionMatcher(motion_matching.MotionDatabase())
            self.reference_data = mujoco.MjData(self.env._model)
            # A lookahead policy sees the reference up to `lead` frames ahead: motion matching runs that far
            # ahead of the simulation (a command takes effect `lead` frames later).
            goal = self.env.obs_container[params["goal_type"]]
            steps = getattr(goal, "lookahead_steps", None)
            self.lead = int(max(steps)) if steps is not None else 0
            self.rows = {}  # buffer row -> (qpos, qvel) written there
            self.start_offset = 0  # frames into the start clip where the next reset begins
        else:
            self.goal = self.env.obs_container["GoalVelocityCommand"]
        self.shaper = CommandShaper() if shape_commands else None
        self.train_state = agent_state.train_state
        self.train_state.params["log_std"] = np.ones_like(self.train_state.params["log_std"]) * -np.inf

        def act(ts, obs, key):
            # The input normalization stays as trained. Updating it with every test step (as training does)
            # made results depend on what ran before: the same test passed or fell depending on the
            # previous one (2026-10-02). Frozen, a test repeats exactly.
            y, _ = self.agent_conf.network.apply({"params": ts.params, "run_stats": ts.run_stats}, obs, mutable=["run_stats"])
            return y[0].sample(seed=key), ts
        self.act = jax.jit(act)
        self.rng = jax.random.key(0)

    def _write_reference(self, row, qpos, qvel):
        """Puts a reference pose into the trajectory buffer row the policy's goal and the reward read."""
        model, d = self.env._model, self.reference_data
        d.qpos[:], d.qvel[:] = qpos, qvel
        mujoco.mj_kinematics(model, d)
        mujoco.mj_comPos(model, d)
        mujoco.mj_comVel(model, d)
        self.rows[row] = (qpos.copy(), qvel.copy())
        traj = self.env.th.traj.data
        traj.qpos[row], traj.qvel[row] = qpos, qvel
        traj.xpos[row], traj.xquat[row], traj.cvel[row] = d.xpos, d.xquat, d.cvel
        traj.subtree_com[row], traj.site_xpos[row], traj.site_xmat[row] = d.subtree_com, d.site_xpos, d.site_xmat

    def _turn_reference(self, now_row, angle):
        """Turns the queued reference (rows after now_row, and the matcher) by `angle` about the current one."""
        pivot = self.rows[now_row][0][:2].copy()
        c, s = np.cos(angle), np.sin(angle)
        turn = np.array([[c, -s], [s, c]])
        spin = np.array([np.cos(angle / 2), 0.0, 0.0, np.sin(angle / 2)])
        for k in range(1, self.lead + 1):
            row = (now_row + k) % self.trajectory_length
            qpos, qvel = self.rows[row]
            qpos, qvel = qpos.copy(), qvel.copy()
            qpos[:2] = pivot + turn @ (qpos[:2] - pivot)
            qpos[3:7] = motion_matching.to_wxyz(motion_matching.to_rotation(spin) * motion_matching.to_rotation(qpos[3:7]))
            qvel[:2] = turn @ qvel[:2]
            self._write_reference(row, qpos, qvel)
        self.matcher.position = pivot + turn @ (self.matcher.position - pivot)
        self.matcher.facing += angle

    def _shaped(self, command):
        """The command as the athlete receives it (through the controller layer, if on)."""
        return self.shaper.step(command, self.env.dt) if self.shaper else tuple(command)

    def reset(self, command=(0.0, 0.0, 0.0)):
        if self.shaper:
            self.shaper.reset()  # the athlete starts standing
        if self.tracking:
            self.rows = {}
            start = self.matcher.db.first_frame(motion_matching.START_CLIP) + self.start_offset
            self._write_reference(0, *self.matcher.reset(frame=start))  # the athlete starts in the reference's first pose
            for row in range(1, self.lead + 1):  # and the policy sees `lead` frames of the future
                self._write_reference(row, *self.matcher.step(self._shaped(command)))
        else:
            self.goal.external_command = self._shaped(command)
        return self.env.reset()

    def step(self, obs, command):
        """One control step (10 ms) toward the command. Returns (obs, fell)."""
        env = self.env
        if self.tracking:
            # The env advances the trajectory row before building the observation, so this step's new frame
            # goes `lead` + 1 rows ahead of the current one.
            now = int(env._additional_carry.traj_state.subtraj_step_no)
            q = env._data.qpos
            if self.lead:
                # Keep the reference facing near the athlete (its absolute heading is observed; position
                # isn't), turning the queued future with it.
                error = motion_matching.wrap(motion_matching.heading(self.rows[now][0][3:7]) - motion_matching.heading(q[3:7]))
                if abs(error) > MAX_REFERENCE_YAW_LEAD_RAD:
                    self._turn_reference(now, -(error - np.sign(error) * MAX_REFERENCE_YAW_LEAD_RAD))
            else:
                self.matcher.follow(q[:2], motion_matching.heading(q[3:7]))
            row = (now + 1 + self.lead) % self.trajectory_length
            self._write_reference(row, *self.matcher.step(self._shaped(command)))
        else:
            self.goal.external_command = self._shaped(command)
        self.rng, key = jax.random.split(self.rng)
        action, self.train_state = self.act(self.train_state, obs, key)
        obs, _, absorbing, _, _ = env.step(jnp.atleast_2d(action))
        return obs, bool(absorbing)

    def run(self, schedule):
        """schedule: [(seconds, (forward, sideways, turn)), ...]. Returns per-step records and whether it fell."""
        env, dt = self.env, self.env.dt
        obs = self.reset(schedule[0][1])
        records, t = [], 0.0
        for duration, command in schedule:
            for _ in range(int(round(duration / dt))):
                obs, fell = self.step(obs, command)
                q, v = env._data.qpos, env._data.qvel
                w, x, y, z = q[3:7]
                yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
                records.append({"t": t, "command": command, "x": q[0], "y": q[1], "z": q[2], "yaw": yaw,
                                "forward": v[0] * np.cos(yaw) + v[1] * np.sin(yaw)})
                t += dt
                if fell:
                    return records, True
        return records, False


def smooth(values, n=50):
    return np.convolve(values, np.ones(n) / n, mode="same") if len(values) > n else np.asarray(values)


def heading_change(records, start, end):
    yaw = np.unwrap([r["yaw"] for r in records[start:end]])
    return float(yaw[-1] - yaw[0]) if len(yaw) > 1 else 0.0


def test_accelerate(driver, dt):
    recs, fell = driver.run([(2.0, (0.0, 0.0, 0.0)), (6.0, (1.5, 0.0, 0.0))])
    speed, switch = smooth([r["forward"] for r in recs]), int(2.0 / dt)
    reached = np.argmax(speed[switch:] >= 0.9 * 1.5) if (speed[switch:] >= 0.9 * 1.5).any() else None
    t90 = None if reached is None else round(float(reached * dt), 2)
    error = float(np.mean(np.abs(speed[-int(2.0 / dt):] - 1.5))) if not fell else None
    passed = not fell and t90 is not None and t90 <= 3.0 and error < 0.25
    return passed, fell, {"time_to_90pct_s": t90, "final_speed_error_mps": None if error is None else round(error, 3)}


def test_brake(driver, dt):
    recs, fell = driver.run([(6.0, (1.5, 0.0, 0.0)), (4.0, (0.0, 0.0, 0.0))])
    speed, switch = smooth([r["forward"] for r in recs]), int(6.0 / dt)
    stop_s = distance = before = None
    if not fell:
        slow = np.abs(speed[switch:]) < 0.15
        stop_index = next((i for i in range(len(slow) - int(1.0 / dt)) if slow[i:i + int(1.0 / dt)].all()), None)
        before = round(float(speed[switch - 1]), 2)
        if stop_index is not None:
            stop_s = round(stop_index * dt, 2)
            a, b = recs[switch], recs[switch + stop_index]
            distance = round(float(np.hypot(b["x"] - a["x"], b["y"] - a["y"])), 2)
    passed = not fell and stop_s is not None and stop_s <= 2.5
    return passed, fell, {"speed_before_mps": before, "time_to_stop_s": stop_s, "stopping_distance_m": distance}


def test_turn(driver, dt):
    recs, fell = driver.run([(3.0, (1.0, 0.0, 0.0)), (4.0, (1.0, 0.0, 0.8)), (2.0, (1.0, 0.0, 0.0))])
    change = heading_change(recs, int(3.0 / dt), int(7.0 / dt)) if not fell else None
    passed = not fell and change is not None and abs(change - 3.2) <= 0.3 * 3.2
    return passed, fell, {"heading_change_rad": None if change is None else round(change, 2)}


def test_turn_on_spot(driver, dt):
    recs, fell = driver.run([(2.0, (0.0, 0.0, 0.0)), (4.0, (0.0, 0.0, 1.0))])
    change = heading_change(recs, int(2.0 / dt), int(6.0 / dt)) if not fell else None
    drift = None if fell else round(float(np.hypot(recs[-1]["x"] - recs[int(2.0 / dt)]["x"],
                                                   recs[-1]["y"] - recs[int(2.0 / dt)]["y"])), 2)
    passed = not fell and change is not None and abs(change - 4.0) <= 0.3 * 4.0
    return passed, fell, {"heading_change_rad": None if change is None else round(change, 2), "drift_m": drift}


def test_run(driver, dt):
    recs, fell = driver.run([(1.0, (0.0, 0.0, 0.0)), (6.0, (2.5, 0.0, 0.0))])
    speed = smooth([r["forward"] for r in recs])
    final = None if fell else round(float(np.mean(speed[-int(2.0 / dt):])), 2)
    return not fell and final is not None and final >= 2.0, fell, {"final_speed_mps": final}


TESTS = [("accelerate", test_accelerate), ("brake", test_brake), ("turn", test_turn),
         ("turn_on_spot", test_turn_on_spot), ("run", test_run)]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("agent")
    parser.add_argument("--random", type=int, default=10, help="episodes of random commands")
    parser.add_argument("--trials", type=int, default=5, help=f"trials per test (at most {len(TRIAL_OFFSETS_FRAMES)})")
    parser.add_argument("--need", type=int, default=4, help="trials that must pass for a test to pass")
    parser.add_argument("--out", default=None)
    parser.add_argument("--no-shaping", action="store_true",
                        help="send the test commands to the athlete unshaped (no controller layer)")
    parser.add_argument("--model", choices=["full", "training"], default="full",
                        help="full: MuJoCo's default solver; training: the GPU training settings (both: the shared contact model)")
    args = parser.parse_args()
    trials = min(args.trials, len(TRIAL_OFFSETS_FRAMES))
    need = min(args.need, trials)
    driver = Driver(args.agent, horizon_s=22.0, model=args.model, shape_commands=not args.no_shaping)
    dt = driver.env.dt
    report = {}

    for name, test in TESTS:
        results = []
        for offset in TRIAL_OFFSETS_FRAMES[:trials]:
            driver.start_offset = offset
            results.append(test(driver, dt))
        n_passed, n_fell = sum(r[0] for r in results), sum(r[1] for r in results)
        measures = {key: [r[2][key] for r in results] for key in results[0][2]}
        report[name] = {"passed": n_passed >= need, "trials_passed": n_passed, "trials": trials, "falls": n_fell,
                        "criteria": f"{CRITERIA[name]} (per trial; at least {need} of {trials} trials)", **measures}
        print(f"{'PASS' if n_passed >= need else 'FAIL'} {name:13s} {n_passed}/{trials} trials, {n_fell} falls | "
              + ", ".join(f"{k} {v}" for k, v in measures.items()), flush=True)

    # random commands: one episode per start offset in turn
    rng = np.random.default_rng(3)
    falls, errors = 0, []
    for episode in range(args.random):
        driver.start_offset = TRIAL_OFFSETS_FRAMES[episode % len(TRIAL_OFFSETS_FRAMES)]
        schedule = [(2.0, (0.0, 0.0, 0.0))]
        for _ in range(6):
            stand = rng.random() < 0.15
            # Forward 0..3 m/s: the mocap has no walking backwards (the AMP runs used -0.5..3).
            command = (0.0, 0.0, 0.0) if stand else (rng.uniform(0.0, 3.0), rng.uniform(-0.5, 0.5) * (rng.random() < 0.3),
                                                     rng.uniform(-1.2, 1.2) * (rng.random() < 0.5))
            schedule.append((3.0, command))
        recs, fell = driver.run(schedule)
        falls += fell
        speed = smooth([r["forward"] for r in recs])
        wanted = np.array([r["command"][0] for r in recs])
        # The second half of each command. (Until 2026-10-05 this was (t % 3 s) > 1.5 s: with the 2 s opening
        # stand those windows straddled every command change, so it measured the first second of each new
        # command, mid-acceleration: 0.69-0.80 m/s for every tracker, the criterion unreachable.)
        starts = np.cumsum([0.0] + [seconds for seconds, _ in schedule])
        segment = np.searchsorted(starts, np.arange(len(recs)) * dt, side="right") - 1
        settled = (np.arange(len(recs)) * dt - starts[segment]) > 0.5 * np.array([schedule[s][0] for s in segment])
        errors.append(float(np.mean(np.abs(speed - wanted)[settled])) if settled.any() else np.nan)
    mean_error = round(float(np.nanmean(errors)), 3)
    # The goal is a rate (Docs/Milestone4_Locomotion.md 9.6). Until 2026-10-07 this was "falls <= 1" whatever
    # the episode count: right for the default 10, four times too strict at 40. Use --random 100 or more for a
    # reliable rate: 10 episodes are mostly luck.
    passed = falls * 10 <= args.random and mean_error < 0.4
    report["random"] = {"passed": passed, "episodes": args.random, "falls": falls, "forward_speed_error_mps": mean_error,
                        "criteria": CRITERIA["random"]}
    print(f"{'PASS' if passed else 'FAIL'} random        {args.random} episodes, {falls} falls | forward_speed_error_mps {mean_error}",
          flush=True)

    n_passed = sum(r["passed"] for r in report.values())
    print(f"\n{n_passed}/{len(report)} tests passed")
    out = Path(args.out) if args.out else Path(args.agent).with_name(f"locomotion_tests_{args.model}_model.json")
    out.write_text(json.dumps(report, indent=2, default=float))
    print(f"report: {out}")


if __name__ == "__main__":
    main()
