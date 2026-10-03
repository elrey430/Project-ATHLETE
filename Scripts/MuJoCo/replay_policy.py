"""Project ATHLETE - replays a trained LocoMuJoCo walking policy on CPU MuJoCo and measures the gait.

Usage (LocoMuJoCo Python):
    Intermediate/LocoMuJoCoPython/Scripts/python.exe Scripts/MuJoCo/replay_policy.py <PPOJax_saved.pkl>
        [--episodes 20] [--video out.avi]

Training ran on the GPU with simplifications (Scripts/MuJoCo/athlete_loco/env.py: foot-only contacts,
capsule soles). This checks the policy on:
  A. the training model (MjxAthleteReference) stepped by CPU MuJoCo: does the policy survive the engine change?
  B. the FULL model (AthleteReference): box feet, every contact, MuJoCo's default solver. The real test.
Same control rate as training (2 ms physics, 10 ms control). Deterministic policy (its mean action), applied
exactly as LocoMuJoCo's own replay does. Each episode starts at a random point of the reference walk.

Per episode: length (of the horizon) and whether it fell; mean imitation reward per step (training's
MimicReward); horizontal walking speed; stance-foot slip; deepest floor penetration; worst joint-limit excess.
"""

import argparse
import json
import sys
from pathlib import Path

import jax
import jax.numpy as jnp
import mujoco
import numpy as np
from omegaconf import OmegaConf, open_dict

sys.path.insert(0, str(Path(__file__).resolve().parent))
import athlete_loco  # noqa: E402,F401
from loco_mujoco import TaskFactory  # noqa: E402
from loco_mujoco.algorithms import PPOJax  # noqa: E402

MJX_ONLY = ("use_mjwarp", "nconmax", "njmax")


def make_env(config, env_name):
    params = OmegaConf.to_container(config.experiment.env_params, resolve=True)
    for key in MJX_ONLY:
        params.pop(key, None)
    params.update(env_name=env_name, timestep=0.002, n_substeps=5, headless=True)
    factory = TaskFactory.get_factory_cls(config.experiment.task_factory.name)
    return factory.make(**params, **OmegaConf.to_container(config.experiment.task_factory.params, resolve=True))


def foot_low_points(model, data):
    """Lowest point of each foot box (world z) and its horizontal position."""
    out = []
    for name in athlete_loco.env.FOOT_GEOMS:
        geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        corners = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]) * model.geom_size[geom]
        world = corners @ data.geom_xmat[geom].reshape(3, 3).T + data.geom_xpos[geom]
        out.append((world[:, 2].min(), data.geom_xpos[geom][:2].copy()))
    return out


def evaluate(label, env, agent_conf, agent_state, episodes, seed, renderer=None, video=None):
    train_state = agent_state.train_state
    train_state.params["log_std"] = np.ones_like(train_state.params["log_std"]) * -np.inf  # deterministic

    def sample_actions(ts, obs, key):
        # Input normalization frozen as trained (PPOJax.play_policy updates it, which made results depend
        # on earlier episodes: see locomotion_tests.Driver).
        y, _ = agent_conf.network.apply({"params": ts.params, "run_stats": ts.run_stats}, obs, mutable=["run_stats"])
        return y[0].sample(seed=key), ts

    policy = jax.jit(sample_actions)
    model, data = env._model, env._data
    hinges = [j for j in range(model.njnt) if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE]
    floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    horizon = env.info.horizon
    rng = jax.random.key(seed)
    np.random.seed(seed)
    results = []
    for episode in range(episodes):
        obs = env.reset()
        rewards, slips, deepest, worst_limit = [], [], 0.0, 0.0
        start_xy = data.qpos[:2].copy()
        previous_feet = foot_low_points(model, data)
        steps, fell = 0, False
        for step in range(horizon):
            rng, key = jax.random.split(rng)
            action, train_state = policy(train_state, obs, key)
            obs, reward, absorbing, done, _ = env.step(jnp.atleast_2d(action))
            steps += 1
            rewards.append(float(reward))
            feet = foot_low_points(model, data)
            for (low, xy), (_, previous_xy) in zip(feet, previous_feet):
                if low < 0.01:  # foot on the ground: how fast does it slide?
                    slips.append(np.linalg.norm(xy - previous_xy) / env.dt)
            previous_feet = feet
            for i in range(data.ncon):
                contact = data.contact[i]
                if floor in (contact.geom1, contact.geom2):
                    deepest = min(deepest, contact.dist)
            for j in hinges:
                q = data.qpos[model.jnt_qposadr[j]]
                low, high = model.jnt_range[j]
                worst_limit = max(worst_limit, low - q, q - high)
            if renderer is not None and episode == 0 and step % 3 == 0:
                video.append(render_frame(renderer, model, data))
            if absorbing or done:
                fell = bool(absorbing) and step < horizon - 1
                break
        distance = np.linalg.norm(data.qpos[:2] - start_xy)
        results.append({"steps": steps, "fell": fell, "reward_per_step": float(np.mean(rewards)),
                        "speed_mps": float(distance / (steps * env.dt)),
                        "stance_slip_mps_median": float(np.median(slips)) if slips else None,
                        "deepest_floor_penetration_mm": float(-deepest * 1000),
                        "worst_limit_excess_deg": float(np.degrees(worst_limit))})
    summary = {
        "episodes": episodes, "falls": int(sum(r["fell"] for r in results)),
        "mean_length": float(np.mean([r["steps"] for r in results])), "horizon": horizon,
        "reward_per_step": float(np.mean([r["reward_per_step"] for r in results])),
        "speed_mps_median": float(np.median([r["speed_mps"] for r in results])),
        "stance_slip_mps_median": float(np.median([r["stance_slip_mps_median"] for r in results if r["stance_slip_mps_median"] is not None])),
        "deepest_floor_penetration_mm": float(max(r["deepest_floor_penetration_mm"] for r in results)),
        "worst_limit_excess_deg": float(max(r["worst_limit_excess_deg"] for r in results)),
    }
    print(f"{label}: falls {summary['falls']}/{episodes}, mean length {summary['mean_length']:.0f}/{horizon}, "
          f"reward/step {summary['reward_per_step']:.3f}, speed {summary['speed_mps_median']:.2f} m/s, "
          f"stance slip {summary['stance_slip_mps_median']:.3f} m/s, penetration {summary['deepest_floor_penetration_mm']:.1f} mm, "
          f"limit excess {summary['worst_limit_excess_deg']:.1f} deg", flush=True)
    return summary, results


CAMERA = mujoco.MjvCamera()


def render_frame(renderer, model, data):
    CAMERA.type = mujoco.mjtCamera.mjCAMERA_FREE
    CAMERA.lookat[:] = [data.qpos[0], data.qpos[1], 0.9]
    CAMERA.distance, CAMERA.elevation = 3.5, -10.0
    CAMERA.azimuth = np.degrees(np.arctan2(data.qvel[1], data.qvel[0] + 1e-9)) + 90.0
    renderer.update_scene(data, CAMERA)
    return renderer.render().copy()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("agent")
    parser.add_argument("--episodes", type=int, default=20)
    parser.add_argument("--video", default=None, help="AVI of the full model's first episode")
    parser.add_argument("--out", default=None, help="JSON report (default: next to the agent)")
    parser.add_argument("--setup", choices=["A", "B", "both"], default="both")
    args = parser.parse_args()
    out = Path(args.out) if args.out else Path(args.agent).with_name("cpu_replay_report.json")

    if args.setup == "both":
        # One process per setup: building the second LocoMuJoCo environment in the same process after the
        # first one ran failed inside LocoMuJoCo (IndexError, 2026-10-01); each setup alone works.
        import subprocess
        report = {}
        for setup in ("A", "B"):
            part = out.with_name(f"{out.stem}_{setup}.json")
            command = [sys.executable, __file__, args.agent, "--episodes", str(args.episodes), "--setup", setup, "--out", str(part)]
            if args.video:
                command += ["--video", args.video]
            subprocess.run(command, check=True)
            report.update(json.loads(part.read_text()))
        out.write_text(json.dumps(report, indent=2))
        print(f"report: {out}")
        return

    agent_conf, agent_state = PPOJax.load_agent(args.agent)
    config = agent_conf.config
    report = {}
    setups = {"A": ("A. training model, CPU MuJoCo", "MjxAthleteReference"), "B": ("B. full model, CPU MuJoCo", "AthleteReference")}
    for label, env_name in [setups[args.setup]]:
        env = make_env(config, env_name)
        renderer, frames = None, None
        if args.video and env_name == "AthleteReference":
            env._model.vis.global_.offwidth, env._model.vis.global_.offheight = 960, 540
            renderer, frames = mujoco.Renderer(env._model, 540, 960), []
        _, agent_state = PPOJax.load_agent(args.agent)  # fresh observation statistics for each setup
        summary, episodes = evaluate(label, env, agent_conf, agent_state, args.episodes, seed=7, renderer=renderer, video=frames)
        report[label] = {"summary": summary, "episodes": episodes}
        if frames:
            import cv2
            writer = cv2.VideoWriter(args.video, cv2.VideoWriter_fourcc(*"MJPG"), 33, (960, 540))
            for frame in frames:
                writer.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
            writer.release()
            print(f"video: {args.video} ({len(frames)} frames)")
    out.write_text(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
