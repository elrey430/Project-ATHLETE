"""Project ATHLETE - LocoMuJoCo PPO training in chunks, with a checkpoint and progress after each.

Usage: python _train_chunked.py <config.yaml> --results <dir> [--total-steps 3e8] [--chunk-steps 2e7]
                                 [--max-minutes 70] [--video-envs 4] [--video-steps 400]

LocoMuJoCo's example compiles the whole training run into one GPU program: nothing is reported or saved
until the very end, so a run that times out or is stopped loses everything (2026-09-30). Here the same
algorithm (experiment.algorithm: PPOJax by default, or AMPJax/GAILJax) runs chunk by chunk through
LocoMuJoCo's own resume function; after each chunk the policy is saved (<results>/agent/<Algorithm>_saved.pkl,
with the discriminator for AMP/GAIL), the learning curve is appended
to <results>/metrics.jsonl and a progress line is printed. Training stops at --total-steps, or before a
chunk would run past --max-minutes. Then a short video of the policy is recorded.

--init-from starts from another trained policy's weights instead of random ones (inputs appended since
are added with zero weight: widen_inputs).
Differences from one long run: each chunk starts from fresh environment resets and a fresh optimizer
state (the network and its observation statistics carry over); validation runs once per chunk.
$ATHLETE_IMPORTS (comma-separated modules next to this file) are imported first, e.g. athlete_loco.
"""

import argparse
import importlib
import json
import os
import shutil
import sys
import time
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
for module in filter(None, os.environ.get("ATHLETE_IMPORTS", "").split(",")):
    importlib.import_module(module)

import dataclasses  # noqa: E402
import gc  # noqa: E402
from typing import Sequence  # noqa: E402

import flax.linen as nn  # noqa: E402
import jax  # noqa: E402
import jax.numpy as jnp  # noqa: E402
import numpy as np  # noqa: E402
from loco_mujoco.algorithms.common.networks import FullyConnectedNet  # noqa: E402
from omegaconf import OmegaConf, open_dict  # noqa: E402
from loco_mujoco import TaskFactory  # noqa: E402
from loco_mujoco import algorithms  # noqa: E402
from loco_mujoco.utils import MetricsHandler  # noqa: E402


class SelectedInputNet(FullyConnectedNet):
    """LocoMuJoCo's FullyConnectedNet that only sees some observation entries (input_ind)."""
    input_ind: Sequence[int] = ()

    @nn.compact
    def __call__(self, x):
        return FullyConnectedNet(hidden_layer_dims=self.hidden_layer_dims, output_dim=self.output_dim,
                                 activation=self.activation, output_activation=self.output_activation,
                                 use_running_mean_stand=self.use_running_mean_stand,
                                 squeeze_output=self.squeeze_output)(x[..., jnp.array(self.input_ind)])


def without_observations(agent_conf, env, names):
    """
    AMP/GAIL: the discriminator judges MOVEMENT, so it must not see observations the motion data can't share,
    such as the commanded velocity (experiment.discriminator_exclude_obs). Measured 2026-10-01: with the
    command visible, the discriminator told policy from motion data apart from the first update (0.01 vs
    0.99) and the style reward was worthless. LocoMuJoCo's AMP example avoids it by using the clip's own
    velocity as the goal.
    """
    excluded = set()
    for name in names:
        excluded.update(int(i) for i in env.obs_container[name].obs_ind)
    keep = tuple(i for i in range(env.info.observation_space.shape[0]) if i not in excluded)
    old = agent_conf.discriminator
    new = SelectedInputNet(hidden_layer_dims=old.hidden_layer_dims, output_dim=old.output_dim, activation=old.activation,
                           output_activation=old.output_activation, use_running_mean_stand=old.use_running_mean_stand,
                           squeeze_output=old.squeeze_output, input_ind=keep)
    print(f"train_chunked: discriminator sees {len(keep)} of {env.info.observation_space.shape[0]} observations (without {list(names)})", flush=True)
    return dataclasses.replace(agent_conf, discriminator=new)


def as_this_run(agent_state, agent_conf):
    """
    Makes a loaded checkpoint look exactly like the state training returns, so every chunk reuses ONE
    compiled program. A loaded state differs in two ways jit treats as a new program:
      - Python numbers where training returns JAX arrays (train_state.step: int vs int32 array);
      - the optimizer and network functions are the checkpoint's own (built while loading), not this run's
        (static parts of the train state, compared by identity).
    Each mismatch recompiled the whole training program, motion data included, on the second chunk; on
    2026-10-01 that doubled memory and killed runs at chunk 2 (GPU OOM, then the VM's 32 GB of RAM).
    """
    agent_state = jax.tree_util.tree_map(lambda x: jnp.asarray(x) if isinstance(x, (bool, int, float)) else x, agent_state)
    if hasattr(agent_state, "train_state"):
        agent_state = agent_state.replace(train_state=agent_state.train_state.replace(
            apply_fn=agent_conf.network.apply, tx=agent_conf.tx))
    if hasattr(agent_state, "disc_train_state") and hasattr(agent_conf, "disc_tx"):
        agent_state = agent_state.replace(disc_train_state=agent_state.disc_train_state.replace(
            apply_fn=agent_conf.discriminator.apply, tx=agent_conf.disc_tx))
    return agent_state


def widen_inputs(agent_state, new_dim):
    """
    Warm start with MORE observations, appended at the end (e.g. athlete_loco's lookahead goal after
    GoalTrajMimic): the input layers get zero weights for the new inputs (with zero optimizer moments),
    and the input normalization neutral statistics (mean 0, variance 1). The widened policy acts exactly
    like the old one at first and learns to use the new inputs.
    """
    train_state = agent_state.train_state
    stats = train_state.run_stats["RunningMeanStd_0"]
    old_dim = stats["mean"].shape[0]
    if old_dim == new_dim:
        return agent_state
    extra = new_dim - old_dim
    assert extra > 0, f"can't narrow the inputs ({old_dim} -> {new_dim})"

    def pad_rows(x):  # input-layer kernels (and their optimizer moments): (old_dim, width)
        if hasattr(x, "shape") and len(x.shape) == 2 and x.shape[0] == old_dim:
            return jnp.concatenate([jnp.asarray(x), jnp.zeros((extra, x.shape[1]), x.dtype)], axis=0)
        return x

    run_stats = {"RunningMeanStd_0": dict(stats, mean=jnp.concatenate([jnp.asarray(stats["mean"]), jnp.zeros(extra, stats["mean"].dtype)]),
                                          var=jnp.concatenate([jnp.asarray(stats["var"]), jnp.ones(extra, stats["var"].dtype)]))}
    train_state = train_state.replace(params=jax.tree_util.tree_map(pad_rows, train_state.params),
                                      opt_state=jax.tree_util.tree_map(pad_rows, train_state.opt_state),
                                      run_stats=run_stats)
    print(f"train_chunked: inputs widened {old_dim} -> {new_dim} (the new ones start with zero weight)", flush=True)
    return agent_state.replace(train_state=train_state)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("config")
    parser.add_argument("--results", required=True)
    parser.add_argument("--total-steps", type=float, default=None, help="default: the config's total_timesteps")
    parser.add_argument("--chunk-steps", type=float, default=2e7)
    parser.add_argument("--max-minutes", type=float, default=70.0)
    parser.add_argument("--video-envs", type=int, default=4)
    parser.add_argument("--video-steps", type=int, default=400)
    parser.add_argument("--no-video", action="store_true")
    parser.add_argument("--resume", default=None,
                        help="checkpoint to continue from (<Algorithm>_saved.pkl with progress.json beside it); ignored if missing")
    parser.add_argument("--init-from", default=None,
                        help="start from this trained policy's weights (same network and observations), at step 0; "
                             "ignored when --resume finds a checkpoint")
    parser.add_argument("--keep-every", type=int, default=0,
                        help="also keep a copy of every Nth chunk's checkpoint (agent/history/), to pick the best later")
    parser.add_argument("--set", nargs="*", default=[], help="config overrides, e.g. experiment.num_envs=16")
    args = parser.parse_args()
    started = time.time()

    config = OmegaConf.merge(OmegaConf.load(args.config), OmegaConf.from_dotlist(args.set))
    total_steps = args.total_steps or float(config.experiment.total_timesteps)
    with open_dict(config.experiment):
        config.experiment.total_timesteps = args.chunk_steps  # one chunk = one compiled training call
        config.experiment.validation.num = 1
    results = Path(args.results)
    (results / "agent").mkdir(parents=True, exist_ok=True)
    metrics_file = open(results / "metrics.jsonl", "a")

    # experiment.algorithm: PPOJax (default; e.g. DeepMimic-style imitation) or AMPJax / GAILJax, whose
    # discriminator learns style from the motion data (the "expert dataset", built from the loaded clips).
    algorithm_name = OmegaConf.select(config, "experiment.algorithm", default="PPOJax")
    Algorithm = getattr(algorithms, algorithm_name)
    factory = TaskFactory.get_factory_cls(config.experiment.task_factory.name)
    env = factory.make(**config.experiment.env_params, **config.experiment.task_factory.params)
    agent_conf = Algorithm.init_agent_conf(env, config)
    if hasattr(agent_conf, "add_expert_dataset"):
        agent_conf = agent_conf.add_expert_dataset(env.create_dataset())
        excluded = OmegaConf.select(config, "experiment.discriminator_exclude_obs", default=None)
        if excluded:
            agent_conf = without_observations(agent_conf, env, list(excluded))
    steps_per_chunk = config.experiment.num_updates * config.experiment.num_steps * config.experiment.num_envs
    mh = MetricsHandler(config, env) if config.experiment.validation.active else None
    first_chunk = jax.jit(Algorithm.build_train_fn(env, agent_conf, mh=mh))
    next_chunk = jax.jit(Algorithm.build_resume_train_fn(env, agent_conf, mh=mh))
    print(f"train_chunked: {algorithm_name}, {type(env).__name__}, obs {env.info.observation_space.shape}, act {env.info.action_space.shape}; "
          f"{steps_per_chunk:,} steps per chunk, up to {total_steps:,.0f} steps or {args.max_minutes} min", flush=True)

    agent_state, done_steps, chunk, last_chunk_s = None, 0, 0, None
    rng = jax.random.PRNGKey(1)
    if args.resume and Path(args.resume).is_file():
        # A previous VM's checkpoint (GpuJob.ps1 puts it back after a Spot VM is reclaimed).
        _, agent_state = Algorithm.load_agent(args.resume)
        agent_state = as_this_run(agent_state, agent_conf)
        progress = json.loads((Path(args.resume).parent / "progress.json").read_text())
        done_steps, chunk = progress["steps"], progress["chunks"]
        rng = jax.random.PRNGKey(1 + chunk)  # don't replay the same random streams
        print(f"train_chunked: resuming from {args.resume} at {done_steps / 1e6:.0f}M steps (chunk {chunk})", flush=True)
    elif args.init_from:
        # Warm start: e.g. the multi-clip tracker from the walking policy (identical observation layout).
        _, agent_state = Algorithm.load_agent(args.init_from)
        agent_state = as_this_run(widen_inputs(agent_state, env.info.observation_space.shape[0]), agent_conf)
        print(f"train_chunked: starting from the weights of {args.init_from}", flush=True)
    compiles_left = 1 if agent_state is not None else 2  # the first and the resume function each compile once
    while done_steps < total_steps:
        elapsed_min = (time.time() - started) / 60
        if last_chunk_s and elapsed_min + 1.2 * last_chunk_s / 60 > args.max_minutes:
            print(f"train_chunked: stopping for the time budget ({elapsed_min:.1f} of {args.max_minutes} min used)", flush=True)
            break
        rng, key = jax.random.split(rng)
        chunk_started = time.time()
        if agent_state is None:
            out = jax.block_until_ready(first_chunk(key))
            # The first-chunk program runs only once: free it before the resume program compiles. Both
            # embed the motion data; together they ran the VM out of RAM (exit 137, 2026-10-01, from scratch
            # with 28 clips). gc first: the cached program is released when nothing references it.
            first_chunk = None
            gc.collect()
            jax.clear_caches()
        else:
            out = jax.block_until_ready(next_chunk(key, agent_state))
        last_chunk_s = time.time() - chunk_started
        agent_state = out["agent_state"]
        done_steps += steps_per_chunk
        chunk += 1

        metrics = out["training_metrics"]
        returns = np.asarray(metrics.mean_episode_return)
        lengths = np.asarray(metrics.mean_episode_length)
        # AMP/GAIL: how "expert-like" the discriminator finds the policy's and the motion data's transitions.
        disc_policy = np.asarray(getattr(metrics, "discriminator_output_policy", np.full_like(returns, np.nan)))
        disc_expert = np.asarray(getattr(metrics, "discriminator_output_expert", np.full_like(returns, np.nan)))
        n = len(returns)
        for i in range(n):
            row = {"step": int(done_steps - steps_per_chunk + (i + 1) * steps_per_chunk / n),
                   "Mean Episode Return": float(returns[i]), "Mean Episode Length": float(lengths[i])}
            if np.isfinite(disc_policy[i]):
                row.update({"Discriminator Policy": float(disc_policy[i]), "Discriminator Expert": float(disc_expert[i])})
            metrics_file.write(json.dumps(row) + "\n")
        metrics_file.flush()
        Algorithm.save_agent(str(results / "agent"), agent_conf, agent_state)  # <results>/agent/<Algorithm>_saved.pkl
        (results / "agent" / "progress.json").write_text(json.dumps({"steps": done_steps, "chunks": chunk}))
        if args.keep_every and chunk % args.keep_every == 0:
            history = results / "agent" / "history"
            history.mkdir(exist_ok=True)
            shutil.copy(results / "agent" / f"{algorithm_name}_saved.pkl", history / f"{algorithm_name}_chunk{chunk:03d}.pkl")
        finite = all(bool(np.isfinite(np.asarray(x)).all()) for x in jax.tree_util.tree_leaves(agent_state)
                     if hasattr(x, "dtype") and np.issubdtype(np.asarray(x).dtype, np.floating))
        print(f"train_chunked: chunk {chunk}: {done_steps / 1e6:.0f}M steps, episode length {lengths[-1]:.0f}/"
              f"{config.experiment.env_params.horizon}, return {returns[-1]:.1f}, {steps_per_chunk / last_chunk_s:,.0f} steps/s "
              f"(chunk {last_chunk_s / 60:.1f} min{', incl. compile' if compiles_left > 0 else ''}), parameters finite: {finite}"
              + (f", discriminator policy {disc_policy[-1]:.2f} / expert {disc_expert[-1]:.2f}" if np.isfinite(disc_policy[-1]) else ""), flush=True)
        compiles_left -= 1

    summary = {"steps": done_steps, "chunks": chunk, "minutes": round((time.time() - started) / 60, 1)}
    (results / "training_summary.json").write_text(json.dumps(summary, indent=2))
    print("train_chunked: done", summary, flush=True)

    if not args.no_video and agent_state is not None:
        Algorithm.play_policy(env, agent_conf, agent_state, deterministic=True, n_steps=args.video_steps,
                           n_envs=args.video_envs, record=True, train_state_seed=0)
        print("train_chunked: video", env.video_file_path, flush=True)


if __name__ == "__main__":
    main()
