"""Project ATHLETE - runs a LocoMuJoCo example script unchanged, without a Weights & Biases account.

Usage: python _run_experiment.py <path/to/experiment.py> [hydra arguments...]

The examples call wandb.login() and log metrics to W&B. Here W&B runs offline (WANDB_MODE=offline),
login is skipped, and every logged metric is also appended to $RESULTS_DIR/metrics.jsonl as plain JSON,
so the learning curve can be read without W&B.

$ATHLETE_IMPORTS (comma-separated module names, found next to this file) are imported first: e.g.
athlete_loco, which registers the athlete's environments with LocoMuJoCo.
"""

import importlib
import json
import os
import runpy
import sys

import wandb

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
for module in filter(None, os.environ.get("ATHLETE_IMPORTS", "").split(",")):
    importlib.import_module(module)
    print(f"_run_experiment: imported {module}")

wandb.login = lambda *args, **kwargs: True  # offline: nothing to log in to

metrics_file = open(os.path.join(os.environ["RESULTS_DIR"], "metrics.jsonl"), "a")
original_init = wandb.init


def plain(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return str(value)


def init(*args, **kwargs):
    run = original_init(*args, **kwargs)
    original_log = run.log

    def log(data, step=None, **log_kwargs):
        metrics_file.write(json.dumps({"step": step, **{key: plain(value) for key, value in data.items()}}) + "\n")
        metrics_file.flush()
        return original_log(data, step=step, **log_kwargs)

    try:
        run.log = log
    except AttributeError:
        print("run_experiment: couldn't mirror metrics to metrics.jsonl; they are in the offline W&B run only")
    return run


wandb.init = init

script = os.path.abspath(sys.argv[1])
sys.argv = [script] + sys.argv[2:]
os.chdir(os.path.dirname(script))  # the example writes its recordings next to itself
runpy.run_path(script, run_name="__main__")
