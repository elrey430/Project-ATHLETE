#!/bin/bash
# Project ATHLETE - MJX smoke test: does the exported athlete simulate on the GPU, and how fast?
# Run:  Scripts\Cloud\GpuJob.ps1 -Job mjx-smoke -ExtraFiles Saved\MuJoCo\athlete_5ft9_190lb.xml
# (Result 2026-09-29, default MuJoCo solver settings: ~57 samples/s at 1,024 worlds, 6% NaN. See the research doc.)
set -euo pipefail
cd "$(dirname "$0")"

nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader
python3 -m venv venv || { sudo apt-get update -qq && sudo apt-get install -y -qq python3-venv && python3 -m venv venv; }
. venv/bin/activate
pip install -q --upgrade pip
pip install -q "mujoco==3.14.0" "mujoco-mjx==3.14.0" "jax[cuda12]"
python -c "import jax; print('jax', jax.__version__, jax.devices())"
python bench_mjx.py athlete_5ft9_190lb.xml results/results.json
