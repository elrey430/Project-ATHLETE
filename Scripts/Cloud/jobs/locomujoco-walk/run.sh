#!/bin/bash
# Project ATHLETE - baseline: reproduce LocoMuJoCo's published DeepMimic walking example unchanged.
# Run:  Scripts\Cloud\GpuJob.ps1 -Job locomujoco-walk -MaxRunDuration 3h
#
# LocoMuJoCo v1.1.0 (MIT), examples/training_examples/jax_rl_mimic: PPO with a DeepMimic reward trains
# the Unitree H1 humanoid to imitate LAFAN1 walking (walk1_subject5) with MuJoCo Warp on the GPU,
# 2,048 parallel environments, 300M steps; published: ~36 min on an RTX 3080 Ti. LAFAN1 is
# research-only (CC BY-NC-ND): fine for a baseline, never shipped.
#
# Dependencies are pinned to the releases current when v1.1.0 came out (2026-03-10); its own
# requirements are unpinned, and today's versions may not match its code.
set -euo pipefail
cd "$(dirname "$0")"
RESULTS="$PWD/results"

echo "== Machine =="
nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader
python3 --version

echo "== Install =="
python3 -m venv venv || { sudo apt-get update -qq && sudo apt-get install -y -qq python3-venv && python3 -m venv venv; }
. venv/bin/activate
pip install -q --upgrade pip
git clone -q --depth 1 --branch v1.1.0 https://github.com/robfiras/loco-mujoco.git
pip install -q "jax[cuda12]==0.9.1" "mujoco==3.5.0" "mujoco-mjx==3.5.0" "mujoco-warp==3.5.0.2" "warp-lang==1.12.0" \
    "flax==0.12.5" wandb -e ./loco-mujoco
pip freeze > "$RESULTS/pip_freeze.txt"
python -c "import jax, mujoco, mujoco_warp; print('jax', jax.__version__, jax.devices(), '| mujoco', mujoco.__version__)"

echo "== Headless rendering (for the final video) =="
# The Deep Learning image has the NVIDIA driver but not the GL/EGL loader libraries PyOpenGL looks
# for (measured 2026-09-29: importing mujoco with MUJOCO_GL=egl failed). Install them, then prove a
# render works BEFORE training: EGL (GPU), else OSMesa (software), else train without the video.
{ sudo DEBIAN_FRONTEND=noninteractive apt-get -o DPkg::Lock::Timeout=300 update -qq &&
  sudo DEBIAN_FRONTEND=noninteractive apt-get -o DPkg::Lock::Timeout=300 install -y -qq libegl1 libgl1 libosmesa6 > /dev/null; } ||
  echo "apt install failed; trying to render anyway"
RENDER_CHECK="import mujoco; r = mujoco.Renderer(mujoco.MjModel.from_xml_string('<mujoco/>'), 64, 64); r.render(); print('render OK')"
if MUJOCO_GL=egl python -c "$RENDER_CHECK"; then
    export MUJOCO_GL=egl
elif MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa python -c "$RENDER_CHECK"; then
    export MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa
else
    echo "No headless rendering: training still runs; the final video will fail after the agent is saved"
fi
echo "MUJOCO_GL=${MUJOCO_GL:-unset}"

echo "== Train =="
export WANDB_MODE=offline     # no Weights & Biases account: metrics stay local (also written to metrics.jsonl)
export WANDB_DIR="$RESULTS"
export RESULTS_DIR="$RESULTS"
export HYDRA_FULL_ERROR=1
EXAMPLE="$PWD/loco-mujoco/examples/training_examples/jax_rl_mimic"
START=$(date +%s)
set +e
python _run_experiment.py "$EXAMPLE/experiment.py" "hydra.run.dir=$RESULTS/hydra"
STATUS=$?
set -e
END=$(date +%s)
echo "training + evaluation wall time: $((END - START)) s (exit $STATUS)" | tee "$RESULTS/timing.txt"

# The trained agent is under results/hydra; the final video (if rendering worked) in the example folder.
cp -r "$EXAMPLE/LocoMuJoCo_recordings" "$RESULTS/" 2>/dev/null || echo "no video recorded"
exit $STATUS
