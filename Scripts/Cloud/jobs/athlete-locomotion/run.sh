#!/bin/bash
# Project ATHLETE - Milestone 4: the athlete follows controller input (velocity commands): AMP with commands that change mid-episode (LocoMuJoCo v1.1.0, MuJoCo Warp, cloud L4). See athlete_locomotion.yaml.
# Run:  Scripts\Cloud\GpuJob.ps1 -Job athlete-locomotion -MaxRunDuration 80m -SpotAttempts 2 `
#           -ExtraFiles Saved\MuJoCo\athlete_reference.xml,Scripts\MuJoCo\athlete_loco,Scripts\MuJoCo\retarget_motion.py `
#           -LiveFiles results/agent/AMPJax_saved.pkl,results/agent/progress.json,results/metrics.jsonl
# The live files are the checkpoint: copied home after every chunk, and put back on the next VM if a
# Spot VM is reclaimed, where training resumes from them.
# Budget: setup (~5 min on the athlete-gpu image, ~12 without) + TRAIN_MINUTES (default 45) + ~10 min
# video and copying; keep -MaxRunDuration above that.
#
# Same PPO settings as the H1 baseline (locomujoco-walk, which trained in ~24 min on the L4); see
# athlete_walk.yaml for the differences. The walking motion is fitted to the athlete's body here (a few
# minutes on the CPU) instead of uploading the 200 MB result. The athlete trains slower than the H1:
# 300M steps hadn't finished after 43 min (2026-09-30), so training runs to a time budget.
set -euo pipefail
cd "$(dirname "$0")"
RESULTS="$PWD/results"

echo "== Machine =="
nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader
python3 --version

if [ -x /opt/athlete/venv/bin/python ]; then
    echo "== Pre-installed (athlete-gpu image) =="
    . /opt/athlete/venv/bin/activate
    export HF_HOME=/opt/athlete/hf
else
    echo "== Install (no athlete-gpu image) =="
    { sudo DEBIAN_FRONTEND=noninteractive apt-get -o DPkg::Lock::Timeout=300 update -qq &&
      sudo DEBIAN_FRONTEND=noninteractive apt-get -o DPkg::Lock::Timeout=300 install -y -qq libegl1 libgl1 libosmesa6 ffmpeg > /dev/null; } ||
      echo "apt install failed; continuing (rendering or the video may fail)"
    python3 -m venv venv || { sudo apt-get install -y -qq python3-venv && python3 -m venv venv; }
    . venv/bin/activate
    pip install -q --upgrade pip
    git clone -q --depth 1 --branch v1.1.0 https://github.com/robfiras/loco-mujoco.git
    pip install -q "jax[cuda12]==0.9.1" "mujoco==3.5.0" "mujoco-mjx==3.5.0" "mujoco-warp==3.5.0.2" "warp-lang==1.12.0" \
        "flax==0.12.5" wandb -e ./loco-mujoco
fi
pip freeze > "$RESULTS/pip_freeze.txt"
python -c "import jax, mujoco, mujoco_warp; print('jax', jax.__version__, jax.devices(), '| mujoco', mujoco.__version__)"

echo "== Headless rendering (for the final video) =="
# Prefer NVIDIA's EGL (Mesa's software renderer took ~9 minutes for the baseline's video).
NVIDIA_EGL=$(ls /usr/share/glvnd/egl_vendor.d/*nvidia*.json 2>/dev/null | head -1 || true)
if [ -n "$NVIDIA_EGL" ]; then export __EGL_VENDOR_LIBRARY_FILENAMES="$NVIDIA_EGL"; fi
RENDER_CHECK="import mujoco; r = mujoco.Renderer(mujoco.MjModel.from_xml_string('<mujoco/>'), 64, 64); r.render(); print('render OK')"
if MUJOCO_GL=egl python -c "$RENDER_CHECK"; then
    export MUJOCO_GL=egl
elif MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa python -c "$RENDER_CHECK"; then
    export MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa
else
    echo "No headless rendering: training still runs; the final video will fail after the agent is saved"
fi
echo "MUJOCO_GL=${MUJOCO_GL:-unset} EGL vendor=${__EGL_VENDOR_LIBRARY_FILENAMES:-default}"

echo "== Motion: LocoMuJoCo's default clips fitted to the athlete (AMP style data) =="
export ATHLETE_MJCF="$PWD/athlete_reference.xml"
export HF_HUB_DISABLE_SYMLINKS_WARNING=1
python retarget_motion.py --source default --datasets walk run walkturn random_walk stepinplace1 stepinplace2 --out "$PWD/motions" | grep -E '"dataset"|"duration_s"|"root_speed'
loco-mujoco-set-all-caches --path "$PWD/motions"
cp motions/DEFAULT/mocap/AthleteReference/*_report.json "$RESULTS/"

echo "== Train (chunked: a checkpoint and a progress line after every chunk) =="
# The example's one-shot training saves nothing until the end; a run stopped or timed out loses all
# (2026-09-30). _train_chunked.py runs the same PPO in chunks within a time budget.
export ATHLETE_IMPORTS=athlete_loco   # registers MjxAthleteReference with LocoMuJoCo
TRAIN_MINUTES="${TRAIN_MINUTES:-45}"
START=$(date +%s)
set +e
python _train_chunked.py athlete_locomotion.yaml --results "$RESULTS" --max-minutes "$TRAIN_MINUTES" \
    --resume "$RESULTS/agent/AMPJax_saved.pkl" 2>&1 | tee "$RESULTS/train.log" | grep --line-buffered -v -E "nefc overflow|CCD overflow"
# (--line-buffered: grep writing to a file buffers ~4 KB, which held back the progress lines on 2026-09-30.)
STATUS=${PIPESTATUS[0]}
set -e
END=$(date +%s)
echo "constraint overflows (nefc > njmax): $(grep -c 'nefc overflow' "$RESULTS/train.log")"
echo "convex-collision overflows (CCD > naccdmax): $(grep -c 'CCD overflow' "$RESULTS/train.log")"
echo "training + video wall time: $((END - START)) s (exit $STATUS)" | tee "$RESULTS/timing.txt"
cp -r LocoMuJoCo_recordings "$RESULTS/" 2>/dev/null || echo "no video recorded"
exit $STATUS
