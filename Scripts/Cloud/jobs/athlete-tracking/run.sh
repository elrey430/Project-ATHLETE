#!/bin/bash
# Project ATHLETE - Milestone 4: the TRACKING policy for controller-driven movement (DReCon-style; LocoMuJoCo
# v1.1.0 PPO, MuJoCo Warp, cloud L4). See athlete_tracking.yaml.
# Run:  Scripts\Cloud\GpuJob.ps1 -Job athlete-tracking -MaxRunDuration 120m -OnDemand `
#           -ExtraFiles Saved\MuJoCo\athlete_reference.xml,Scripts\MuJoCo\athlete_loco,Scripts\MuJoCo\retarget_motion.py,`
#                       Scripts\MuJoCo\motion_matching.py,Scripts\MuJoCo\mirror_motion.py,Saved\Cloud\athlete-athlete-walk-0930-2014\results\agent\PPOJax_saved.pkl `
#           -LiveFiles results/agent/PPOJax_saved.pkl,results/agent/progress.json,results/metrics.jsonl
# The uploaded PPOJax_saved.pkl (job folder) is the policy training starts from (run 1: the walking policy,
# Saved/Cloud/athlete-athlete-walk-0930-2014; run 2: run 1's tracker); the live files
# (results/agent/) are this run's checkpoint: copied home after every chunk, and put back on the next VM
# if a Spot VM is reclaimed, where training resumes from them.
# Run 9 (100STYLE): also upload Scripts/MuJoCo/bvh.py, Scripts/MuJoCo/import_100style.py and the staged BVH
# folder Saved/MotionData/100STYLE/100style_bvh (import_100style.py --stage <it>; 21 files, ~73 MB): if that
# folder is here, its clips are fitted, mirrored and added to the matcher (ATHLETE_MOTION_SET=100style).
# Budget: VM boot (~5 min) + motion (~10 min in parallel: fit 6 clips, generate 12 matcher clips) +
# TRAIN_MINUTES (default 120) + a few min copying: run with -MaxRunDuration 150m. No video: recording it after training
# ran out of GPU memory (2026-10-01); record on the PC from the checkpoint instead.
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

echo "== Motion: mocap fitted to the athlete, plus motion-matched clips from random controller input =="
export ATHLETE_MJCF="$PWD/athlete_reference.xml"
export HF_HUB_DISABLE_SYMLINKS_WARNING=1
# Foot placement of the LocoMuJoCo clips: "frame" (a foot on the floor in every frame, as runs 1-8; removes
# running's flight phases) or "clip" (one floor height per clip; keeps them). See retarget_motion.py.
FLOOR=clip  # run 9 (user, 2026-10-04): flight phases back in the run clip
if [ -d 100style_bvh ]; then export ATHLETE_MOTION_SET=100style; fi  # the matcher's clips (motion_matching.py)
echo "motion set: ${ATHLETE_MOTION_SET:-base}; LocoMuJoCo clip floor: $FLOOR"
# In parallel (8 vCPUs): one after another this took ~25 min of a paid GPU VM (2026-10-01).
run_parallel() {  # run_parallel <name> <command>...: start in the background, log to $RESULTS/<name>.log
    local name=$1; shift
    # CPU only: these don't need the GPU, and several processes each grabbing it failed
    # ("Unable to initialize backend 'cuda'", 2026-10-01).
    JAX_PLATFORMS=cpu "$@" > "$RESULTS/$name.log" 2>&1 &
    PIDS+=($!); NAMES+=("$name")
}
wait_parallel() {  # fails (with the log tail) if any of them failed
    local i failed=0
    for i in "${!PIDS[@]}"; do
        if ! wait "${PIDS[$i]}"; then echo "FAILED: ${NAMES[$i]}"; tail -n 20 "$RESULTS/${NAMES[$i]}.log"; failed=1; fi
    done
    PIDS=(); NAMES=()
    return $failed
}
PIDS=(); NAMES=()
for clip in walk run walkturn random_walk stepinplace1 stepinplace2; do
    run_parallel "retarget_$clip" python retarget_motion.py --source default --datasets "$clip" --out "$PWD/motions" --floor "$FLOOR"
done
wait_parallel
grep -h -E '"dataset"|"duration_s"|"root_speed' "$RESULTS"/retarget_*.log
for clip in walk run walkturn random_walk; do  # left/right mirrors: turns both ways (the matcher uses them too)
    run_parallel "mirror_$clip" python mirror_motion.py --clips "$clip" --motions "$PWD/motions"
done
wait_parallel
grep -h -E '^[{]' "$RESULTS"/mirror_*.log
if [ "${ATHLETE_MOTION_SET:-base}" = 100style ]; then  # 21 clips, ~25 min of motion: ~3 min on 8 cores
    JAX_PLATFORMS=cpu python import_100style.py --data 100style_bvh --out "$PWD/motions" --jobs 8 > "$RESULTS/import_100style.log" 2>&1 ||
        { echo "FAILED: import_100style"; tail -n 20 "$RESULTS/import_100style.log"; exit 1; }
    grep -E '^[{]' "$RESULTS/import_100style.log"
    CLIPS100=$(python -c "import motion_matching as m; print(' '.join(c for c in m.CLIPS_100STYLE if not c.endswith('_mirror')))")
    JAX_PLATFORMS=cpu python mirror_motion.py --clips $CLIPS100 --motions "$PWD/motions" > "$RESULTS/mirror_100style.log" 2>&1 ||
        { echo "FAILED: mirror_100style"; tail -n 20 "$RESULTS/mirror_100style.log"; exit 1; }
fi
python stand_motion.py --motions "$PWD/motions" | grep -E '^[{]'  # quiet standing (needs the mirror map)
# 4 turn-heavy, 4 speed-heavy and 6 running-manoeuvre clips, 2-3 per process (names and seeds as one process).
# The whole motion dataset is built into the compiled training program, and 14 x 300 s clips plus the
# repeated mocap (~820k frames) ran the VM's 32 GB of RAM out (exit 137, 2026-10-02). ~680k fits (200 s).
# Run 9 adds the 100STYLE clips once each (~152k) and drops walk_mirror (88k): 170 s keeps it at ~700k.
# Run 11 adds 4 clips in the random-command test's own pattern (mm_test; half the remaining falls were hard
# stops from a run, often turning) and makes every clip from SHAPED commands (--shaped: through the controller
# layer, as at run time; until now training clips had sharper transitions than the athlete ever meets).
# 18 clips x 150 s: ~737k frames.
CLIP_SECONDS=150
SHAPED=--shaped
run_parallel "generate_turn" python motion_matching.py generate --motions "$PWD/motions" --name mm_turn --turn-heavy --count 4 --seconds $CLIP_SECONDS $SHAPED
run_parallel "generate_speed" python motion_matching.py generate --motions "$PWD/motions" --name mm_speed --speed-heavy --count 4 --seconds $CLIP_SECONDS $SHAPED
for first in 0 3; do
    run_parallel "generate_runman_$first" python motion_matching.py generate --motions "$PWD/motions" --name mm_runman --run-maneuvers --count 3 --first "$first" --seconds $CLIP_SECONDS $SHAPED
done
for first in 0 2; do
    run_parallel "generate_test_$first" python motion_matching.py generate --motions "$PWD/motions" --name mm_test --test-pattern --count 2 --first "$first" --seed 101 --seconds $CLIP_SECONDS $SHAPED
done
wait_parallel
grep -h -E '^[{]' "$RESULTS"/generate_*.log
loco-mujoco-set-all-caches --path "$PWD/motions"
cp motions/DEFAULT/mocap/AthleteReference/*_report.json "$RESULTS/"

echo "== Train (chunked: a checkpoint and a progress line after every chunk) =="
# The example's one-shot training saves nothing until the end; a run stopped or timed out loses all
# (2026-09-30). _train_chunked.py runs the same PPO in chunks within a time budget.
export ATHLETE_IMPORTS=athlete_loco   # registers MjxAthleteReference with LocoMuJoCo
# JAX takes 75% of the GPU by default; MuJoCo Warp loads its kernels into what's left. With this run's motion
# data (12 clips) that ran out: "Failed to load in-memory CUBIN ... out of memory" in chunk 2 (2026-10-01).
export XLA_PYTHON_CLIENT_MEM_FRACTION=0.6
# Warm start only if a policy to start from was uploaded (a new observation layout trains from scratch).
INIT_FROM=()
if [ -f PPOJax_saved.pkl ]; then INIT_FROM=(--init-from PPOJax_saved.pkl); else echo "No PPOJax_saved.pkl uploaded: training from scratch"; fi
TRAIN_MINUTES="${TRAIN_MINUTES:-235}"  # runs 6-9: ~2 h; runs 10-11: ~4 h (user: max 4.5 h for the VM, 2026-10-04; setup ~24 min)
START=$(date +%s)
set +e
python _train_chunked.py athlete_tracking.yaml --results "$RESULTS" --max-minutes "$TRAIN_MINUTES" \
    --resume "$RESULTS/agent/PPOJax_saved.pkl" "${INIT_FROM[@]}" --no-video --total-steps 5e9 --keep-every 5 2>&1 | tee "$RESULTS/train.log" | grep --line-buffered -v -E "nefc overflow|CCD overflow"
# (--line-buffered: grep writing to a file buffers ~4 KB, which held back the progress lines on 2026-09-30.)
STATUS=${PIPESTATUS[0]}
set -e
END=$(date +%s)
echo "constraint overflows (nefc > njmax): $(grep -c 'nefc overflow' "$RESULTS/train.log")"
echo "convex-collision overflows (CCD > naccdmax): $(grep -c 'CCD overflow' "$RESULTS/train.log")"
echo "all overflow warnings, by kind:"; grep -a -o -i '[a-z_ ]*overflow' "$RESULTS/train.log" | sort | uniq -c || echo "  none"
echo "training + video wall time: $((END - START)) s (exit $STATUS)" | tee "$RESULTS/timing.txt"
cp -r LocoMuJoCo_recordings "$RESULTS/" 2>/dev/null || echo "no video recorded"
exit $STATUS
