#!/bin/bash
# Project ATHLETE - installs everything the training jobs need into /opt/athlete, on the Deep Learning
# image. BuildImage.ps1 then saves the disk as image family "athlete-gpu", which GPU jobs boot from: no
# ~10 min install and no ~3.5 GiB of downloads through the NAT per run. Runs on a cheap CPU VM (the
# packages don't need a GPU to install). Versions = the athlete-walk / locomujoco-walk jobs' pins.
set -euo pipefail
cd "$(dirname "$0")"
RESULTS="$PWD/results"

echo "== System packages =="
sudo DEBIAN_FRONTEND=noninteractive apt-get -o DPkg::Lock::Timeout=300 update -qq
sudo DEBIAN_FRONTEND=noninteractive apt-get -o DPkg::Lock::Timeout=300 install -y -qq \
    libegl1 libgl1 libosmesa6 ffmpeg python3-venv > /dev/null

echo "== /opt/athlete: LocoMuJoCo v1.1.0 stack =="
sudo mkdir -p /opt/athlete
sudo chmod 777 /opt/athlete
python3 -m venv /opt/athlete/venv
. /opt/athlete/venv/bin/activate
pip install -q --upgrade pip
git clone -q --depth 1 --branch v1.1.0 https://github.com/robfiras/loco-mujoco.git /opt/athlete/loco-mujoco
pip install -q "jax[cuda12]==0.9.1" "mujoco==3.5.0" "mujoco-mjx==3.5.0" "mujoco-warp==3.5.0.2" "warp-lang==1.12.0" \
    "flax==0.12.5" wandb -e /opt/athlete/loco-mujoco
pip cache purge > /dev/null
pip freeze > "$RESULTS/pip_freeze.txt"

echo "== Motion data (LocoMuJoCo's default mocap on its human skeleton) =="
export HF_HOME=/opt/athlete/hf HF_HUB_DISABLE_SYMLINKS_WARNING=1
python - <<'EOF'
from huggingface_hub import hf_hub_download
for task in ["walk", "run"]:
    print(hf_hub_download(repo_id="robfiras/loco-mujoco-datasets", repo_type="dataset",
                          filename=f"DefaultDatasets/mocap/SkeletonTorque/{task}.npz"))
EOF
python -c "import loco_mujoco; from loco_mujoco.environments import LocoEnv; LocoEnv.registered_envs['SkeletonTorque'](); print('LocoMuJoCo models OK')"

sudo chmod -R a+rwX /opt/athlete   # jobs run as whichever account logs in, and LocoMuJoCo writes its cache paths there
du -sh /opt/athlete/* | tee "$RESULTS/sizes.txt"
echo "image setup done"
