# Project ATHLETE - builds the local Python environment for LocoMuJoCo development (CPU).
# Usage:  powershell -ExecutionPolicy Bypass -File Scripts\SetupLocoMuJoCoPython.ps1
#
# LocoMuJoCo v1.1.0 (MIT) is the imitation-learning stack used for training on cloud GPUs
# (Scripts\Cloud\jobs\). Locally it's for building and testing the athlete's environment and motion
# data on the CPU. Versions match the cloud job: the releases current when v1.1.0 came out (2026-03-10).
# Environment in Intermediate\LocoMuJoCoPython, LocoMuJoCo source in Intermediate\LocoMuJoCo. ~0.6-1 GB.

. (Join-Path $PSScriptRoot 'AthleteEnv.ps1')
$ErrorActionPreference = 'Continue' # git and pip report progress on stderr; exit codes are checked

$EnginePython = Join-Path $UERoot 'Engine\Binaries\ThirdParty\Python3\Win64\python.exe'
$VenvDir = Join-Path $ProjectRoot 'Intermediate\LocoMuJoCoPython'
$VenvPython = Join-Path $VenvDir 'Scripts\python.exe'
$SourceDir = Join-Path $ProjectRoot 'Intermediate\LocoMuJoCo'

if (-not (Test-Path $VenvPython)) {
    Write-Host "Creating $VenvDir"
    & $EnginePython -m venv $VenvDir
    if ($LASTEXITCODE -ne 0) { throw "venv creation failed ($LASTEXITCODE)" }
}
if (-not (Test-Path $SourceDir)) {
    git clone -q --depth 1 --branch v1.1.0 https://github.com/robfiras/loco-mujoco.git $SourceDir
    if ($LASTEXITCODE -ne 0) { throw "git clone failed ($LASTEXITCODE)" }
}

& $VenvPython -m pip install -q --disable-pip-version-check --upgrade pip
& $VenvPython -m pip install -q --disable-pip-version-check "jax==0.9.1" "mujoco==3.5.0" "mujoco-mjx==3.5.0" `
    "mujoco-warp==3.5.0.2" "warp-lang==1.12.0" "flax==0.12.5" -e $SourceDir
if ($LASTEXITCODE -ne 0) { throw "pip install failed ($LASTEXITCODE)" }

& $VenvPython -c "import jax, mujoco, loco_mujoco; print('jax', jax.__version__, jax.devices(), '| mujoco', mujoco.__version__, '| loco_mujoco OK')"
if ($LASTEXITCODE -ne 0) { throw "import check failed ($LASTEXITCODE)" }
Write-Host "LocoMuJoCo Python ready: $VenvPython" -ForegroundColor Green
