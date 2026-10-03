# Project ATHLETE - builds the Python environment the Learning Agents trainer runs in.
# Usage:  powershell -ExecutionPolicy Bypass -File Scripts\SetupLearningPython.ps1
#
# The trainer runs <Project>\Intermediate\PipInstall\Scripts\python.exe. Unreal would normally build
# that environment itself at editor start (see bRunPipInstallOnStartup in Config\DefaultEngine.ini),
# installing the CUDA build of PyTorch (~2.5 GB). This machine trains on the CPU, so this script
# builds it from the engine's bundled Python with the CPU build of the same PyTorch version (~200 MB).
# Safe to re-run; re-run it after deleting Intermediate\.

. (Join-Path $PSScriptRoot 'AthleteEnv.ps1')

$EnginePython = Join-Path $UERoot 'Engine\Binaries\ThirdParty\Python3\Win64\python.exe'
$VenvDir = Join-Path $ProjectRoot 'Intermediate\PipInstall'
$VenvPython = Join-Path $VenvDir 'Scripts\python.exe'

if (-not (Test-Path $EnginePython)) { throw "Engine Python not found: $EnginePython" }

if (-not (Test-Path $VenvPython)) {
    Write-Host "Creating the trainer's Python environment in $VenvDir"
    & $EnginePython -m venv $VenvDir
    if ($LASTEXITCODE -ne 0) { throw "venv creation failed ($LASTEXITCODE)" }
}

# The PyTorch version Unreal's PythonMLPackages plugin pins (torch 2.5.1), CPU build.
& $VenvPython -m pip install --disable-pip-version-check torch==2.5.1 --index-url https://download.pytorch.org/whl/cpu
if ($LASTEXITCODE -ne 0) { throw "torch install failed ($LASTEXITCODE)" }

# What the Learning Agents training scripts import besides torch (TensorBoard and MLflow are optional).
& $VenvPython -m pip install --disable-pip-version-check numpy smart_open boto3
if ($LASTEXITCODE -ne 0) { throw "package install failed ($LASTEXITCODE)" }

& $VenvPython -c "import torch, numpy, smart_open, boto3; print('torch', torch.__version__, '| threads', torch.get_num_threads(), '| cuda', torch.cuda.is_available())"
if ($LASTEXITCODE -ne 0) { throw "import check failed ($LASTEXITCODE)" }
Write-Host "Trainer Python ready: $VenvPython" -ForegroundColor Green
