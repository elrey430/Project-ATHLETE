# Project ATHLETE - builds the Python environment for the MuJoCo engine spike.
# Usage:  powershell -ExecutionPolicy Bypass -File Scripts\SetupMuJoCoPython.ps1
#
# A separate environment from the Learning Agents one (Intermediate\PipInstall), so the two can't
# break each other. Built from the engine's bundled Python 3.11, in Intermediate\MuJoCoPython.
# Installs MuJoCo (Apache-2.0, ~17.5 MB wheel, ~35 MB with dependencies). Safe to re-run.

. (Join-Path $PSScriptRoot 'AthleteEnv.ps1')

$MuJoCoVersion = '3.14.0'
$EnginePython = Join-Path $UERoot 'Engine\Binaries\ThirdParty\Python3\Win64\python.exe'
$VenvDir = Join-Path $ProjectRoot 'Intermediate\MuJoCoPython'
$VenvPython = Join-Path $VenvDir 'Scripts\python.exe'

if (-not (Test-Path $EnginePython)) { throw "Engine Python not found: $EnginePython" }

if (-not (Test-Path $VenvPython)) {
    Write-Host "Creating the MuJoCo Python environment in $VenvDir"
    & $EnginePython -m venv $VenvDir
    if ($LASTEXITCODE -ne 0) { throw "venv creation failed ($LASTEXITCODE)" }
}

& $VenvPython -m pip install --disable-pip-version-check "mujoco==$MuJoCoVersion"
if ($LASTEXITCODE -ne 0) { throw "mujoco install failed ($LASTEXITCODE)" }

& $VenvPython -c "import mujoco, numpy; print('mujoco', mujoco.__version__, '| numpy', numpy.__version__)"
if ($LASTEXITCODE -ne 0) { throw "import check failed ($LASTEXITCODE)" }
Write-Host "MuJoCo Python ready: $VenvPython" -ForegroundColor Green
