# Project ATHLETE - builds the local Python environment for video motion capture (CPU).
# Usage:  powershell -ExecutionPolicy Bypass -File Scripts\SetupCapturePython.ps1
#
# MediaPipe Pose (Google, Apache 2.0): 33 body landmarks per frame, heel and toe included, in the image and
# as rough 3D. Used on our own recorded video (phones); Scripts\Capture\pose_video.py runs it. Separate from
# the training environment (Intermediate\LocoMuJoCoPython): MediaPipe brings its own jax and OpenCV.
# Environment in Intermediate\CapturePython, model in Saved\Capture\models. ~0.3 GB. CPU only (no CUDA here).

. (Join-Path $PSScriptRoot 'AthleteEnv.ps1')
$ErrorActionPreference = 'Continue' # pip reports progress on stderr; exit codes are checked

$EnginePython = Join-Path $UERoot 'Engine\Binaries\ThirdParty\Python3\Win64\python.exe'
$VenvDir = Join-Path $ProjectRoot 'Intermediate\CapturePython'
$VenvPython = Join-Path $VenvDir 'Scripts\python.exe'
$ModelDir = Join-Path $ProjectRoot 'Saved\Capture\models'
$Model = Join-Path $ModelDir 'pose_landmarker_heavy.task'
$ModelUrl = 'https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_heavy/float16/latest/pose_landmarker_heavy.task'

if (-not (Test-Path $VenvPython)) {
    Write-Host "Creating $VenvDir"
    & $EnginePython -m venv $VenvDir
    if ($LASTEXITCODE -ne 0) { throw "venv creation failed ($LASTEXITCODE)" }
}
& $VenvPython -m pip install -q --disable-pip-version-check --upgrade pip
& $VenvPython -m pip install -q --disable-pip-version-check "mediapipe"
if ($LASTEXITCODE -ne 0) { throw "pip install failed ($LASTEXITCODE)" }

if (-not (Test-Path $Model)) {
    New-Item -ItemType Directory -Force $ModelDir | Out-Null
    Write-Host "Downloading $ModelUrl"
    Invoke-WebRequest -Uri $ModelUrl -OutFile $Model -UseBasicParsing
}

& $VenvPython -c "import mediapipe, cv2; print('mediapipe', mediapipe.__version__, '| opencv', cv2.__version__)"
if ($LASTEXITCODE -ne 0) { throw "import check failed ($LASTEXITCODE)" }

# Desktop shortcut to the app (Scripts\Capture\capture_app.py), started without a console window.
$Shortcut = (New-Object -ComObject WScript.Shell).CreateShortcut((Join-Path ([Environment]::GetFolderPath('Desktop')) 'ATHLETE Capture.lnk'))
$Shortcut.TargetPath = Join-Path $VenvDir 'Scripts\pythonw.exe'
$Shortcut.Arguments = '"' + (Join-Path $ProjectRoot 'Scripts\Capture\capture_app.py') + '"'
$Shortcut.WorkingDirectory = $ProjectRoot
$Shortcut.Description = 'Project ATHLETE: body landmarks from recorded video (MediaPipe Pose)'
$Shortcut.Save()
Write-Host "Desktop shortcut: ATHLETE Capture"
Write-Host "Capture Python ready: $VenvPython (model $Model)" -ForegroundColor Green
