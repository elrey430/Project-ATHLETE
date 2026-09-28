# Project ATHLETE - run AthleteLab headlessly for a fixed amount of simulated time and record telemetry.
# Usage:  powershell -ExecutionPolicy Bypass -File Scripts\RunLab.ps1 [-Seconds 6] [-Seed 1] [-Fps 60]
#
# -benchmark + -fps makes every frame exactly 1/Fps seconds long, so the same seed and inputs
# reproduce the same simulation. Telemetry lands in Saved\Telemetry\<timestamp>_AthleteLab\.

param(
    [double]$Seconds = 6,
    [long]$Seed = 1,
    [int]$Fps = 60
)

. "$PSScriptRoot\AthleteEnv.ps1"

# Quote every -arg=value: Windows PowerShell 5.1 splits unquoted arguments at '.'.
& $EditorCmd $ProjectFile '/Game/Lab/Maps/AthleteLab' -game -NullRHI -unattended -nosplash -benchmark `
    "-fps=$Fps" "-benchmarkseconds=$Seconds" "-AthleteSeed=$Seed" "-log=AthleteLab.log" | Out-Null
$ExitCode = $LASTEXITCODE

$Latest = Get-ChildItem (Join-Path $ProjectRoot 'Saved\Telemetry') -Directory -Filter '*_AthleteLab' |
    Sort-Object Name | Select-Object -Last 1
if ($Latest) { Write-Host "Telemetry: $($Latest.FullName)" }
exit $ExitCode
