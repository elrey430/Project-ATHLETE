# Project ATHLETE - generate lab content headlessly:
#   1. Scripts\Editor\create_sample_athletes.py  (creates/updates athlete Data Assets)
#   2. Scripts\Editor\build_athlete_lab.py       (creates the AthleteLab level)
# Usage:  powershell -ExecutionPolicy Bypass -File Scripts\BuildLab.ps1 [-Rebuild]
#   -Rebuild  deletes the existing AthleteLab level first. The level is a generated artifact:
#             hand edits made in the editor are lost; put permanent changes in the script.
# Requires a successful build first (Scripts\Build.ps1). Close the editor before running.

param(
    [switch]$Rebuild
)

. "$PSScriptRoot\AthleteEnv.ps1"

$MapFile = Join-Path $ProjectRoot 'Content\Lab\Maps\AthleteLab.umap'
if ($Rebuild -and (Test-Path $MapFile)) {
    Remove-Item $MapFile
    Write-Host "Deleted $MapFile"
}

foreach ($Name in @('create_sample_athletes.py', 'build_athlete_lab.py')) {
    $Script = Join-Path $PSScriptRoot "Editor\$Name"
    Write-Host "Running $Name ..."
    & $EditorCmd $ProjectFile -run=pythonscript "-script=$Script" -unattended -nopause -nosplash -stdout -FullStdOutLogOutput |
        Select-String -Pattern '\[(CreateSampleAthletes|BuildAthleteLab)\]|LogPython: Error|Traceback' | ForEach-Object { $_.Line }
    if ($LASTEXITCODE -ne 0) {
        Write-Host "$Name failed (exit $LASTEXITCODE). See Saved\Logs\ProjectAthlete.log" -ForegroundColor Red
        exit $LASTEXITCODE
    }
}
exit 0
