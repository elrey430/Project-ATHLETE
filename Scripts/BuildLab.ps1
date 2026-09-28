# Project ATHLETE - generate the AthleteLab level headlessly from Scripts\Editor\build_athlete_lab.py.
# Requires a successful build first (Scripts\Build.ps1). Close the editor before running.

. "$PSScriptRoot\AthleteEnv.ps1"

$Script = Join-Path $PSScriptRoot 'Editor\build_athlete_lab.py'
& $EditorCmd $ProjectFile -run=pythonscript "-script=$Script" -unattended -nopause -nosplash -stdout -FullStdOutLogOutput
exit $LASTEXITCODE
