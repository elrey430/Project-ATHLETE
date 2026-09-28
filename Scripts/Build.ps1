# Project ATHLETE - compile the editor build (the build you develop and test in).
# Usage:  powershell -ExecutionPolicy Bypass -File Scripts\Build.ps1
# Close the Unreal Editor first, or use Live Coding (Ctrl+Alt+F11) inside the editor instead.

. "$PSScriptRoot\AthleteEnv.ps1"

& $BuildBat ProjectAthleteEditor Win64 Development "-Project=$ProjectFile" -WaitMutex
exit $LASTEXITCODE
