# Project ATHLETE - (re)generate the Visual Studio solution (ProjectAthlete.sln).
# Run after adding, removing, or renaming C++ files or modules.
# Equivalent to right-clicking ProjectAthlete.uproject > "Generate Visual Studio project files".

. "$PSScriptRoot\AthleteEnv.ps1"

& $UBTExe -projectfiles "-project=$ProjectFile" -game -rocket -progress
exit $LASTEXITCODE
