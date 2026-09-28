# Project ATHLETE - shared paths for the helper scripts. Dot-source this file; don't run it directly.
# Override the engine location by setting the ATHLETE_UE_ROOT environment variable.

$ErrorActionPreference = 'Stop'

$UERoot = if ($env:ATHLETE_UE_ROOT) { $env:ATHLETE_UE_ROOT } else { 'C:\Program Files\Epic Games\UE_5.8' }
$ProjectRoot = Split-Path -Parent $PSScriptRoot
$ProjectFile = Join-Path $ProjectRoot 'ProjectAthlete.uproject'

$BuildBat  = Join-Path $UERoot 'Engine\Build\BatchFiles\Build.bat'
$EditorExe = Join-Path $UERoot 'Engine\Binaries\Win64\UnrealEditor.exe'
$EditorCmd = Join-Path $UERoot 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
$UBTExe    = Join-Path $UERoot 'Engine\Binaries\DotNET\UnrealBuildTool\UnrealBuildTool.exe'

foreach ($Required in @($BuildBat, $EditorCmd, $ProjectFile)) {
    if (-not (Test-Path $Required)) { throw "Not found: $Required" }
}
