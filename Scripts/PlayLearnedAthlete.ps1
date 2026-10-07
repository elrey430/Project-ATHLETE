# Project ATHLETE - play the learned athlete (Milestone 4's tracker, running in MuJoCo inside Unreal) on the lab map.
# Usage:  powershell -ExecutionPolicy Bypass -File Scripts\PlayLearnedAthlete.ps1
#
# Controls: W / left stick forward (Shift or right trigger: run), A/D sidestep, Q/E or right stick turn,
# R or Y reset. Needs the bundle (Saved\MuJoCo\unreal, from Scripts\MuJoCo\export_unreal_bundle.py) and the
# MuJoCo library set up for Unreal (Scripts\SetupMuJoCoUnreal.ps1, then a build). See Docs\LearnedAthleteInUnreal.md.

param(
    [int]$ResX = 1600,
    [int]$ResY = 900
)

. "$PSScriptRoot\AthleteEnv.ps1"

$Manifest = Join-Path $ProjectRoot 'Saved\MuJoCo\unreal\manifest.json'
if (-not (Test-Path $Manifest)) {
    throw "No learned-athlete bundle at $Manifest. Export one first (Docs\LearnedAthleteInUnreal.md)."
}

# The lab map with the learned athlete's game mode instead of the lab's. Quote: PowerShell 5.1 splits at '.'.
& $EditorExe $ProjectFile '/Game/Lab/Maps/AthleteLab?game=/Script/AthleteTracker.AthleteTrackerGameMode' `
    -game -windowed "-ResX=$ResX" "-ResY=$ResY" -nosplash "-log=LearnedAthlete.log"
