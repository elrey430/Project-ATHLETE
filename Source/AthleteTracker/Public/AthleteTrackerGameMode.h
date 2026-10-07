// Project ATHLETE

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "AthleteTrackerGameMode.generated.h"

/**
 * Plays the learned athlete: the player controls an AAthleteTrackerPawn. Use it on the lab map:
 *   UnrealEditor ProjectAthlete.uproject /Game/Lab/Maps/AthleteLab?game=/Script/AthleteTracker.AthleteTrackerGameMode -game
 * (Scripts/PlayLearnedAthlete.ps1), or pick it as the GameMode Override in the level's World Settings.
 */
UCLASS()
class ATHLETETRACKER_API AAthleteTrackerGameMode : public AGameModeBase
{
	GENERATED_BODY()

public:
	AAthleteTrackerGameMode();
};
