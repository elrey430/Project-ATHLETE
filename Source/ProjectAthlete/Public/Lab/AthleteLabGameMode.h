// Project ATHLETE

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "AthleteLabGameMode.generated.h"

/**
 * Game mode for the AthleteLab test level.
 *
 * A Game Mode defines the rules of a level: which pawn the player gets, which controller, HUD, etc.
 * The lab has no gameplay, so the player receives a free-flying spectator camera
 * (WASD + mouse, E/Q up/down) to observe experiments.
 *
 * Football rules will NOT live here. They will be a separate system (see project principle 21).
 */
UCLASS()
class PROJECTATHLETE_API AAthleteLabGameMode : public AGameModeBase
{
	GENERATED_BODY()

public:
	AAthleteLabGameMode();
};
