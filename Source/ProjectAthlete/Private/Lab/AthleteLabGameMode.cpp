// Project ATHLETE

#include "Lab/AthleteLabGameMode.h"
#include "GameFramework/SpectatorPawn.h"

AAthleteLabGameMode::AAthleteLabGameMode()
{
	DefaultPawnClass = ASpectatorPawn::StaticClass();
}
