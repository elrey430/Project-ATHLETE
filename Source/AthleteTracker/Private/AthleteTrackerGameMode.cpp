// Project ATHLETE

#include "AthleteTrackerGameMode.h"

#include "AthleteTrackerPawn.h"

AAthleteTrackerGameMode::AAthleteTrackerGameMode()
{
	DefaultPawnClass = AAthleteTrackerPawn::StaticClass();
}
