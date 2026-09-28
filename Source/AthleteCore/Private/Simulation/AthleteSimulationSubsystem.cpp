// Project ATHLETE

#include "Simulation/AthleteSimulationSubsystem.h"
#include "AthleteCore.h"
#include "Simulation/AthleteSimulationSettings.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

void UAthleteSimulationSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);

	MasterSeed = UAthleteSimulationSettings::Get()->DefaultSimulationSeed;

	int64 CommandLineSeed = 0;
	if (FParse::Value(FCommandLine::Get(), TEXT("AthleteSeed="), CommandLineSeed))
	{
		MasterSeed = CommandLineSeed;
	}

	UE_LOG(LogAthlete, Log, TEXT("ATHLETE simulation initialized for world '%s' with master seed %lld."),
		*GetWorld()->GetName(), MasterSeed);
}

FAthleteRandomStream UAthleteSimulationSubsystem::MakeRandomStream(FStringView ConsumerName) const
{
	return FAthleteRandomStream(static_cast<uint64>(MasterSeed)).MakeSubstream(ConsumerName);
}

bool UAthleteSimulationSubsystem::DoesSupportWorldType(const EWorldType::Type WorldType) const
{
	// Only worlds that actually simulate: packaged/standalone game worlds and Play-In-Editor.
	// The editor's own viewport world is excluded.
	return WorldType == EWorldType::Game || WorldType == EWorldType::PIE;
}
