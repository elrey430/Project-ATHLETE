// Project ATHLETE

#pragma once

#include "CoreMinimal.h"
#include "Engine/DeveloperSettings.h"
#include "AthleteSimulationSettings.generated.h"

/**
 * Project-wide ATHLETE simulation settings.
 *
 * Editable in the editor at: Edit > Project Settings > Game > ATHLETE Simulation.
 * Saved to Config/DefaultGame.ini under [/Script/AthleteCore.AthleteSimulationSettings],
 * so every value here is version-controlled and never a hidden magic number.
 */
UCLASS(Config = Game, DefaultConfig, meta = (DisplayName = "ATHLETE Simulation"))
class ATHLETECORE_API UAthleteSimulationSettings : public UDeveloperSettings
{
	GENERATED_BODY()

public:
	/** Convenience accessor for the settings' Class Default Object (the single instance holding config values). */
	static const UAthleteSimulationSettings* Get() { return GetDefault<UAthleteSimulationSettings>(); }

	/**
	 * Master seed for all deterministic randomness in a world.
	 * Override from the command line with -AthleteSeed=<number> to reproduce a recorded run.
	 */
	UPROPERTY(Config, EditAnywhere, Category = "Reproducibility")
	int64 DefaultSimulationSeed = 1;

	/** When false, telemetry tables are not created and nothing is written to disk. */
	UPROPERTY(Config, EditAnywhere, Category = "Telemetry")
	bool bTelemetryEnabled = true;

	/** Folder under the project's Saved/ directory that receives telemetry sessions. */
	UPROPERTY(Config, EditAnywhere, Category = "Telemetry")
	FString TelemetryFolderName = TEXT("Telemetry");
};
