// Project ATHLETE

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "Random/AthleteRandom.h"
#include "AthleteSimulationSubsystem.generated.h"

/**
 * Per-world owner of simulation-wide state that must be reproducible.
 *
 * Today it only owns the master random seed. Later it is the natural home for the
 * simulation clock and fixed-step bookkeeping.
 *
 * A World Subsystem is created automatically by Unreal once per UWorld (the editor world,
 * each PIE session, the game world, test worlds) and destroyed with it. No placement needed.
 */
UCLASS()
class ATHLETECORE_API UAthleteSimulationSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()

public:
	virtual void Initialize(FSubsystemCollectionBase& Collection) override;

	/** The seed every random stream in this world is derived from. */
	UFUNCTION(BlueprintPure, Category = "ATHLETE|Simulation")
	int64 GetMasterSeed() const { return MasterSeed; }

	/**
	 * Creates the random stream owned by one named consumer, e.g. TEXT("Athlete3.MotorNoise").
	 * The same name and master seed always produce the same sequence.
	 */
	FAthleteRandomStream MakeRandomStream(FStringView ConsumerName) const;

protected:
	virtual bool DoesSupportWorldType(const EWorldType::Type WorldType) const override;

private:
	int64 MasterSeed = 0;
};
