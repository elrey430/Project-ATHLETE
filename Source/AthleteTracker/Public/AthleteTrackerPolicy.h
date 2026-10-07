// Project ATHLETE

#pragma once

#include "CoreMinimal.h"

class FAthleteTrackerBundle;

/**
 * The tracking policy (LocoMuJoCo PPO actor): the observation normalized by the training's running mean and
 * variance (frozen), two tanh layers (512, 256), and a linear output: one action in about [-1, 1] per motor.
 * Deterministic: the mean action, as in the acceptance tests.
 */
class ATHLETETRACKER_API FAthleteTrackerPolicy
{
public:
	bool Initialize(const FAthleteTrackerBundle& Bundle, FString& OutError);

	/** Observation (ObservationDim) -> action (ActionDim). */
	void Act(TConstArrayView<double> Observation, TArrayView<double> OutAction) const;

	int32 ObservationDim() const { return Mean.Num(); }
	int32 ActionDim() const { return B2.Num(); }

private:
	TArray<float> Mean, InvStd, W0, B0, W1, B1, W2, B2;
	mutable TArray<float> Input, Hidden0, Hidden1;
};
