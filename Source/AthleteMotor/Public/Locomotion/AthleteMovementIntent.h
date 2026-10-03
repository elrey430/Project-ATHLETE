// Project ATHLETE

#pragma once

#include "CoreMinimal.h"
#include "AthleteMovementIntent.generated.h"

/**
 * What the athlete WANTS to do with his body right now: the only input the motor system takes for
 * moving around. It comes from a player's controller today and from the athlete's own decision
 * making later (pipeline: ... -> Intent -> Motor Planning -> Physical Athlete Controller -> Body).
 *
 * An intent is a wish, not a command to the world: nothing here moves the body. Whether and how
 * quickly the athlete reaches this velocity depends on his body: where his feet are, his balance,
 * his strength, his speed.
 */
USTRUCT(BlueprintType)
struct ATHLETEMOTOR_API FAthleteMovementIntent
{
	GENERATED_BODY()

	/** Desired horizontal velocity over the ground (world X/Y, m/s). Zero = stand still. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Intent", meta = (Units = "m/s"))
	FVector2D DesiredVelocityMps = FVector2D::ZeroVector;

	/** Face the direction of travel (true), or DesiredFacingYawDeg (false; e.g. backpedaling). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Intent")
	bool bFaceMovementDirection = true;

	/** World yaw the athlete wants to face when not facing the direction of travel (degrees). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Intent", meta = (Units = "deg"))
	double DesiredFacingYawDeg = 0.0;

	/**
	 * Keep stepping even with no velocity wished for: stepping in place ("chopping the feet"),
	 * staying ready to move instead of settling into a stance.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Intent")
	bool bKeepStepping = false;

	/** Below this desired speed the athlete wants to stand. */
	static constexpr double StillSpeedMps = 0.05;
	bool WantsToMove() const { return DesiredVelocityMps.Size() > StillSpeedMps; }
	bool WantsToStep() const { return bKeepStepping || WantsToMove(); }
};
