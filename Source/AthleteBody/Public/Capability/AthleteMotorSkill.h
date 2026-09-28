// Project ATHLETE

#pragma once

#include "CoreMinimal.h"
#include "AthleteMotorSkill.generated.h"

/**
 * How well an athlete controls his body: neuromuscular control parameters, not ratings.
 *
 * Every value here changes HOW the athlete's own muscles are commanded. None of them adds force,
 * speed, or stability from outside the body. Two athletes with identical bodies and strength can
 * still balance differently because of these values, which is the point: coordination is real.
 */
USTRUCT(BlueprintType)
struct ATHLETEBODY_API FAthleteMotorSkill
{
	GENERATED_BODY()

	/**
	 * Muscle tone: joint stiffness as a multiple of the gravitational load that joint supports
	 * (m * g * d). Below 1 a joint cannot hold its load by stiffness alone; above 1 it can.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor Skill", meta = (ClampMin = "0.1", ClampMax = "10"))
	double MuscleToneGain = 2.0;

	/** Joint damping as a fraction of critical damping (1 = critically damped, no overshoot). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor Skill", meta = (ClampMin = "0.05", ClampMax = "3"))
	double DampingRatio = 1.0;

	/**
	 * Ankle strategy: how strongly body lean is corrected toward the base of support.
	 *
	 * Muscle tone already makes the ankles stiff enough to hold the body up; what the neural loop
	 * mainly adds is DAMPING of sway (human posturography: about 0.3 m*g*h*s of neural damping,
	 * with the stiffness from tone and a smaller neural share; Peterka 2002). With tone 2 the
	 * defaults here (0.25 with VelocityFeedbackGain 2) give about 0.3 m*g*h*s of damping and add
	 * about 0.45 m*g*h of stiffness. Measured (Milestone 3 study): quiet-stance sway about 1 mm;
	 * gain 1.0 with the 0.1 s reaction delay is unstable.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor Skill|Balance", meta = (ClampMin = "0", ClampMax = "5"))
	double AnkleStrategyGain = 0.25;

	/**
	 * Weight of velocity in the balance feedback, relative to the extrapolated center of mass
	 * (1 = pure XCoM: velocity / omega0). Higher = the athlete reacts more to how fast he is
	 * swaying than to where he is, which damps sway.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor Skill|Balance", meta = (ClampMin = "0", ClampMax = "5"))
	double VelocityFeedbackGain = 2.0;

	/** Hip strategy: how strongly the hips flex/extend once the ankles alone cannot recover. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor Skill|Balance", meta = (ClampMin = "0", ClampMax = "5"))
	double HipStrategyGain = 1.0;

	/**
	 * Delay between the body's state and the balance controller acting on it (sensing + neural
	 * transmission). Human postural responses begin roughly 0.1 s after a perturbation.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor Skill|Balance", meta = (Units = "s", ClampMin = "0", ClampMax = "0.3"))
	double ReactionTimeS = 0.1;

	/**
	 * Delay of the postural reflexes that keep each body segment oriented in space (stretch and
	 * vestibulospinal reflexes, roughly 40-60 ms in people): faster than a balance decision.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor Skill|Balance", meta = (Units = "s", ClampMin = "0", ClampMax = "0.3"))
	double PostureReflexDelayS = 0.05;
};
