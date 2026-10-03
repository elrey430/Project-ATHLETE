// Project ATHLETE

#pragma once

#include "CoreMinimal.h"
#include "AthleteStrength.generated.h"

/** Joint actions whose strength is represented. Extended when a milestone needs more. */
UENUM(BlueprintType)
enum class EAthleteJointAction : uint8
{
	HipExtension,
	HipFlexion,
	KneeExtension,
	KneeFlexion,
	AnklePlantarFlexion,
	AnkleDorsiflexion,
	ShoulderAbduction,
	ShoulderAdduction,
	ElbowFlexion,
	ElbowExtension,
	// Added in Milestone 3 (appended so existing assets keep their values).
	WristFlexion,
	TrunkExtension,
	NeckExtension,

	Count UMETA(Hidden)
};

UENUM(BlueprintType)
enum class EAthleteStrengthTestMode : uint8
{
	/** Joint held still: maximum torque at zero velocity. */
	Isometric,
	/** Joint moving at a fixed angular velocity (see TestVelocityDegPerSec). */
	Isokinetic,
};

/**
 * Peak torque a joint action can produce, WITH the condition it was measured under.
 *
 * Muscles produce less torque the faster they shorten, so "peak torque" means nothing without
 * the test speed. Keeping the condition next to the number lets Milestones 3-4 convert
 * everything into one torque-angle-velocity model instead of silently mixing measurements.
 */
USTRUCT(BlueprintType)
struct ATHLETEBODY_API FAthleteJointActionStrength
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Strength", meta = (Units = "Nm", ClampMin = "0"))
	double PeakTorqueNm = 0.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Strength")
	EAthleteStrengthTestMode TestMode = EAthleteStrengthTestMode::Isometric;

	/** Joint angular velocity during an isokinetic test; 0 for isometric. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Strength", meta = (Units = "deg/s", ClampMin = "0"))
	double TestVelocityDegPerSec = 0.0;

	/**
	 * Fastest the joint can move in this direction with no load: the speed at which the muscles'
	 * torque drops to zero (force-velocity relation). 0 = unspecified (an estimate is used).
	 * Explosive athletes have higher values; it bounds acceleration and top speed.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Strength", meta = (Units = "deg/s", ClampMin = "0"))
	double MaxVelocityDegPerSec = 0.0;
};

/**
 * An athlete's measured (or estimated) joint strength.
 *
 * These are physical capabilities, not ratings: they will be inputs to the motor system's
 * torque limits. Missing entries mean "not specified". Nothing consumes this yet; the
 * locomotion and balance milestones (3-4) will, and will calibrate it against measurable
 * performance such as jump height and sprint acceleration.
 */
USTRUCT(BlueprintType)
struct ATHLETEBODY_API FAthleteStrengthProfile
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Strength")
	TMap<EAthleteJointAction, FAthleteJointActionStrength> Actions;

	const FAthleteJointActionStrength* Find(EAthleteJointAction Action) const { return Actions.Find(Action); }

	/**
	 * Population-baseline strength for a man of this age, height, and mass, from the regression
	 * equations of Harbo, Brincks & Andersen (2012), Eur J Appl Physiol 112:267-275, Tables 1-2
	 * (93 healthy Danish men aged 15-83, Biodex System 3 dynamometer).
	 *
	 * This is a GENERAL-POPULATION baseline: trained football players are considerably stronger.
	 * It gives every athlete sensible, sourced values that scale with body size, which authors
	 * then raise or lower per athlete. Test modes differ per action exactly as in the study.
	 */
	static FAthleteStrengthProfile MakeGeneralPopulationMaleBaseline(double AgeYears, double StatureM, double BodyMassKg);
};

namespace AthleteStrength
{
	ATHLETEBODY_API const TCHAR* GetActionName(EAthleteJointAction Action);

	/**
	 * ESTIMATE of an adult man's unloaded maximum joint angular velocity for an action (deg/s),
	 * used where a profile doesn't specify one. Order-of-magnitude values from peak joint speeds in
	 * unloaded movements (kicking, throwing, jumping: several hundred to ~1500 deg/s); to be
	 * calibrated against sprint and jump performance.
	 */
	ATHLETEBODY_API double GetEstimatedMaxVelocityDegPerSec(EAthleteJointAction Action);
}
