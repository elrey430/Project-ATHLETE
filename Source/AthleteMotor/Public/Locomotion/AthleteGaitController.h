// Project ATHLETE

#pragma once

#include "CoreMinimal.h"
#include "Balance/AthleteBalanceController.h"
#include "Locomotion/AthleteMovementIntent.h"

class FAthleteBodyModel;
struct FAthleteMotorSkill;

/** Leg dimensions the gait needs, from the athlete's own body model (meters). */
struct FAthleteLegGeometry
{
	double ThighLengthM = 0.0;     // hip center to knee center
	double ShankLengthM = 0.0;     // knee center to ankle center
	double AnkleHeightM = 0.0;     // ankle center above the sole when standing
	double AnkleToFootCenterM = 0.0; // how far the foot's center is ahead of the ankle
	double StanceHalfWidthM = 0.0; // half the distance between the feet when standing

	/**
	 * For holding the swinging leg up against gravity: segment masses, and each segment's center of
	 * mass relative to its proximal joint in the reference pose (body frame), per leg [left, right].
	 */
	double ThighMassKg = 0.0;
	double ShankMassKg = 0.0;
	double FootMassKg = 0.0;
	FVector ThighComFromHipM[2] = { FVector::ZeroVector, FVector::ZeroVector };
	FVector ShankComFromKneeM[2] = { FVector::ZeroVector, FVector::ZeroVector };
	FVector FootComFromAnkleM[2] = { FVector::ZeroVector, FVector::ZeroVector };
	/** Principal moments of inertia about each segment's center of mass, in its reference-pose axes (kg*m^2). */
	FVector ThighInertiaKgM2 = FVector::ZeroVector;
	FVector ShankInertiaKgM2 = FVector::ZeroVector;
	FVector FootInertiaKgM2 = FVector::ZeroVector;

	static FAthleteLegGeometry FromModel(const FAthleteBodyModel& Model, double FootCenterXM, double FootHalfHeightM);
};

/** What the gait is doing this frame (for telemetry and debug drawing). */
struct FAthleteGaitState
{
	bool bStepping = false;
	/** Shifting weight onto the stance foot before the first step (anticipatory postural adjustment). */
	bool bShifting = false;
	/** 0 = left, 1 = right. */
	int32 SwingLeg = 1;
	int32 StepCount = 0;
	double PhaseFraction = 0.0;
	/** Where the swing foot will be put down (ground point under the foot's center, world, m). */
	FVector FootTargetM = FVector::ZeroVector;
	/** Where the swing ankle is being steered this frame (world, m). */
	FVector SwingAnkleTargetM = FVector::ZeroVector;
	/** Extrapolated center of mass (capture point) on the ground (world, m). */
	FVector CapturePointM = FVector::ZeroVector;
	/** Where the capture point should be now on the planned walk (world ground point, m). */
	FVector PlannedCapturePointM = FVector::ZeroVector;
	/** Center of pressure the stance ankle aims for, relative to the stance ankle (X ahead, Y to the right, heading frame, m). */
	FVector2D PressureOffsetM = FVector2D::ZeroVector;
	/** The velocity the gait is currently aiming for (after limiting how fast the wish can change). */
	FVector2D PlannedVelocityMps = FVector2D::ZeroVector;
	/** World yaw the athlete is turning to face (degrees). */
	double HeadingYawDeg = 0.0;
};

/**
 * Walking, as a sequence of controlled falls (Milestone 4).
 *
 * The body is an inverted pendulum over the stance foot. Where the swing foot lands decides how the
 * next step goes, following the linear inverted pendulum (LIPM) and its capture point
 * xi = CoM + v / omega0, the same extrapolated center of mass that governs standing balance:
 *  - to walk on at velocity v with step time T, land the foot  L / (exp(omega0 T) - 1)  BEHIND the
 *    capture point (L = v T, the step length): the capture point then grows to exactly one step on;
 *  - to stop, land ON the capture point; to speed up or slow down, land it shorter or longer;
 *  - sideways, land a little outside the capture point, so the body sways back over the other foot.
 * Nothing pushes the body: every change of speed comes from where the feet go and what the legs do.
 *
 * Legs:
 *  - STANCE leg: the knee holds almost straight (a strut); the hip keeps the pelvis upright and
 *    facing the heading (turning happens here, over the planted foot) and takes up the swing hip's
 *    reaction; the ankle moves the center of pressure along the foot (feedforward torque, within
 *    what the foot can take) to keep the capture point on its planned path.
 *  - SWING leg: the ankle follows a trajectory (lift, carry, set down) to the target; two-link leg
 *    geometry gives the thigh and shank directions, as joint targets from a level pelvis. The joints
 *    are commanded the plan's velocity too, and an internal model (inverse dynamics of the planned
 *    motion, foot to hip) supplies the torque it needs against gravity and inertia. Walking, the
 *    trailing foot rolls onto its toes and pushes off.
 * The spine, neck and arms follow the standing posture rules.
 *
 * Measured limits (Milestone 4 doc): steps in place and walks slowly (~0.3 m/s); faster walking
 * falls within a few steps, each landing a little short of where it was aimed.
 *
 * Plain C++ (no engine objects), driven once per frame by UAthleteMotorComponent.
 */
class ATHLETEMOTOR_API FAthleteGaitController
{
public:
	void Initialize(const FAthleteLegGeometry& InLegs, double InGroundHeightM, double InBodyFrameYawDeg, double InitialHeadingYawDeg);

	/**
	 * Advance the gait one frame. Returns false while standing (the caller runs standing balance),
	 * true while stepping (OutPosture holds the joint targets).
	 *   ForPlanning: sensing at the balance reaction delay (where to step).
	 *   ForPosture:  sensing at the postural reflex delay (holding segments).
	 *   Now:         current sensing (foot touch: the fastest reflex).
	 */
	bool Update(const FAthleteMovementIntent& Intent, const FAthleteBalanceSensing& ForPlanning, const FAthleteBalanceSensing& ForPosture,
		const FAthleteBalanceSensing& Now, const FAthleteMotorSkill& Skill, const FAthleteFootSupport& Feet, double GravityMps2, double DeltaTimeS, FAthletePosture& OutPosture);

	const FAthleteGaitState& GetState() const { return State; }

	/** The heading (as a body-frame deviation, pure yaw) the athlete currently holds. */
	FQuat GetHeadingDeviation() const;

private:
	void StartStep(int32 SwingLeg, const FAthleteBalanceSensing& Now);
	FQuat WorldYaw(double YawDeg) const;

	FAthleteLegGeometry Legs;
	double GroundHeightM = 0.0;
	double BodyFrameYawDeg = 0.0;
	FAthleteGaitState State;
	double StepStartTimeS = 0.0;
	double ShiftStartTimeS = 0.0;
	double PlantedSinceS = -1.0; // when the landing foot came down and has stayed down since
	FVector LiftoffAnkleM = FVector::ZeroVector;
	/** Where the stance foot's ankle was when it touched down (world, m): a planted foot's place is known at the touch, not a reaction time later. */
	FVector StanceAnkleM = FVector::ZeroVector;
};
