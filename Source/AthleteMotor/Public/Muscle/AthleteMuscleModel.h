// Project ATHLETE

#pragma once

#include "CoreMinimal.h"
#include "Anatomy/AthleteJoints.h"
#include "Capability/AthleteStrength.h"

class FAthleteBodyModel;
struct FAthleteMotorSkill;

/**
 * The muscles crossing one joint, modeled as a torque-limited spring-damper about a target
 * orientation, applied as explicit torques every physics substep. The lowest-complexity model
 * that keeps what matters for football:
 *  - STIFFNESS resists being displaced, instantly (muscle short-range stiffness / tone),
 *  - DAMPING resists being moved quickly,
 *  - a TORQUE LIMIT set by the athlete's measured strength: a load bigger than that wins,
 *  - FORCE-VELOCITY (Hill 1938): the limit falls as the muscle shortens faster (zero at the
 *    joint's maximum speed) and rises up to 1.5x when it is stretched. This bounds how fast a
 *    body can accelerate and move, and gives a muscle at its limit resistance to being stretched.
 *
 * Real muscles also have force-length curves, activation dynamics, and different
 * strengths per direction. Those are simplified away here (see Docs/Milestone3_StandingBalance.md).
 */
struct FAthleteJointImpedance
{
	EAthleteJoint Joint = EAthleteJoint::Count;

	/** Gravitational load this joint supports in standing: supported mass * g * lever (N*m/rad). */
	double GravityStiffnessNmPerRad = 0.0;

	/** Inertia of the supported mass about the joint (kg*m^2). */
	double EffectiveInertiaKgM2 = 0.0;

	/** Main (sagittal) stiffness and damping: the axis that carries the standing load. */
	double StiffnessNmPerRad = 0.0;
	double DampingNmsPerRad = 0.0;
	/** ISOMETRIC strength (zero joint speed); see ForceVelocityFactor for how it changes with speed. */
	double TorqueLimitNm = 0.0;
	/** Joint speed at which the muscles can no longer produce torque while shortening (rad/s). */
	double MaxVelocityRadPerS = 0.0;

	/**
	 * Per axis of the joint frame (X twist, Y sagittal, Z frontal): different muscle groups act
	 * about different axes (hip flexors/extensors vs abductors/adductors) and carry different loads.
	 */
	FVector AxisStiffnessNmPerRad = FVector::ZeroVector;
	FVector AxisDampingNmsPerRad = FVector::ZeroVector;

	/** Which strength measurement limits this joint's torque. */
	EAthleteJointAction LimitingAction = EAthleteJointAction::Count;
};

namespace AthleteMuscleModel
{
	/**
	 * Impedance of one joint for upright standing.
	 *   stiffness = MuscleToneGain * (m_supported * g * lever)
	 *   damping   = 2 * DampingRatio * sqrt(stiffness * I_supported)
	 *   limit     = ISOMETRIC peak torque of the joint's anti-gravity action (e.g. knee: extension);
	 *               a strength measured while moving (isokinetic) is converted with the Hill curve
	 * Leg joints support the body above them, shared between both legs.
	 *
	 * All three axes get that stiffness, except the hips' frontal axis. With both feet down, legs,
	 * pelvis, and ground form a closed loop, and the whole body swaying sideways turns both hips:
	 * the hip abductors/adductors carry the whole body's sideways sway (shared by both hips), not
	 * just the trunk above them.
	 */
	ATHLETEMOTOR_API FAthleteJointImpedance ComputeStandingImpedance(const FAthleteBodyModel& Model, EAthleteJoint Joint,
		const FAthleteStrengthProfile& Strength, const FAthleteMotorSkill& Skill, double GravityMps2);

	/** Which strength measurement bounds this joint's torque. */
	ATHLETEMOTOR_API EAthleteJointAction GetLimitingAction(EAthleteJoint Joint);

	/** Hill curvature a/F0: how quickly torque falls with shortening speed (Hill 1938: 0.15-0.4). */
	inline constexpr double HillCurvature = 0.25;

	/** Torque available when lengthening fast, as a multiple of isometric (eccentric plateau). */
	inline constexpr double EccentricPlateau = 1.5;

	/**
	 * Force-velocity factor: available torque / isometric torque, for a joint moving at
	 * ShorteningFraction = (speed in the direction the muscle pushes) / MaxVelocity.
	 *   shortening (x > 0): (1 - x) / (1 + x / HillCurvature), 0 at x >= 1
	 *   lengthening (y = -x > 0): rises from 1 toward EccentricPlateau, reached at y = 1,
	 *   about 2.5x more steeply than the shortening side near zero (Katz 1939: lengthening is steeper).
	 */
	ATHLETEMOTOR_API double ForceVelocityFactor(double ShorteningFraction);
}
