// Project ATHLETE

#pragma once

#include "CoreMinimal.h"
#include "Anatomy/AthleteJoints.h"
#include "Capability/AthleteStrength.h"

class FAthleteBodyModel;
struct FAthleteMotorSkill;

/**
 * The muscles crossing one joint, modeled as a torque-limited spring-damper about a target
 * orientation, applied as explicit torques every physics substep. This is the lowest-complexity model that keeps what matters for
 * football:
 *  - STIFFNESS resists being displaced, instantly (muscle short-range stiffness / tone),
 *  - DAMPING resists being moved quickly,
 *  - a TORQUE LIMIT set by the athlete's measured strength: a load bigger than that wins.
 *
 * Real muscles have force-length and force-velocity curves, activation dynamics, and different
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
	double TorqueLimitNm = 0.0;

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
	 *   limit     = peak torque of the joint's anti-gravity action (e.g. knee: extension)
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
}
