// Project ATHLETE

#pragma once

#include "CoreMinimal.h"
#include "Chaos/SimCallbackInput.h"
#include "Chaos/SimCallbackObject.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"

/** One joint's muscle command, pushed from the game thread once per frame. */
struct FAthleteMuscleCommand
{
	Chaos::FSingleParticlePhysicsProxy* Child = nullptr;
	Chaos::FSingleParticlePhysicsProxy* Parent = nullptr;

	/** Joint frames in each body's local space (Unreal constraint Frame1 = child, Frame2 = parent). */
	FQuat ChildFrameLocal = FQuat::Identity;
	FQuat ParentFrameLocal = FQuat::Identity;
	/**
	 * The joint's anatomical axes in the parent body's local space (X long axis/twist, Y flexion axis,
	 * Z frontal axis), which the per-axis gains refer to. Not the parent constraint frame: that one
	 * is turned by the range-of-motion neutral.
	 */
	FQuat AxisFrameLocal = FQuat::Identity;

	/** Target orientation of the child frame relative to the parent frame. */
	FQuat Target = FQuat::Identity;
	/**
	 * How fast the target itself moves: the planned relative spin, in the parent joint frame
	 * (Unreal rad/s). Zero when holding a posture. In a planned movement the nervous system commands
	 * the motion, not only its end point (equilibrium-point control), so the reflex damping resists
	 * deviation FROM the planned motion rather than the motion itself.
	 */
	FVector TargetSpin = FVector::ZeroVector;
	/**
	 * Index of another joint on the same parent whose reaction this joint takes up (adds the opposite
	 * of its torque, within this joint's own strength), or INDEX_NONE. The stance hip does this for the
	 * swing hip so swinging a leg doesn't tip the pelvis.
	 */
	int32 CancelReactionOf = INDEX_NONE;
	/**
	 * Torque the muscle produces by activation alone, independent of stretch (world frame, Unreal
	 * units, on the child). Added to the reflex spring and damper, then capped with them at the
	 * joint's strength. Not a stiffness, so the substep stiffness cap does not apply to it.
	 */
	FVector FeedforwardTorque = FVector::ZeroVector;

	/** Per joint-frame axis (X twist, Y sagittal, Z frontal). Unreal units: torque kg*cm^2/s^2 (1 N*m = 1e4). */
	FVector StiffnessPerRad = FVector::ZeroVector;
	FVector DampingPerRadPerS = FVector::ZeroVector;
	/** Isometric strength (Unreal torque units) and the joint speed at which it falls to zero (rad/s; 0 = no force-velocity effect). */
	double TorqueLimit = 0.0;
	double MaxVelocityRadPerS = 0.0;
};

struct FAthleteMuscleInput : public Chaos::FSimCallbackInput
{
	TArray<FAthleteMuscleCommand> Joints;
	void Reset() { Joints.Reset(); }
};

/** What the muscles did during one substep, per joint. */
struct FAthleteMuscleOutput : public Chaos::FSimCallbackOutput
{
	/** Torque applied (world frame, N*m, acting on the child; the parent got the opposite). */
	TArray<FVector> JointTorquesNm;
	/** Rotation still needed to reach the target, along the joint's anatomical axes (X twist, Y sagittal, Z frontal; radians). */
	TArray<FVector> JointErrorsRad;
	/** Wall-clock time this substep's muscle computation took (seconds; performance telemetry). */
	double ComputeSeconds = 0.0;
	void Reset() { JointTorquesNm.Reset(); JointErrorsRad.Reset(); ComputeSeconds = 0.0; }
};

/**
 * Runs the athlete's muscles inside the physics solver, once per SUBSTEP.
 *
 * For each joint: torque = stiffness * (rotation to target) - damping * (relative spin - planned spin),
 * capped at the joint's strength, applied +torque to the child segment and -torque to the parent:
 * an internal torque, exactly like a muscle crossing the joint. Nothing external is applied.
 *
 * Runs on the physics thread: it may only touch physics-thread particle data.
 */
class FAthleteMuscleSimCallback : public Chaos::TSimCallbackObject<FAthleteMuscleInput, FAthleteMuscleOutput>
{
protected:
	virtual void OnPreSimulate_Internal() override;
};
