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

	/** Target orientation of the child frame relative to the parent frame. */
	FQuat Target = FQuat::Identity;

	/** Per joint-frame axis (X twist, Y sagittal, Z frontal). Unreal units: torque kg*cm^2/s^2 (1 N*m = 1e4). */
	FVector StiffnessPerRad = FVector::ZeroVector;
	FVector DampingPerRadPerS = FVector::ZeroVector;
	double TorqueLimit = 0.0;
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
	/** Rotation still needed to reach the target, in the parent joint frame (X twist, Y sagittal, Z frontal; radians). */
	TArray<FVector> JointErrorsRad;
	void Reset() { JointTorquesNm.Reset(); JointErrorsRad.Reset(); }
};

/**
 * Runs the athlete's muscles inside the physics solver, once per SUBSTEP.
 *
 * For each joint: torque = stiffness * (rotation to target) - damping * (relative angular velocity),
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
