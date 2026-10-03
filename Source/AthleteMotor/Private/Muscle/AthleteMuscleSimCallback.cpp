// Project ATHLETE

#include "Muscle/AthleteMuscleSimCallback.h"
#include "Muscle/AthleteMuscleModel.h"
#include "Chaos/Utilities.h"
#include "HAL/IConsoleManager.h"

namespace
{
	constexpr double UnrealTorquePerNm = 100.0 * 100.0;

	/*
	 * Numerical limits of an explicit muscle.
	 *
	 * A muscle's stiffness and damping are sized for the load it moves about its main axis (the ankle
	 * holds up the whole body when it pitches). About other axes, or when the segment on one side is
	 * light and free (a foot about its long axis, a forearm about its own axis, an airborne foot), the
	 * same stiffness and damping act on a tiny inertia, and one physics substep is too long to follow
	 * the resulting motion: an explicit spring overshoots once k*dt^2/I passes 4 and an explicit
	 * damper once d*dt/I passes 2, and the joint rings with growing amplitude.
	 *
	 * Real tissue has no timestep, so these are limits of the numerical method, not of the model: in
	 * the direction the spring (or the damper) pushes, its effective gain is
	 * capped at what one substep can represent, using the two segments' inertia about their own
	 * centers of mass. That is the inertia Chaos integrates an applied torque with, before its joint
	 * constraints pull the segments back together; measured: using the (larger) inertia about the
	 * joint instead let the damper overshoot and the trunk fold within a second.
	 * The caps scale with 1/dt^2 and 1/dt, so a higher substep rate relaxes them (see the Milestone 3
	 * doc). At 240 Hz they trim the ankle's sagittal stiffness by a few percent and leave the other
	 * weight-bearing directions alone.
	 */
	TAutoConsoleVariable<float> CVarMaxStiffnessStepRatio(
		TEXT("athlete.Muscle.MaxStiffnessStepRatio"), 2.0f,
		TEXT("Cap on k*dt^2/I for muscle stiffness in any direction (explicit spring is unstable above 4)."));
	TAutoConsoleVariable<float> CVarMaxSpinRemovedPerSubstep(
		TEXT("athlete.Muscle.MaxSpinRemovedPerSubstep"), 0.5f,
		TEXT("Cap on d*dt/I: the largest fraction of a joint's relative spin its muscle damping may remove in one physics substep."));

	/**
	 * How easily a torque about Axis spins the body: Axis . I^-1 . Axis (world, about its center of
	 * mass), in 1/(kg*cm^2). Zero for bodies the muscle cannot turn (kinematic or static).
	 */
	double InverseInertiaAbout(Chaos::FRigidBodyHandle_Internal& Body, const Chaos::FVec3& Axis)
	{
		const Chaos::EObjectStateType State = Body.ObjectState();
		if (State != Chaos::EObjectStateType::Dynamic && State != Chaos::EObjectStateType::Sleeping)
		{
			return 0.0;
		}
		const Chaos::FMatrix33 InverseInertiaWorld = Chaos::Utilities::ComputeWorldSpaceInertia(Body.R() * Body.RotationOfMass(), Chaos::FVec3(Body.InvI()));
		return Chaos::FVec3::DotProduct(Axis, InverseInertiaWorld * Axis);
	}

	/**
	 * A muscle torque (world) capped at the joint's strength AT ITS CURRENT SPEED (force-velocity):
	 * the speed that matters is the joint's relative spin in the direction the muscle pushes.
	 * Positive = the joint moves the way the muscle pushes (shortening, less torque available);
	 * negative = it's being forced the other way (lengthening, more torque available).
	 */
	FVector CapAtStrength(const FAthleteMuscleCommand& Muscle, const FVector& Torque, const FVector& RelativeSpinWorld)
	{
		const double Magnitude = Torque.Size();
		if (Magnitude <= 0.0)
		{
			return Torque;
		}
		double Limit = Muscle.TorqueLimit;
		if (Muscle.MaxVelocityRadPerS > 0.0)
		{
			const double ShorteningSpeed = FVector::DotProduct(Torque / Magnitude, RelativeSpinWorld);
			Limit *= AthleteMuscleModel::ForceVelocityFactor(ShorteningSpeed / Muscle.MaxVelocityRadPerS);
		}
		return Magnitude > Limit ? Torque * (Limit / Magnitude) : Torque;
	}
}

void FAthleteMuscleSimCallback::OnPreSimulate_Internal()
{
	const FAthleteMuscleInput* Input = GetConsumerInput_Internal();
	if (!Input)
	{
		return;
	}
	const uint64 StartCycles = FPlatformTime::Cycles64();
	const double Dt = GetDeltaTime_Internal();
	const double MaxStiffnessStepRatio = CVarMaxStiffnessStepRatio.GetValueOnAnyThread();
	const double MaxSpinRemovedPerSubstep = CVarMaxSpinRemovedPerSubstep.GetValueOnAnyThread();

	FAthleteMuscleOutput& Output = GetProducerOutputData_Internal();
	Output.JointTorquesNm.SetNumZeroed(Input->Joints.Num());
	Output.JointErrorsRad.SetNumZeroed(Input->Joints.Num());

	// Pass 1 works out each joint's torque; pass 2 lets a joint take up another's reaction; pass 3 applies.
	const int32 NumJoints = Input->Joints.Num();
	TArray<FVector, TInlineAllocator<32>> Torques;          // world, Unreal units, on the child
	TArray<FVector, TInlineAllocator<32>> RelativeSpinsWorld;
	Torques.SetNumZeroed(NumJoints);
	RelativeSpinsWorld.SetNumZeroed(NumJoints);

	for (int32 Index = 0; Index < NumJoints; ++Index)
	{
		const FAthleteMuscleCommand& Muscle = Input->Joints[Index];
		Chaos::FRigidBodyHandle_Internal* Child = Muscle.Child ? Muscle.Child->GetPhysicsThreadAPI() : nullptr;
		Chaos::FRigidBodyHandle_Internal* Parent = Muscle.Parent ? Muscle.Parent->GetPhysicsThreadAPI() : nullptr;
		if (!Child || !Parent || Dt <= 0.0)
		{
			continue;
		}

		// Current joint frames in world space, and the child frame relative to the parent frame.
		const FQuat ChildFrame = FQuat(Child->R()) * Muscle.ChildFrameLocal;
		const FQuat ParentFrame = FQuat(Parent->R()) * Muscle.ParentFrameLocal;
		const FQuat Relative = ParentFrame.Inverse() * ChildFrame;

		// Rotation still needed to reach the target, as a rotation vector (axis * angle, shortest way
		// round), first in the parent constraint frame...
		FQuat ToTarget = Muscle.Target * Relative.Inverse();
		ToTarget.EnforceShortestArcWith(FQuat::Identity);
		FVector Axis;
		double Angle;
		ToTarget.ToAxisAndAngle(Axis, Angle);

		// ...then along the joint's ANATOMICAL axes, where the per-axis gains belong: X the long axis
		// (twist), Y the flexion axis (sagittal), Z the frontal axis. The constraint frame is turned by
		// the range-of-motion neutral (the hip's by about 45 deg of flexion), and decomposing there put
		// the hip's frontal stiffness mostly about its yaw and left its abductors at about half.
		const FQuat AxisFrame = FQuat(Parent->R()) * Muscle.AxisFrameLocal;
		const FVector Error = AxisFrame.UnrotateVector(ParentFrame.RotateVector(Axis * Angle));

		// Relative spin, and its departure from the planned motion (no plan = hold still), on the same axes.
		const FVector RelativeSpin = AxisFrame.UnrotateVector(FVector(Child->W()) - FVector(Parent->W()));
		const FVector SpinError = RelativeSpin - AxisFrame.UnrotateVector(ParentFrame.RotateVector(Muscle.TargetSpin));

		// Spring toward the target and damper against spin away from the plan, with per-axis gains. Each is then
		// capped along the direction it actually pushes, at what one substep can represent for the
		// two segments' inertia about that direction (see above): the effective gain in that
		// direction is |torque| / |error| (or / |spin|).
		auto CapAlongTorque = [&](const FVector& TorqueInFrame, double Displacement, double MaxRatio, double StepPower)
		{
			const double Magnitude = TorqueInFrame.Size();
			if (Magnitude <= UE_SMALL_NUMBER || Displacement <= UE_SMALL_NUMBER)
			{
				return TorqueInFrame;
			}
			const FVector Direction = AxisFrame.RotateVector(TorqueInFrame / Magnitude);
			const double InverseInertia = InverseInertiaAbout(*Child, Direction) + InverseInertiaAbout(*Parent, Direction);
			const double Gain = Magnitude / Displacement;
			const double MaxGain = InverseInertia > 0.0 ? MaxRatio / (StepPower * InverseInertia) : Gain;
			return Gain > MaxGain ? TorqueInFrame * (MaxGain / Gain) : TorqueInFrame;
		};
		const FVector SpringTorque = CapAlongTorque(Muscle.StiffnessPerRad * Error, Error.Size(), MaxStiffnessStepRatio, Dt * Dt);
		const FVector DamperTorque = CapAlongTorque(-Muscle.DampingPerRadPerS * SpinError, SpinError.Size(), MaxSpinRemovedPerSubstep, Dt);

		RelativeSpinsWorld[Index] = AxisFrame.RotateVector(RelativeSpin);
		Torques[Index] = CapAtStrength(Muscle, AxisFrame.RotateVector(SpringTorque + DamperTorque) + Muscle.FeedforwardTorque, RelativeSpinsWorld[Index]);
		Output.JointErrorsRad[Index] = Error;
	}

	// A joint told to take up another joint's reaction (the stance hip, for the swing hip: both pull
	// on the pelvis) adds the opposite of that joint's torque, so the shared parent feels none of
	// it; the difference goes down its own leg to the ground. Still an internal torque, still within
	// its own strength. (SIMBICON, Yin et al. 2007: stance hip torque = -torso torque - swing hip torque.)
	for (int32 Index = 0; Index < NumJoints; ++Index)
	{
		const int32 Partner = Input->Joints[Index].CancelReactionOf;
		if (Torques.IsValidIndex(Partner) && Input->Joints[Partner].CancelReactionOf == INDEX_NONE)
		{
			Torques[Index] = CapAtStrength(Input->Joints[Index], Torques[Index] - Torques[Partner], RelativeSpinsWorld[Index]);
		}
	}

	// Equal and opposite: muscles are internal to the body.
	for (int32 Index = 0; Index < NumJoints; ++Index)
	{
		const FAthleteMuscleCommand& Muscle = Input->Joints[Index];
		Chaos::FRigidBodyHandle_Internal* Child = Muscle.Child ? Muscle.Child->GetPhysicsThreadAPI() : nullptr;
		Chaos::FRigidBodyHandle_Internal* Parent = Muscle.Parent ? Muscle.Parent->GetPhysicsThreadAPI() : nullptr;
		if (!Child || !Parent || Dt <= 0.0)
		{
			continue;
		}
		Child->AddTorque(Torques[Index]);
		Parent->AddTorque(-Torques[Index]);
		Output.JointTorquesNm[Index] = Torques[Index] / UnrealTorquePerNm;
	}
	Output.ComputeSeconds = FPlatformTime::ToSeconds64(FPlatformTime::Cycles64() - StartCycles);
}
