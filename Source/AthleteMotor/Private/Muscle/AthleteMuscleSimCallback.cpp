// Project ATHLETE

#include "Muscle/AthleteMuscleSimCallback.h"
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
}

void FAthleteMuscleSimCallback::OnPreSimulate_Internal()
{
	const FAthleteMuscleInput* Input = GetConsumerInput_Internal();
	if (!Input)
	{
		return;
	}
	const double Dt = GetDeltaTime_Internal();
	const double MaxStiffnessStepRatio = CVarMaxStiffnessStepRatio.GetValueOnAnyThread();
	const double MaxSpinRemovedPerSubstep = CVarMaxSpinRemovedPerSubstep.GetValueOnAnyThread();

	FAthleteMuscleOutput& Output = GetProducerOutputData_Internal();
	Output.JointTorquesNm.SetNumZeroed(Input->Joints.Num());
	Output.JointErrorsRad.SetNumZeroed(Input->Joints.Num());

	for (int32 Index = 0; Index < Input->Joints.Num(); ++Index)
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
		// round) in the parent joint frame: X twist, Y sagittal, Z frontal.
		FQuat ToTarget = Muscle.Target * Relative.Inverse();
		ToTarget.EnforceShortestArcWith(FQuat::Identity);
		FVector Axis;
		double Angle;
		ToTarget.ToAxisAndAngle(Axis, Angle);
		const FVector Error = Axis * Angle;

		// Relative spin, in the same joint frame.
		const FVector RelativeSpin = ParentFrame.UnrotateVector(FVector(Child->W()) - FVector(Parent->W()));

		// Spring toward the target and damper against relative spin, with per-axis gains. Each is then
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
			const FVector Direction = ParentFrame.RotateVector(TorqueInFrame / Magnitude);
			const double InverseInertia = InverseInertiaAbout(*Child, Direction) + InverseInertiaAbout(*Parent, Direction);
			const double Gain = Magnitude / Displacement;
			const double MaxGain = InverseInertia > 0.0 ? MaxRatio / (StepPower * InverseInertia) : Gain;
			return Gain > MaxGain ? TorqueInFrame * (MaxGain / Gain) : TorqueInFrame;
		};
		const FVector SpringTorque = CapAlongTorque(Muscle.StiffnessPerRad * Error, Error.Size(), MaxStiffnessStepRatio, Dt * Dt);
		const FVector DamperTorque = CapAlongTorque(-Muscle.DampingPerRadPerS * RelativeSpin, RelativeSpin.Size(), MaxSpinRemovedPerSubstep, Dt);

		// Total muscle torque, capped at the joint's strength.
		FVector Torque = ParentFrame.RotateVector(SpringTorque + DamperTorque);
		const double Magnitude = Torque.Size();
		if (Magnitude > Muscle.TorqueLimit && Magnitude > 0.0)
		{
			Torque *= Muscle.TorqueLimit / Magnitude;
		}

		// Equal and opposite: muscles are internal to the body.
		Child->AddTorque(Torque);
		Parent->AddTorque(-Torque);
		Output.JointTorquesNm[Index] = Torque / UnrealTorquePerNm;
		Output.JointErrorsRad[Index] = Axis * Angle;
	}
}
