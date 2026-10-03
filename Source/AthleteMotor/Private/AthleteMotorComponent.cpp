// Project ATHLETE

#include "AthleteMotorComponent.h"
#include "AthleteMotor.h"
#include "Articulation/AthletePhysicalBodyComponent.h"
#include "Components/ShapeComponent.h"
#include "Definition/AthleteDefinition.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "PBDRigidsSolver.h"
#include "Physics/Experimental/PhysScene_Chaos.h"
#include "PhysicsEngine/BodyInstance.h"
#include "PhysicsEngine/ConstraintInstance.h"
#include "Units/AthleteUnits.h"

namespace
{
	// Unreal torque units: kg*cm^2/s^2. 1 N*m = 1e4.
	constexpr double UnrealTorquePerNm = 100.0 * 100.0;

	/** Enough history to cover the longest allowed reaction time plus a margin. */
	constexpr double SensingHistorySeconds = 0.5;
}

UAthleteMotorComponent::UAthleteMotorComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	// Decide the next physics step's targets from the state the last step produced.
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
}

UAthletePhysicalBodyComponent* UAthleteMotorComponent::FindBody() const
{
	return GetOwner() ? GetOwner()->FindComponentByClass<UAthletePhysicalBodyComponent>() : nullptr;
}

bool UAthleteMotorComponent::InitializeMuscles(const FAthleteStrengthProfile& Strength, const FAthleteMotorSkill& Skill)
{
	UAthletePhysicalBodyComponent* Body = FindBody();
	FPhysScene* Scene = GetWorld() ? GetWorld()->GetPhysicsScene() : nullptr;
	if (!Body || !Body->IsBuilt() || !Scene || !Scene->GetSolver())
	{
		return false;
	}
	ReleaseMuscles();
	MotorSkill = Skill;
	const double Gravity = FMath::Abs(AthleteUnits::UnrealToMeters(GetWorld()->GetGravityZ()));

	Impedances.SetNum(AthleteJoints::NumJoints);
	MuscleCommands.SetNum(AthleteJoints::NumJoints);
	LastJointTorquesNm.Init(FVector::ZeroVector, AthleteJoints::NumJoints);
	LastJointErrorsRad.Init(FVector::ZeroVector, AthleteJoints::NumJoints);
	for (int32 Index = 0; Index < AthleteJoints::NumJoints; ++Index)
	{
		const EAthleteJoint Joint = AthleteJoints::FromIndex(Index);
		const FAthleteJointImpedance& Impedance = Impedances[Index] = AthleteMuscleModel::ComputeStandingImpedance(Body->GetBodyModel(), Joint, Strength, Skill, Gravity);
		DisableSolverDrive(Joint);

		const FConstraintInstance* Constraint = Body->GetJointConstraint(Joint);
		FAthleteMuscleCommand& Muscle = MuscleCommands[Index];
		Muscle.Child = Body->GetSegmentBody(AthleteJoints::GetChildSegment(Joint))->BodyInstance.GetPhysicsActor();
		Muscle.Parent = Body->GetSegmentBody(AthleteJoints::GetParentSegment(Joint))->BodyInstance.GetPhysicsActor();
		Muscle.ChildFrameLocal = Constraint->GetRefFrame(EConstraintFrame::Frame1).GetRotation();
		Muscle.ParentFrameLocal = Constraint->GetRefFrame(EConstraintFrame::Frame2).GetRotation();
		// The parent constraint frame is (neutral * joint frame) in body terms; take the neutral back
		// out to get the anatomical axes, in the parent's local space.
		const FAthleteJointSetup& Setup = Body->GetJointSetup(Joint);
		Muscle.AxisFrameLocal = Muscle.ParentFrameLocal * (Setup.JointFrame.Inverse() * Setup.NeutralRotation.Inverse() * Setup.JointFrame);
		Muscle.StiffnessPerRad = Impedance.AxisStiffnessNmPerRad * UnrealTorquePerNm;
		Muscle.DampingPerRadPerS = Impedance.AxisDampingNmsPerRad * UnrealTorquePerNm;
		Muscle.TorqueLimit = Impedance.TorqueLimitNm * UnrealTorquePerNm;
		Muscle.MaxVelocityRadPerS = Impedance.MaxVelocityRadPerS;
	}
	ApplyPosture(FAthletePosture()); // reference pose

	// What each foot can take: half the body's weight, on the lever from the ankle to the toes or heel.
	const FAthleteSegmentCollisionShape& Foot = Body->GetCollisionShape(EAthleteSegment::FootLeft);
	const double AnkleX = Body->GetBodyModel().GetJointCenterM(EAthleteJoint::AnkleLeft).X;
	const double WeightPerFootN = 0.5 * Body->GetBodyModel().GetTotalMassKg() * Gravity;
	FootSupport.TippingTorqueToesNm = WeightPerFootN * FMath::Max(Foot.CenterM.X + Foot.BoxHalfExtentsM.X - AnkleX, 0.0);
	FootSupport.TippingTorqueHeelNm = WeightPerFootN * FMath::Max(AnkleX - (Foot.CenterM.X - Foot.BoxHalfExtentsM.X), 0.0);
	const double AnkleY = Body->GetBodyModel().GetJointCenterM(EAthleteJoint::AnkleLeft).Y;
	FootSupport.TippingTorqueSideNm = WeightPerFootN * FMath::Max(FMath::Min(Foot.CenterM.Y + Foot.BoxHalfExtentsM.Y - AnkleY, AnkleY - (Foot.CenterM.Y - Foot.BoxHalfExtentsM.Y)), 0.0);
	FootSupport.AnkleStiffnessNmPerRad = GetJointImpedance(EAthleteJoint::AnkleLeft).StiffnessNmPerRad;
	FootSupport.LoadPerFootN = WeightPerFootN;

	// He starts on his feet; remember how high his center of mass stands above them.
	const FAthleteBalanceSensing Start = SenseNow();
	GroundHeightM = Start.SupportCenterM.Z;
	StandingComHeightM = Start.CenterOfMassM.Z - GroundHeightM;
	MotorState = EAthleteMotorState::Standing;
	SensingHistory.Reset();

	// Walking: his own leg dimensions, facing the way his body was built.
	const double BodyYawDeg = Body->GetComponentRotation().Yaw;
	Gait.Initialize(FAthleteLegGeometry::FromModel(Body->GetBodyModel(), Foot.CenterM.X, Foot.BoxHalfExtentsM.Z), GroundHeightM, BodyYawDeg, BodyYawDeg);

	MuscleCallback = Scene->GetSolver()->CreateAndRegisterSimCallbackObject_External<FAthleteMuscleSimCallback>();
	bMusclesInitialized = true;

	const FAthleteJointImpedance& Ankle = GetJointImpedance(EAthleteJoint::AnkleLeft);
	UE_LOG(LogAthleteMotor, Log, TEXT("%s: muscles on. Ankle stiffness %.0f N*m/rad (gravity load %.0f), damping %.0f N*m*s/rad, limit %.0f N*m; foot tips at %.0f N*m (toes) / %.0f N*m (heel) / %.0f N*m (side)."),
		*GetOwner()->GetName(), Ankle.StiffnessNmPerRad, Ankle.GravityStiffnessNmPerRad, Ankle.DampingNmsPerRad, Ankle.TorqueLimitNm,
		FootSupport.TippingTorqueToesNm, FootSupport.TippingTorqueHeelNm, FootSupport.TippingTorqueSideNm);
	return true;
}

void UAthleteMotorComponent::DisableSolverDrive(EAthleteJoint Joint)
{
	// Muscles are applied as explicit torques, not Chaos joint drives: a solver drive's effective
	// stiffness depends on the mass ratio of the bodies it connects, and a light foot against a
	// heavy body made the ankle yield (see the Milestone 3 doc). Make sure no drive is left on.
	FConstraintInstance* Constraint = FindBody()->GetJointConstraint(Joint);
	Constraint->SetOrientationDriveTwistAndSwing(false, false);
	Constraint->SetAngularVelocityDriveTwistAndSwing(false, false);
	Constraint->SetOrientationDriveSLERP(false);
	Constraint->SetAngularVelocityDriveSLERP(false);
}

void UAthleteMotorComponent::ApplyPosture(const FAthletePosture& Posture, double DeltaTimeS)
{
	UAthletePhysicalBodyComponent* Body = FindBody();
	bPlannedLastFrame.SetNumZeroed(MuscleCommands.Num());
	for (int32 Index = 0; Index < MuscleCommands.Num(); ++Index)
	{
		const EAthleteJoint Joint = AthleteJoints::FromIndex(Index);
		FAthleteMuscleCommand& Muscle = MuscleCommands[Index];
		const FQuat PreviousTarget = Muscle.Target;
		Muscle.Target = AthleteJointSetup::ComputeDriveTarget(Body->GetJointSetup(Joint), Posture.ChildRelativeToParent[Index]);

		// A planned movement: command its velocity too, the rate the target moved since last frame
		// (a spin in the parent joint frame, like the muscle's error). Only once the plan has run for
		// a frame: when a joint starts following a plan its target jumps, which is a new end point,
		// not a velocity. Never faster than the joint can turn.
		Muscle.TargetSpin = FVector::ZeroVector;
		if (Posture.bPlannedMotion[Index] && bPlannedLastFrame[Index] && DeltaTimeS > 0.0)
		{
			FQuat Step = Muscle.Target * PreviousTarget.Inverse();
			Step.EnforceShortestArcWith(FQuat::Identity);
			FVector Axis;
			double Angle;
			Step.ToAxisAndAngle(Axis, Angle);
			const double MaxSpeed = Impedances[Index].MaxVelocityRadPerS > 0.0 ? Impedances[Index].MaxVelocityRadPerS : UE_BIG_NUMBER;
			Muscle.TargetSpin = Axis * FMath::Min(Angle / DeltaTimeS, MaxSpeed);
		}
		bPlannedLastFrame[Index] = Posture.bPlannedMotion[Index] && DeltaTimeS > 0.0;
		Muscle.CancelReactionOf = Posture.CancelReactionOf[Index];
		Muscle.FeedforwardTorque = Posture.FeedforwardTorqueNm[Index] * UnrealTorquePerNm;

		// Activation: stiffness scales directly, damping with its square root (same damping ratio).
		const double Scale = FMath::Max(Posture.StiffnessScale[Index], 0.0);
		Muscle.StiffnessPerRad = Impedances[Index].AxisStiffnessNmPerRad * Scale * UnrealTorquePerNm;
		Muscle.DampingPerRadPerS = Impedances[Index].AxisDampingNmsPerRad * FMath::Sqrt(Scale) * UnrealTorquePerNm;
	}
}

FVector UAthleteMotorComponent::GetJointTorqueNm(EAthleteJoint Joint) const
{
	const int32 Index = AthleteJoints::ToIndex(Joint);
	return LastJointTorquesNm.IsValidIndex(Index) ? LastJointTorquesNm[Index] : FVector::ZeroVector;
}

FVector UAthleteMotorComponent::GetJointErrorDeg(EAthleteJoint Joint) const
{
	const int32 Index = AthleteJoints::ToIndex(Joint);
	return LastJointErrorsRad.IsValidIndex(Index) ? FMath::RadiansToDegrees(LastJointErrorsRad[Index]) : FVector::ZeroVector;
}

void UAthleteMotorComponent::PushMuscleCommands()
{
	// Chaos hands this input to every substep of the coming physics step, then discards it, so it
	// is sent every frame. (The physics side never keeps it: a body pointer must not outlive the
	// frame it was known to be valid in.)
	if (FAthleteMuscleInput* Input = MuscleCallback->GetProducerInputData_External())
	{
		Input->Joints = MuscleCommands;
	}
}

void UAthleteMotorComponent::ReadMuscleOutputs()
{
	// One output per completed substep; keep the newest. Draining also returns them to the pool.
	SubstepsLastRead = 0;
	MuscleMsLastRead = 0.0;
	while (Chaos::TSimCallbackOutputHandle<FAthleteMuscleOutput> Output = MuscleCallback->PopFutureOutputData_External())
	{
		++SubstepsLastRead;
		MuscleMsLastRead += 1000.0 * Output->ComputeSeconds;
		if (Output->JointTorquesNm.Num() == LastJointTorquesNm.Num())
		{
			LastJointTorquesNm = Output->JointTorquesNm;
			LastJointErrorsRad = Output->JointErrorsRad;
		}
	}
}

void UAthleteMotorComponent::ReleaseMuscles()
{
	if (MuscleCallback)
	{
		UWorld* World = GetWorld();
		FPhysScene* Scene = World ? World->GetPhysicsScene() : nullptr;
		if (Scene && Scene->GetSolver())
		{
			Scene->GetSolver()->UnregisterAndFreeSimCallbackObject_External(MuscleCallback);
		}
		MuscleCallback = nullptr;
	}
	bMusclesInitialized = false;
}

void UAthleteMotorComponent::OnUnregister()
{
	ReleaseMuscles();
	Super::OnUnregister();
}

FAthleteBalanceSensing UAthleteMotorComponent::SenseNow() const
{
	UAthletePhysicalBodyComponent* Body = FindBody();
	FAthleteBalanceSensing Sensing;
	Sensing.TimeS = GetWorld()->GetTimeSeconds();
	Sensing.CenterOfMassM = Body->GetCenterOfMassM();
	Sensing.CenterOfMassVelocityMps = Body->GetLinearMomentumKgMps() / Body->GetTotalMassKg();

	// Support: midpoint of the two feet's centers, dropped to the soles.
	const FVector LeftFoot = AthleteUnits::UnrealToMeters(Body->GetSegmentBody(EAthleteSegment::FootLeft)->GetComponentLocation());
	const FVector RightFoot = AthleteUnits::UnrealToMeters(Body->GetSegmentBody(EAthleteSegment::FootRight)->GetComponentLocation());
	Sensing.SupportCenterM = 0.5 * (LeftFoot + RightFoot);
	Sensing.SupportCenterM.Z -= Body->GetCollisionShape(EAthleteSegment::FootLeft).BoxHalfExtentsM.Z;
	Sensing.FootHalfLengthM = Body->GetCollisionShape(EAthleteSegment::FootLeft).BoxHalfExtentsM.X;

	// The heading he holds (turned by the gait) on top of the body frame he was built in.
	const FQuat BodyFrame = Body->GetComponentQuat();
	Sensing.BodyFrameRotation = BodyFrame;
	Sensing.HeadingDeviation = Gait.GetHeadingDeviation();
	const FQuat Heading = BodyFrame * Sensing.HeadingDeviation;
	Sensing.ForwardAxis = Heading.GetForwardVector().GetSafeNormal2D();
	Sensing.RightAxis = Heading.GetRightVector().GetSafeNormal2D();
	for (int32 Segment = 0; Segment < AthleteSegments::NumSegments; ++Segment)
	{
		Sensing.SegmentRotations[Segment] = Body->GetSegmentBody(static_cast<EAthleteSegment>(Segment))->GetComponentQuat();
	}
	for (int32 Joint = 0; Joint < AthleteJoints::NumJoints; ++Joint)
	{
		Sensing.JointCentersM[Joint] = Body->GetJointCenterWorldM(AthleteJoints::FromIndex(Joint));
	}
	Sensing.FootSoleHeightM[0] = Body->GetSegmentLowestPointM(EAthleteSegment::FootLeft) - GroundHeightM;
	Sensing.FootSoleHeightM[1] = Body->GetSegmentLowestPointM(EAthleteSegment::FootRight) - GroundHeightM;
	return Sensing;
}

void UAthleteMotorComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	UAthletePhysicalBodyComponent* Body = FindBody();
	if (!Body || !Body->IsBuilt())
	{
		return;
	}
	if (!bMusclesInitialized)
	{
		const UAthleteDefinition* Athlete = Body->Athlete;
		if (!InitializeMuscles(Athlete ? Athlete->Strength : FAthleteStrengthProfile::MakeGeneralPopulationMaleBaseline(24.0, Body->GetBodyModel().GetStatureM(), Body->GetBodyModel().GetTotalMassKg()),
			Athlete ? Athlete->MotorSkill : MotorSkill))
		{
			return;
		}
	}
	const uint64 ControlStartCycles = FPlatformTime::Cycles64();
	ReadMuscleOutputs();

	if (bBalanceEnabled && MotorState == EAthleteMotorState::Standing)
	{
		// Sense now; each neural loop acts on what was sensed its own latency ago.
		const FAthleteBalanceSensing Now = SenseNow();
		SensingHistory.Add(Now);
		auto SensedAgo = [this, &Now](double DelayS) -> const FAthleteBalanceSensing&
		{
			// The newest sample at least DelayS old (the oldest one if the history is shorter).
			for (int32 Index = SensingHistory.Num() - 1; Index >= 0; --Index)
			{
				if (SensingHistory[Index].TimeS <= Now.TimeS - DelayS + UE_KINDA_SMALL_NUMBER)
				{
					return SensingHistory[Index];
				}
			}
			return SensingHistory[0];
		};
		const FAthleteBalanceSensing ForBalance = SensedAgo(MotorSkill.ReactionTimeS);
		const FAthleteBalanceSensing ForPosture = SensedAgo(MotorSkill.PostureReflexDelayS);
		SensingHistory.RemoveAll([&Now](const FAthleteBalanceSensing& Sample) { return Sample.TimeS < Now.TimeS - SensingHistorySeconds; });

		if (ForBalance.CenterOfMassM.Z - GroundHeightM < FallenComHeightFraction * StandingComHeightM)
		{
			// He knows he's down (as perceived, with his reaction delay). Stop trying to stand:
			// muscles hold their reference joint angles with normal tone.
			MotorState = EAthleteMotorState::Fallen;
			LastCommand = FAthleteBalanceCommand();
			ApplyPosture(FAthletePosture());
			UE_LOG(LogAthleteMotor, Log, TEXT("%s: fallen at t = %.2f s (center of mass %.2f m above the ground, standing %.2f m). Balance control off."),
				*GetOwner()->GetName(), Now.TimeS, ForBalance.CenterOfMassM.Z - GroundHeightM, StandingComHeightM);
		}
		else
		{
			const double Gravity = FMath::Abs(AthleteUnits::UnrealToMeters(GetWorld()->GetGravityZ()));
			FAthletePosture Posture;
			if (Gait.Update(MovementIntent, ForBalance, ForPosture, Now, MotorSkill, FootSupport, Gravity, DeltaTime, Posture))
			{
				// Walking: the gait places the feet and drives the legs.
				LastCommand = FAthleteBalanceCommand();
				ApplyPosture(Posture, DeltaTime);
			}
			else
			{
				// Standing: balance decides where the body should be; postural reflexes hold each
				// segment there in space, facing the heading he holds.
				LastCommand = AthleteBalanceController::Compute(ForBalance, MotorSkill, Gravity);
				ApplyPosture(AthleteBalanceController::SolveJointTargets(
					AthleteBalanceController::MakeSegmentTargets(LastCommand, Gait.GetHeadingDeviation()), ForPosture, FootSupport));
			}
		}
	}
	PushMuscleCommands();
	ControlMsLastTick = 1000.0 * FPlatformTime::ToSeconds64(FPlatformTime::Cycles64() - ControlStartCycles);
}
