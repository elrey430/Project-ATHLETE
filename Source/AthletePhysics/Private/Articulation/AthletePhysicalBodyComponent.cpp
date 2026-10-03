// Project ATHLETE

#include "Articulation/AthletePhysicalBodyComponent.h"
#include "AthletePhysics.h"
#include "Chaos/ChaosEngineInterface.h"
#include "Components/BoxComponent.h"
#include "Components/CapsuleComponent.h"
#include "Definition/AthleteDefinition.h"
#include "Engine/CollisionProfile.h"
#include "Engine/World.h"
#include "Physics/Experimental/PhysInterface_Chaos.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"
#include "PhysicsEngine/BodyInstance.h"
#include "PhysicsEngine/ConstraintInstance.h"
#include "Units/AthleteUnits.h"

namespace
{
	// Unreal inertia is kg*cm^2; ATHLETE uses kg*m^2.
	constexpr double KgCm2PerKgM2 = 100.0 * 100.0;

	/**
	 * Unreal clamps every body's spin to 3600 deg/s by default. Measured: that clamp removed the
	 * trunk's spin the instant it was struck (before the joints could share the impulse), losing
	 * angular momentum. Human segments exceed 3600 deg/s (a thrower's arm reaches ~7000-8000 deg/s),
	 * so the default would silently cap football. This value is a numerical-explosion guard only,
	 * about 4x beyond anything physiological.
	 */
	constexpr float MaxSegmentAngularVelocityRadPerS = 30000.0f * UE_PI / 180.0f;

	const FColor LeftBodyColor(80, 140, 255);
	const FColor RightBodyColor(255, 90, 80);
	const FColor MidlineBodyColor(230, 230, 230);

	FColor ColorFor(EAthleteSegment Segment)
	{
		switch (AthleteSegments::GetSide(Segment))
		{
		case EAthleteBodySide::Left:  return LeftBodyColor;
		case EAthleteBodySide::Right: return RightBodyColor;
		default:                      return MidlineBodyColor;
		}
	}

	EAngularConstraintMotion MotionFor(double HalfRangeDeg)
	{
		return HalfRangeDeg > 0.0 ? EAngularConstraintMotion::ACM_Limited : EAngularConstraintMotion::ACM_Locked;
	}

	bool AreJointed(EAthleteSegment A, EAthleteSegment B)
	{
		for (int32 Index = 0; Index < AthleteJoints::NumJoints; ++Index)
		{
			const EAthleteJoint Joint = AthleteJoints::FromIndex(Index);
			const EAthleteSegment Parent = AthleteJoints::GetParentSegment(Joint);
			const EAthleteSegment Child = AthleteJoints::GetChildSegment(Joint);
			if ((Parent == A && Child == B) || (Parent == B && Child == A))
			{
				return true;
			}
		}
		return false;
	}
}

UAthletePhysicalBodyComponent::UAthletePhysicalBodyComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
}

void UAthletePhysicalBodyComponent::BeginPlay()
{
	Super::BeginPlay();
	if (bBuildOnBeginPlay && Athlete)
	{
		BuildBody();
	}
}

void UAthletePhysicalBodyComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	DestroyBody();
	Super::EndPlay(EndPlayReason);
}

bool UAthletePhysicalBodyComponent::BuildBody()
{
	FAthleteBodyModel NewModel;
	if (!Athlete || !Athlete->BuildBodyModel(NewModel))
	{
		UE_LOG(LogAthletePhysics, Warning, TEXT("%s: no valid athlete to build a body from."), *GetPathName());
		return false;
	}
	return BuildBodyFrom(NewModel, Athlete->Mobility);
}

bool UAthletePhysicalBodyComponent::BuildBodyFrom(const FAthleteBodyModel& InModel, const FAthleteMobilityProfile& InMobility)
{
	if (!GetWorld() || !GetOwner())
	{
		return false;
	}
	DestroyBody();
	Model = InModel;

	CollisionShapes.SetNum(AthleteSegments::NumSegments);
	for (int32 Index = 0; Index < AthleteSegments::NumSegments; ++Index)
	{
		CollisionShapes[Index] = AthleteCollisionGeometry::ComputeShape(Model, AthleteSegments::FromIndex(Index));
	}

	JointSetups.SetNum(AthleteJoints::NumJoints);
	for (int32 Index = 0; Index < AthleteJoints::NumJoints; ++Index)
	{
		const EAthleteJoint Joint = AthleteJoints::FromIndex(Index);
		JointSetups[Index] = AthleteJointSetup::Compute(Model, Joint, InMobility.Get(Joint));
	}

	// 1. Rigid bodies.
	SegmentBodies.SetNum(AthleteSegments::NumSegments);
	for (int32 Index = 0; Index < AthleteSegments::NumSegments; ++Index)
	{
		SegmentBodies[Index] = CreateSegmentBody(AthleteSegments::FromIndex(Index));
	}

	// 2. Joints.
	Joints.Reset();
	for (int32 Index = 0; Index < AthleteJoints::NumJoints; ++Index)
	{
		CreateJoint(AthleteJoints::FromIndex(Index));
	}

	// 3. Self-collision filtering for overlapping, non-jointed pairs.
	IgnoreOverlappingPairs();

	UE_LOG(LogAthletePhysics, Log, TEXT("%s: built %d bodies, %d joints, %d ignored self-collision pairs, %.1f kg."),
		*GetOwner()->GetName(), SegmentBodies.Num(), Joints.Num(), IgnoredCollisionPairs.Num(), GetTotalMassKg());
	return true;
}

UShapeComponent* UAthletePhysicalBodyComponent::CreateSegmentBody(EAthleteSegment Segment)
{
	const FAthleteSegmentCollisionShape& Shape = GetCollisionShape(Segment);
	const FAthleteBodySegment& Seg = Model.GetSegment(Segment);
	const FName Name(*FString::Printf(TEXT("Body_%s"), AthleteSegments::GetName(Segment)));

	UShapeComponent* Body = nullptr;
	if (Shape.Type == EAthleteCollisionShapeType::Box)
	{
		UBoxComponent* Box = NewObject<UBoxComponent>(GetOwner(), Name);
		Box->InitBoxExtent(AthleteUnits::MetersToUnreal(Shape.BoxHalfExtentsM));
		Body = Box;
	}
	else
	{
		UCapsuleComponent* Capsule = NewObject<UCapsuleComponent>(GetOwner(), Name);
		Capsule->InitCapsuleSize(AthleteUnits::MetersToUnreal(Shape.CapsuleRadiusM), AthleteUnits::MetersToUnreal(Shape.CapsuleHalfHeightM));
		Body = Capsule;
	}

	// Every body starts aligned with the body frame, centered on its collision shape.
	const FTransform& BodyFrame = GetComponentTransform();
	Body->SetWorldLocationAndRotation(BodyFrame.TransformPosition(AthleteUnits::MetersToUnreal(Shape.CenterM)), BodyFrame.GetRotation());

	Body->SetCollisionProfileName(UCollisionProfile::PhysicsActor_ProfileName);
	Body->SetSimulatePhysics(true);
	Body->SetHiddenInGame(false);
	Body->ShapeColor = ColorFor(Segment);

	FBodyInstance& Instance = Body->BodyInstance;
	// Mass override keeps Unreal's own bookkeeping consistent; the exact mass properties are written
	// straight to the physics body afterwards (see ApplyMassProperties).
	Instance.SetMassOverride(static_cast<float>(Seg.MassKg), true);
	Instance.SetInertiaConditioningEnabled(false); // no hidden inertia changes
	Instance.LinearDamping = 0.0f;                 // no hidden drag
	Instance.AngularDamping = 0.0f;

	GetOwner()->AddInstanceComponent(Body);
	Body->RegisterComponent();

	Instance.SetMaxAngularVelocityInRadians(MaxSegmentAngularVelocityRadPerS, /*bAddToCurrent=*/false);
	Instance.SetGyroscopicTorqueEnabled(bGyroscopicTorque);
	if (PositionIterations > 0 || VelocityIterations > 0)
	{
		Instance.SetOverrideIterationCounts(true);
		if (PositionIterations > 0)
		{
			Instance.SetPositionSolverIterationCount(static_cast<uint8>(PositionIterations));
		}
		if (VelocityIterations > 0)
		{
			Instance.SetVelocitySolverIterationCount(static_cast<uint8>(VelocityIterations));
		}
	}
	ApplyMassProperties(Body, Segment);
	if (!bCanSleep)
	{
		FPhysicsCommand::ExecuteWrite(Instance.GetPhysicsActor(), [](const FPhysicsActorHandle& Actor)
		{
			Actor->GetGameThreadAPI().SetSleepType(Chaos::ESleepType::NeverSleep);
		});
	}
	return Body;
}

void UAthletePhysicalBodyComponent::ApplyMassProperties(UShapeComponent* Body, EAthleteSegment Segment)
{
	// Write de Leva's mass, center of mass, and principal inertia directly into the Chaos body.
	//
	// Why not FBodyInstance's COMNudge / InertiaTensorScale? Measured and read from engine source
	// (BodyUtils.cpp, Chaos::Utilities::ScaleInertia): InertiaTensorScale is a geometric scale of an
	// "equivalent box", not a per-axis inertia multiplier, and a COMNudge also adds m*d^2 to the
	// inertia along the nudge axis. Neither produces a prescribed inertia.
	//
	// Caveat: anything that makes Unreal recompute this body's mass properties later (changing its
	// scale, mass scale, or welding it) would overwrite these values. This component never does.
	const FAthleteBodySegment& Seg = Model.GetSegment(Segment);
	const FVector ComLocalCm = AthleteUnits::MetersToUnreal(Seg.CenterOfMassM - GetCollisionShape(Segment).CenterM);
	const FVector InertiaKgCm2 = Seg.PrincipalInertiaKgM2 * KgCm2PerKgM2;
	const float MassKg = static_cast<float>(Seg.MassKg);

	// Principal axes = the body's own axes: every segment starts aligned with the body frame, and
	// de Leva's radii are given on those anatomical axes.
	FPhysicsCommand::ExecuteWrite(Body->BodyInstance.GetPhysicsActor(), [&](const FPhysicsActorHandle& Actor)
	{
		FPhysicsInterface::SetMass_AssumesLocked(Actor, MassKg);
		FPhysicsInterface::SetComLocalPose_AssumesLocked(Actor, FTransform(FQuat::Identity, ComLocalCm));
		FPhysicsInterface::SetMassSpaceInertiaTensor_AssumesLocked(Actor, InertiaKgCm2);
	});
}

void UAthletePhysicalBodyComponent::CreateJoint(EAthleteJoint Joint)
{
	const FAthleteJointSetup& Setup = GetJointSetup(Joint);
	UShapeComponent* Child = GetSegmentBody(AthleteJoints::GetChildSegment(Joint));
	UShapeComponent* Parent = GetSegmentBody(AthleteJoints::GetParentSegment(Joint));

	TUniquePtr<FConstraintInstance> Constraint = MakeUnique<FConstraintInstance>();
	Constraint->JointName = FName(AthleteJoints::GetName(Joint));

	// Frames are expressed relative to each body's transform. All bodies start aligned with the
	// body frame, so only positions differ; the parent frame carries the range-of-motion center.
	const FVector CenterCm = AthleteUnits::MetersToUnreal(Setup.CenterM);
	const FVector ChildCenterCm = AthleteUnits::MetersToUnreal(GetCollisionShape(AthleteJoints::GetChildSegment(Joint)).CenterM);
	const FVector ParentCenterCm = AthleteUnits::MetersToUnreal(GetCollisionShape(AthleteJoints::GetParentSegment(Joint)).CenterM);
	Constraint->SetRefFrame(EConstraintFrame::Frame1, FTransform(Setup.JointFrame, CenterCm - ChildCenterCm));
	Constraint->SetRefFrame(EConstraintFrame::Frame2, FTransform(Setup.GetParentFrame(), CenterCm - ParentCenterCm));

	// Ball joint: no translation, anatomical rotation limits.
	Constraint->SetLinearLimits(ELinearConstraintMotion::LCM_Locked, ELinearConstraintMotion::LCM_Locked, ELinearConstraintMotion::LCM_Locked, 0.0f);
	Constraint->SetAngularSwing1Limit(MotionFor(Setup.SwingFrontalHalfRangeDeg), static_cast<float>(Setup.SwingFrontalHalfRangeDeg));
	Constraint->SetAngularSwing2Limit(MotionFor(Setup.SwingSagittalHalfRangeDeg), static_cast<float>(Setup.SwingSagittalHalfRangeDeg));
	Constraint->SetAngularTwistLimit(MotionFor(Setup.TwistHalfRangeDeg), static_cast<float>(Setup.TwistHalfRangeDeg));

	FConstraintProfileProperties& Profile = Constraint->ProfileInstance;
	Profile.ConeLimit.bSoftConstraint = false;  // hard limits: bone and ligament stops
	Profile.TwistLimit.bSoftConstraint = false;
	Profile.bDisableCollision = true;           // jointed segments overlap at the joint by design
	Profile.bEnableProjection = false;          // no teleport fix-ups
	Profile.bEnableMassConditioning = false;    // no hidden mass/inertia scaling in the joint solve
	Profile.bEnableShockPropagation = false;
	Profile.bUseLinearJointSolver = false;      // the nonlinear solver is the accurate one

	// Body1 = child, Body2 = parent (Unreal's convention).
	Constraint->InitConstraint(&Child->BodyInstance, &Parent->BodyInstance, 1.0f, this);
	Joints.Add(MoveTemp(Constraint));
}

void UAthletePhysicalBodyComponent::IgnoreOverlappingPairs()
{
	IgnoredCollisionPairs.Reset();
	TMap<FPhysicsActorHandle, TArray<FPhysicsActorHandle>> Disabled;
	const FCollisionQueryParams Params(SCENE_QUERY_STAT(AthleteSelfOverlap), /*bTraceComplex=*/false);

	for (int32 A = 0; A < AthleteSegments::NumSegments; ++A)
	{
		for (int32 B = A + 1; B < AthleteSegments::NumSegments; ++B)
		{
			const EAthleteSegment SegmentA = AthleteSegments::FromIndex(A);
			const EAthleteSegment SegmentB = AthleteSegments::FromIndex(B);
			if (AreJointed(SegmentA, SegmentB))
			{
				continue; // handled by the joint's bDisableCollision
			}

			UShapeComponent* BodyA = SegmentBodies[A];
			UShapeComponent* BodyB = SegmentBodies[B];
			if (BodyA->ComponentOverlapComponent(BodyB, BodyB->GetComponentLocation(), BodyB->GetComponentQuat(), Params))
			{
				// Overlapping at rest means the pair is anatomically in contact (an arm hanging against
				// the trunk). Colliding them would push the body apart the moment physics starts.
				IgnoredCollisionPairs.Emplace(SegmentA, SegmentB);
				Disabled.FindOrAdd(BodyA->BodyInstance.GetPhysicsActor()).Add(BodyB->BodyInstance.GetPhysicsActor());
				Disabled.FindOrAdd(BodyB->BodyInstance.GetPhysicsActor()).Add(BodyA->BodyInstance.GetPhysicsActor());
			}
		}
	}

	if (!Disabled.IsEmpty())
	{
		// The same mechanism skeletal-mesh ragdolls use for their Physics Asset collision table.
		FPhysicsCommand::ExecuteWrite(GetWorld()->GetPhysicsScene(), [&Disabled]()
		{
			FChaosEngineInterface::AddDisabledCollisionsFor_AssumesLocked(Disabled);
		});
	}
}

void UAthletePhysicalBodyComponent::DestroyBody()
{
	for (TUniquePtr<FConstraintInstance>& Constraint : Joints)
	{
		Constraint->TermConstraint();
	}
	Joints.Reset();

	for (UShapeComponent* Body : SegmentBodies)
	{
		if (IsValid(Body))
		{
			Body->DestroyComponent();
		}
	}
	SegmentBodies.Reset();
	IgnoredCollisionPairs.Reset();
}

UShapeComponent* UAthletePhysicalBodyComponent::GetSegmentBody(EAthleteSegment Segment) const
{
	return SegmentBodies.IsValidIndex(AthleteSegments::ToIndex(Segment)) ? SegmentBodies[AthleteSegments::ToIndex(Segment)].Get() : nullptr;
}

FConstraintInstance* UAthletePhysicalBodyComponent::GetJointConstraint(EAthleteJoint Joint) const
{
	return Joints.IsValidIndex(AthleteJoints::ToIndex(Joint)) ? Joints[AthleteJoints::ToIndex(Joint)].Get() : nullptr;
}

double UAthletePhysicalBodyComponent::GetTotalMassKg() const
{
	double Mass = 0.0;
	for (const UShapeComponent* Body : SegmentBodies)
	{
		Mass += Body->BodyInstance.GetBodyMass();
	}
	return Mass;
}

FVector UAthletePhysicalBodyComponent::GetCenterOfMassM() const
{
	FVector Weighted = FVector::ZeroVector;
	double Mass = 0.0;
	for (const UShapeComponent* Body : SegmentBodies)
	{
		const double BodyMass = Body->BodyInstance.GetBodyMass();
		Weighted += Body->BodyInstance.GetCOMPosition() * BodyMass;
		Mass += BodyMass;
	}
	return AthleteUnits::UnrealToMeters(Weighted / Mass);
}

FVector UAthletePhysicalBodyComponent::GetLinearMomentumKgMps() const
{
	FVector Momentum = FVector::ZeroVector;
	for (const UShapeComponent* Body : SegmentBodies)
	{
		Momentum += AthleteUnits::UnrealToMeters(Body->GetPhysicsLinearVelocity()) * Body->BodyInstance.GetBodyMass();
	}
	return Momentum;
}

FMatrix UAthletePhysicalBodyComponent::GetSegmentWorldInertiaKgM2(EAthleteSegment Segment) const
{
	const UShapeComponent* Body = GetSegmentBody(Segment);
	const FBodyInstance& Instance = Body->BodyInstance;
	const FVector Principal = Instance.GetBodyInertiaTensor() / KgCm2PerKgM2;
	const FQuat WorldMassRotation = Body->GetComponentQuat() * Instance.GetMassSpaceLocal().GetRotation();

	// I_world = sum_i I_i * a_i a_i^T, with a_i the i-th principal axis in world space.
	FMatrix Inertia(ForceInitToZero);
	for (int32 Axis = 0; Axis < 3; ++Axis)
	{
		FVector Unit = FVector::ZeroVector;
		Unit[Axis] = 1.0;
		const FVector A = WorldMassRotation.RotateVector(Unit);
		for (int32 Row = 0; Row < 3; ++Row)
		{
			for (int32 Col = 0; Col < 3; ++Col)
			{
				Inertia.M[Row][Col] += Principal[Axis] * A[Row] * A[Col];
			}
		}
	}
	return Inertia;
}

FVector UAthletePhysicalBodyComponent::GetAngularMomentumAboutComKgM2ps() const
{
	// L = sum over bodies of (spin: I_world * omega) + (orbit: m * r x v), r relative to the whole-body CoM.
	const FVector Com = GetCenterOfMassM();
	FVector Momentum = FVector::ZeroVector;
	for (int32 Index = 0; Index < SegmentBodies.Num(); ++Index)
	{
		const UShapeComponent* Body = SegmentBodies[Index];
		const FMatrix I = GetSegmentWorldInertiaKgM2(AthleteSegments::FromIndex(Index));
		const FVector Omega = Body->GetPhysicsAngularVelocityInRadians();
		const FVector Spin(
			I.M[0][0] * Omega.X + I.M[0][1] * Omega.Y + I.M[0][2] * Omega.Z,
			I.M[1][0] * Omega.X + I.M[1][1] * Omega.Y + I.M[1][2] * Omega.Z,
			I.M[2][0] * Omega.X + I.M[2][1] * Omega.Y + I.M[2][2] * Omega.Z);
		const FVector R = AthleteUnits::UnrealToMeters(Body->BodyInstance.GetCOMPosition()) - Com;
		const FVector V = AthleteUnits::UnrealToMeters(Body->GetPhysicsLinearVelocity());
		Momentum += Spin + (R ^ V) * Body->BodyInstance.GetBodyMass();
	}
	return Momentum;
}

double UAthletePhysicalBodyComponent::GetKineticEnergyJ() const
{
	double Energy = 0.0;
	for (int32 Index = 0; Index < SegmentBodies.Num(); ++Index)
	{
		const UShapeComponent* Body = SegmentBodies[Index];
		const FMatrix I = GetSegmentWorldInertiaKgM2(AthleteSegments::FromIndex(Index));
		const FVector W = Body->GetPhysicsAngularVelocityInRadians();
		const FVector IW(
			I.M[0][0] * W.X + I.M[0][1] * W.Y + I.M[0][2] * W.Z,
			I.M[1][0] * W.X + I.M[1][1] * W.Y + I.M[1][2] * W.Z,
			I.M[2][0] * W.X + I.M[2][1] * W.Y + I.M[2][2] * W.Z);
		const FVector V = AthleteUnits::UnrealToMeters(Body->GetPhysicsLinearVelocity());
		Energy += 0.5 * Body->BodyInstance.GetBodyMass() * V.SizeSquared() + 0.5 * (W | IW);
	}
	return Energy;
}

double UAthletePhysicalBodyComponent::GetMaxJointSeparationM() const
{
	double MaxSeparationCm = 0.0;
	for (int32 Index = 0; Index < Joints.Num(); ++Index)
	{
		const EAthleteJoint Joint = AthleteJoints::FromIndex(Index);
		const FConstraintInstance& Constraint = *Joints[Index];
		const FVector ChildAnchor = GetSegmentBody(AthleteJoints::GetChildSegment(Joint))->GetComponentTransform().TransformPosition(Constraint.GetRefFrame(EConstraintFrame::Frame1).GetLocation());
		const FVector ParentAnchor = GetSegmentBody(AthleteJoints::GetParentSegment(Joint))->GetComponentTransform().TransformPosition(Constraint.GetRefFrame(EConstraintFrame::Frame2).GetLocation());
		MaxSeparationCm = FMath::Max(MaxSeparationCm, FVector::Dist(ChildAnchor, ParentAnchor));
	}
	return AthleteUnits::UnrealToMeters(MaxSeparationCm);
}

double UAthletePhysicalBodyComponent::GetSegmentLowestPointM(EAthleteSegment Segment) const
{
	// Exact geometry (not Unreal's bounding boxes, which are loose for tilted capsules).
	const FAthleteSegmentCollisionShape& Shape = GetCollisionShape(Segment);
	const FTransform& Transform = GetSegmentBody(Segment)->GetComponentTransform();
	double LowestCm;
	if (Shape.Type == EAthleteCollisionShapeType::Box)
	{
		// Lowest of the 8 corners.
		const FVector Half = AthleteUnits::MetersToUnreal(Shape.BoxHalfExtentsM);
		LowestCm = TNumericLimits<double>::Max();
		for (int32 Corner = 0; Corner < 8; ++Corner)
		{
			const FVector Local((Corner & 1) ? Half.X : -Half.X, (Corner & 2) ? Half.Y : -Half.Y, (Corner & 4) ? Half.Z : -Half.Z);
			LowestCm = FMath::Min(LowestCm, Transform.TransformPosition(Local).Z);
		}
	}
	else
	{
		// Lower of the two hemisphere centers, minus the radius.
		const double RadiusCm = AthleteUnits::MetersToUnreal(Shape.CapsuleRadiusM);
		const double SegmentHalfCm = AthleteUnits::MetersToUnreal(Shape.CapsuleHalfHeightM) - RadiusCm;
		const double EndA = Transform.TransformPosition(FVector(0, 0, SegmentHalfCm)).Z;
		const double EndB = Transform.TransformPosition(FVector(0, 0, -SegmentHalfCm)).Z;
		LowestCm = FMath::Min(EndA, EndB) - RadiusCm;
	}
	return AthleteUnits::UnrealToMeters(LowestCm);
}

double UAthletePhysicalBodyComponent::GetLowestPointM(EAthleteSegment* OutLowestSegment) const
{
	double Lowest = TNumericLimits<double>::Max();
	for (int32 Index = 0; Index < SegmentBodies.Num(); ++Index)
	{
		const EAthleteSegment Segment = AthleteSegments::FromIndex(Index);
		const double SegmentLowest = GetSegmentLowestPointM(Segment);
		if (SegmentLowest < Lowest)
		{
			Lowest = SegmentLowest;
			if (OutLowestSegment)
			{
				*OutLowestSegment = Segment;
			}
		}
	}
	return Lowest;
}

FVector UAthletePhysicalBodyComponent::GetJointCenterWorldM(EAthleteJoint Joint) const
{
	// The joint's anchor on the child segment (the two anchors coincide while the joint holds together).
	const FConstraintInstance& Constraint = *Joints[AthleteJoints::ToIndex(Joint)];
	return AthleteUnits::UnrealToMeters(GetSegmentBody(AthleteJoints::GetChildSegment(Joint))->GetComponentTransform().TransformPosition(Constraint.GetRefFrame(EConstraintFrame::Frame1).GetLocation()));
}

FAthleteJointAngles UAthletePhysicalBodyComponent::GetJointAngles(EAthleteJoint Joint) const
{
	const FConstraintInstance& Constraint = *Joints[AthleteJoints::ToIndex(Joint)];
	const FAthleteJointSetup& Setup = GetJointSetup(Joint);
	const FQuat ChildFrame = GetSegmentBody(AthleteJoints::GetChildSegment(Joint))->GetComponentQuat() * Constraint.GetRefFrame(EConstraintFrame::Frame1).GetRotation();
	const FQuat ParentFrame = GetSegmentBody(AthleteJoints::GetParentSegment(Joint))->GetComponentQuat() * Constraint.GetRefFrame(EConstraintFrame::Frame2).GetRotation();

	// Child frame relative to parent frame, split as Swing * Twist with twist about X.
	FQuat Relative = ParentFrame.Inverse() * ChildFrame;
	Relative.EnforceShortestArcWith(FQuat::Identity);
	FQuat Swing, Twist;
	Relative.ToSwingTwist(FVector::XAxisVector, Swing, Twist);

	FAthleteJointAngles Angles;
	FVector SwingAxis;
	double SwingAngle;
	Swing.ToAxisAndAngle(SwingAxis, SwingAngle);
	const double SwingDeg = FMath::RadiansToDegrees(SwingAngle);
	Angles.SwingSagittalDeg = SwingDeg * SwingAxis.Y;
	Angles.SwingFrontalDeg = SwingDeg * SwingAxis.Z;
	FVector TwistAxis;
	double TwistAngle;
	Twist.ToAxisAndAngle(TwistAxis, TwistAngle);
	Angles.TwistDeg = FMath::RadiansToDegrees(TwistAngle) * FMath::Sign(TwistAxis.X);

	// Swing limit. Both axes free: elliptical cone, allowed swing in direction (ny, nz) is
	// 1 / sqrt((ny/a)^2 + (nz/b)^2). One axis locked (a hinge): the free axis has a simple limit and
	// the locked axis's violation is simply its angle. Both locked: any swing is a violation.
	const double A = Setup.SwingSagittalHalfRangeDeg;
	const double B = Setup.SwingFrontalHalfRangeDeg;
	double SwingExcess;
	if (A > 0.0 && B > 0.0)
	{
		const double AllowedSwingDeg = 1.0 / FMath::Sqrt(FMath::Square(SwingAxis.Y / A) + FMath::Square(SwingAxis.Z / B));
		SwingExcess = SwingDeg - AllowedSwingDeg;
	}
	else if (A > 0.0)
	{
		SwingExcess = FMath::Max(FMath::Abs(Angles.SwingSagittalDeg) - A, FMath::Abs(Angles.SwingFrontalDeg));
	}
	else if (B > 0.0)
	{
		SwingExcess = FMath::Max(FMath::Abs(Angles.SwingFrontalDeg) - B, FMath::Abs(Angles.SwingSagittalDeg));
	}
	else
	{
		SwingExcess = SwingDeg;
	}
	const double TwistExcess = FMath::Abs(Angles.TwistDeg) - Setup.TwistHalfRangeDeg;
	Angles.LimitExcessDeg = FMath::Max3(0.0, SwingExcess, TwistExcess);
	return Angles;
}

void UAthletePhysicalBodyComponent::AddImpulseAtPoint(EAthleteSegment Segment, const FVector& ImpulseNs, const FVector& WorldPointM)
{
	// Unreal impulse units are kg*cm/s: 1 N*s = 100.
	GetSegmentBody(Segment)->AddImpulseAtLocation(AthleteUnits::MetersToUnreal(ImpulseNs), AthleteUnits::MetersToUnreal(WorldPointM));
}

void UAthletePhysicalBodyComponent::SetGravityEnabled(bool bEnabled)
{
	for (UShapeComponent* Body : SegmentBodies)
	{
		Body->SetEnableGravity(bEnabled);
	}
}

void UAthletePhysicalBodyComponent::ResetToReferencePose()
{
	// Same placement as when the segments were created: each at its shape center in the body frame.
	const FTransform& BodyFrame = GetComponentTransform();
	for (int32 Index = 0; Index < SegmentBodies.Num(); ++Index)
	{
		UShapeComponent* Body = SegmentBodies[Index];
		Body->SetWorldLocationAndRotation(BodyFrame.TransformPosition(AthleteUnits::MetersToUnreal(CollisionShapes[Index].CenterM)), BodyFrame.GetRotation(),
			false, nullptr, ETeleportType::ResetPhysics);
		Body->SetPhysicsLinearVelocity(FVector::ZeroVector);
		Body->SetPhysicsAngularVelocityInRadians(FVector::ZeroVector);
	}
}
