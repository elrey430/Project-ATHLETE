// Project ATHLETE

#include "Lab/AthleteLabRagdoll.h"
#include "Articulation/AthletePhysicalBodyComponent.h"
#include "Components/ShapeComponent.h"
#include "Debug/AthleteDebugDraw.h"
#include "Definition/AthleteDefinition.h"
#include "Engine/World.h"
#include "PhysicsEngine/BodyInstance.h"
#include "Telemetry/AthleteTelemetrySubsystem.h"
#include "Units/AthleteUnits.h"

AAthleteLabRagdoll::AAthleteLabRagdoll()
{
	PrimaryActorTick.bCanEverTick = true;
	// Measure after physics has stepped this frame.
	PrimaryActorTick.TickGroup = TG_PostPhysics;

	PhysicalBody = CreateDefaultSubobject<UAthletePhysicalBodyComponent>(TEXT("PhysicalBody"));
	PhysicalBody->bBuildOnBeginPlay = false; // this actor decides where and how the body starts
	RootComponent = PhysicalBody;
}

void AAthleteLabRagdoll::BeginPlay()
{
	Super::BeginPlay();
	if (!Athlete)
	{
		return;
	}

	if (Scenario == EAthleteLabRagdollScenario::Drop)
	{
		// The body frame is the component's transform at build time: raise and tilt it first.
		PhysicalBody->AddWorldOffset(FVector(0.0, 0.0, AthleteUnits::MetersToUnreal(DropHeightM)));
		PhysicalBody->AddLocalRotation(FRotator(-DropPitchDeg, 0.0, 0.0));
	}

	PhysicalBody->Athlete = Athlete;
	if (!PhysicalBody->BuildBody())
	{
		return;
	}
	ReleaseTimeS = GetWorld()->GetTimeSeconds();

	if (UAthleteTelemetrySubsystem* TelemetrySubsystem = GetWorld()->GetSubsystem<UAthleteTelemetrySubsystem>())
	{
		Telemetry = TelemetrySubsystem->CreateTable(FName(*FString::Printf(TEXT("Ragdoll_%s"), *GetName())),
			{
				TEXT("time_s"),
				TEXT("com_x_m"), TEXT("com_y_m"), TEXT("com_z_m"),
				TEXT("com_vel_x_mps"), TEXT("com_vel_y_mps"), TEXT("com_vel_z_mps"),
				TEXT("ang_mom_x_kg_m2ps"), TEXT("ang_mom_y_kg_m2ps"), TEXT("ang_mom_z_kg_m2ps"),
				TEXT("kinetic_energy_J"), TEXT("max_joint_separation_mm"), TEXT("lowest_point_m"), TEXT("max_limit_excess_deg"),
			});
	}
}

void AAthleteLabRagdoll::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	if (!PhysicalBody->IsBuilt())
	{
		return;
	}

	const double Elapsed = GetWorld()->GetTimeSeconds() - ReleaseTimeS;
	if (Scenario == EAthleteLabRagdollScenario::Push && !bPushed && Elapsed >= PushTimeS)
	{
		// Strike the chest 10 cm above the upper trunk's center of mass.
		const FVector ChestM = AthleteUnits::UnrealToMeters(PhysicalBody->GetSegmentBody(EAthleteSegment::UpperTrunk)->BodyInstance.GetCOMPosition()) + FVector(0.0, 0.0, 0.1);
		PhysicalBody->AddImpulseAtPoint(EAthleteSegment::UpperTrunk, GetActorRotation().RotateVector(PushImpulseNs), ChestM);
		bPushed = true;
	}

	RecordSample();

	const UWorld* World = GetWorld();
	const FVector ComM = PhysicalBody->GetCenterOfMassM();
	const FVector ComCm = AthleteUnits::MetersToUnreal(ComM);
	const FVector ComVelocityMps = PhysicalBody->GetLinearMomentumKgMps() / PhysicalBody->GetTotalMassKg();
	AthleteDebug::DrawPoint(World, EAthleteDebugChannel::CenterOfMass, ComCm, FColor::Magenta);
	AthleteDebug::DrawLine(World, EAthleteDebugChannel::CenterOfMass, ComCm, FVector(ComCm.X, ComCm.Y, 0.0), FColor::Magenta);
	AthleteDebug::DrawArrow(World, EAthleteDebugChannel::Kinematics, ComCm, ComCm + ComVelocityMps * AthleteDebug::GetVelocityArrowScale(), FColor::Green);
}

void AAthleteLabRagdoll::RecordSample()
{
	if (!Telemetry)
	{
		return;
	}
	double MaxLimitExcessDeg = 0.0;
	for (int32 Index = 0; Index < AthleteJoints::NumJoints; ++Index)
	{
		MaxLimitExcessDeg = FMath::Max(MaxLimitExcessDeg, PhysicalBody->GetJointAngles(AthleteJoints::FromIndex(Index)).LimitExcessDeg);
	}
	const FVector Com = PhysicalBody->GetCenterOfMassM();
	const FVector ComVelocity = PhysicalBody->GetLinearMomentumKgMps() / PhysicalBody->GetTotalMassKg();
	const FVector AngularMomentum = PhysicalBody->GetAngularMomentumAboutComKgM2ps();

	Telemetry->AddRow({
		GetWorld()->GetTimeSeconds() - ReleaseTimeS,
		Com.X, Com.Y, Com.Z,
		ComVelocity.X, ComVelocity.Y, ComVelocity.Z,
		AngularMomentum.X, AngularMomentum.Y, AngularMomentum.Z,
		PhysicalBody->GetKineticEnergyJ(), PhysicalBody->GetMaxJointSeparationM() * 1000.0, PhysicalBody->GetLowestPointM(), MaxLimitExcessDeg,
	});
}
