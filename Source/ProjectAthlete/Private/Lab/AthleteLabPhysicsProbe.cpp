// Project ATHLETE

#include "Lab/AthleteLabPhysicsProbe.h"
#include "Components/StaticMeshComponent.h"
#include "Debug/AthleteDebugDraw.h"
#include "Engine/CollisionProfile.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "Telemetry/AthleteTelemetrySubsystem.h"
#include "Units/AthleteUnits.h"
#include "UObject/ConstructorHelpers.h"
#include <limits>

AAthleteLabPhysicsProbe::AAthleteLabPhysicsProbe()
{
	PrimaryActorTick.bCanEverTick = true;

	// Sample AFTER physics has stepped this frame so telemetry records the simulated result,
	// not last frame's state.
	PrimaryActorTick.TickGroup = TG_PostPhysics;

	Body = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Body"));
	RootComponent = Body;

	// Engine-provided 1 m cube. Placeholder geometry only.
	static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeMesh(TEXT("/Engine/BasicShapes/Cube.Cube"));
	if (CubeMesh.Succeeded())
	{
		Body->SetStaticMesh(CubeMesh.Object);
	}

	Body->SetCollisionProfileName(UCollisionProfile::PhysicsActor_ProfileName);
	Body->SetSimulatePhysics(true);
}

void AAthleteLabPhysicsProbe::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);

	Body->SetMassOverrideInKg(NAME_None, MassKg, /*bOverrideMass=*/true);

	// Unreal adds a small linear damping (0.01) to every body by default. That is a hidden,
	// non-physical drag force; a measurement instrument must not have it.
	// Real aerodynamic drag will be modeled explicitly when it matters (the football, Milestone 10).
	Body->SetLinearDamping(0.0f);
	Body->SetAngularDamping(0.0f);
}

void AAthleteLabPhysicsProbe::BeginPlay()
{
	Super::BeginPlay();

	if (UAthleteTelemetrySubsystem* Telemetry = GetWorld()->GetSubsystem<UAthleteTelemetrySubsystem>())
	{
		TelemetryTable = Telemetry->CreateTable(
			FName(*FString::Printf(TEXT("Probe_%s"), *GetName())),
			{
				TEXT("time_s"),
				TEXT("pos_x_m"), TEXT("pos_y_m"), TEXT("pos_z_m"),
				TEXT("vel_x_mps"), TEXT("vel_y_mps"), TEXT("vel_z_mps"),
				TEXT("speed_mps"),
				TEXT("acc_x_mps2"), TEXT("acc_y_mps2"), TEXT("acc_z_mps2"),
			});
	}
}

void AAthleteLabPhysicsProbe::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	const FVector CenterOfMassCm = Body->GetCenterOfMass();
	const FVector VelocityCmps = Body->GetPhysicsLinearVelocity();

	const FVector PositionM = AthleteUnits::UnrealToMeters(CenterOfMassCm);
	const FVector VelocityMps = AthleteUnits::UnrealToMeters(VelocityCmps);

	// Finite-difference acceleration. Undefined on the first sample; NaN says so honestly
	// instead of recording a fake zero.
	FVector AccelerationMps2(std::numeric_limits<double>::quiet_NaN());
	if (bHasPreviousSample && DeltaSeconds > 0.0f)
	{
		AccelerationMps2 = (VelocityMps - PreviousVelocityMps) / DeltaSeconds;
	}
	PreviousVelocityMps = VelocityMps;
	bHasPreviousSample = true;

	if (TelemetryTable)
	{
		TelemetryTable->AddRow({
			GetWorld()->GetTimeSeconds(),
			PositionM.X, PositionM.Y, PositionM.Z,
			VelocityMps.X, VelocityMps.Y, VelocityMps.Z,
			VelocityMps.Length(),
			AccelerationMps2.X, AccelerationMps2.Y, AccelerationMps2.Z,
		});
	}

	const UWorld* World = GetWorld();
	AthleteDebug::DrawPoint(World, EAthleteDebugChannel::CenterOfMass, CenterOfMassCm, FColor::Yellow);
	AthleteDebug::DrawArrow(World, EAthleteDebugChannel::Kinematics, CenterOfMassCm,
		CenterOfMassCm + VelocityMps * AthleteDebug::GetVelocityArrowScale(), FColor::Green);
}
