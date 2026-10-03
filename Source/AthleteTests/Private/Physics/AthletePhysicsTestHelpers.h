// Project ATHLETE

#pragma once

#include "CoreMinimal.h"
#include "Anatomy/AthleteBodyModel.h"
#include "Anatomy/AthleteMobility.h"
#include "Articulation/AthletePhysicalBodyComponent.h"
#include "AthleteMotorComponent.h"
#include "Components/BoxComponent.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "HAL/PlatformTime.h"
#include "Tests/AutomationCommon.h"
#include "Units/AthleteUnits.h"

#if WITH_DEV_AUTOMATION_TESTS

/**
 * A test world containing (optionally) a large static floor with its top at Z = 0, and one
 * athlete body standing in the reference pose on it. Ticks at a fixed frame time.
 */
class FAthletePhysicsTestScene
{
public:
	static constexpr float FrameDeltaSeconds = 1.0f / 60.0f;

	bool Initialize(FAutomationTestBase& Test, bool bWithFloor, const FAthleteMorphology& Morphology = FAthleteMorphology(), const FVector& BodyOriginM = FVector::ZeroVector,
		int32 PositionIterations = 0, int32 VelocityIterations = 0, bool bGyroscopicTorque = true, const FRotator& BodyRotation = FRotator::ZeroRotator)
	{
		if (!TestWorld.CreateTestWorld(EWorldType::Game))
		{
			Test.AddError(TEXT("Could not create test world"));
			return false;
		}
		UWorld* World = TestWorld.GetTestWorld();

		if (bWithFloor)
		{
			// 100 m x 100 m, 1 m thick, top surface at Z = 0.
			AActor* Floor = World->SpawnActor<AActor>();
			UBoxComponent* FloorBox = NewObject<UBoxComponent>(Floor, TEXT("Floor"));
			FloorBox->InitBoxExtent(FVector(5000.0, 5000.0, 50.0));
			FloorBox->SetCollisionProfileName(TEXT("BlockAll"));
			Floor->SetRootComponent(FloorBox);
			FloorBox->SetWorldLocation(FVector(0.0, 0.0, -50.0));
			FloorBox->RegisterComponent();
		}

		BodyActor = World->SpawnActor<AActor>(AthleteUnits::MetersToUnreal(BodyOriginM), FRotator::ZeroRotator);
		Body = NewObject<UAthletePhysicalBodyComponent>(BodyActor, TEXT("PhysicalBody"));
		Body->bBuildOnBeginPlay = false;
		Body->PositionIterations = PositionIterations;
		Body->VelocityIterations = VelocityIterations;
		Body->bGyroscopicTorque = bGyroscopicTorque;
		BodyActor->SetRootComponent(Body);
		Body->SetWorldLocationAndRotation(AthleteUnits::MetersToUnreal(BodyOriginM), BodyRotation);
		Body->RegisterComponent();

		if (!TestWorld.BeginPlayInTestWorld())
		{
			Test.AddError(TEXT("BeginPlay failed"));
			return false;
		}

		FAthleteBodyModel Model;
		FString Error;
		if (!FAthleteBodyModel::Build(Morphology, Model, &Error))
		{
			Test.AddError(Error);
			return false;
		}
		if (!Body->BuildBodyFrom(Model, FAthleteMobilityProfile()))
		{
			Test.AddError(TEXT("BuildBodyFrom failed"));
			return false;
		}
		return true;
	}

	/** Adds another athlete body (after Initialize), standing in the reference pose at OriginM. */
	UAthletePhysicalBodyComponent* AddBody(FAutomationTestBase& Test, const FVector& OriginM, const FRotator& Rotation = FRotator::ZeroRotator,
		const FAthleteMorphology& Morphology = FAthleteMorphology())
	{
		AActor* Actor = GetWorld()->SpawnActor<AActor>(AthleteUnits::MetersToUnreal(OriginM), Rotation);
		UAthletePhysicalBodyComponent* NewBody = NewObject<UAthletePhysicalBodyComponent>(Actor, TEXT("PhysicalBody"));
		NewBody->bBuildOnBeginPlay = false;
		Actor->SetRootComponent(NewBody);
		NewBody->SetWorldLocationAndRotation(AthleteUnits::MetersToUnreal(OriginM), Rotation);
		NewBody->RegisterComponent();
		FAthleteBodyModel Model;
		FString Error;
		if (!FAthleteBodyModel::Build(Morphology, Model, &Error) || !NewBody->BuildBodyFrom(Model, FAthleteMobilityProfile()))
		{
			Test.AddError(FString::Printf(TEXT("Could not build an extra body: %s"), *Error));
			return nullptr;
		}
		return NewBody;
	}

	/** Adds muscles (and optionally balance control) to the body. */
	UAthleteMotorComponent* AddMotor(const FAthleteStrengthProfile& Strength, const FAthleteMotorSkill& Skill, bool bBalance)
	{
		return AddMotorTo(Body, Strength, Skill, bBalance);
	}

	/** Adds muscles (and optionally balance control) to any body in the scene. */
	static UAthleteMotorComponent* AddMotorTo(UAthletePhysicalBodyComponent* TargetBody, const FAthleteStrengthProfile& Strength, const FAthleteMotorSkill& Skill, bool bBalance)
	{
		UAthleteMotorComponent* Motor = NewObject<UAthleteMotorComponent>(TargetBody->GetOwner(), TEXT("Motor"));
		Motor->bBalanceEnabled = bBalance;
		Motor->MotorSkill = Skill;
		Motor->RegisterComponent();
		Motor->InitializeMuscles(Strength, Skill);
		return Motor;
	}

	/** Ticks the world; returns the mean wall-clock milliseconds per tick. */
	double Simulate(double Seconds)
	{
		const int32 Frames = FMath::RoundToInt(Seconds / FrameDeltaSeconds);
		const double Start = FPlatformTime::Seconds();
		for (int32 Frame = 0; Frame < Frames; ++Frame)
		{
			TestWorld.TickTestWorld(FrameDeltaSeconds);
		}
		return Frames > 0 ? 1000.0 * (FPlatformTime::Seconds() - Start) / Frames : 0.0;
	}

	UWorld* GetWorld() const { return TestWorld.GetTestWorld(); }

	~FAthletePhysicsTestScene()
	{
		if (TestWorld.GetTestWorld() && TestWorld.GetTestWorld()->HasBegunPlay())
		{
			TestWorld.EndPlayInTestWorld();
		}
	}

	FTestWorldWrapper TestWorld;
	AActor* BodyActor = nullptr;
	UAthletePhysicalBodyComponent* Body = nullptr;
};

#endif // WITH_DEV_AUTOMATION_TESTS
