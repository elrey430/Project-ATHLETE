// Project ATHLETE

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "AthleteLabPhysicsProbe.generated.h"

class UStaticMeshComponent;
class FAthleteTelemetryTable;

/**
 * A lab instrument: a single simulated rigid body with a known mass that records its own
 * kinematics every frame and can draw them.
 *
 * It is NOT an athlete and contains no athlete logic. Its job is to prove, before any athlete
 * exists, that the pipeline "Chaos physics -> measurement -> telemetry CSV -> debug view" works
 * and matches known physics (e.g. free fall accelerates at the configured gravity).
 *
 * Telemetry columns (SI units): time_s, pos_{x,y,z}_m, vel_{x,y,z}_mps, speed_mps, acc_{x,y,z}_mps2
 * Acceleration is a finite difference of consecutive velocity samples, so the first row is NaN.
 */
UCLASS()
class PROJECTATHLETE_API AAthleteLabPhysicsProbe : public AActor
{
	GENERATED_BODY()

public:
	AAthleteLabPhysicsProbe();

	virtual void OnConstruction(const FTransform& Transform) override;
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;

	UStaticMeshComponent* GetBody() const { return Body; }

	/** Null if telemetry is disabled or play has not begun. */
	TSharedPtr<FAthleteTelemetryTable> GetTelemetryTable() const { return TelemetryTable; }

protected:
	/** The simulated rigid body. Component = a reusable piece attached to an Actor. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Probe")
	TObjectPtr<UStaticMeshComponent> Body;

	/** Mass of the body. Overrides Unreal's volume-based mass so experiments use a known value. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Probe", meta = (ClampMin = "0.1", Units = "kg"))
	float MassKg = 100.0f;

private:
	TSharedPtr<FAthleteTelemetryTable> TelemetryTable;

	FVector PreviousVelocityMps = FVector::ZeroVector;
	bool bHasPreviousSample = false;
};
