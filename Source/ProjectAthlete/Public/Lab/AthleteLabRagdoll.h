// Project ATHLETE

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "AthleteLabRagdoll.generated.h"

class UAthleteDefinition;
class UAthletePhysicalBodyComponent;
class FAthleteTelemetryTable;

UENUM(BlueprintType)
enum class EAthleteLabRagdollScenario : uint8
{
	/** Released standing: with no muscles yet, the body crumples. */
	Collapse,
	/** Released standing, then struck at the chest with a known impulse. */
	Push,
	/** Released tilted, from a height. */
	Drop,
};

/**
 * Lab experiment: one athlete's physical body (Milestone 2, passive: no muscles, no balance)
 * in a repeatable scenario, recorded to telemetry every frame.
 *
 * Telemetry table "Ragdoll_<name>" (SI units): time_s, com_{x,y,z}_m, com_vel_{x,y,z}_mps,
 * ang_mom_{x,y,z}_kg_m2ps, kinetic_energy_J, max_joint_separation_mm, lowest_point_m,
 * max_limit_excess_deg.
 *
 * Debug: athlete.Debug.CenterOfMass 1 (CoM and its floor projection),
 *        athlete.Debug.Kinematics 1 (CoM velocity arrow).
 */
UCLASS()
class PROJECTATHLETE_API AAthleteLabRagdoll : public AActor
{
	GENERATED_BODY()

public:
	AAthleteLabRagdoll();

	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;

	UAthletePhysicalBodyComponent* GetPhysicalBody() const { return PhysicalBody; }

protected:
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ragdoll")
	TObjectPtr<UAthletePhysicalBodyComponent> PhysicalBody;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ragdoll")
	TObjectPtr<UAthleteDefinition> Athlete;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ragdoll")
	EAthleteLabRagdollScenario Scenario = EAthleteLabRagdollScenario::Collapse;

	/**
	 * Push: impulse (N*s) delivered to the upper trunk, in the actor's local frame (X = athlete's
	 * forward). Default: 60 N*s from the front, about a 90 kg defender arriving at 0.7 m/s and
	 * stopping (an intentionally mild first test).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ragdoll|Push")
	FVector PushImpulseNs = FVector(-60.0, 0.0, 0.0);

	/** Push: simulated time after release at which the push lands. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ragdoll|Push", meta = (Units = "s", ClampMin = "0"))
	float PushTimeS = 0.1f;

	/** Drop: height of the feet above the floor at release. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ragdoll|Drop", meta = (Units = "m", ClampMin = "0"))
	float DropHeightM = 1.5f;

	/** Drop: forward tilt of the whole body at release. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ragdoll|Drop", meta = (Units = "deg"))
	float DropPitchDeg = 30.0f;

private:
	void RecordSample();

	TSharedPtr<FAthleteTelemetryTable> Telemetry;
	double ReleaseTimeS = 0.0;
	bool bPushed = false;
};
