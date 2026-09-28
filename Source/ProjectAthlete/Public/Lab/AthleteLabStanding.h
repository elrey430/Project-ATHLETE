// Project ATHLETE

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "AthleteLabStanding.generated.h"

class UAthleteDefinition;
class UAthleteMotorComponent;
class UAthletePhysicalBodyComponent;
class FAthleteTelemetryTable;

UENUM(BlueprintType)
enum class EAthleteLabStandingScenario : uint8
{
	/** Muscles and balance on: stands on his own. */
	QuietStanding,
	/** Stands, then is pushed at the pelvis with a known impulse. */
	Push,
	/** Muscles hold their reference angles, no neural control: the stack of segments collapses. */
	NoNeuralControl,
};

/**
 * Lab experiment (Milestone 3): one athlete standing by his own muscles and balance control, in a
 * repeatable scenario, recorded to telemetry every frame.
 *
 * Telemetry table "Standing_<name>" (SI units): time_s, com_{x,y,z}_m, com_vel_{x,y,z}_mps,
 * xcom_err_fwd_m, xcom_err_right_m, lean_fwd_deg, lean_right_deg, hip_flex_deg,
 * ankle_torque_{l,r}_Nm, knee_torque_l_Nm, hip_torque_{l,r}_Nm, lumbar_torque_Nm, kinetic_energy_J,
 * fallen (1 once his motor system has concluded he is down and stopped balancing).
 * Joint torques are the muscle torque magnitudes of the most recent physics substep.
 *
 * Debug: athlete.Debug.Balance 1 (support center yellow, extrapolated CoM cyan, on the floor),
 *        athlete.Debug.CenterOfMass 1, athlete.Debug.Kinematics 1.
 */
UCLASS()
class PROJECTATHLETE_API AAthleteLabStanding : public AActor
{
	GENERATED_BODY()

public:
	AAthleteLabStanding();

	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;

protected:
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Standing")
	TObjectPtr<UAthletePhysicalBodyComponent> PhysicalBody;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Standing")
	TObjectPtr<UAthleteMotorComponent> Motor;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Standing")
	TObjectPtr<UAthleteDefinition> Athlete;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Standing")
	EAthleteLabStandingScenario Scenario = EAthleteLabStandingScenario::QuietStanding;

	/**
	 * Push: total impulse (N*s) at the pelvis, in the actor's local frame (X = athlete's forward),
	 * spread over PushDurationS. Default 10 N*s from behind: inside the measured recovery limit
	 * (about 15 N*s forward for the reference athlete).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Standing|Push")
	FVector PushImpulseNs = FVector(10.0, 0.0, 0.0);

	/** Push: when it starts, after release. Quiet standing settles in about 2.5 s (the reference
	 * pose starts with the CoM over the ankles and glides forward over the feet); a push during
	 * that transient finds him mid-sway and is harder to recover from. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Standing|Push", meta = (Units = "s", ClampMin = "0"))
	float PushTimeS = 3.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Standing|Push", meta = (Units = "s", ClampMin = "0.01"))
	float PushDurationS = 0.1f;

private:
	void RecordSample();
	void DrawDebug() const;

	TSharedPtr<FAthleteTelemetryTable> Telemetry;
	double ReleaseTimeS = 0.0;
	double PushedNs = 0.0;
};
