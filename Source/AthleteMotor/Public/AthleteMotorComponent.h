// Project ATHLETE

#pragma once

#include "CoreMinimal.h"
#include "Balance/AthleteBalanceController.h"
#include "Capability/AthleteMotorSkill.h"
#include "Capability/AthleteStrength.h"
#include "Components/ActorComponent.h"
#include "Muscle/AthleteMuscleModel.h"
#include "Muscle/AthleteMuscleSimCallback.h"
#include "Locomotion/AthleteGaitController.h"
#include "Locomotion/AthleteMovementIntent.h"
#include "AthleteMotorComponent.generated.h"

class UAthletePhysicalBodyComponent;

/** What the athlete's motor system is trying to do. */
UENUM(BlueprintType)
enum class EAthleteMotorState : uint8
{
	/** On his feet: balance and posture control are active. */
	Standing,
	/**
	 * Down: his center of mass has dropped too far to be a stance he can hold. Balance and posture
	 * control stop (trying to hold body parts upright in space while lying on the ground only makes
	 * muscles fight the ground); the muscles hold their reference joint angles with normal tone.
	 * Getting up is Milestone 7.
	 */
	Fallen,
};

/**
 * The athlete's motor system (Milestone 3: standing balance).
 *
 * Two layers, mirroring biology:
 *  1. MUSCLES: every joint produces a torque-limited spring-damper torque, computed and applied
 *     inside the physics solver on every substep (FAthleteMuscleSimCallback), equal and opposite
 *     on the two segments it connects. Stiffness/damping come from the body's own loads and the
 *     athlete's motor skill; the torque limit from his strength.
 *  2. NEURAL CONTROL: once per frame, the balance controller senses the body (with the athlete's
 *     reaction delay), computes a posture, and moves the muscles' TARGETS. It never touches the body.
 *
 * The only things that can move the body are its joint torques, gravity, the ground, and
 * whatever hits it. Nothing locks it upright.
 *
 * Needs a UAthletePhysicalBodyComponent on the same actor; activates once that body is built.
 */
UCLASS(ClassGroup = (Athlete), meta = (BlueprintSpawnableComponent))
class ATHLETEMOTOR_API UAthleteMotorComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UAthleteMotorComponent();

	/** Balance control on/off. Off = muscles hold the reference pose with no postural corrections. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor")
	bool bBalanceEnabled = true;

	/** Used when the body has no athlete definition (e.g. tests); otherwise taken from the athlete. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor")
	FAthleteMotorSkill MotorSkill;

	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	/** Sets up the muscles. Called automatically on the first tick after the body is built. */
	bool InitializeMuscles(const FAthleteStrengthProfile& Strength, const FAthleteMotorSkill& Skill);
	bool IsInitialized() const { return bMusclesInitialized; }

	/** Sets the muscles' targets (the body still has to get there physically). DeltaTimeS: time since the last posture, to command planned movements' velocity (0 = hold). */
	void ApplyPosture(const FAthletePosture& Posture, double DeltaTimeS = 0.0);

	const FAthleteJointImpedance& GetJointImpedance(EAthleteJoint Joint) const { return Impedances[AthleteJoints::ToIndex(Joint)]; }
	const FAthleteBalanceCommand& GetLastCommand() const { return LastCommand; }
	EAthleteMotorState GetMotorState() const { return MotorState; }

	/**
	 * What he wants to do with his body (Milestone 4: where to move). The motor system tries; the
	 * body decides. Zero velocity = stand.
	 */
	UFUNCTION(BlueprintCallable, Category = "Motor")
	void SetMovementIntent(const FAthleteMovementIntent& Intent) { MovementIntent = Intent; }
	const FAthleteMovementIntent& GetMovementIntent() const { return MovementIntent; }
	const FAthleteGaitState& GetGaitState() const { return Gait.GetState(); }

	/**
	 * He counts as down once his perceived center of mass is below this fraction of its standing
	 * height above the feet. Well below anything a recovered push or a full hip strategy reaches.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor", meta = (ClampMin = "0.1", ClampMax = "0.95"))
	double FallenComHeightFraction = 0.7;

	/** Muscle torque at a joint in the most recent completed physics substep (world frame, N*m, acting on the child segment). */
	FVector GetJointTorqueNm(EAthleteJoint Joint) const;

	/** Muscle error at a joint in the most recent completed substep: rotation still needed to reach the target, along the joint's anatomical axes (X twist, Y sagittal, Z frontal; degrees). */
	FVector GetJointErrorDeg(EAthleteJoint Joint) const;

	// --- Performance telemetry (the most recent tick) ---
	/** Physics substeps whose muscle outputs arrived this tick (normally the substeps per frame). */
	int32 GetSubstepsLastTick() const { return SubstepsLastRead; }
	/** Physics-thread time spent computing muscle torques over those substeps (ms). */
	double GetMuscleMsLastTick() const { return MuscleMsLastRead; }
	/** Game-thread time spent in this component's tick: sensing, balance, gait, posture (ms). */
	double GetControlMsLastTick() const { return ControlMsLastTick; }

	/** Current sensing of the body, undelayed (world frame). */
	FAthleteBalanceSensing SenseNow() const;

protected:
	virtual void OnUnregister() override;

private:
	UAthletePhysicalBodyComponent* FindBody() const;
	void DisableSolverDrive(EAthleteJoint Joint);
	void PushMuscleCommands();
	void ReadMuscleOutputs();
	void ReleaseMuscles();

	TArray<FAthleteJointImpedance> Impedances;
	TArray<FAthleteMuscleCommand> MuscleCommands; // one per joint, sent to the physics thread every frame
	TArray<bool> bPlannedLastFrame;               // per joint: its target was a planned movement last frame
	TArray<FVector> LastJointTorquesNm;
	TArray<FVector> LastJointErrorsRad;
	FAthleteMuscleSimCallback* MuscleCallback = nullptr; // owned by the Chaos solver
	TArray<FAthleteBalanceSensing> SensingHistory; // oldest first, for the reaction delay
	FAthleteBalanceCommand LastCommand;
	FAthleteFootSupport FootSupport;
	EAthleteMotorState MotorState = EAthleteMotorState::Standing;
	FAthleteMovementIntent MovementIntent;
	FAthleteGaitController Gait;
	double StandingComHeightM = 0.0; // center of mass above the ground when the muscles came on
	double GroundHeightM = 0.0;      // the ground his feet stood on then (flat ground assumed)
	bool bMusclesInitialized = false;
	int32 SubstepsLastRead = 0;
	double MuscleMsLastRead = 0.0;
	double ControlMsLastTick = 0.0;
};
