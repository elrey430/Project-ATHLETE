// Project ATHLETE

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "AthleteMotionMatcher.h"
#include "AthleteTrackerPawn.generated.h"

class FAthleteTrackerSim;
class UCameraComponent;
class UInputAction;
class UInputMappingContext;
class UInstancedStaticMeshComponent;
class USpringArmComponent;

/**
 * The learned athlete, playable: controller input -> command -> FAthleteTrackerSim (MuJoCo, 100 Hz fixed
 * steps) -> the body drawn from MuJoCo's geometry each frame, with a chase camera.
 *
 * Controls (keyboard / gamepad):
 *   W / left stick up      forward (walk 1.5 m/s; hold Shift or right trigger to run 3 m/s; the stick is analog)
 *   A, D / left stick      sidestep (0.5 m/s)
 *   Q, E / right stick     turn (1.2 rad/s)
 *   R / Y button           reset
 * The athlete starts where the pawn spawns (on the floor). MuJoCo's ground is the plane Z = 0.
 * Command line (demos, checks): -AthleteDemo=forward,sideways,turn drives it without a controller;
 * -AthleteShotAt=S saves Saved/Screenshots/LearnedAthlete.png at S seconds and quits a second later.
 */
UCLASS()
class ATHLETETRACKER_API AAthleteTrackerPawn : public APawn
{
	GENERATED_BODY()

public:
	AAthleteTrackerPawn();
	virtual ~AAthleteTrackerPawn() override;

	virtual void Tick(float DeltaSeconds) override;
	virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;
	virtual void PossessedBy(AController* NewController) override;

	/** Bundle folder (default: Saved/MuJoCo/unreal, written by Scripts/MuJoCo/export_unreal_bundle.py). */
	UPROPERTY(EditAnywhere, Category = "Athlete")
	FString BundleDirectory;

	UPROPERTY(EditAnywhere, Category = "Athlete|Controls")
	float WalkSpeedMps = 1.5f;

	UPROPERTY(EditAnywhere, Category = "Athlete|Controls")
	float RunSpeedMps = 3.0f;

	UPROPERTY(EditAnywhere, Category = "Athlete|Controls")
	float SidestepSpeedMps = 0.5f;

	UPROPERTY(EditAnywhere, Category = "Athlete|Controls")
	float TurnRateRadPerS = 1.2f;

	/** Seconds on the ground before the athlete is put back on its feet. */
	UPROPERTY(EditAnywhere, Category = "Athlete")
	float RestartAfterFallS = 2.0f;

	/** Show speed, command and step timing on screen. */
	UPROPERTY(EditAnywhere, Category = "Athlete")
	bool bShowDebugText = true;

	/** The command sent last step (after reading input; before the controller layer shapes it). */
	const FAthleteCommand& Command() const { return CurrentCommand; }

	/** The command to use when no player controls this pawn (tests, demos). Player input overrides it. */
	void SetCommand(const FAthleteCommand& InCommand) { CurrentCommand = InCommand; }

	/** The simulation (null if the bundle didn't load). */
	const FAthleteTrackerSim* Simulation() const { return Sim.Get(); }

	/** Drawn shapes: boxes, spheres, cylinders. */
	int32 NumShapeInstances() const;

	/** World transform of the shape drawn for MuJoCo geom Geom (a capsule's: its shaft). False if not drawn. */
	bool GetDrawnShape(int32 Geom, FTransform& OutWorld) const;

	/** Unreal world location of MuJoCo world point PointM (metres). */
	FVector ToWorld(const FVector& PointM) const { return Origin + FVector(PointM.X, -PointM.Y, PointM.Z) * 100.0; }

protected:
	virtual void BeginPlay() override;

private:
	void CreateInput();
	void ReadInput();
	void Restart();
	void UpdateBody();

	UPROPERTY(VisibleAnywhere)
	TObjectPtr<USceneComponent> Root;

	/** Follows the pelvis (position, and heading with lag): the camera hangs from it. */
	UPROPERTY(VisibleAnywhere)
	TObjectPtr<USceneComponent> CameraTarget;

	UPROPERTY(VisibleAnywhere)
	TObjectPtr<USpringArmComponent> SpringArm;

	UPROPERTY(VisibleAnywhere)
	TObjectPtr<UCameraComponent> Camera;

	UPROPERTY(VisibleAnywhere)
	TObjectPtr<UInstancedStaticMeshComponent> Boxes;

	UPROPERTY(VisibleAnywhere)
	TObjectPtr<UInstancedStaticMeshComponent> Spheres;

	UPROPERTY(VisibleAnywhere)
	TObjectPtr<UInstancedStaticMeshComponent> Cylinders;

	UPROPERTY(Transient)
	TObjectPtr<UInputMappingContext> InputContext;

	UPROPERTY(Transient)
	TObjectPtr<UInputAction> MoveAction;

	UPROPERTY(Transient)
	TObjectPtr<UInputAction> TurnAction;

	UPROPERTY(Transient)
	TObjectPtr<UInputAction> RunAction;

	UPROPERTY(Transient)
	TObjectPtr<UInputAction> ResetAction;

	TSharedPtr<FAthleteTrackerSim> Sim; // shared: the deleter is captured where the full type is known
	FAthleteCommand CurrentCommand;
	FVector Origin = FVector::ZeroVector;
	double Accumulator = 0.0;
	double FallenFor = 0.0;
	double LastStepMs = 0.0;
	bool bResetHeld = false;
	bool bInputBound = false;
	bool bDemo = false;
	double ElapsedS = 0.0;
	double ShotAtS = -1.0;
	bool bShotTaken = false;

	struct FGeomInstance
	{
		int32 Geom;
		int32 Kind; // 0 box, 1 sphere, 2 cylinder, 3 capsule end (+axis), 4 capsule end (-axis)
		int32 Instance;
	};
	TArray<FGeomInstance> GeomInstances;
};
