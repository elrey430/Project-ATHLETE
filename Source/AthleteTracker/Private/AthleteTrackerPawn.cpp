// Project ATHLETE

#include "AthleteTrackerPawn.h"

#include "AthleteTrackerBundle.h"
#include "AthleteTrackerSim.h"
#include "Camera/CameraComponent.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Engine/Engine.h"
#include "Engine/LocalPlayer.h"
#include "Engine/StaticMesh.h"
#include "EnhancedInputComponent.h"
#include "EnhancedInputSubsystems.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/SpringArmComponent.h"
#include "HAL/PlatformTime.h"
#include "Misc/CommandLine.h"
#include "Misc/Paths.h"
#include "UnrealClient.h"
#include "InputAction.h"
#include "InputMappingContext.h"
#include "InputModifiers.h"
#include "UObject/ConstructorHelpers.h"

THIRD_PARTY_INCLUDES_START
#include <mujoco/mujoco.h>
THIRD_PARTY_INCLUDES_END

DEFINE_LOG_CATEGORY_STATIC(LogAthleteTrackerPawn, Log, All);

namespace
{
	constexpr double MetersToCm = 100.0;

	/** MuJoCo (right-handed, metres) to Unreal (left-handed, cm): Y flips, as in the MJCF exporter. */
	FVector PositionToUnreal(const double* P)
	{
		return FVector(P[0], -P[1], P[2]) * MetersToCm;
	}

	/** MuJoCo row-major rotation to Unreal: S R S with S = diag(1, -1, 1). */
	FQuat RotationToUnreal(const mjtNum* R)
	{
		FMatrix M = FMatrix::Identity;
		// Columns of S R S are the rotated axes; FMatrix rows hold axes (row-vector convention).
		const double C[3][3] = {
			{R[0], -R[3], R[6]},   // x axis
			{-R[1], R[4], -R[7]},  // y axis
			{R[2], -R[5], R[8]}};  // z axis
		for (int32 Axis = 0; Axis < 3; ++Axis)
		{
			M.M[Axis][0] = C[Axis][0];
			M.M[Axis][1] = C[Axis][1];
			M.M[Axis][2] = C[Axis][2];
		}
		return FQuat(M).GetNormalized();
	}

	UInstancedStaticMeshComponent* MakeShapes(AActor* Owner, const TCHAR* Name, UStaticMesh* Mesh, USceneComponent* Parent)
	{
		UInstancedStaticMeshComponent* Shapes = Owner->CreateDefaultSubobject<UInstancedStaticMeshComponent>(Name);
		Shapes->SetupAttachment(Parent);
		Shapes->SetStaticMesh(Mesh);
		Shapes->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		Shapes->SetUsingAbsoluteLocation(true);
		Shapes->SetUsingAbsoluteRotation(true);
		Shapes->SetUsingAbsoluteScale(true);
		Shapes->SetMobility(EComponentMobility::Movable);
		return Shapes;
	}
}

AAthleteTrackerPawn::AAthleteTrackerPawn()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickGroup = TG_PrePhysics;
	AutoPossessPlayer = EAutoReceiveInput::Disabled;

	Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	SetRootComponent(Root);
	CameraTarget = CreateDefaultSubobject<USceneComponent>(TEXT("CameraTarget"));
	CameraTarget->SetupAttachment(Root);
	CameraTarget->SetUsingAbsoluteLocation(true);
	CameraTarget->SetUsingAbsoluteRotation(true);
	SpringArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("SpringArm"));
	SpringArm->SetupAttachment(CameraTarget);
	SpringArm->TargetArmLength = 450.0f;
	SpringArm->SetRelativeRotation(FRotator(-15.0, 0.0, 0.0));
	SpringArm->bDoCollisionTest = false;
	SpringArm->bEnableCameraLag = true;
	SpringArm->CameraLagSpeed = 8.0f;
	Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
	Camera->SetupAttachment(SpringArm);

	static ConstructorHelpers::FObjectFinder<UStaticMesh> Cube(TEXT("/Engine/BasicShapes/Cube.Cube"));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> Sphere(TEXT("/Engine/BasicShapes/Sphere.Sphere"));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> Cylinder(TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
	Boxes = MakeShapes(this, TEXT("Boxes"), Cube.Object, Root);
	Spheres = MakeShapes(this, TEXT("Spheres"), Sphere.Object, Root);
	Cylinders = MakeShapes(this, TEXT("Cylinders"), Cylinder.Object, Root);
}

AAthleteTrackerPawn::~AAthleteTrackerPawn() = default;

void AAthleteTrackerPawn::BeginPlay()
{
	Super::BeginPlay();
	Origin = FVector(GetActorLocation().X, GetActorLocation().Y, 0.0);

	FString Error;
	const FString Directory = BundleDirectory.IsEmpty() ? FAthleteTrackerBundle::DefaultDirectory() : BundleDirectory;
	TSharedPtr<const FAthleteTrackerBundle> Bundle = FAthleteTrackerBundle::Load(Directory, Error);
	Sim = MakeShared<FAthleteTrackerSim>();
	if (!Bundle || !Sim->Initialize(Bundle, Error))
	{
		UE_LOG(LogAthleteTrackerPawn, Error, TEXT("Learned athlete unavailable: %s"), *Error);
		Sim.Reset();
		return;
	}

	// One instance per visible geom (body shapes; the translucent sole capsules are contact-only).
	const mjModel* M = Sim->Model();
	for (int32 G = 0; G < M->ngeom; ++G)
	{
		if (M->geom_bodyid[G] == 0 || M->geom_rgba[4 * G + 3] < 0.5f)
		{
			continue;
		}
		switch (M->geom_type[G])
		{
		case mjGEOM_BOX:
			GeomInstances.Add({G, 0, Boxes->AddInstance(FTransform::Identity, true)});
			break;
		case mjGEOM_SPHERE:
			GeomInstances.Add({G, 1, Spheres->AddInstance(FTransform::Identity, true)});
			break;
		case mjGEOM_CYLINDER:
			GeomInstances.Add({G, 2, Cylinders->AddInstance(FTransform::Identity, true)});
			break;
		case mjGEOM_CAPSULE:
			GeomInstances.Add({G, 2, Cylinders->AddInstance(FTransform::Identity, true)});
			GeomInstances.Add({G, 3, Spheres->AddInstance(FTransform::Identity, true)});
			GeomInstances.Add({G, 4, Spheres->AddInstance(FTransform::Identity, true)});
			break;
		default:
			break;
		}
	}
	FString Demo;
	if (FParse::Value(FCommandLine::Get(), TEXT("AthleteDemo="), Demo))
	{
		TArray<FString> Parts;
		Demo.ParseIntoArray(Parts, TEXT(","));
		if (Parts.Num() == 3)
		{
			CurrentCommand = {FCString::Atod(*Parts[0]), FCString::Atod(*Parts[1]), FCString::Atod(*Parts[2])};
			bDemo = true;
		}
	}
	FParse::Value(FCommandLine::Get(), TEXT("AthleteShotAt="), ShotAtS);
	Restart();
}

void AAthleteTrackerPawn::CreateInput()
{
	if (InputContext)
	{
		return;
	}
	MoveAction = NewObject<UInputAction>(this, TEXT("Move"));
	MoveAction->ValueType = EInputActionValueType::Axis2D;
	TurnAction = NewObject<UInputAction>(this, TEXT("Turn"));
	TurnAction->ValueType = EInputActionValueType::Axis1D;
	RunAction = NewObject<UInputAction>(this, TEXT("Run"));
	RunAction->ValueType = EInputActionValueType::Boolean;
	ResetAction = NewObject<UInputAction>(this, TEXT("Reset"));
	ResetAction->ValueType = EInputActionValueType::Boolean;

	InputContext = NewObject<UInputMappingContext>(this, TEXT("AthleteControls"));
	auto Swizzle = [this]() { return NewObject<UInputModifierSwizzleAxis>(this); }; // X -> Y
	auto Negate = [this]() { return NewObject<UInputModifierNegate>(this); };
	InputContext->MapKey(MoveAction, EKeys::W).Modifiers.Add(Swizzle());
	{
		FEnhancedActionKeyMapping& Back = InputContext->MapKey(MoveAction, EKeys::S);
		Back.Modifiers.Add(Swizzle());
		Back.Modifiers.Add(Negate());
	}
	InputContext->MapKey(MoveAction, EKeys::D);
	InputContext->MapKey(MoveAction, EKeys::A).Modifiers.Add(Negate());
	InputContext->MapKey(MoveAction, EKeys::Gamepad_Left2D);
	InputContext->MapKey(TurnAction, EKeys::Q);
	InputContext->MapKey(TurnAction, EKeys::E).Modifiers.Add(Negate());
	InputContext->MapKey(TurnAction, EKeys::Gamepad_RightX).Modifiers.Add(Negate());
	InputContext->MapKey(RunAction, EKeys::LeftShift);
	InputContext->MapKey(RunAction, EKeys::Gamepad_RightTrigger);
	InputContext->MapKey(ResetAction, EKeys::R);
	InputContext->MapKey(ResetAction, EKeys::Gamepad_FaceButton_Top);
}

void AAthleteTrackerPawn::PossessedBy(AController* NewController)
{
	Super::PossessedBy(NewController);
	CreateInput();
	if (const APlayerController* PC = Cast<APlayerController>(NewController))
	{
		if (UEnhancedInputLocalPlayerSubsystem* Subsystem = ULocalPlayer::GetSubsystem<UEnhancedInputLocalPlayerSubsystem>(PC->GetLocalPlayer()))
		{
			Subsystem->AddMappingContext(InputContext, 0);
		}
	}
}

void AAthleteTrackerPawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);
	CreateInput();
	if (UEnhancedInputComponent* Input = Cast<UEnhancedInputComponent>(PlayerInputComponent))
	{
		// Values are read each tick (GetBoundActionValue); the bindings array may grow, so no pointers kept.
		Input->BindActionValue(MoveAction);
		Input->BindActionValue(TurnAction);
		Input->BindActionValue(RunAction);
		Input->BindActionValue(ResetAction);
		bInputBound = true;
	}
}

void AAthleteTrackerPawn::ReadInput()
{
	const UEnhancedInputComponent* Input = Cast<UEnhancedInputComponent>(InputComponent);
	if (!bInputBound || !Input)
	{
		return;
	}
	const FVector2D Move = Input->GetBoundActionValue(MoveAction).Get<FVector2D>();
	const bool bRun = Input->GetBoundActionValue(RunAction).Get<bool>();
	// Forward only: the mocap has no walking backwards. S (or stick down) means stop.
	CurrentCommand.Forward = FMath::Clamp(Move.Y, 0.0, 1.0) * (bRun ? RunSpeedMps : WalkSpeedMps);
	CurrentCommand.Sideways = -FMath::Clamp(Move.X, -1.0, 1.0) * SidestepSpeedMps; // right on the stick = sideways right (MuJoCo: left +)
	CurrentCommand.Turn = FMath::Clamp(Input->GetBoundActionValue(TurnAction).Get<float>(), -1.0f, 1.0f) * TurnRateRadPerS;
	const bool bReset = Input->GetBoundActionValue(ResetAction).Get<bool>();
	if (bReset && !bResetHeld)
	{
		Restart();
	}
	bResetHeld = bReset;
}

void AAthleteTrackerPawn::Restart()
{
	if (!Sim)
	{
		return;
	}
	Sim->Reset();
	Accumulator = 0.0;
	FallenFor = 0.0;
	UpdateBody();
}

void AAthleteTrackerPawn::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	if (!Sim)
	{
		return;
	}
	if (!bDemo)
	{
		ReadInput();
	}
	ElapsedS += DeltaSeconds;
	if (ShotAtS > 0.0 && ElapsedS >= ShotAtS && !bShotTaken)
	{
		FScreenshotRequest::RequestScreenshot(FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("Screenshots"), TEXT("LearnedAthlete.png")), false, false);
		bShotTaken = true;
	}
	if (bShotTaken && ElapsedS >= ShotAtS + 1.0)
	{
		FGenericPlatformMisc::RequestExit(false);
	}

	// Fixed 10 ms control steps (the policy's rate), at most 10 per frame.
	Accumulator = FMath::Min(Accumulator + DeltaSeconds, 0.1);
	const double Start = FPlatformTime::Seconds();
	int32 Steps = 0;
	while (Accumulator >= Sim->ControlDt() && Steps < 10)
	{
		if (!Sim->HasFallen())
		{
			Sim->Step(CurrentCommand);
		}
		Accumulator -= Sim->ControlDt();
		++Steps;
	}
	if (Steps > 0)
	{
		LastStepMs = 1000.0 * (FPlatformTime::Seconds() - Start) / Steps;
	}
	if (Sim->HasFallen())
	{
		FallenFor += DeltaSeconds;
		if (FallenFor >= RestartAfterFallS)
		{
			Restart();
		}
	}
	UpdateBody();

	if (bShowDebugText && GEngine)
	{
		const FAthleteCommand& Shaped = Sim->ShapedCommand();
		GEngine->AddOnScreenDebugMessage(static_cast<uint64>(GetUniqueID()), 0.0f, Sim->HasFallen() ? FColor::Red : FColor::White,
			FString::Printf(TEXT("Learned athlete (MuJoCo in Unreal)  speed %.2f m/s  command fwd %.2f side %.2f turn %.2f  (shaped %.2f %.2f %.2f)  "
				"matcher jumps %d  %.2f ms/step%s\nW/S/A/D or left stick: move   Shift or RT: run   Q/E or right stick: turn   R or Y: reset"),
				Sim->ForwardSpeedMps(), CurrentCommand.Forward, CurrentCommand.Sideways, CurrentCommand.Turn,
				Shaped.Forward, Shaped.Sideways, Shaped.Turn, Sim->Matcher().Jumps(), LastStepMs,
				Sim->HasFallen() ? TEXT("  FELL - restarting") : TEXT("")));
	}
}

int32 AAthleteTrackerPawn::NumShapeInstances() const
{
	return Boxes->GetInstanceCount() + Spheres->GetInstanceCount() + Cylinders->GetInstanceCount();
}

bool AAthleteTrackerPawn::GetDrawnShape(int32 Geom, FTransform& OutWorld) const
{
	for (const FGeomInstance& Item : GeomInstances)
	{
		if (Item.Geom == Geom && Item.Kind <= 2)
		{
			const UInstancedStaticMeshComponent* Shapes = Item.Kind == 0 ? Boxes : Item.Kind == 1 ? Spheres : Cylinders;
			return Shapes->GetInstanceTransform(Item.Instance, OutWorld, true);
		}
	}
	return false;
}

void AAthleteTrackerPawn::UpdateBody()
{
	const mjModel* M = Sim->Model();
	const mjData* D = Sim->Data();
	for (const FGeomInstance& Item : GeomInstances)
	{
		const int32 G = Item.Geom;
		const double* Size = M->geom_size + 3 * G;
		const FQuat Rotation = RotationToUnreal(D->geom_xmat + 9 * G);
		FVector Location = Origin + PositionToUnreal(D->geom_xpos + 3 * G);
		FVector Scale;
		UInstancedStaticMeshComponent* Target = nullptr;
		switch (Item.Kind)
		{
		case 0: // box: the engine cube is 100 cm per side; MuJoCo sizes are half-extents in metres
			Scale = FVector(2.0 * Size[0], 2.0 * Size[1], 2.0 * Size[2]);
			Target = Boxes;
			break;
		case 1: // sphere: the engine sphere is 100 cm across
			Scale = FVector(2.0 * Size[0]);
			Target = Spheres;
			break;
		case 2: // cylinder (or a capsule's shaft): 100 cm across and tall, along local Z
			Scale = FVector(2.0 * Size[0], 2.0 * Size[0], 2.0 * Size[1]);
			Target = Cylinders;
			break;
		default: // capsule ends
		{
			const FVector Axis = Rotation.GetAxisZ();
			Location += Axis * (Item.Kind == 3 ? 1.0 : -1.0) * Size[1] * MetersToCm;
			Scale = FVector(2.0 * Size[0]);
			Target = Spheres;
			break;
		}
		}
		// The engine cylinder and cube are centred; the sphere too.
		Target->UpdateInstanceTransform(Item.Instance, FTransform(Rotation, Location, Scale), true, false, true);
	}
	Boxes->MarkRenderStateDirty();
	Spheres->MarkRenderStateDirty();
	Cylinders->MarkRenderStateDirty();

	// Camera: follow the pelvis, turning with the athlete's heading (lagged by the spring arm).
	const FVector Pelvis = Origin + FVector(D->qpos[0], -D->qpos[1], 0.9) * MetersToCm;
	CameraTarget->SetWorldLocation(Pelvis);
	const double YawDeg = -FMath::RadiansToDegrees(Sim->HeadingRad());
	const FRotator Current = CameraTarget->GetComponentRotation();
	const double Blend = FMath::Clamp(GetWorld() ? GetWorld()->GetDeltaSeconds() * 2.0 : 1.0, 0.0, 1.0);
	CameraTarget->SetWorldRotation(FRotator(0.0, Current.Yaw + FMath::FindDeltaAngleDegrees(Current.Yaw, YawDeg) * Blend, 0.0));
}
