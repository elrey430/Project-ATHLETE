// Project ATHLETE

#include "Lab/AthleteLabStanding.h"
#include "AthleteMotorComponent.h"
#include "Articulation/AthletePhysicalBodyComponent.h"
#include "Components/ShapeComponent.h"
#include "Debug/AthleteDebugDraw.h"
#include "Definition/AthleteDefinition.h"
#include "Engine/World.h"
#include "PhysicsEngine/BodyInstance.h"
#include "Telemetry/AthleteTelemetrySubsystem.h"
#include "Units/AthleteUnits.h"

AAthleteLabStanding::AAthleteLabStanding()
{
	PrimaryActorTick.bCanEverTick = true;
	// Measure after physics has stepped this frame.
	PrimaryActorTick.TickGroup = TG_PostPhysics;

	PhysicalBody = CreateDefaultSubobject<UAthletePhysicalBodyComponent>(TEXT("PhysicalBody"));
	PhysicalBody->bBuildOnBeginPlay = false; // built in BeginPlay once the athlete is assigned
	RootComponent = PhysicalBody;

	Motor = CreateDefaultSubobject<UAthleteMotorComponent>(TEXT("Motor"));
}

void AAthleteLabStanding::BeginPlay()
{
	Super::BeginPlay();
	if (!Athlete)
	{
		return;
	}
	Motor->bBalanceEnabled = Scenario != EAthleteLabStandingScenario::NoNeuralControl;

	PhysicalBody->Athlete = Athlete;
	if (!PhysicalBody->BuildBody())
	{
		return;
	}
	ReleaseTimeS = GetWorld()->GetTimeSeconds();

	if (UAthleteTelemetrySubsystem* TelemetrySubsystem = GetWorld()->GetSubsystem<UAthleteTelemetrySubsystem>())
	{
		Telemetry = TelemetrySubsystem->CreateTable(FName(*FString::Printf(TEXT("Standing_%s"), *GetName())),
			{
				TEXT("time_s"),
				TEXT("com_x_m"), TEXT("com_y_m"), TEXT("com_z_m"),
				TEXT("com_vel_x_mps"), TEXT("com_vel_y_mps"), TEXT("com_vel_z_mps"),
				TEXT("xcom_err_fwd_m"), TEXT("xcom_err_right_m"),
				TEXT("lean_fwd_deg"), TEXT("lean_right_deg"), TEXT("hip_flex_deg"),
				TEXT("ankle_torque_l_Nm"), TEXT("ankle_torque_r_Nm"), TEXT("knee_torque_l_Nm"),
				TEXT("hip_torque_l_Nm"), TEXT("hip_torque_r_Nm"), TEXT("lumbar_torque_Nm"),
				TEXT("kinetic_energy_J"), TEXT("fallen"),
			});
	}
}

void AAthleteLabStanding::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	if (!PhysicalBody->IsBuilt())
	{
		return;
	}

	// Push: a constant force at the pelvis for PushDurationS, delivered as this frame's share of
	// the impulse (it acts during the next physics step).
	const double Elapsed = GetWorld()->GetTimeSeconds() - ReleaseTimeS;
	const double Total = PushImpulseNs.Size();
	if (Scenario == EAthleteLabStandingScenario::Push && Total > 0.0 && Elapsed >= PushTimeS && PushedNs < Total)
	{
		const double Share = FMath::Min(Total * DeltaSeconds / PushDurationS, Total - PushedNs);
		const FVector PelvisM = AthleteUnits::UnrealToMeters(PhysicalBody->GetSegmentBody(EAthleteSegment::LowerTrunk)->BodyInstance.GetCOMPosition());
		PhysicalBody->AddImpulseAtPoint(EAthleteSegment::LowerTrunk, GetActorRotation().RotateVector(PushImpulseNs / Total * Share), PelvisM);
		PushedNs += Share;
	}

	RecordSample();
	DrawDebug();
}

void AAthleteLabStanding::RecordSample()
{
	if (!Telemetry)
	{
		return;
	}
	const FVector Com = PhysicalBody->GetCenterOfMassM();
	const FVector ComVelocity = PhysicalBody->GetLinearMomentumKgMps() / PhysicalBody->GetTotalMassKg();
	const FAthleteBalanceCommand& Command = Motor->GetLastCommand();
	auto Torque = [this](EAthleteJoint Joint) { return Motor->GetJointTorqueNm(Joint).Size(); };

	Telemetry->AddRow({
		GetWorld()->GetTimeSeconds() - ReleaseTimeS,
		Com.X, Com.Y, Com.Z,
		ComVelocity.X, ComVelocity.Y, ComVelocity.Z,
		Command.XcomErrorForwardM, Command.XcomErrorRightM,
		Command.LeanForwardDeg, Command.LeanRightDeg, Command.HipFlexionDeg,
		Torque(EAthleteJoint::AnkleLeft), Torque(EAthleteJoint::AnkleRight), Torque(EAthleteJoint::KneeLeft),
		Torque(EAthleteJoint::HipLeft), Torque(EAthleteJoint::HipRight), Torque(EAthleteJoint::Lumbar),
		PhysicalBody->GetKineticEnergyJ(), Motor->GetMotorState() == EAthleteMotorState::Fallen ? 1.0 : 0.0,
	});
}

void AAthleteLabStanding::DrawDebug() const
{
	const UWorld* World = GetWorld();
	const FVector ComCm = AthleteUnits::MetersToUnreal(PhysicalBody->GetCenterOfMassM());
	const FVector ComVelocityMps = PhysicalBody->GetLinearMomentumKgMps() / PhysicalBody->GetTotalMassKg();
	AthleteDebug::DrawPoint(World, EAthleteDebugChannel::CenterOfMass, ComCm, FColor::Magenta);
	AthleteDebug::DrawLine(World, EAthleteDebugChannel::CenterOfMass, ComCm, FVector(ComCm.X, ComCm.Y, 0.0), FColor::Magenta);
	AthleteDebug::DrawArrow(World, EAthleteDebugChannel::Kinematics, ComCm, ComCm + ComVelocityMps * AthleteDebug::GetVelocityArrowScale(), FColor::Green);

	if (Motor->IsInitialized() && AthleteDebug::IsChannelEnabled(EAthleteDebugChannel::Balance))
	{
		// On the floor: where the feet support the body (yellow) and where the balance controller
		// believes the extrapolated center of mass is (cyan, as perceived, so delayed).
		const FAthleteBalanceSensing Now = Motor->SenseNow();
		const FAthleteBalanceCommand& Command = Motor->GetLastCommand();
		const FVector SupportCm = AthleteUnits::MetersToUnreal(Now.SupportCenterM);
		const FVector XcomCm = SupportCm + AthleteUnits::MetersToUnreal(Now.ForwardAxis * Command.XcomErrorForwardM + Now.RightAxis * Command.XcomErrorRightM);
		AthleteDebug::DrawPoint(World, EAthleteDebugChannel::Balance, SupportCm, FColor::Yellow);
		AthleteDebug::DrawPoint(World, EAthleteDebugChannel::Balance, XcomCm, FColor::Cyan);
		AthleteDebug::DrawLine(World, EAthleteDebugChannel::Balance, SupportCm, XcomCm, FColor::Cyan);
	}
}
