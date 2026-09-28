// Project ATHLETE

#include "Lab/AthleteLabBodyPreview.h"
#include "AthleteBody.h"
#include "Components/SceneComponent.h"
#include "Debug/AthleteDebugDraw.h"
#include "Definition/AthleteDefinition.h"
#include "Engine/World.h"
#include "Telemetry/AthleteTelemetrySubsystem.h"
#include "Units/AthleteUnits.h"
#include <limits>

namespace
{
	/** Average human body density, used only to size the preview capsules. VISUAL ONLY. */
	constexpr double VisualBodyDensityKgPerM3 = 1050.0;

	constexpr float SegmentComSphereRadiusCm = 2.5f;
	constexpr float BodyComSphereRadiusCm = 6.0f;

	const FColor LeftColor(80, 140, 255);
	const FColor RightColor(255, 90, 80);
	const FColor MidlineColor(230, 230, 230);
	const FColor SegmentComColor = FColor::Yellow;
	const FColor BodyComColor = FColor::Magenta;

	FColor ColorFor(EAthleteSegment Segment)
	{
		switch (AthleteSegments::GetSide(Segment))
		{
		case EAthleteBodySide::Left:  return LeftColor;
		case EAthleteBodySide::Right: return RightColor;
		default:                      return MidlineColor;
		}
	}

	/** Radius (m) of a cylinder with this mass and length at body density: m = rho * pi * r^2 * L. */
	double EquivalentCylinderRadiusM(double MassKg, double LengthM)
	{
		return FMath::Sqrt(MassKg / (VisualBodyDensityKgPerM3 * UE_DOUBLE_PI * FMath::Max(LengthM, UE_DOUBLE_KINDA_SMALL_NUMBER)));
	}
}

AAthleteLabBodyPreview::AAthleteLabBodyPreview()
{
	PrimaryActorTick.bCanEverTick = true;
	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
}

void AAthleteLabBodyPreview::BeginPlay()
{
	Super::BeginPlay();
	RebuildModel();
	if (bModelValid)
	{
		RecordTelemetry();
	}
}

void AAthleteLabBodyPreview::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	RebuildModel();
	if (bModelValid)
	{
		Draw();
	}
}

void AAthleteLabBodyPreview::RebuildModel()
{
	bModelValid = Athlete && Athlete->BuildBodyModel(Model);
}

void AAthleteLabBodyPreview::Draw() const
{
	const UWorld* World = GetWorld();
	const FTransform& ActorTransform = GetActorTransform();

	// Body frame (meters, origin on the floor between the ankles) -> world (Unreal cm).
	auto ToWorld = [&ActorTransform](const FVector& BodyMeters)
	{
		return ActorTransform.TransformPosition(AthleteUnits::MetersToUnreal(BodyMeters));
	};

	for (const FAthleteBodySegment& Segment : Model.GetSegments())
	{
		if (bDrawSegments)
		{
			const float RadiusCm = static_cast<float>(AthleteUnits::MetersToUnreal(EquivalentCylinderRadiusM(Segment.MassKg, Segment.LengthM)));
			AthleteDebug::DrawCapsuleBetween(World, EAthleteDebugChannel::Anatomy, ToWorld(Segment.OriginM), ToWorld(Segment.EndM), RadiusCm, ColorFor(Segment.Segment));
		}
		if (bDrawCentersOfMass)
		{
			AthleteDebug::DrawSphere(World, EAthleteDebugChannel::Anatomy, ToWorld(Segment.CenterOfMassM), SegmentComSphereRadiusCm, SegmentComColor);
		}
	}

	if (bDrawCentersOfMass)
	{
		const FVector Com = Model.GetCenterOfMassM();
		AthleteDebug::DrawSphere(World, EAthleteDebugChannel::Anatomy, ToWorld(Com), BodyComSphereRadiusCm, BodyComColor);
		AthleteDebug::DrawLine(World, EAthleteDebugChannel::Anatomy, ToWorld(Com), ToWorld(FVector(Com.X, Com.Y, 0.0)), BodyComColor);
	}
}

void AAthleteLabBodyPreview::RecordTelemetry() const
{
	UAthleteTelemetrySubsystem* Telemetry = GetWorld()->GetSubsystem<UAthleteTelemetrySubsystem>();
	if (!Telemetry || !Telemetry->IsEnabled())
	{
		return;
	}

	// Segment id -> name legend, so the numeric CSVs are readable.
	TStringBuilder<512> Legend;
	for (int32 Index = 0; Index < AthleteSegments::NumSegments; ++Index)
	{
		Legend.Appendf(TEXT("%s%d %s"), Index > 0 ? TEXT(";") : TEXT(""), Index, AthleteSegments::GetName(AthleteSegments::FromIndex(Index)));
	}
	Telemetry->SetSessionMetadata(TEXT("segment_ids"), FString(Legend.ToView()));

	const FString Prefix = FString::Printf(TEXT("Body_%s"), *Athlete->GetName());

	if (TSharedPtr<FAthleteTelemetryTable> Segments = Telemetry->CreateTable(FName(Prefix + TEXT("_Segments")),
		{ TEXT("segment_id"), TEXT("length_m"), TEXT("mass_kg"), TEXT("com_x_m"), TEXT("com_y_m"), TEXT("com_z_m"),
		  TEXT("i_xx_kg_m2"), TEXT("i_yy_kg_m2"), TEXT("i_zz_kg_m2") }))
	{
		for (const FAthleteBodySegment& Segment : Model.GetSegments())
		{
			Segments->AddRow({
				static_cast<double>(AthleteSegments::ToIndex(Segment.Segment)), Segment.LengthM, Segment.MassKg,
				Segment.CenterOfMassM.X, Segment.CenterOfMassM.Y, Segment.CenterOfMassM.Z,
				Segment.PrincipalInertiaKgM2.X, Segment.PrincipalInertiaKgM2.Y, Segment.PrincipalInertiaKgM2.Z });
		}
	}

	// One-row summary: whole-body properties plus strength (NaN = not specified).
	TArray<FName> SummaryColumns = { TEXT("stature_m"), TEXT("mass_kg"), TEXT("com_x_m"), TEXT("com_z_m"), TEXT("com_height_ratio"),
		TEXT("hip_joint_height_m"), TEXT("i_xx_kg_m2"), TEXT("i_yy_kg_m2"), TEXT("i_zz_kg_m2"), TEXT("i_xz_kg_m2") };
	const FAthleteInertiaTensor& I = Model.GetInertiaAboutCom();
	TArray<double> SummaryValues = { Model.GetStatureM(), Model.GetTotalMassKg(), Model.GetCenterOfMassM().X, Model.GetCenterOfMassM().Z,
		Model.GetComHeightRatio(), Model.GetHipJointHeightM(), I.XX, I.YY, I.ZZ, I.XZ };

	for (int32 Index = 0; Index < static_cast<int32>(EAthleteJointAction::Count); ++Index)
	{
		const EAthleteJointAction Action = static_cast<EAthleteJointAction>(Index);
		SummaryColumns.Add(FName(FString::Printf(TEXT("%s_Nm"), AthleteStrength::GetActionName(Action))));
		const FAthleteJointActionStrength* Strength = Athlete->Strength.Find(Action);
		SummaryValues.Add(Strength ? Strength->PeakTorqueNm : std::numeric_limits<double>::quiet_NaN());
	}

	if (TSharedPtr<FAthleteTelemetryTable> Summary = Telemetry->CreateTable(FName(Prefix + TEXT("_Summary")), MoveTemp(SummaryColumns)))
	{
		Summary->AddRow(SummaryValues);
	}

	UE_LOG(LogAthleteBody, Log, TEXT("%s: %.3f m, %.1f kg, CoM %.3f m (%.1f%% of stature), yaw inertia %.3f kg*m^2"),
		*Athlete->GetName(), Model.GetStatureM(), Model.GetTotalMassKg(), Model.GetCenterOfMassM().Z,
		Model.GetComHeightRatio() * 100.0, I.ZZ);
}
