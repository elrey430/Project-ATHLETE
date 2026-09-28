// Project ATHLETE

#include "Anatomy/AthleteBodyModel.h"
#include "Anatomy/AthleteAnthropometry.h"

namespace
{
	using namespace AthleteAnthropometry;

	/**
	 * A head-and-trunk shorter or longer than this fraction of its reference length means the
	 * proportion scales contradict the stature (e.g. legs so long there is no room for a trunk).
	 */
	constexpr double MinTrunkLengthFactor = 0.6;
	constexpr double MaxTrunkLengthFactor = 1.4;

	bool IsVerticalChainKind(EAthleteSegmentKind Kind)
	{
		return GetVerticalChainKinds().Contains(Kind);
	}

	bool IsLegKind(EAthleteSegmentKind Kind)
	{
		return Kind == EAthleteSegmentKind::Thigh || Kind == EAthleteSegmentKind::Shank;
	}

	/**
	 * Segment principal moments on body axes (X forward, Y right, Z up) in the reference pose.
	 * de Leva radii: sagittal = about the front-to-back axis, transverse = about the side-to-side
	 * axis, longitudinal = about the segment's long axis.
	 */
	FVector ComputePrincipalInertia(EAthleteSegmentKind Kind, double MassKg, double LengthM)
	{
		const FAthleteSegmentInertiaReference& Ref = GetDeLevaMale(Kind);
		auto Moment = [MassKg, LengthM](double RadiusPercent)
		{
			const double Radius = RadiusPercent * 0.01 * LengthM;
			return MassKg * Radius * Radius;
		};

		const double Sagittal = Moment(Ref.RadiusSagittalPercent);
		const double Transverse = Moment(Ref.RadiusTransversePercent);
		const double Longitudinal = Moment(Ref.RadiusLongitudinalPercent);

		if (Kind == EAthleteSegmentKind::Foot)
		{
			// The foot lies horizontally with its long axis pointing forward (X). Of the two remaining
			// axes, side-to-side (Y) is the anatomical transverse axis, leaving vertical (Z) as sagittal.
			return FVector(Longitudinal, Transverse, Sagittal);
		}
		// Every other segment is vertical in the reference pose: long axis = Z.
		return FVector(Sagittal, Transverse, Longitudinal);
	}
}

bool FAthleteBodyModel::Build(const FAthleteMorphology& In, FAthleteBodyModel& Out, FString* OutError)
{
	if (!In.Validate(OutError))
	{
		return false;
	}

	const double H = In.StatureM;
	const double ChainNormalization = GetVerticalChainNormalization();

	// ---------------------------------------------------------------------------------------
	// 1. Segment lengths.
	//
	// "Neutral" length = what this segment would be at this stature with reference proportions.
	// Vertical-chain segments carry the normalization so ankle height + chain = stature exactly.
	// ---------------------------------------------------------------------------------------
	double NeutralLength[static_cast<int32>(EAthleteSegmentKind::Count)];
	for (int32 KindIndex = 0; KindIndex < static_cast<int32>(EAthleteSegmentKind::Count); ++KindIndex)
	{
		const EAthleteSegmentKind Kind = static_cast<EAthleteSegmentKind>(KindIndex);
		NeutralLength[KindIndex] = H * GetReferenceLengthRatio(Kind) * (IsVerticalChainKind(Kind) ? ChainNormalization : 1.0);
	}
	auto Neutral = [&NeutralLength](EAthleteSegmentKind Kind) { return NeutralLength[static_cast<int32>(Kind)]; };

	// Legs take their scaled length; the head and trunk share whatever height remains, so a
	// longer-legged athlete of the same stature has a proportionally shorter head and trunk.
	const double ThighLength = Neutral(EAthleteSegmentKind::Thigh) * In.LegLengthScale;
	const double ShankLength = Neutral(EAthleteSegmentKind::Shank) * In.LegLengthScale;
	const double AnkleHeight = AnkleHeightRatio * H;

	double NeutralUpperChain = 0.0;
	for (const EAthleteSegmentKind Kind : GetVerticalChainKinds())
	{
		NeutralUpperChain += IsLegKind(Kind) ? 0.0 : Neutral(Kind);
	}
	const double UpperChainAvailable = H - AnkleHeight - ThighLength - ShankLength;
	const double TrunkLengthFactor = UpperChainAvailable / NeutralUpperChain;
	if (TrunkLengthFactor < MinTrunkLengthFactor || TrunkLengthFactor > MaxTrunkLengthFactor)
	{
		if (OutError)
		{
			*OutError = FString::Printf(TEXT("LegLengthScale %.3f leaves a head-and-trunk %.0f%% of normal length; proportions contradict stature."),
				In.LegLengthScale, TrunkLengthFactor * 100.0);
		}
		return false;
	}

	auto LengthOf = [&](EAthleteSegmentKind Kind) -> double
	{
		switch (Kind)
		{
		case EAthleteSegmentKind::Thigh:    return ThighLength;
		case EAthleteSegmentKind::Shank:    return ShankLength;
		case EAthleteSegmentKind::UpperArm:
		case EAthleteSegmentKind::Forearm:
		case EAthleteSegmentKind::Hand:     return Neutral(Kind) * In.ArmLengthScale;
		case EAthleteSegmentKind::Foot:     return Neutral(Kind) * In.FootLengthScale;
		default:                            return Neutral(Kind) * TrunkLengthFactor; // head and trunk
		}
	};

	// ---------------------------------------------------------------------------------------
	// 2. Joint positions in the reference pose (body frame, meters).
	// ---------------------------------------------------------------------------------------
	const double KneeZ = AnkleHeight + ShankLength;
	const double HipZ = KneeZ + ThighLength;
	const double OmphalionZ = HipZ + LengthOf(EAthleteSegmentKind::LowerTrunk);
	const double XiphionZ = OmphalionZ + LengthOf(EAthleteSegmentKind::MiddleTrunk);
	const double CervicaleZ = XiphionZ + LengthOf(EAthleteSegmentKind::UpperTrunk);
	const double VertexZ = CervicaleZ + LengthOf(EAthleteSegmentKind::Head); // equals H by construction
	const double ShoulderZ = CervicaleZ - GetShoulderDropFractionOfUpperTrunk() * LengthOf(EAthleteSegmentKind::UpperTrunk);

	const double HipHalfWidth = HipJointHalfSeparationRatio * H * In.HipWidthScale;
	const double ShoulderHalfWidth = ShoulderJointHalfSeparationRatio * H * In.ShoulderWidthScale;

	const double UpperArmLength = LengthOf(EAthleteSegmentKind::UpperArm);
	const double ForearmLength = LengthOf(EAthleteSegmentKind::Forearm);
	const double HandLength = LengthOf(EAthleteSegmentKind::Hand);
	const double FootLength = LengthOf(EAthleteSegmentKind::Foot);
	const double HeelX = -HeelToAnkleFootLengthRatio * FootLength;

	auto Endpoints = [&](EAthleteSegment Segment, FVector& OutOrigin, FVector& OutEnd)
	{
		// Left side is -Y, right side is +Y.
		const double Side = AthleteSegments::GetSide(Segment) == EAthleteBodySide::Left ? -1.0 : 1.0;
		const double ShoulderY = Side * ShoulderHalfWidth;
		const double HipY = Side * HipHalfWidth;

		switch (AthleteSegments::GetKind(Segment))
		{
		case EAthleteSegmentKind::Head:        OutOrigin = FVector(0, 0, VertexZ);    OutEnd = FVector(0, 0, CervicaleZ); break;
		case EAthleteSegmentKind::UpperTrunk:  OutOrigin = FVector(0, 0, CervicaleZ); OutEnd = FVector(0, 0, XiphionZ); break;
		case EAthleteSegmentKind::MiddleTrunk: OutOrigin = FVector(0, 0, XiphionZ);   OutEnd = FVector(0, 0, OmphalionZ); break;
		case EAthleteSegmentKind::LowerTrunk:  OutOrigin = FVector(0, 0, OmphalionZ); OutEnd = FVector(0, 0, HipZ); break;
		case EAthleteSegmentKind::UpperArm:
			OutOrigin = FVector(0, ShoulderY, ShoulderZ);
			OutEnd = FVector(0, ShoulderY, ShoulderZ - UpperArmLength);
			break;
		case EAthleteSegmentKind::Forearm:
			OutOrigin = FVector(0, ShoulderY, ShoulderZ - UpperArmLength);
			OutEnd = FVector(0, ShoulderY, ShoulderZ - UpperArmLength - ForearmLength);
			break;
		case EAthleteSegmentKind::Hand:
			OutOrigin = FVector(0, ShoulderY, ShoulderZ - UpperArmLength - ForearmLength);
			OutEnd = FVector(0, ShoulderY, ShoulderZ - UpperArmLength - ForearmLength - HandLength);
			break;
		case EAthleteSegmentKind::Thigh:       OutOrigin = FVector(0, HipY, HipZ);  OutEnd = FVector(0, HipY, KneeZ); break;
		case EAthleteSegmentKind::Shank:       OutOrigin = FVector(0, HipY, KneeZ); OutEnd = FVector(0, HipY, AnkleHeight); break;
		case EAthleteSegmentKind::Foot:
			// Foot axis drawn along the floor. de Leva projects the foot CM onto its heel-toe axis, so
			// the true CM is a few cm higher; the effect on whole-body CM height is about 1 mm.
			OutOrigin = FVector(HeelX, HipY, 0);
			OutEnd = FVector(HeelX + FootLength, HipY, 0);
			break;
		default: checkNoEntry(); break;
		}
	};

	// ---------------------------------------------------------------------------------------
	// 3. Segment masses.
	//
	// Start from de Leva's mass fractions. A segment longer (or shorter) than neutral is assumed
	// to keep its cross-section, so its mass weight scales with its length. Region mass scales
	// apply on top. Weights are renormalized so segment masses sum exactly to body mass.
	// Consequence: uniform size changes leave mass fractions untouched (as in de Leva), while
	// longer legs at the same stature and weight carry a larger share of the body's mass.
	// ---------------------------------------------------------------------------------------
	double Weights[AthleteSegments::NumSegments];
	double WeightSum = 0.0;
	for (int32 Index = 0; Index < AthleteSegments::NumSegments; ++Index)
	{
		const EAthleteSegment Segment = AthleteSegments::FromIndex(Index);
		const EAthleteSegmentKind Kind = AthleteSegments::GetKind(Segment);
		const double LengthRatio = LengthOf(Kind) / Neutral(Kind);
		Weights[Index] = GetDeLevaMale(Kind).MassPercent * LengthRatio * In.GetRegionMassScale(AthleteSegments::GetRegion(Segment));
		WeightSum += Weights[Index];
	}

	// ---------------------------------------------------------------------------------------
	// 4. Assemble segments, then whole-body center of mass and inertia.
	// ---------------------------------------------------------------------------------------
	Out = FAthleteBodyModel();
	Out.StatureM = H;
	Out.TotalMassKg = 0.0;

	FVector MassWeightedCom = FVector::ZeroVector;
	for (int32 Index = 0; Index < AthleteSegments::NumSegments; ++Index)
	{
		const EAthleteSegment Segment = AthleteSegments::FromIndex(Index);
		const EAthleteSegmentKind Kind = AthleteSegments::GetKind(Segment);
		FAthleteBodySegment& Seg = Out.Segments[Index];

		Seg.Segment = Segment;
		Endpoints(Segment, Seg.OriginM, Seg.EndM);
		Seg.LengthM = FVector::Dist(Seg.OriginM, Seg.EndM);
		Seg.MassKg = In.BodyMassKg * Weights[Index] / WeightSum;
		Seg.CenterOfMassM = Seg.OriginM + (Seg.EndM - Seg.OriginM) * (GetDeLevaMale(Kind).ComPercentFromOrigin * 0.01);
		Seg.PrincipalInertiaKgM2 = ComputePrincipalInertia(Kind, Seg.MassKg, Seg.LengthM);

		Out.TotalMassKg += Seg.MassKg;
		MassWeightedCom += Seg.CenterOfMassM * Seg.MassKg;
	}
	Out.CenterOfMassM = MassWeightedCom / Out.TotalMassKg;

	// Parallel-axis theorem: each segment's own inertia, plus its mass acting at its offset from
	// the whole-body center of mass.
	for (const FAthleteBodySegment& Seg : Out.Segments)
	{
		Out.InertiaAboutCom += Seg.GetInertiaAboutOwnCom();
		Out.InertiaAboutCom += FAthleteInertiaTensor::PointMass(Seg.MassKg, Seg.CenterOfMassM - Out.CenterOfMassM);
	}

	return true;
}

double FAthleteBodyModel::GetRegionMassKg(EAthleteBodyRegion Region) const
{
	double Mass = 0.0;
	for (const FAthleteBodySegment& Seg : Segments)
	{
		Mass += AthleteSegments::GetRegion(Seg.Segment) == Region ? Seg.MassKg : 0.0;
	}
	return Mass;
}
