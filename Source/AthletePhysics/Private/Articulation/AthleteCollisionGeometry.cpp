// Project ATHLETE

#include "Articulation/AthleteCollisionGeometry.h"
#include "Anatomy/AthleteBodyModel.h"

namespace
{
	/** Volume of a capsule with the given radius and tip-to-tip length: cylinder + sphere. */
	double CapsuleVolume(double Radius, double TotalLength)
	{
		const double CylinderLength = TotalLength - 2.0 * Radius;
		return UE_DOUBLE_PI * Radius * Radius * CylinderLength + (4.0 / 3.0) * UE_DOUBLE_PI * Radius * Radius * Radius;
	}

	double DepthToBreadth(EAthleteSegment Segment)
	{
		switch (Segment)
		{
		case EAthleteSegment::UpperTrunk:  return AthleteCollisionGeometry::UpperTrunkDepthToBreadth;
		case EAthleteSegment::MiddleTrunk: return AthleteCollisionGeometry::MiddleTrunkDepthToBreadth;
		default:                           return AthleteCollisionGeometry::LowerTrunkDepthToBreadth;
		}
	}
}

double FAthleteSegmentCollisionShape::GetVolumeM3() const
{
	if (Type == EAthleteCollisionShapeType::Box)
	{
		return 8.0 * BoxHalfExtentsM.X * BoxHalfExtentsM.Y * BoxHalfExtentsM.Z;
	}
	return CapsuleVolume(CapsuleRadiusM, 2.0 * CapsuleHalfHeightM);
}

double AthleteCollisionGeometry::SolveCapsuleRadius(double TotalLengthM, double VolumeM3)
{
	// Volume grows monotonically with radius on [0, L/2] (from 0 up to a sphere of diameter L),
	// so bisection always converges. If the volume exceeds that sphere, return the sphere radius.
	double Low = 0.0;
	double High = 0.5 * TotalLengthM;
	if (CapsuleVolume(High, TotalLengthM) <= VolumeM3)
	{
		return High;
	}
	for (int32 Iteration = 0; Iteration < 60; ++Iteration)
	{
		const double Mid = 0.5 * (Low + High);
		(CapsuleVolume(Mid, TotalLengthM) < VolumeM3 ? Low : High) = Mid;
	}
	return 0.5 * (Low + High);
}

FAthleteSegmentCollisionShape AthleteCollisionGeometry::ComputeShape(const FAthleteBodyModel& Model, EAthleteSegment Segment)
{
	const FAthleteBodySegment& Seg = Model.GetSegment(Segment);
	const double VolumeM3 = Seg.MassKg / ShapeSizingDensityKgPerM3;

	FAthleteSegmentCollisionShape Shape;
	Shape.Segment = Segment;

	switch (AthleteSegments::GetKind(Segment))
	{
	case EAthleteSegmentKind::UpperTrunk:
	case EAthleteSegmentKind::MiddleTrunk:
	case EAthleteSegmentKind::LowerTrunk:
	{
		// Size the box as the ellipse that fits it: an elliptical cylinder of this length, depth/breadth
		// ratio, and volume. The box keeps the ellipse's breadth and depth (flat chest for contact).
		const double Ratio = DepthToBreadth(Segment);
		const double Breadth = FMath::Sqrt(4.0 * VolumeM3 / (UE_DOUBLE_PI * Ratio * Seg.LengthM));
		Shape.Type = EAthleteCollisionShapeType::Box;
		Shape.CenterM = 0.5 * (Seg.OriginM + Seg.EndM);
		Shape.BoxHalfExtentsM = FVector(0.5 * Breadth * Ratio, 0.5 * Breadth, 0.5 * Seg.LengthM);
		break;
	}
	case EAthleteSegmentKind::Foot:
	{
		// Flat box on the floor from heel to toe; height follows from mass.
		const double Breadth = FootBreadthToLength * Seg.LengthM;
		const double Height = VolumeM3 / (Seg.LengthM * Breadth);
		Shape.Type = EAthleteCollisionShapeType::Box;
		Shape.CenterM = 0.5 * (Seg.OriginM + Seg.EndM) + FVector(0, 0, 0.5 * Height);
		Shape.BoxHalfExtentsM = FVector(0.5 * Seg.LengthM, 0.5 * Breadth, 0.5 * Height);
		break;
	}
	case EAthleteSegmentKind::Hand:
	{
		const double TotalLength = Seg.LengthM * HandCollisionLengthScale;
		const FVector Direction = (Seg.EndM - Seg.OriginM).GetSafeNormal();
		Shape.Type = EAthleteCollisionShapeType::Capsule;
		Shape.CapsuleRadiusM = SolveCapsuleRadius(TotalLength, VolumeM3);
		Shape.CapsuleHalfHeightM = 0.5 * TotalLength;
		Shape.CenterM = Seg.OriginM + Direction * (0.5 * TotalLength);
		break;
	}
	default:
	{
		// Head and limb segments: capsule spanning the segment tip to tip.
		Shape.Type = EAthleteCollisionShapeType::Capsule;
		Shape.CapsuleRadiusM = SolveCapsuleRadius(Seg.LengthM, VolumeM3);
		Shape.CapsuleHalfHeightM = 0.5 * Seg.LengthM;
		Shape.CenterM = 0.5 * (Seg.OriginM + Seg.EndM);
		break;
	}
	}
	return Shape;
}
