// Project ATHLETE

#pragma once

#include "CoreMinimal.h"
#include "Anatomy/AthleteSegments.h"

class FAthleteBodyModel;

enum class EAthleteCollisionShapeType : uint8
{
	/** Rounded cylinder along the body Z axis (limbs, head, hands). */
	Capsule,
	/** Axis-aligned box (trunk segments, feet: flat faces for stable standing and chest contact). */
	Box,
};

/**
 * Collision geometry for one segment, in the reference pose (body frame, meters).
 *
 * Collision shapes decide WHERE bodies touch. They do NOT decide mass or inertia: those always
 * come from FAthleteBodyModel and are imposed on the physics body exactly.
 *
 * Shapes are sized from each segment's mass and length at an assumed tissue density, so a
 * heavier athlete of the same height really is wider and deeper. That matters for contact.
 */
struct ATHLETEPHYSICS_API FAthleteSegmentCollisionShape
{
	EAthleteSegment Segment = EAthleteSegment::Count;
	EAthleteCollisionShapeType Type = EAthleteCollisionShapeType::Capsule;

	/** Geometric center of the shape. */
	FVector CenterM = FVector::ZeroVector;

	/** Box half-extents on the body axes (Box only). */
	FVector BoxHalfExtentsM = FVector::ZeroVector;

	/** Capsule radius and half-height to the tip, including the rounded end (Capsule only; axis = Z). */
	double CapsuleRadiusM = 0.0;
	double CapsuleHalfHeightM = 0.0;

	double GetVolumeM3() const;
};

namespace AthleteCollisionGeometry
{
	/**
	 * ESTIMATE. Average soft-tissue/bone density used only to size collision shapes. Real segment
	 * densities range from ~0.9 (thorax, lungs) to ~1.16 g/cm^3 (hand).
	 */
	inline constexpr double ShapeSizingDensityKgPerM3 = 1060.0;

	/** ESTIMATES. Front-to-back depth / side-to-side breadth of the trunk cross-sections. */
	inline constexpr double UpperTrunkDepthToBreadth = 0.75;
	inline constexpr double MiddleTrunkDepthToBreadth = 0.75;
	inline constexpr double LowerTrunkDepthToBreadth = 0.65;

	/** ESTIMATE. Foot breadth / foot length. */
	inline constexpr double FootBreadthToLength = 0.36;

	/**
	 * The hand segment in de Leva ends at the 3rd metacarpal (knuckle), but the fingers exist
	 * and touch things. de Leva Table 4 gives the wrist-to-fingertip length as 187.9 mm vs.
	 * 86.2 mm to the knuckle (males), so the hand's collision shape spans 187.9 / 86.2 of it.
	 */
	inline constexpr double HandCollisionLengthScale = 187.9 / 86.2;

	ATHLETEPHYSICS_API FAthleteSegmentCollisionShape ComputeShape(const FAthleteBodyModel& Model, EAthleteSegment Segment);

	/** Radius of a capsule of total tip-to-tip length L with volume V (solved numerically). */
	ATHLETEPHYSICS_API double SolveCapsuleRadius(double TotalLengthM, double VolumeM3);
}
