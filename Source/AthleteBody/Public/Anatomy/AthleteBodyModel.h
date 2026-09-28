// Project ATHLETE

#pragma once

#include "CoreMinimal.h"
#include "Anatomy/AthleteInertiaTensor.h"
#include "Anatomy/AthleteMorphology.h"
#include "Anatomy/AthleteSegments.h"
#include "Containers/StaticArray.h"

/**
 * One rigid segment of the body in the reference pose.
 *
 * Positions are in the BODY FRAME, in meters: origin on the floor midway between the ankles,
 * +X forward, +Y right, +Z up. The reference pose is standing upright with arms hanging
 * straight down, palms facing inward, and feet flat and pointing forward.
 */
struct FAthleteBodySegment
{
	EAthleteSegment Segment = EAthleteSegment::Count;

	/** de Leva's origin endpoint (cranial / proximal; the heel for the foot). */
	FVector OriginM = FVector::ZeroVector;

	/** The other endpoint. */
	FVector EndM = FVector::ZeroVector;

	double LengthM = 0.0;
	double MassKg = 0.0;
	FVector CenterOfMassM = FVector::ZeroVector;

	/**
	 * Principal moments of inertia about the segment's own center of mass, expressed on the
	 * body axes (X forward, Y right, Z up). In the reference pose every segment's principal
	 * axes line up with the body axes, so the tensor is diagonal.
	 */
	FVector PrincipalInertiaKgM2 = FVector::ZeroVector;

	FAthleteInertiaTensor GetInertiaAboutOwnCom() const
	{
		return FAthleteInertiaTensor::Diagonal(PrincipalInertiaKgM2.X, PrincipalInertiaKgM2.Y, PrincipalInertiaKgM2.Z);
	}
};

/**
 * The computed physical body of an athlete: 16 segments with length, mass, center of mass,
 * and rotational inertia, plus whole-body totals.
 *
 * Built purely from FAthleteMorphology plus published anthropometric data. Plain C++, not a
 * UObject: cheap to copy, trivially testable, and safe to build on any thread.
 *
 * This is the data Milestone 2 turns into physics bodies and joints.
 */
class ATHLETEBODY_API FAthleteBodyModel
{
public:
	/** Computes the body. Returns false and fills OutError if the morphology is invalid. */
	static bool Build(const FAthleteMorphology& Morphology, FAthleteBodyModel& OutModel, FString* OutError = nullptr);

	const FAthleteBodySegment& GetSegment(EAthleteSegment Segment) const { return Segments[AthleteSegments::ToIndex(Segment)]; }
	TConstArrayView<FAthleteBodySegment> GetSegments() const { return MakeArrayView(Segments.GetData(), Segments.Num()); }

	double GetStatureM() const { return StatureM; }
	double GetTotalMassKg() const { return TotalMassKg; }

	/** Whole-body center of mass in the reference pose (body frame, meters). */
	const FVector& GetCenterOfMassM() const { return CenterOfMassM; }

	/** Center-of-mass height as a fraction of stature. */
	double GetComHeightRatio() const { return CenterOfMassM.Z / StatureM; }

	/**
	 * Whole-body inertia tensor about the whole-body center of mass, reference pose.
	 *   ZZ: turning about the vertical axis (spins, cuts)
	 *   YY: pitching forward/backward (being tackled from the front or back, diving)
	 *   XX: tipping sideways (being hit from the side)
	 */
	const FAthleteInertiaTensor& GetInertiaAboutCom() const { return InertiaAboutCom; }

	/** Height of the hip joint centers above the floor. */
	double GetHipJointHeightM() const { return GetSegment(EAthleteSegment::ThighLeft).OriginM.Z; }

	double GetRegionMassKg(EAthleteBodyRegion Region) const;

private:
	TStaticArray<FAthleteBodySegment, AthleteSegments::NumSegments> Segments;
	double StatureM = 0.0;
	double TotalMassKg = 0.0;
	FVector CenterOfMassM = FVector::ZeroVector;
	FAthleteInertiaTensor InertiaAboutCom;
};
