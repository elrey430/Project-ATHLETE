// Project ATHLETE

#pragma once

#include "CoreMinimal.h"
#include "Anatomy/AthleteJoints.h"
#include "AthleteMobility.generated.h"

/**
 * Range of motion of one joint, in degrees, measured from the reference (anatomical) position.
 *
 * Three planes, named with their usual clinical terms. Per-joint meaning:
 *  - Sagittal (about the side-to-side axis):
 *      Flexion / Extension. For the ANKLE, Flexion = dorsiflexion (toes up) and
 *      Extension = plantarflexion (toes down).
 *  - Frontal (about the front-to-back axis; for the ankle, about the vertical axis):
 *      Abduction (away from the midline) / Adduction (toward it).
 *      Spine and neck: lateral flexion to either side. Wrist: radial / ulnar deviation.
 *      Ankle: toe-out / toe-in.
 *  - Axial (about the segment's long axis):
 *      InternalRotation / ExternalRotation. Forearm (on the elbow): pronation / supination.
 *      Ankle (about the foot's long axis): inversion / eversion.
 * A plane whose two values are both 0 is locked (e.g. the knee does not abduct).
 */
USTRUCT(BlueprintType)
struct ATHLETEBODY_API FAthleteJointRangeOfMotion
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sagittal", meta = (Units = "deg", ClampMin = "0", ClampMax = "180"))
	double FlexionDeg = 0.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sagittal", meta = (Units = "deg", ClampMin = "0", ClampMax = "180"))
	double ExtensionDeg = 0.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Frontal", meta = (Units = "deg", ClampMin = "0", ClampMax = "180"))
	double AbductionDeg = 0.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Frontal", meta = (Units = "deg", ClampMin = "0", ClampMax = "180"))
	double AdductionDeg = 0.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Axial", meta = (Units = "deg", ClampMin = "0", ClampMax = "180"))
	double InternalRotationDeg = 0.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Axial", meta = (Units = "deg", ClampMin = "0", ClampMax = "180"))
	double ExternalRotationDeg = 0.0;
};

/**
 * An athlete's joint mobility (flexibility). A physical attribute like strength: a flexible
 * athlete can reach positions a stiff one cannot, and that difference has to come from here,
 * not from a "flexibility rating".
 */
USTRUCT(BlueprintType)
struct ATHLETEBODY_API FAthleteMobilityProfile
{
	GENERATED_BODY()

	/** Defaults to normal adult values (see MakeNormalAdult). */
	FAthleteMobilityProfile();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mobility")
	TMap<EAthleteJoint, FAthleteJointRangeOfMotion> Joints;

	/** The joint's range, or the normal-adult range if the joint is missing from the map. */
	FAthleteJointRangeOfMotion Get(EAthleteJoint Joint) const;

	/**
	 * Normal adult ranges. Sources:
	 *  - Limbs and neck: American Academy of Orthopaedic Surgeons (1965), Joint Motion: Method of
	 *    Measuring and Recording (the AAOS normative table used in clinical goniometry).
	 *  - Thoracolumbar totals: AAOS values (flexion 80, extension 25, lateral 35, rotation 45).
	 *    NOT independently verified this session; the split between the lumbar and thoracic
	 *    joints is an ESTIMATE.
	 *  - Shoulder adduction and ankle toe-in/out are not in the AAOS table: ESTIMATES.
	 */
	static FAthleteJointRangeOfMotion GetNormalAdult(EAthleteJointKind Kind);
};
