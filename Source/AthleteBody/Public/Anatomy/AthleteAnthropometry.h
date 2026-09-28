// Project ATHLETE

#pragma once

#include "CoreMinimal.h"
#include "Anatomy/AthleteSegments.h"

/**
 * Published reference data for one segment kind (de Leva 1996, Table 4, males).
 *
 * de Leva adjusted Zatsiorsky et al.'s gamma-ray-scan data (100 male physical-education
 * students, mean age 24) so segments are measured between joint centers. Reference male:
 * 73.0 kg, 1.741 m.
 *
 * Percentages follow the paper:
 *  - MassPercent: % of whole-body mass.
 *  - ComPercentFromOrigin: center-of-mass position, % of segment length measured from the
 *    ORIGIN endpoint (the cranial/proximal one; the heel for the foot).
 *  - Radius*Percent: radius of gyration about the segment's own center of mass, % of segment
 *    length. Moment of inertia I = mass * (radius% * length)^2.
 *      Sagittal   = about the anteroposterior (front-to-back) axis
 *      Transverse = about the mediolateral (side-to-side) axis
 *      Longitudinal = about the segment's long axis
 *    The paper does not state the axis convention explicitly. We inferred it from the data:
 *    segments that are wider than they are deep (trunk, upper trunk, hand) have the larger
 *    "sagittal" radius, which is only true for rotation about the front-to-back axis.
 */
struct FAthleteSegmentInertiaReference
{
	double LengthMm = 0.0;
	double MassPercent = 0.0;
	double ComPercentFromOrigin = 0.0;
	double RadiusSagittalPercent = 0.0;
	double RadiusTransversePercent = 0.0;
	double RadiusLongitudinalPercent = 0.0;

	/** False where de Leva marks the row "not adjusted" (endpoints are bony landmarks, not joint centers). */
	bool bAdjustedToJointCenters = true;
};

namespace AthleteAnthropometry
{
	/** de Leva (1996) reference male. */
	inline constexpr double ReferenceStatureMm = 1741.0;
	inline constexpr double ReferenceBodyMassKg = 73.0;

	/** de Leva (1996) Table 4, male columns, for the segment definitions ATHLETE uses. */
	ATHLETEBODY_API const FAthleteSegmentInertiaReference& GetDeLevaMale(EAthleteSegmentKind Kind);

	/** Reference segment length as a fraction of stature (de Leva length / 1741 mm). */
	ATHLETEBODY_API double GetReferenceLengthRatio(EAthleteSegmentKind Kind);

	// ---------------------------------------------------------------------------------------
	// Landmarks not covered by de Leva. Each is either sourced or explicitly an ESTIMATE.
	// Estimates are calibration candidates: change them here, never inline in model code.
	// ---------------------------------------------------------------------------------------

	/**
	 * Ankle joint center height / stature, standing. Winter, "Biomechanics and Motor Control of
	 * Human Movement" (4th ed., 2009), Fig. 4.1, after Drillis & Contini (1966).
	 */
	inline constexpr double AnkleHeightRatio = 0.039;

	/**
	 * Wrist height / stature with arms hanging at the sides (same source as above).
	 * Used to place the shoulder joint so that de Leva's arm lengths land the wrist here.
	 */
	inline constexpr double HangingWristHeightRatio = 0.485;

	/**
	 * ESTIMATE. Hip joint center lateral offset from the midline / stature.
	 * Bell et al. (1990) place each hip joint center 14% of inter-ASIS width medial to the ASIS.
	 * Assuming a typical male inter-ASIS width of 0.25 m gives a 0.18 m separation for the
	 * 1.741 m reference male: 0.09 m / 1.741 m.
	 */
	inline constexpr double HipJointHalfSeparationRatio = 0.0517;

	/**
	 * ESTIMATE. Shoulder joint center lateral offset from the midline / stature.
	 * Chosen as ~0.17 m for the reference male (joint centers sit medial to the ~0.40 m
	 * biacromial breadth). Affects arm placement and whole-body yaw inertia moderately.
	 */
	inline constexpr double ShoulderJointHalfSeparationRatio = 0.100;

	/**
	 * ESTIMATE. Horizontal distance from the heel to the ankle joint center, as a fraction of
	 * foot length (heel to toe tip).
	 */
	inline constexpr double HeelToAnkleFootLengthRatio = 0.22;

	// ---------------------------------------------------------------------------------------
	// Derived quantities (computed from the constants above, never typed in).
	// ---------------------------------------------------------------------------------------

	/** Segments stacked vertically from the top of the head to the ankle joint center. */
	ATHLETEBODY_API TConstArrayView<EAthleteSegmentKind> GetVerticalChainKinds();

	/**
	 * de Leva's mean segment lengths come from different subjects' measurements, so stacking them
	 * does not exactly reproduce the mean stature: head-to-ankle sums to 1708.7 mm, leaving only
	 * 32 mm for ankle height instead of Winter's 0.039 * 1741 = 68 mm.
	 * This factor (about 0.979) shrinks the vertical chain uniformly so that
	 *   ankle height + chain = stature
	 * exactly, while keeping de Leva's relative proportions.
	 */
	ATHLETEBODY_API double GetVerticalChainNormalization();

	/**
	 * Where the shoulder joint centers sit on the upper trunk: the fraction of upper-trunk
	 * length measured down from cervicale. Derived so that, in the reference body, arms of
	 * de Leva length hanging from the shoulder put the wrist at Winter's HangingWristHeightRatio.
	 */
	ATHLETEBODY_API double GetShoulderDropFractionOfUpperTrunk();
}
