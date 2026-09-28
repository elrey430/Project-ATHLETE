// Project ATHLETE

#include "Anatomy/AthleteAnthropometry.h"

namespace
{
	// de Leva P. (1996) Adjustments to Zatsiorsky-Seluyanov's segment inertia parameters.
	// Journal of Biomechanics 29(9):1223-1230. Table 4, male (M) columns.
	// Values were read directly from the published table and cross-checked against the
	// Visual3D documentation of the same parameters.
	//
	// Where the table offers alternative endpoints, the row chosen makes the segments form a
	// continuous chain of joint centers from the top of the head to the ankle:
	//   Head: VERT -> CERV (alternative row, not adjusted)
	//   UpperTrunk: CERV -> XYPH (alternative row, not adjusted)
	//   MiddleTrunk: XYPH -> OMPH (not adjusted)
	//   LowerTrunk: OMPH -> MIDH (hip joint centers)
	//   Shank: KJC -> AJC (ankle joint center, alternative row; the primary row ends at the lateral malleolus)
	//
	//                                        Length  Mass%  CM%     Sag r  Trans r Long r  Adjusted
	const FAthleteSegmentInertiaReference GHead        { 242.9,  6.94, 50.02,  30.3,  31.5,  26.1, false };
	const FAthleteSegmentInertiaReference GUpperTrunk  { 242.1, 15.96, 50.66,  50.5,  32.0,  46.5, false };
	const FAthleteSegmentInertiaReference GMiddleTrunk { 215.5, 16.33, 45.02,  48.2,  38.3,  46.8, false };
	const FAthleteSegmentInertiaReference GLowerTrunk  { 145.7, 11.17, 61.15,  61.5,  55.1,  58.7, true  };
	const FAthleteSegmentInertiaReference GUpperArm    { 281.7,  2.71, 57.72,  28.5,  26.9,  15.8, true  };
	const FAthleteSegmentInertiaReference GForearm     { 268.9,  1.62, 45.74,  27.6,  26.5,  12.1, true  };
	const FAthleteSegmentInertiaReference GHand        {  86.2,  0.61, 79.00,  62.8,  51.3,  40.1, true  };
	const FAthleteSegmentInertiaReference GThigh       { 422.2, 14.16, 40.95,  32.9,  32.9,  14.9, true  };
	const FAthleteSegmentInertiaReference GShank       { 440.3,  4.33, 43.95,  25.1,  24.6,  10.2, true  };
	const FAthleteSegmentInertiaReference GFoot        { 258.1,  1.37, 44.15,  25.7,  24.5,  12.4, false };
}

const FAthleteSegmentInertiaReference& AthleteAnthropometry::GetDeLevaMale(EAthleteSegmentKind Kind)
{
	switch (Kind)
	{
	case EAthleteSegmentKind::Head:        return GHead;
	case EAthleteSegmentKind::UpperTrunk:  return GUpperTrunk;
	case EAthleteSegmentKind::MiddleTrunk: return GMiddleTrunk;
	case EAthleteSegmentKind::LowerTrunk:  return GLowerTrunk;
	case EAthleteSegmentKind::UpperArm:    return GUpperArm;
	case EAthleteSegmentKind::Forearm:     return GForearm;
	case EAthleteSegmentKind::Hand:        return GHand;
	case EAthleteSegmentKind::Thigh:       return GThigh;
	case EAthleteSegmentKind::Shank:       return GShank;
	case EAthleteSegmentKind::Foot:        return GFoot;
	default:                               checkNoEntry(); return GHead;
	}
}

double AthleteAnthropometry::GetReferenceLengthRatio(EAthleteSegmentKind Kind)
{
	return GetDeLevaMale(Kind).LengthMm / ReferenceStatureMm;
}

TConstArrayView<EAthleteSegmentKind> AthleteAnthropometry::GetVerticalChainKinds()
{
	static const EAthleteSegmentKind Chain[] =
	{
		EAthleteSegmentKind::Head,
		EAthleteSegmentKind::UpperTrunk,
		EAthleteSegmentKind::MiddleTrunk,
		EAthleteSegmentKind::LowerTrunk,
		EAthleteSegmentKind::Thigh,
		EAthleteSegmentKind::Shank,
	};
	return Chain;
}

double AthleteAnthropometry::GetVerticalChainNormalization()
{
	double ChainRatio = 0.0;
	for (const EAthleteSegmentKind Kind : GetVerticalChainKinds())
	{
		ChainRatio += GetReferenceLengthRatio(Kind);
	}
	return (1.0 - AnkleHeightRatio) / ChainRatio;
}

double AthleteAnthropometry::GetShoulderDropFractionOfUpperTrunk()
{
	const double Normalization = GetVerticalChainNormalization();

	// Heights as fractions of stature in the reference body.
	const double CervicaleHeight = 1.0 - Normalization * GetReferenceLengthRatio(EAthleteSegmentKind::Head);
	const double UpperTrunkLength = Normalization * GetReferenceLengthRatio(EAthleteSegmentKind::UpperTrunk);
	const double ShoulderHeight = HangingWristHeightRatio
		+ GetReferenceLengthRatio(EAthleteSegmentKind::UpperArm)
		+ GetReferenceLengthRatio(EAthleteSegmentKind::Forearm);

	return (CervicaleHeight - ShoulderHeight) / UpperTrunkLength;
}
