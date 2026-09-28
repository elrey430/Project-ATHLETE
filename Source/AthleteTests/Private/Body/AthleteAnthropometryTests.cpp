// Project ATHLETE

#include "AthleteTestFlags.h"
#include "Anatomy/AthleteAnthropometry.h"
#include "Anatomy/AthleteBodyModel.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteAnthropometryMassFractionTest, "Athlete.Body.Anthropometry.MassFractionsSumTo100", AthleteTestFlags)

bool FAthleteAnthropometryMassFractionTest::RunTest(const FString& Parameters)
{
	// Guards against transcription errors in the de Leva table: the 16 segments must account
	// for exactly 100% of body mass.
	double TotalPercent = 0.0;
	for (int32 Index = 0; Index < AthleteSegments::NumSegments; ++Index)
	{
		TotalPercent += AthleteAnthropometry::GetDeLevaMale(AthleteSegments::GetKind(AthleteSegments::FromIndex(Index))).MassPercent;
	}
	TestEqual(TEXT("Sum of segment mass percentages"), TotalPercent, 100.0, 1e-9);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteAnthropometryLandmarkTest, "Athlete.Body.Anthropometry.ReferenceLandmarksMatchWinter", AthleteTestFlags)

bool FAthleteAnthropometryLandmarkTest::RunTest(const FString& Parameters)
{
	// Build the de Leva reference male and compare landmark heights against an INDEPENDENT
	// source: Winter (2009) Fig. 4.1 / Drillis & Contini (1966) segment-length ratios.
	// Only ankle height and hanging-wrist height are inputs to our model; knee and hip heights
	// come purely from de Leva's lengths, so agreement here is a genuine cross-check.
	constexpr double WinterKneeHeightRatio = 0.285;
	constexpr double WinterGreaterTrochanterHeightRatio = 0.530; // hip joint center sits close to this level
	constexpr double CrossSourceTolerance = 0.015;               // 1.5% of stature (about 2.6 cm)

	const FAthleteMorphology Reference; // defaults are the de Leva reference male
	FAthleteBodyModel Model;
	UTEST_TRUE(TEXT("Reference body builds"), FAthleteBodyModel::Build(Reference, Model));

	const double H = Reference.StatureM;
	TestEqual(TEXT("Top of head is exactly at stature (m)"), Model.GetSegment(EAthleteSegment::Head).OriginM.Z, H, 1e-9);
	TestEqual(TEXT("Ankle joint at Winter height (m)"), Model.GetSegment(EAthleteSegment::ShankLeft).EndM.Z, AthleteAnthropometry::AnkleHeightRatio * H, 1e-9);
	TestEqual(TEXT("Hanging wrist at Winter height (m)"), Model.GetSegment(EAthleteSegment::HandLeft).OriginM.Z, AthleteAnthropometry::HangingWristHeightRatio * H, 1e-9);

	const double KneeRatio = Model.GetSegment(EAthleteSegment::ShankLeft).OriginM.Z / H;
	const double HipRatio = Model.GetHipJointHeightM() / H;
	TestEqual(TEXT("Knee height ratio vs Winter"), KneeRatio, WinterKneeHeightRatio, CrossSourceTolerance);
	TestEqual(TEXT("Hip joint height ratio vs Winter trochanter"), HipRatio, WinterGreaterTrochanterHeightRatio, CrossSourceTolerance);

	const double Normalization = AthleteAnthropometry::GetVerticalChainNormalization();
	const double ShoulderDrop = AthleteAnthropometry::GetShoulderDropFractionOfUpperTrunk();
	TestTrue(TEXT("Vertical chain normalization is a small correction"), Normalization > 0.95 && Normalization < 1.0);
	TestTrue(TEXT("Shoulder joint lies within the upper trunk"), ShoulderDrop > 0.0 && ShoulderDrop < 1.0);

	AddInfo(FString::Printf(TEXT("Knee %.4f H (Winter 0.285), hip %.4f H (Winter 0.530), chain normalization %.4f, shoulder %.3f down upper trunk"),
		KneeRatio, HipRatio, Normalization, ShoulderDrop));
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
