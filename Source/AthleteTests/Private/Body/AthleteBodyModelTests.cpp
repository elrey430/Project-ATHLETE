// Project ATHLETE

#include "AthleteTestFlags.h"
#include "Anatomy/AthleteBodyModel.h"
#include "Units/AthleteUnits.h"
#include <limits>

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	constexpr double ExactTolerance = 1e-9;

	bool TensorScaledEquals(const FAthleteInertiaTensor& Actual, const FAthleteInertiaTensor& Base, double Scale)
	{
		const FAthleteInertiaTensor Expected = Base * Scale;
		return Actual.Equals(Expected, ExactTolerance * FMath::Max(1.0, FMath::Abs(Expected.YY)));
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteBodyReferenceTotalsTest, "Athlete.Body.Model.ReferenceBodyTotals", AthleteTestFlags)

bool FAthleteBodyReferenceTotalsTest::RunTest(const FString& Parameters)
{
	const FAthleteMorphology Reference;
	FAthleteBodyModel Model;
	UTEST_TRUE(TEXT("Reference body builds"), FAthleteBodyModel::Build(Reference, Model));

	double SegmentMassSum = 0.0;
	for (const FAthleteBodySegment& Segment : Model.GetSegments())
	{
		SegmentMassSum += Segment.MassKg;
	}
	TestEqual(TEXT("Segment masses sum to body mass (kg)"), SegmentMassSum, Reference.BodyMassKg, ExactTolerance);
	TestEqual(TEXT("Center of mass on the midline (m)"), Model.GetCenterOfMassM().Y, 0.0, ExactTolerance);

	// Sanity range: adult male standing center-of-mass height is commonly reported at roughly
	// 55-57% of stature. A wide band catches gross errors without pretending to more precision.
	const double ComRatio = Model.GetComHeightRatio();
	TestTrue(TEXT("Center-of-mass height is 54-59% of stature"), ComRatio > 0.54 && ComRatio < 0.59);

	const FAthleteInertiaTensor& I = Model.GetInertiaAboutCom();
	TestTrue(TEXT("Turning about the vertical axis is far easier than tipping over"), I.ZZ < 0.25 * I.YY && I.ZZ < 0.25 * I.XX);

	AddInfo(FString::Printf(TEXT("Reference male %.3f m / %.1f kg: CoM %.4f m (%.2f%% of stature), CoM x %.4f m. Inertia about CoM: roll XX %.2f, pitch YY %.2f, yaw ZZ %.3f kg*m^2"),
		Reference.StatureM, Reference.BodyMassKg, Model.GetCenterOfMassM().Z, ComRatio * 100.0, Model.GetCenterOfMassM().X, I.XX, I.YY, I.ZZ));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteBodyStatureScalingTest, "Athlete.Body.Model.StatureScalingLaw", AthleteTestFlags)

bool FAthleteBodyStatureScalingTest::RunTest(const FString& Parameters)
{
	// Scaling every length by s at constant mass must scale positions by s and inertia by s^2.
	// This is pure dimensional analysis: any hidden size-dependent rule would break it.
	constexpr double Scale = 1.1;
	FAthleteMorphology Small;
	FAthleteMorphology Tall = Small;
	Tall.StatureM *= Scale;

	FAthleteBodyModel SmallModel, TallModel;
	UTEST_TRUE(TEXT("Build small"), FAthleteBodyModel::Build(Small, SmallModel));
	UTEST_TRUE(TEXT("Build tall"), FAthleteBodyModel::Build(Tall, TallModel));

	int32 LengthMismatches = 0;
	int32 MassMismatches = 0;
	for (int32 Index = 0; Index < AthleteSegments::NumSegments; ++Index)
	{
		const EAthleteSegment Segment = AthleteSegments::FromIndex(Index);
		LengthMismatches += FMath::IsNearlyEqual(TallModel.GetSegment(Segment).LengthM, SmallModel.GetSegment(Segment).LengthM * Scale, ExactTolerance) ? 0 : 1;
		MassMismatches += FMath::IsNearlyEqual(TallModel.GetSegment(Segment).MassKg, SmallModel.GetSegment(Segment).MassKg, ExactTolerance) ? 0 : 1;
	}
	TestEqual(TEXT("Every segment length scales by s"), LengthMismatches, 0);
	TestEqual(TEXT("Mass distribution unchanged by uniform scaling"), MassMismatches, 0);
	TestEqual(TEXT("CoM height scales by s (m)"), TallModel.GetCenterOfMassM().Z, SmallModel.GetCenterOfMassM().Z * Scale, ExactTolerance);
	TestTrue(TEXT("Inertia tensor scales by s^2"), TensorScaledEquals(TallModel.GetInertiaAboutCom(), SmallModel.GetInertiaAboutCom(), Scale * Scale));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteBodyMassScalingTest, "Athlete.Body.Model.MassScalingLaw", AthleteTestFlags)

bool FAthleteBodyMassScalingTest::RunTest(const FString& Parameters)
{
	constexpr double Scale = 1.3;
	FAthleteMorphology Light;
	FAthleteMorphology Heavy = Light;
	Heavy.BodyMassKg *= Scale;

	FAthleteBodyModel LightModel, HeavyModel;
	UTEST_TRUE(TEXT("Build light"), FAthleteBodyModel::Build(Light, LightModel));
	UTEST_TRUE(TEXT("Build heavy"), FAthleteBodyModel::Build(Heavy, HeavyModel));

	TestTrue(TEXT("CoM unchanged by mass alone"), HeavyModel.GetCenterOfMassM().Equals(LightModel.GetCenterOfMassM(), ExactTolerance));
	TestTrue(TEXT("Inertia tensor scales linearly with mass"), TensorScaledEquals(HeavyModel.GetInertiaAboutCom(), LightModel.GetInertiaAboutCom(), Scale));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteBodySymmetryTest, "Athlete.Body.Model.LeftRightSymmetry", AthleteTestFlags)

bool FAthleteBodySymmetryTest::RunTest(const FString& Parameters)
{
	FAthleteMorphology Morphology = FAthleteMorphology::FromImperial(6, 1, 215);
	Morphology.ShoulderWidthScale = 1.15;
	Morphology.LegLengthScale = 1.05;
	FAthleteBodyModel Model;
	UTEST_TRUE(TEXT("Build"), FAthleteBodyModel::Build(Morphology, Model));

	int32 Asymmetries = 0;
	for (const FAthleteBodySegment& Segment : Model.GetSegments())
	{
		const FAthleteBodySegment& Mirror = Model.GetSegment(AthleteSegments::GetMirror(Segment.Segment));
		const FVector MirroredCom(Mirror.CenterOfMassM.X, -Mirror.CenterOfMassM.Y, Mirror.CenterOfMassM.Z);
		Asymmetries += (FMath::IsNearlyEqual(Segment.MassKg, Mirror.MassKg, ExactTolerance) && Segment.CenterOfMassM.Equals(MirroredCom, ExactTolerance)) ? 0 : 1;
	}
	TestEqual(TEXT("Every segment matches its mirror"), Asymmetries, 0);

	const FAthleteInertiaTensor& I = Model.GetInertiaAboutCom();
	TestEqual(TEXT("CoM on the midline"), Model.GetCenterOfMassM().Y, 0.0, ExactTolerance);
	TestEqual(TEXT("No left-right coupling (XY)"), I.XY, 0.0, ExactTolerance);
	TestEqual(TEXT("No left-right coupling (YZ)"), I.YZ, 0.0, ExactTolerance);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteBodyLegLengthTest, "Athlete.Body.Model.LongerLegsPreserveStatureAndRaiseCom", AthleteTestFlags)

bool FAthleteBodyLegLengthTest::RunTest(const FString& Parameters)
{
	FAthleteMorphology Base = FAthleteMorphology::FromImperial(6, 2, 205);
	FAthleteMorphology LongLegs = Base;
	LongLegs.LegLengthScale = 1.08;

	FAthleteBodyModel BaseModel, LongModel;
	UTEST_TRUE(TEXT("Build base"), FAthleteBodyModel::Build(Base, BaseModel));
	UTEST_TRUE(TEXT("Build long legs"), FAthleteBodyModel::Build(LongLegs, LongModel));

	TestEqual(TEXT("Stature preserved (m)"), LongModel.GetSegment(EAthleteSegment::Head).OriginM.Z, LongLegs.StatureM, ExactTolerance);
	TestEqual(TEXT("Body mass preserved (kg)"), LongModel.GetTotalMassKg(), LongLegs.BodyMassKg, ExactTolerance);
	TestTrue(TEXT("Hips higher"), LongModel.GetHipJointHeightM() > BaseModel.GetHipJointHeightM());
	TestTrue(TEXT("Trunk shorter"), LongModel.GetSegment(EAthleteSegment::MiddleTrunk).LengthM < BaseModel.GetSegment(EAthleteSegment::MiddleTrunk).LengthM);
	TestTrue(TEXT("Legs carry more of the mass"), LongModel.GetRegionMassKg(EAthleteBodyRegion::Legs) > BaseModel.GetRegionMassKg(EAthleteBodyRegion::Legs));
	TestTrue(TEXT("Center of mass higher"), LongModel.GetComHeightRatio() > BaseModel.GetComHeightRatio());

	AddInfo(FString::Printf(TEXT("Leg length x1.08: CoM %.2f%% -> %.2f%% of stature; leg mass %.1f -> %.1f kg"),
		BaseModel.GetComHeightRatio() * 100.0, LongModel.GetComHeightRatio() * 100.0,
		BaseModel.GetRegionMassKg(EAthleteBodyRegion::Legs), LongModel.GetRegionMassKg(EAthleteBodyRegion::Legs)));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteBodyRegionMassTest, "Athlete.Body.Model.RegionMassScaleRedistributes", AthleteTestFlags)

bool FAthleteBodyRegionMassTest::RunTest(const FString& Parameters)
{
	FAthleteMorphology Base;
	FAthleteMorphology HeavyArms = Base;
	HeavyArms.ArmMassScale = 1.25;

	FAthleteBodyModel BaseModel, ArmsModel;
	UTEST_TRUE(TEXT("Build base"), FAthleteBodyModel::Build(Base, BaseModel));
	UTEST_TRUE(TEXT("Build heavy arms"), FAthleteBodyModel::Build(HeavyArms, ArmsModel));

	TestEqual(TEXT("Total mass unchanged (kg)"), ArmsModel.GetTotalMassKg(), Base.BodyMassKg, ExactTolerance);
	TestTrue(TEXT("Arms heavier"), ArmsModel.GetRegionMassKg(EAthleteBodyRegion::Arms) > BaseModel.GetRegionMassKg(EAthleteBodyRegion::Arms));
	TestTrue(TEXT("Legs lighter"), ArmsModel.GetRegionMassKg(EAthleteBodyRegion::Legs) < BaseModel.GetRegionMassKg(EAthleteBodyRegion::Legs));

	// Exact expectation: arm weight x1.25, everything renormalized.
	const double ArmFraction = BaseModel.GetRegionMassKg(EAthleteBodyRegion::Arms) / Base.BodyMassKg;
	const double ExpectedArmFraction = 1.25 * ArmFraction / (1.0 + 0.25 * ArmFraction);
	TestEqual(TEXT("Arm mass fraction after renormalization"), ArmsModel.GetRegionMassKg(EAthleteBodyRegion::Arms) / Base.BodyMassKg, ExpectedArmFraction, ExactTolerance);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteBodyInvalidMorphologyTest, "Athlete.Body.Model.InvalidMorphologyRejected", AthleteTestFlags)

bool FAthleteBodyInvalidMorphologyTest::RunTest(const FString& Parameters)
{
	FAthleteBodyModel Model;
	FString Error;

	FAthleteMorphology NoHeight;
	NoHeight.StatureM = 0.0;
	TestFalse(TEXT("Zero stature rejected"), FAthleteBodyModel::Build(NoHeight, Model, &Error));
	TestFalse(TEXT("Error message given"), Error.IsEmpty());

	FAthleteMorphology AllLegs;
	AllLegs.LegLengthScale = 1.9; // valid scale range, but leaves no room for a trunk
	Error.Reset();
	TestFalse(TEXT("Legs longer than the body allows rejected"), FAthleteBodyModel::Build(AllLegs, Model, &Error));
	TestTrue(TEXT("Error explains proportions"), Error.Contains(TEXT("proportions")));

	FAthleteMorphology NotANumber;
	NotANumber.BodyMassKg = std::numeric_limits<double>::quiet_NaN();
	TestFalse(TEXT("NaN mass rejected"), FAthleteBodyModel::Build(NotANumber, Model));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteInertiaTensorTest, "Athlete.Body.Model.InertiaTensorMath", AthleteTestFlags)

bool FAthleteInertiaTensorTest::RunTest(const FString& Parameters)
{
	// Two 2 kg point masses at x = +/-0.5 m: textbook dumbbell.
	constexpr double Mass = 2.0;
	constexpr double Distance = 0.5;
	const FAthleteInertiaTensor Dumbbell = FAthleteInertiaTensor::PointMass(Mass, FVector(Distance, 0, 0)) + FAthleteInertiaTensor::PointMass(Mass, FVector(-Distance, 0, 0));
	const double Expected = 2.0 * Mass * Distance * Distance;

	TestEqual(TEXT("No inertia about the dumbbell's own axis"), Dumbbell.XX, 0.0, ExactTolerance);
	TestEqual(TEXT("YY = 2 m d^2"), Dumbbell.YY, Expected, ExactTolerance);
	TestEqual(TEXT("ZZ = 2 m d^2"), Dumbbell.ZZ, Expected, ExactTolerance);
	TestEqual(TEXT("About an axis at 45 deg in the XY plane: m d^2"), Dumbbell.AboutAxis(FVector(1, 1, 0).GetSafeNormal()), Expected * 0.5, ExactTolerance);

	// Off-diagonal sign convention: tensor entry XY = -m*x*y.
	const FAthleteInertiaTensor Diagonal = FAthleteInertiaTensor::PointMass(1.0, FVector(1, 1, 0));
	TestEqual(TEXT("Tensor XY component"), Diagonal.XY, -1.0, ExactTolerance);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteBodyMorphologyPreviewTest, "Athlete.Body.Model.MorphologyExperimentPreview", AthleteTestFlags)

bool FAthleteBodyMorphologyPreviewTest::RunTest(const FString& Parameters)
{
	// Milestone 5 preview: the two athletes from the project brief, with identical proportions.
	// Their difference in rotational inertia must be exactly what size alone implies,
	// I proportional to mass * length^2. No archetype rule, no bonus.
	const FAthleteMorphology A = FAthleteMorphology::FromImperial(5, 9, 190);
	const FAthleteMorphology B = FAthleteMorphology::FromImperial(6, 4, 240);
	FAthleteBodyModel ModelA, ModelB;
	UTEST_TRUE(TEXT("Build A"), FAthleteBodyModel::Build(A, ModelA));
	UTEST_TRUE(TEXT("Build B"), FAthleteBodyModel::Build(B, ModelB));

	const double ExpectedRatio = (B.BodyMassKg / A.BodyMassKg) * FMath::Square(B.StatureM / A.StatureM);
	const double YawRatio = ModelB.GetInertiaAboutCom().ZZ / ModelA.GetInertiaAboutCom().ZZ;
	TestEqual(TEXT("Yaw inertia ratio equals (mB/mA)(hB/hA)^2"), YawRatio, ExpectedRatio, 1e-9);

	for (const TPair<const TCHAR*, const FAthleteBodyModel*>& Athlete : { TPair<const TCHAR*, const FAthleteBodyModel*>(TEXT("A 5'9\" 190 lb"), &ModelA), TPair<const TCHAR*, const FAthleteBodyModel*>(TEXT("B 6'4\" 240 lb"), &ModelB) })
	{
		const FAthleteBodyModel& M = *Athlete.Value;
		const FAthleteInertiaTensor& I = M.GetInertiaAboutCom();
		AddInfo(FString::Printf(TEXT("%s: %.2f m, %.1f kg, CoM %.3f m, yaw %.3f / pitch %.2f / roll %.2f kg*m^2"),
			Athlete.Key, M.GetStatureM(), M.GetTotalMassKg(), M.GetCenterOfMassM().Z, I.ZZ, I.YY, I.XX));
	}
	AddInfo(FString::Printf(TEXT("B/A: mass x%.3f, yaw inertia x%.3f"), B.BodyMassKg / A.BodyMassKg, YawRatio));
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
