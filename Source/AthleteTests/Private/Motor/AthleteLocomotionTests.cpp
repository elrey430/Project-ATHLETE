// Project ATHLETE
// Milestone 4: locomotion. What the hand-built gait can do, measured over several trials each
// (see FAthleteWalkingRobustness). The gait's known limit (faster walking falls within a few steps)
// is reported by the WalkingSpeeds study, not asserted: see the Milestone 4 doc.

#include "AthleteTestFlags.h"
#include "AthleteWalkingTestHelpers.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	constexpr int32 RobustnessTrials = 5;
	constexpr double RobustnessTrialS = 12.0;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteStepInPlaceTest, "Athlete.Motor.Locomotion.StepInPlace", AthleteTestFlags)

bool FAthleteStepInPlaceTest::RunTest(const FString& Parameters)
{
	// Asked to keep stepping without going anywhere ("chopping the feet"), he steps for 11 s in every
	// trial: every foot put down near where it was aimed, and he stays about where he started. The
	// only way to stay up is to put each foot where the balance of his body needs it.
	FAthleteWalkingRobustness Result;
	UTEST_TRUE(TEXT("Trials ran"), RunWalkingRobustness(*this, FAthleteMotorSkill(), 0.0, RobustnessTrials, RobustnessTrialS, Result));
	TestEqual(TEXT("Never falls"), Result.Falls, 0);
	TestTrue(TEXT("Keeps stepping (at least 20 steps in 11 s, every trial)"), Result.MinSteps() >= 20);
	TestTrue(TEXT("Feet land within 10 cm fore-aft of their targets on average"), Result.Pooled.MeanAbsLandingErrorM().X < 0.10);
	TestTrue(TEXT("Feet land within 5 cm sideways of their targets on average"), Result.Pooled.MeanAbsLandingErrorM().Y < 0.05);
	TestTrue(TEXT("Stays within 1 m of where he started"), Result.MaxDriftM() < 1.0);
	AddInfo(Result.Summary());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteWalkSlowTest, "Athlete.Motor.Locomotion.WalkSlow", AthleteTestFlags)

bool FAthleteWalkSlowTest::RunTest(const FString& Parameters)
{
	// Asked to walk forward at 0.3 m/s. The hand-built gait's working range: it walks at about the
	// speed asked, feet landing close to their targets. A regression guard at the measured baseline
	// (at most one fall in five trials), not the goal: see the Milestone 4 doc.
	constexpr double WishedSpeedMps = 0.3;
	FAthleteWalkingRobustness Result;
	UTEST_TRUE(TEXT("Trials ran"), RunWalkingRobustness(*this, FAthleteMotorSkill(), WishedSpeedMps, RobustnessTrials, RobustnessTrialS, Result));
	TestTrue(TEXT("Falls in at most one of five trials"), Result.Falls <= 1);
	TestTrue(TEXT("Walks at the speed asked (within 0.15 m/s)"), FMath::Abs(Result.AverageSpeedMps() - WishedSpeedMps) < 0.15);
	TestTrue(TEXT("Feet land within 6 cm fore-aft of their targets on average"), Result.Pooled.MeanAbsLandingErrorM().X < 0.06);
	TestTrue(TEXT("Feet land within 6 cm sideways of their targets on average"), Result.Pooled.MeanAbsLandingErrorM().Y < 0.06);
	AddInfo(Result.Summary());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteLocomotionDeterminismTest, "Athlete.Motor.Locomotion.Deterministic", AthleteTestFlags)

bool FAthleteLocomotionDeterminismTest::RunTest(const FString& Parameters)
{
	// The same walk twice gives the same walk: every frame of the center-of-mass path identical.
	FAthleteWalkingTrial Trial;
	Trial.DurationS = 6.0;
	Trial.Script.Add(FAthleteIntentKey::Velocity(1.0, 0.3, 0.0));
	FAthleteWalkingResult First, Second;
	UTEST_TRUE(TEXT("First run"), RunWalkingTrial(*this, Trial, First));
	UTEST_TRUE(TEXT("Second run"), RunWalkingTrial(*this, Trial, Second));
	UTEST_EQUAL(TEXT("Same number of frames"), First.ComPath.Num(), Second.ComPath.Num());
	double LargestDifferenceM = 0.0;
	for (int32 Frame = 0; Frame < First.ComPath.Num(); ++Frame)
	{
		LargestDifferenceM = FMath::Max(LargestDifferenceM, FVector::Dist(First.ComPath[Frame], Second.ComPath[Frame]));
	}
	TestEqual(TEXT("Same steps"), First.Steps, Second.Steps);
	TestTrue(TEXT("Identical path (under 1 micrometer)"), LargestDifferenceM < 1e-6);
	AddInfo(FString::Printf(TEXT("%d frames, %d steps, largest difference %.3g m"), First.ComPath.Num(), First.Steps, LargestDifferenceM));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteWalkingSpeedsStudy, "Athlete.Motor.Study.WalkingSpeeds", AthleteTestFlags)

bool FAthleteWalkingSpeedsStudy::RunTest(const FString& Parameters)
{
	// Reports, doesn't judge: how the hand-built gait holds up from stepping in place to 1 m/s.
	for (const double SpeedMps : { 0.0, 0.3, 0.6, 1.0 })
	{
		FAthleteWalkingRobustness Result;
		UTEST_TRUE(TEXT("Trials ran"), RunWalkingRobustness(*this, FAthleteMotorSkill(), SpeedMps, RobustnessTrials, RobustnessTrialS, Result));
		AddInfo(FString::Printf(TEXT("%.1f m/s: %s"), SpeedMps, *Result.Summary()));
	}
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
