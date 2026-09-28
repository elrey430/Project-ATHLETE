// Project ATHLETE

#include "AthleteTestFlags.h"
#include "AthleteStandingTestHelpers.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	FString Describe(const FAthleteStandingResult& Result)
	{
		return FString::Printf(TEXT("CoM height %.3f -> %.3f m (min %.3f); drift max %.3f final %.3f m; sway %.1f mm RMS; foot slide %.1f mm; final KE %.2f J, peak KE in last 2 s %.2f J; %.2f ms/frame"),
			Result.InitialComHeightM, Result.FinalComHeightM, Result.MinComHeightM, Result.MaxHorizontalComDriftM, Result.FinalHorizontalComDriftM,
			1000.0 * Result.LateSwayRmsM, 1000.0 * Result.MaxFootSlideM, Result.FinalKineticEnergyJ, Result.LateMaxKineticEnergyJ, Result.MeanTickMs);
	}

	/** Largest pelvis push (N*s, 2.5 N*s steps) the athlete recovers from without falling, in one direction. */
	double FindLargestRecoveredPush(FAutomationTestBase& Test, const FAthleteMorphology& Morphology, const FAthleteMotorSkill& Skill, const FVector& Direction)
	{
		double Largest = 0.0;
		for (double Impulse = 2.5; Impulse <= 60.0; Impulse += 2.5)
		{
			FAthleteStandingTrial Trial;
			Trial.Morphology = Morphology;
			Trial.Skill = Skill;
			Trial.PushTimeS = 3.0;
			Trial.DurationS = 6.0;
			Trial.PushImpulseNs = Direction * Impulse;
			FAthleteStandingResult Result;
			if (!RunStandingTrial(Test, Trial, Result) || !Result.IsUpright())
			{
				break;
			}
			Largest = Impulse;
		}
		return Largest;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteQuietStandingTest, "Athlete.Motor.Balance.QuietStanding", AthleteTestFlags)

bool FAthleteQuietStandingTest::RunTest(const FString& Parameters)
{
	// Released in the reference pose on flat ground, the athlete must stay up for 10 s by his own
	// muscles: small sway, feet planted, and never frozen by the physics engine's sleep system.
	FAthleteStandingTrial Trial;
	Trial.DurationS = 10.0;
	FAthleteStandingResult Result;
	UTEST_TRUE(TEXT("Trial ran"), RunStandingTrial(*this, Trial, Result));
	TestTrue(TEXT("Upright after 10 s"), Result.IsUpright());
	TestTrue(TEXT("Quiet sway under 5 mm RMS"), Result.LateSwayRmsM < 0.005);
	TestTrue(TEXT("Feet stay planted (under 5 mm)"), Result.MaxFootSlideM < 0.005);
	TestEqual(TEXT("No segment ever put to sleep"), Result.FramesWithSleepingSegment, 0);
	TestFalse(TEXT("Never judged himself fallen"), Result.bMotorFallen);
	AddInfo(Describe(Result));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteNoNeuralControlFallsTest, "Athlete.Motor.Balance.NoNeuralControlFalls", AthleteTestFlags)

bool FAthleteNoNeuralControlFallsTest::RunTest(const FString& Parameters)
{
	// Muscle tone alone (every joint held at its reference angle) cannot keep a stack of segments
	// upright: when every joint gives a little in the same direction, the top moves by the sum.
	// Without the neural loops the athlete must collapse. If he stayed up, something would be
	// holding him that isn't his body.
	FAthleteStandingTrial Trial;
	Trial.bBalance = false;
	Trial.DurationS = 10.0;
	FAthleteStandingResult Result;
	UTEST_TRUE(TEXT("Trial ran"), RunStandingTrial(*this, Trial, Result));
	TestFalse(TEXT("Collapses without neural control"), Result.IsUpright());
	AddInfo(Describe(Result));
	return true;
}

IMPLEMENT_COMPLEX_AUTOMATION_TEST(FAthleteSmallPushRecoveredTest, "Athlete.Motor.Balance.SmallPushRecovered", AthleteTestFlags)

void FAthleteSmallPushRecoveredTest::GetTests(TArray<FString>& OutBeautifiedNames, TArray<FString>& OutTestCommands) const
{
	// Pushes at the pelvis, spread over 0.1 s, well inside the measured recovery limits
	// (Milestone 3 study: about 15 N*s forward, 10 backward, 12.5 sideways).
	OutBeautifiedNames.Add(TEXT("Forward10Ns"));  OutTestCommands.Add(TEXT("10 0"));
	OutBeautifiedNames.Add(TEXT("Backward7Ns"));  OutTestCommands.Add(TEXT("-7.5 0"));
	OutBeautifiedNames.Add(TEXT("Sideways10Ns")); OutTestCommands.Add(TEXT("0 10"));
	// The same push in HIS frame while he faces another way: balance must not depend on heading.
	OutBeautifiedNames.Add(TEXT("Forward10NsFacingBack")); OutTestCommands.Add(TEXT("10 0 180 3"));
	OutBeautifiedNames.Add(TEXT("Forward10NsFacingSideways")); OutTestCommands.Add(TEXT("10 0 90 3"));
}

bool FAthleteSmallPushRecoveredTest::RunTest(const FString& Parameters)
{
	TArray<FString> Parts;
	Parameters.ParseIntoArrayWS(Parts);
	UTEST_TRUE(TEXT("Parameters"), Parts.Num() == 2 || Parts.Num() == 4);
	FAthleteStandingTrial Trial;
	Trial.PushTimeS = Parts.Num() == 4 ? FCString::Atod(*Parts[3]) : 3.0;
	Trial.BodyYawDeg = Parts.Num() == 4 ? FCString::Atod(*Parts[2]) : 0.0;
	Trial.DurationS = 8.0;
	Trial.PushImpulseNs = FVector(FCString::Atod(*Parts[0]), FCString::Atod(*Parts[1]), 0.0);
	FAthleteStandingResult Result;
	UTEST_TRUE(TEXT("Trial ran"), RunStandingTrial(*this, Trial, Result));
	TestTrue(TEXT("Upright at the end"), Result.IsUpright());
	TestTrue(TEXT("Back near where he stood (within 10 cm)"), Result.FinalHorizontalComDriftM < 0.10);
	TestTrue(TEXT("Feet stayed planted (no stepping in Milestone 3; under 2 cm)"), Result.MaxFootSlideM < 0.02);
	TestTrue(TEXT("Settled again (sway under 1 cm RMS in the last 4 s)"), Result.LateSwayRmsM < 0.01);
	TestFalse(TEXT("Never judged himself fallen"), Result.bMotorFallen);
	AddInfo(Describe(Result));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteLargePushFallsTest, "Athlete.Motor.Balance.LargePushFalls", AthleteTestFlags)

bool FAthleteLargePushFallsTest::RunTest(const FString& Parameters)
{
	// 40 N*s at the pelvis gives the body about 0.5 m/s: its extrapolated center of mass lands far
	// beyond the toes. No torque the feet can transmit brings that back; only a step could, and
	// stepping is Milestone 7. He must fall. Standing through it would mean the balance is faked.
	FAthleteStandingTrial Trial;
	Trial.PushTimeS = 3.0;
	Trial.DurationS = 10.0;
	Trial.PushImpulseNs = FVector(40.0, 0.0, 0.0);
	FAthleteStandingResult Result;
	UTEST_TRUE(TEXT("Trial ran"), RunStandingTrial(*this, Trial, Result));
	TestFalse(TEXT("Falls"), Result.IsUpright());
	// Once down, his motor system must know it and stop fighting to stand. Holding body parts
	// "upright in space" while lying on the ground only makes muscles thrash against the floor
	// (found in the lab: 5.4 J of twitching kinetic energy 5 s after landing).
	TestTrue(TEXT("Motor system knows he is down"), Result.bMotorFallen);
	TestTrue(TEXT("Comes to rest on the ground (peak kinetic energy in the last 2 s under 0.5 J)"), Result.LateMaxKineticEnergyJ < 0.5);
	AddInfo(Describe(Result));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteBalanceDeterminismTest, "Athlete.Motor.Balance.Deterministic", AthleteTestFlags)

bool FAthleteBalanceDeterminismTest::RunTest(const FString& Parameters)
{
	// Same athlete, same push, same frame rate: the same outcome, to the last digit.
	FAthleteStandingTrial Trial;
	Trial.PushTimeS = 2.0;
	Trial.DurationS = 5.0;
	Trial.PushImpulseNs = FVector(10.0, 5.0, 0.0);
	FAthleteStandingResult First, Second;
	UTEST_TRUE(TEXT("First run"), RunStandingTrial(*this, Trial, First));
	UTEST_TRUE(TEXT("Second run"), RunStandingTrial(*this, Trial, Second));
	TestEqual(TEXT("Final CoM height identical"), First.FinalComHeightM, Second.FinalComHeightM);
	TestEqual(TEXT("Final drift identical"), First.FinalHorizontalComDriftM, Second.FinalHorizontalComDriftM);
	TestEqual(TEXT("Maximum drift identical"), First.MaxHorizontalComDriftM, Second.MaxHorizontalComDriftM);
	TestEqual(TEXT("Foot slide identical"), First.MaxFootSlideM, Second.MaxFootSlideM);
	AddInfo(FString::Printf(TEXT("Run 1: %s"), *Describe(First)));
	AddInfo(FString::Printf(TEXT("Run 2: %s"), *Describe(Second)));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthletePushThresholdStudy, "Athlete.Motor.Study.PushThresholds", AthleteTestFlags)

bool FAthletePushThresholdStudy::RunTest(const FString& Parameters)
{
	// Study (Milestone 3): the largest pelvis push each athlete recovers from with his feet in place,
	// and how that depends on reaction time. Reports; asserts only that the athletes can take a
	// small push at all. All athletes use the same general-population strength regression for
	// their size and the same motor skill: only the body differs.
	struct FAthlete { const TCHAR* Name; FAthleteMorphology Morphology; };
	const FAthlete Athletes[] =
	{
		{ TEXT("Reference (1.74 m, 73 kg)"), FAthleteMorphology() },
		{ TEXT("A (5'9\", 190 lb)"), FAthleteMorphology::FromImperial(5, 9, 190) },
		{ TEXT("B (6'4\", 240 lb)"), FAthleteMorphology::FromImperial(6, 4, 240) },
	};
	for (const FAthlete& Athlete : Athletes)
	{
		const double Forward = FindLargestRecoveredPush(*this, Athlete.Morphology, FAthleteMotorSkill(), FVector(1, 0, 0));
		const double Backward = FindLargestRecoveredPush(*this, Athlete.Morphology, FAthleteMotorSkill(), FVector(-1, 0, 0));
		const double Sideways = FindLargestRecoveredPush(*this, Athlete.Morphology, FAthleteMotorSkill(), FVector(0, 1, 0));
		TestTrue(FString::Printf(TEXT("%s recovers from a small push"), Athlete.Name), FMath::Min3(Forward, Backward, Sideways) >= 5.0);
		const double Mass = Athlete.Morphology.BodyMassKg;
		AddInfo(FString::Printf(TEXT("%-26s largest recovered push: forward %4.1f N*s (%.2f m/s), backward %4.1f N*s (%.2f m/s), sideways %4.1f N*s (%.2f m/s)"),
			Athlete.Name, Forward, Forward / Mass, Backward, Backward / Mass, Sideways, Sideways / Mass));
	}

	FString Line = TEXT("Reference, forward push vs reaction time:");
	for (const double Reaction : { 0.05, 0.1, 0.15, 0.2 })
	{
		FAthleteMotorSkill Skill;
		Skill.ReactionTimeS = Reaction;
		Line += FString::Printf(TEXT(" %.2f s -> %.1f N*s;"), Reaction, FindLargestRecoveredPush(*this, FAthleteMorphology(), Skill, FVector(1, 0, 0)));
	}
	AddInfo(Line);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
