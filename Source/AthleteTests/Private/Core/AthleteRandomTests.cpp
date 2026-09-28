// Project ATHLETE

#include "AthleteTestFlags.h"
#include "Random/AthleteRandom.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteRandomReferenceTest, "Athlete.Core.Random.MatchesPcg32Reference", AthleteTestFlags)

bool FAthleteRandomReferenceTest::RunTest(const FString& Parameters)
{
	// Published output of the PCG reference demo (pcg32-demo, seed 42, sequence 54).
	// If this ever fails, recorded seeds from earlier sessions no longer reproduce.
	const TArray<uint32> Expected = { 0xa15c02b7u, 0x7b47f409u, 0xba1d3330u, 0x83d2f293u, 0xbfa4784bu, 0xcbed606eu };

	FAthleteRandomStream Stream(42u, 54u);
	for (int32 Index = 0; Index < Expected.Num(); ++Index)
	{
		TestEqual(*FString::Printf(TEXT("Output %d"), Index), static_cast<int64>(Stream.NextUInt32()), static_cast<int64>(Expected[Index]));
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteRandomDeterminismTest, "Athlete.Core.Random.SameSeedSameSequence", AthleteTestFlags)

bool FAthleteRandomDeterminismTest::RunTest(const FString& Parameters)
{
	constexpr int32 SampleCount = 1000;

	FAthleteRandomStream A(12345u);
	FAthleteRandomStream B(12345u);
	FAthleteRandomStream C(12346u);

	int32 Mismatches = 0;
	int32 DifferencesFromOtherSeed = 0;
	for (int32 Index = 0; Index < SampleCount; ++Index)
	{
		const uint32 ValueA = A.NextUInt32();
		Mismatches += (ValueA != B.NextUInt32()) ? 1 : 0;
		DifferencesFromOtherSeed += (ValueA != C.NextUInt32()) ? 1 : 0;
	}

	TestEqual(TEXT("Identical seeds produce identical sequences"), Mismatches, 0);
	TestTrue(TEXT("Different seeds produce different sequences"), DifferencesFromOtherSeed > SampleCount * 9 / 10);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteRandomSubstreamTest, "Athlete.Core.Random.SubstreamsAreIndependent", AthleteTestFlags)

bool FAthleteRandomSubstreamTest::RunTest(const FString& Parameters)
{
	const FAthleteRandomStream Master(777u);

	// Same name -> same child, regardless of how much the parent has been used.
	FAthleteRandomStream UsedMaster(777u);
	for (int32 Index = 0; Index < 50; ++Index)
	{
		UsedMaster.NextUInt32();
	}
	FAthleteRandomStream ChildA = Master.MakeSubstream(TEXT("Athlete1.Motor"));
	FAthleteRandomStream ChildAFromUsed = UsedMaster.MakeSubstream(TEXT("Athlete1.Motor"));
	FAthleteRandomStream ChildB = Master.MakeSubstream(TEXT("Athlete2.Motor"));

	const uint32 FirstA = ChildA.NextUInt32();
	TestEqual(TEXT("Substream ignores parent consumption"), static_cast<int64>(FirstA), static_cast<int64>(ChildAFromUsed.NextUInt32()));
	TestNotEqual(TEXT("Different consumers get different streams"), static_cast<int64>(FirstA), static_cast<int64>(ChildB.NextUInt32()));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteRandomRangeTest, "Athlete.Core.Random.UnitDoubleRange", AthleteTestFlags)

bool FAthleteRandomRangeTest::RunTest(const FString& Parameters)
{
	constexpr int32 SampleCount = 100000;
	FAthleteRandomStream Stream(2026u);

	double Sum = 0.0;
	bool bAllInRange = true;
	for (int32 Index = 0; Index < SampleCount; ++Index)
	{
		const double Value = Stream.NextUnitDouble();
		bAllInRange &= (Value >= 0.0 && Value < 1.0);
		Sum += Value;
	}

	TestTrue(TEXT("All samples in [0, 1)"), bAllInRange);
	// Mean of U(0,1) is 0.5 with standard error 1/sqrt(12 N) ~= 0.0009; 0.005 is > 5 sigma.
	TestEqual(TEXT("Mean near 0.5"), Sum / SampleCount, 0.5, 0.005);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
