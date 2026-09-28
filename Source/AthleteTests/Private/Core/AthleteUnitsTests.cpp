// Project ATHLETE

#include "AthleteTestFlags.h"
#include "Units/AthleteUnits.h"

#if WITH_DEV_AUTOMATION_TESTS

// Test names form a hierarchy in the Session Frontend: Athlete > Core > Units > ...
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteUnitsConversionTest, "Athlete.Core.Units.Conversions", AthleteTestFlags)

bool FAthleteUnitsConversionTest::RunTest(const FString& Parameters)
{
	using namespace AthleteUnits;
	constexpr double Tolerance = 1e-9;

	// Unreal <-> SI
	TestEqual(TEXT("100 cm is 1 m"), UnrealToMeters(100.0), 1.0, Tolerance);
	TestEqual(TEXT("1 m is 100 cm"), MetersToUnreal(1.0), 100.0, Tolerance);
	TestEqual(TEXT("Vector cm -> m"), UnrealToMeters(FVector(100.0, -250.0, 980.0)), FVector(1.0, -2.5, 9.8), 1e-6f);
	TestEqual(TEXT("1 N is 100 kg*cm/s^2"), NewtonsToUnreal(1.0), 100.0, Tolerance);

	// Imperial -> SI, checked against exact definitions
	TestEqual(TEXT("1 inch is 2.54 cm"), InchesToMeters(1.0), 0.0254, Tolerance);
	TestEqual(TEXT("5'9\" is 1.7526 m"), FeetInchesToMeters(5.0, 9.0), 1.7526, Tolerance);
	TestEqual(TEXT("6'4\" is 1.9304 m"), FeetInchesToMeters(6.0, 4.0), 1.9304, Tolerance);
	TestEqual(TEXT("100 yards is 91.44 m"), YardsToMeters(100.0), 91.44, Tolerance);
	TestEqual(TEXT("190 lb in kg"), PoundsToKilograms(190.0), 86.1825503, 1e-6);
	TestEqual(TEXT("240 lb in kg"), PoundsToKilograms(240.0), 108.8621688, 1e-6);
	TestEqual(TEXT("22 mph in m/s"), MphToMetersPerSecond(22.0), 9.83488, Tolerance);

	// Round trips
	TestEqual(TEXT("lb round trip"), KilogramsToPounds(PoundsToKilograms(212.5)), 212.5, Tolerance);
	TestEqual(TEXT("mph round trip"), MetersPerSecondToMph(MphToMetersPerSecond(21.3)), 21.3, Tolerance);
	TestEqual(TEXT("yard round trip"), MetersToYards(YardsToMeters(53.3)), 53.3, Tolerance);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
