// Project ATHLETE

#include "AthleteTestFlags.h"
#include "Capability/AthleteStrength.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteStrengthBaselineTest, "Athlete.Body.Strength.GeneralPopulationBaseline", AthleteTestFlags)

bool FAthleteStrengthBaselineTest::RunTest(const FString& Parameters)
{
	// Worked by hand from Harbo et al. (2012) Table 2, male rows, for age 22, 1.85 m, 100 kg:
	//   Knee extension (isometric):   -107.9 - 1.40*22 + 158*1.85 + 1.75*100 = 328.60 Nm
	//   Hip extension (isokinetic 60): -49.8 - 0.97*22 + 103*1.85 + 1.14*100 = 233.41 Nm
	const FAthleteStrengthProfile Profile = FAthleteStrengthProfile::MakeGeneralPopulationMaleBaseline(22.0, 1.85, 100.0);

	const FAthleteJointActionStrength* Knee = Profile.Find(EAthleteJointAction::KneeExtension);
	const FAthleteJointActionStrength* Hip = Profile.Find(EAthleteJointAction::HipExtension);
	UTEST_NOT_NULL(TEXT("Knee extension present"), Knee);
	UTEST_NOT_NULL(TEXT("Hip extension present"), Hip);

	TestEqual(TEXT("Knee extension peak torque (Nm)"), Knee->PeakTorqueNm, 328.60, 1e-9);
	TestEqual(TEXT("Knee extension measured isometrically"), static_cast<int32>(Knee->TestMode), static_cast<int32>(EAthleteStrengthTestMode::Isometric));
	TestEqual(TEXT("Hip extension peak torque (Nm)"), Hip->PeakTorqueNm, 233.41, 1e-9);
	TestEqual(TEXT("Hip extension test velocity (deg/s)"), Hip->TestVelocityDegPerSec, 60.0, 1e-9);

	// Every represented action is filled, and test metadata is self-consistent.
	int32 Inconsistent = 0;
	for (int32 Index = 0; Index < static_cast<int32>(EAthleteJointAction::Count); ++Index)
	{
		const FAthleteJointActionStrength* Strength = Profile.Find(static_cast<EAthleteJointAction>(Index));
		const bool bConsistent = Strength && Strength->PeakTorqueNm > 0.0
			&& ((Strength->TestMode == EAthleteStrengthTestMode::Isometric) == (Strength->TestVelocityDegPerSec == 0.0));
		Inconsistent += bConsistent ? 0 : 1;
	}
	TestEqual(TEXT("All actions present with consistent test metadata"), Inconsistent, 0);

	// Strength grows with body mass at fixed age and height (positive mass coefficients).
	const FAthleteStrengthProfile Heavier = FAthleteStrengthProfile::MakeGeneralPopulationMaleBaseline(22.0, 1.85, 120.0);
	TestTrue(TEXT("Heavier baseline is stronger at the knee"), Heavier.Find(EAthleteJointAction::KneeExtension)->PeakTorqueNm > Knee->PeakTorqueNm);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
