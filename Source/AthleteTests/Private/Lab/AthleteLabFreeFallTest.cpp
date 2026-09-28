// Project ATHLETE

#include "AthleteTestFlags.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/World.h"
#include "Lab/AthleteLabPhysicsProbe.h"
#include "Telemetry/AthleteTelemetryTable.h"
#include "Tests/AutomationCommon.h"
#include "Units/AthleteUnits.h"

#if WITH_DEV_AUTOMATION_TESTS

/**
 * Integration test: the whole measurement pipeline against a known physical answer.
 *
 * A 100 kg probe is released in an empty world and ticked 60 times at a fixed 60 Hz frame rate.
 * With no damping and no drag, the only force is gravity, so:
 *   - samples must be evenly spaced by the frame time (no dropped or duplicated samples),
 *   - the first sample is one frame after release from rest: v0 = g * dt,
 *   - measured acceleration must equal the world's gravity (independent of mass),
 *   - velocity change between samples must equal g * (elapsed telemetry time).
 * If Chaos settings, unit conversions, sampling order, or telemetry are wrong, this fails.
 *
 * All timing checks use the telemetry's own timestamps rather than "ticks * dt", because the
 * test world does not start simulating on the same tick in every context: run as a commandlet
 * (how Visual Studio's Test Explorer runs tests), the first tick after BeginPlay neither steps
 * physics nor ticks actors, so one fewer sample is recorded than in the editor.
 */
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteLabFreeFallTest, "Athlete.Lab.PhysicsProbe.FreeFallMatchesGravity", AthleteTestFlags)

bool FAthleteLabFreeFallTest::RunTest(const FString& Parameters)
{
	constexpr float FrameDeltaSeconds = 1.0f / 60.0f;
	constexpr int32 FrameCount = 60;
	constexpr double ReleaseHeightM = 100.0; // High enough that nothing is hit in one second.
	constexpr double RelativeTolerance = 0.01; // 1%

	FTestWorldWrapper TestWorld;
	UTEST_TRUE(TEXT("Create test world"), TestWorld.CreateTestWorld(EWorldType::Game));
	UWorld* World = TestWorld.GetTestWorld();

	AAthleteLabPhysicsProbe* Probe = World->SpawnActor<AAthleteLabPhysicsProbe>(
		FVector(0.0, 0.0, AthleteUnits::MetersToUnreal(ReleaseHeightM)), FRotator::ZeroRotator);
	UTEST_NOT_NULL(TEXT("Spawn probe"), Probe);

	UTEST_TRUE(TEXT("Begin play"), TestWorld.BeginPlayInTestWorld());
	for (int32 Frame = 0; Frame < FrameCount; ++Frame)
	{
		TestWorld.TickTestWorld(FrameDeltaSeconds);
	}
	TestWorld.ForwardErrorMessages(this);

	const double GravityMps2 = AthleteUnits::UnrealToMeters(World->GetGravityZ()); // negative (down)

	TSharedPtr<FAthleteTelemetryTable> Table = Probe->GetTelemetryTable();
	UTEST_TRUE(TEXT("Telemetry table exists (is telemetry enabled in project settings?)"), Table.IsValid());
	const int32 Rows = Table->NumRows();
	UTEST_TRUE(*FString::Printf(TEXT("One sample per simulated frame (%d or %d rows), got %d"), FrameCount - 1, FrameCount, Rows),
		Rows == FrameCount || Rows == FrameCount - 1);

	const int32 TimeColumn = Table->FindColumn(TEXT("time_s"));
	const int32 VelColumn = Table->FindColumn(TEXT("vel_z_mps"));
	const int32 AccColumn = Table->FindColumn(TEXT("acc_z_mps2"));
	UTEST_TRUE(TEXT("Expected telemetry columns exist"), TimeColumn != INDEX_NONE && VelColumn != INDEX_NONE && AccColumn != INDEX_NONE);

	// Evenly spaced samples: every consecutive pair is exactly one frame apart.
	constexpr double TimeToleranceSeconds = 1e-5;
	int32 IrregularSteps = 0;
	for (int32 Row = 1; Row < Rows; ++Row)
	{
		const double Step = Table->GetValue(Row, TimeColumn) - Table->GetValue(Row - 1, TimeColumn);
		IrregularSteps += FMath::IsNearlyEqual(Step, static_cast<double>(FrameDeltaSeconds), TimeToleranceSeconds) ? 0 : 1;
	}
	TestEqual(TEXT("Samples are exactly one frame apart"), IrregularSteps, 0);

	// First sample: exactly one frame of free fall from rest.
	const double FirstVelocityMps = Table->GetValue(0, VelColumn);
	TestEqual(TEXT("First sample is one frame after release from rest (m/s)"),
		FirstVelocityMps, GravityMps2 * FrameDeltaSeconds, FMath::Abs(GravityMps2 * FrameDeltaSeconds) * RelativeTolerance);

	// Mean measured vertical acceleration, skipping row 0 (undefined finite difference)
	// and row 1 (the first frame may include the transition from rest).
	double AccelerationSum = 0.0;
	int32 AccelerationSamples = 0;
	for (int32 Row = 2; Row < Table->NumRows(); ++Row)
	{
		AccelerationSum += Table->GetValue(Row, AccColumn);
		++AccelerationSamples;
	}
	const double MeasuredAccelerationMps2 = AccelerationSum / AccelerationSamples;
	TestEqual(TEXT("Measured acceleration equals gravity (m/s^2)"),
		MeasuredAccelerationMps2, GravityMps2, FMath::Abs(GravityMps2) * RelativeTolerance);

	// Integrated check over the whole run: velocity gained equals g * elapsed telemetry time.
	const double ElapsedSeconds = Table->GetValue(Rows - 1, TimeColumn) - Table->GetValue(0, TimeColumn);
	const double VelocityGainMps = Table->GetValue(Rows - 1, VelColumn) - FirstVelocityMps;
	TestEqual(TEXT("Velocity gained equals g * elapsed time (m/s)"),
		VelocityGainMps, GravityMps2 * ElapsedSeconds, FMath::Abs(GravityMps2 * ElapsedSeconds) * RelativeTolerance);

	// The telemetry must agree with the physics body's own state at the end of the run.
	const double BodyVelocityMps = AthleteUnits::UnrealToMeters(Probe->GetBody()->GetPhysicsLinearVelocity().Z);
	TestEqual(TEXT("Last telemetry sample matches the body's velocity (m/s)"),
		Table->GetValue(Rows - 1, VelColumn), BodyVelocityMps, 1e-6);

	AddInfo(FString::Printf(TEXT("Gravity %.4f m/s^2, measured acceleration %.4f m/s^2, %d samples over %.4f s, velocity gain %.4f m/s"),
		GravityMps2, MeasuredAccelerationMps2, Rows, ElapsedSeconds, VelocityGainMps));

	TestWorld.EndPlayInTestWorld();
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
