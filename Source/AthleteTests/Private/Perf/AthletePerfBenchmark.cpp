// Project ATHLETE
// Performance benchmark: what does simulating a full field of athletes cost, at each physics rate?
//
// Not part of the normal suite (the name deliberately doesn't contain "Athlete", which is the
// default RunTests filter). Run it on purpose:
//   powershell -ExecutionPolicy Bypass -File Scripts\RunTests.ps1 -Filter ProjectPerf -ShowInfo
//
// It measures time; it doesn't judge it. The only checks are that the run was valid: the physics
// rate really changed (substeps per frame) and the athletes stayed on their feet (a fallen body is
// a different workload).

#include "AthleteTestFlags.h"
#include "../Physics/AthletePhysicsTestHelpers.h"
#include "Async/TaskGraphInterfaces.h"
#include "HAL/PlatformMisc.h"
#include "Misc/ScopeExit.h"
#include "PhysicsEngine/PhysicsSettings.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	constexpr double WarmUpS = 1.0;
	constexpr double MeasureS = 5.0;
	constexpr double RealTimeBudgetMs = 1000.0 / 60.0;

	/** Where athlete Index stands: two lines of 11 facing each other, 3 m apart along the line, 4 m between lines (nobody touches). */
	FVector StandingSpotM(int32 Index, FRotator& OutRotation)
	{
		const int32 Line = Index / 11;
		const int32 InLine = Index % 11;
		OutRotation = FRotator(0.0, Line == 0 ? 90.0 : -90.0, 0.0);
		return FVector(3.0 * (InLine - 5), Line == 0 ? -2.0 : 2.0, 0.0);
	}

	double Percentile(TArray<double> Values, double Fraction)
	{
		if (Values.IsEmpty())
		{
			return 0.0;
		}
		Values.Sort();
		return Values[FMath::Clamp(FMath::CeilToInt(Fraction * Values.Num()) - 1, 0, Values.Num() - 1)];
	}
}

IMPLEMENT_COMPLEX_AUTOMATION_TEST(FAthletePerfBenchmark, "ProjectPerf.Benchmark.Standing", AthleteTestFlags)

void FAthletePerfBenchmark::GetTests(TArray<FString>& OutBeautifiedNames, TArray<FString>& OutTestCommands) const
{
	for (const int32 Count : { 1, 22 })
	{
		for (const int32 Hz : { 240, 480, 1000 })
		{
			OutBeautifiedNames.Add(FString::Printf(TEXT("N%02d_%dHz"), Count, Hz));
			OutTestCommands.Add(FString::Printf(TEXT("%d %d"), Count, Hz));
		}
	}
}

bool FAthletePerfBenchmark::RunTest(const FString& Parameters)
{
	FString CountText, HzText;
	Parameters.Split(TEXT(" "), &CountText, &HzText);
	const int32 Count = FCString::Atoi(*CountText);
	const int32 Hz = FCString::Atoi(*HzText);

	// The physics rate for this run (the engine reads these every frame), restored afterwards.
	UPhysicsSettings* Physics = UPhysicsSettings::Get();
	const float OldSubstepS = Physics->MaxSubstepDeltaTime;
	const int32 OldMaxSubsteps = Physics->MaxSubsteps;
	ON_SCOPE_EXIT { Physics->MaxSubstepDeltaTime = OldSubstepS; Physics->MaxSubsteps = OldMaxSubsteps; };
	const double SubstepS = 1.0 / Hz;
	const int32 ExpectedSubsteps = FMath::CeilToInt(FAthletePhysicsTestScene::FrameDeltaSeconds / SubstepS - 1e-6);
	Physics->MaxSubstepDeltaTime = SubstepS;
	Physics->MaxSubsteps = ExpectedSubsteps + 1;

	// The athletes: standing, muscles and balance on (today's most representative full workload).
	FAthletePhysicsTestScene Scene;
	FRotator Rotation;
	const FVector FirstSpot = StandingSpotM(0, Rotation);
	if (!Scene.Initialize(*this, /*bWithFloor=*/true, FAthleteMorphology(), FirstSpot, 0, 0, true, Rotation))
	{
		return false;
	}
	const FAthleteMorphology Morphology;
	const FAthleteStrengthProfile Strength = FAthleteStrengthProfile::MakeGeneralPopulationMaleBaseline(Morphology.AgeYears, Morphology.StatureM, Morphology.BodyMassKg);
	TArray<UAthletePhysicalBodyComponent*> Bodies = { Scene.Body };
	for (int32 Index = 1; Index < Count; ++Index)
	{
		const FVector Spot = StandingSpotM(Index, Rotation);
		UAthletePhysicalBodyComponent* Body = Scene.AddBody(*this, Spot, Rotation, Morphology);
		if (!Body)
		{
			return false;
		}
		Bodies.Add(Body);
	}
	TArray<UAthleteMotorComponent*> Motors;
	TArray<double> StartComHeightM;
	for (UAthletePhysicalBodyComponent* Body : Bodies)
	{
		Motors.Add(FAthletePhysicsTestScene::AddMotorTo(Body, Strength, FAthleteMotorSkill(), /*bBalance=*/true));
		StartComHeightM.Add(Body->GetCenterOfMassM().Z);
	}

	Scene.Simulate(WarmUpS);

	// Measure frame by frame.
	const int32 Frames = FMath::RoundToInt(MeasureS / FAthletePhysicsTestScene::FrameDeltaSeconds);
	TArray<double> FrameMs;
	double MuscleMs = 0.0, ControlMs = 0.0;
	int32 MinSubsteps = MAX_int32, MaxSubsteps = 0;
	for (int32 Frame = 0; Frame < Frames; ++Frame)
	{
		const uint64 Start = FPlatformTime::Cycles64();
		Scene.TestWorld.TickTestWorld(FAthletePhysicsTestScene::FrameDeltaSeconds);
		FrameMs.Add(1000.0 * FPlatformTime::ToSeconds64(FPlatformTime::Cycles64() - Start));
		for (const UAthleteMotorComponent* Motor : Motors)
		{
			MuscleMs += Motor->GetMuscleMsLastTick();
			ControlMs += Motor->GetControlMsLastTick();
		}
		MinSubsteps = FMath::Min(MinSubsteps, Motors[0]->GetSubstepsLastTick());
		MaxSubsteps = FMath::Max(MaxSubsteps, Motors[0]->GetSubstepsLastTick());
	}

	double TotalMs = 0.0;
	for (const double Ms : FrameMs)
	{
		TotalMs += Ms;
	}
	const double MeanMs = TotalMs / Frames;
	const double MeanMuscleMs = MuscleMs / Frames;
	const double MeanControlMs = ControlMs / Frames;
	int32 Fallen = 0;
	for (int32 Index = 0; Index < Bodies.Num(); ++Index)
	{
		if (Motors[Index]->GetMotorState() == EAthleteMotorState::Fallen || Bodies[Index]->GetCenterOfMassM().Z < 0.8 * StartComHeightM[Index])
		{
			++Fallen;
		}
	}

	AddInfo(FString::Printf(TEXT("Machine: %d cores (%d logical), %d task-graph workers"),
		FPlatformMisc::NumberOfCores(), FPlatformMisc::NumberOfCoresIncludingHyperthreads(), FTaskGraphInterface::Get().GetNumWorkerThreads()));
	AddInfo(FString::Printf(TEXT("%d athlete(s) at %d Hz (%d substeps per 60 Hz frame): frame %.2f ms mean, %.2f ms p95, %.2f ms max; %.3f ms per athlete"),
		Count, Hz, MaxSubsteps, MeanMs, Percentile(FrameMs, 0.95), Percentile(FrameMs, 1.0), MeanMs / Count));
	AddInfo(FString::Printf(TEXT("  of which: muscles %.2f ms (physics thread), controllers %.2f ms (game thread), Chaos solve + engine %.2f ms"),
		MeanMuscleMs, MeanControlMs, MeanMs - MeanMuscleMs - MeanControlMs));
	AddInfo(FString::Printf(TEXT("  real-time budget (60 FPS, %.1f ms for everything): physics alone uses %.0f%%"), RealTimeBudgetMs, 100.0 * MeanMs / RealTimeBudgetMs));
	AddInfo(FString::Printf(TEXT("BENCH n=%d hz=%d substeps=%d mean_ms=%.3f p95_ms=%.3f max_ms=%.3f muscle_ms=%.3f control_ms=%.3f fallen=%d"),
		Count, Hz, MaxSubsteps, MeanMs, Percentile(FrameMs, 0.95), Percentile(FrameMs, 1.0), MeanMuscleMs, MeanControlMs, Fallen));

	// Validity only.
	TestEqual(TEXT("Physics ran at the requested rate (substeps per frame)"), MinSubsteps, ExpectedSubsteps);
	TestEqual(TEXT("Physics ran at the requested rate (substeps per frame, max)"), MaxSubsteps, ExpectedSubsteps);
	if (Fallen > 0)
	{
		AddWarning(FString::Printf(TEXT("%d athlete(s) fell: the workload wasn't all standing bodies"), Fallen));
	}
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
