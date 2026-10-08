// Project ATHLETE

#include "AthleteTestFlags.h"
#include "AthleteTrackerBundle.h"
#include "AthleteTrackerSim.h"
#include "HAL/PlatformTime.h"
#include "Math/RandomStream.h"

#if WITH_DEV_AUTOMATION_TESTS

// Milestone 4's acceptance tests (Scripts/MuJoCo/locomotion_tests.py), on the learned athlete running inside
// Unreal: same command schedules, trials, measures and criteria. The commands pass through the controller
// layer (command shaper), as in the game. Each test: 5 trials from different start moments, 4 must pass.

namespace AthleteTrackerAcceptance
{
	const int32 TrialOffsets[] = {0, 17, 34, 51, 68};
	constexpr int32 NeedTrials = 4;

	struct FRecord
	{
		double X, Y, Yaw, Forward;
	};

	struct FSegment
	{
		double Seconds;
		FAthleteCommand Command;
	};

	/** Runs a schedule from a standing start. Returns the records; bOutFell if the athlete fell. */
	TArray<FRecord> Run(FAthleteTrackerSim& Sim, const TArray<FSegment>& Schedule, int32 StartOffset, bool& bOutFell)
	{
		TArray<FRecord> Records;
		Sim.Reset(Schedule[0].Command, StartOffset);
		bOutFell = false;
		for (const FSegment& Segment : Schedule)
		{
			for (int32 I = 0; I < FMath::RoundToInt(Segment.Seconds / Sim.ControlDt()); ++I)
			{
				bOutFell = Sim.Step(Segment.Command);
				const FVector P = Sim.PelvisPositionM();
				Records.Add({P.X, P.Y, Sim.HeadingRad(), Sim.ForwardSpeedMps()});
				if (bOutFell)
				{
					return Records;
				}
			}
		}
		return Records;
	}

	/** numpy.convolve(values, ones(50) / 50, "same"). */
	TArray<double> Smooth(const TArray<double>& Values, int32 N = 50)
	{
		if (Values.Num() <= N)
		{
			return Values;
		}
		TArray<double> Out;
		Out.SetNumZeroed(Values.Num());
		for (int32 I = 0; I < Values.Num(); ++I)
		{
			double Sum = 0.0;
			for (int32 J = FMath::Max(0, I - N / 2); J <= FMath::Min(Values.Num() - 1, I + N / 2 - 1); ++J)
			{
				Sum += Values[J];
			}
			Out[I] = Sum / N;
		}
		return Out;
	}

	TArray<double> Forward(const TArray<FRecord>& Records)
	{
		TArray<double> Out;
		for (const FRecord& R : Records) Out.Add(R.Forward);
		return Out;
	}

	double HeadingChange(const TArray<FRecord>& Records, int32 Start, int32 End)
	{
		double Change = 0.0;
		for (int32 I = Start + 1; I < FMath::Min(End, Records.Num()); ++I)
		{
			Change += FMath::UnwindRadians(Records[I].Yaw - Records[I - 1].Yaw);
		}
		return Change;
	}

	TSharedPtr<FAthleteTrackerSim> MakeSim(FAutomationTestBase& Test)
	{
		FString Error;
		TSharedPtr<const FAthleteTrackerBundle> Bundle = FAthleteTrackerBundle::Load(FAthleteTrackerBundle::DefaultDirectory(), Error);
		TSharedPtr<FAthleteTrackerSim> Sim = MakeShared<FAthleteTrackerSim>();
		if (!Bundle || !Sim->Initialize(Bundle, Error))
		{
			Test.AddError(Error);
			return nullptr;
		}
		return Sim;
	}

	/** Runs a trial function over the 5 start offsets; passes if NeedTrials pass. */
	bool RunTrials(FAutomationTestBase& Test, const TCHAR* Name, const TCHAR* Criteria,
		TFunctionRef<bool(FAthleteTrackerSim&, int32, bool&, FString&)> Trial)
	{
		TSharedPtr<FAthleteTrackerSim> Sim = MakeSim(Test);
		if (!Sim) return false;
		int32 Passed = 0, Falls = 0;
		TArray<FString> Measures;
		const double Start = FPlatformTime::Seconds();
		for (const int32 Offset : TrialOffsets)
		{
			bool bFell = false;
			FString Measure;
			Passed += Trial(*Sim, Offset, bFell, Measure) ? 1 : 0;
			Falls += bFell ? 1 : 0;
			Measures.Add(Measure);
		}
		Test.AddInfo(FString::Printf(TEXT("%s: %d/5 trials passed, %d falls | %s | %.1f s wall"), Name, Passed, Falls,
			*FString::Join(Measures, TEXT("; ")), FPlatformTime::Seconds() - Start));
		Test.TestTrue(FString::Printf(TEXT("%s (%s), at least %d of 5 trials"), Name, Criteria, NeedTrials), Passed >= NeedTrials);
		return true;
	}
}

using namespace AthleteTrackerAcceptance;

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteTrackerAccelerateTest, "Athlete.Tracker.Acceptance.Accelerate", AthleteTestFlags)

bool FAthleteTrackerAccelerateTest::RunTest(const FString& Parameters)
{
	return RunTrials(*this, TEXT("accelerate"), TEXT("no fall; 90% of 1.5 m/s within 3 s; mean speed error < 0.25 m/s over the last 2 s"),
		[](FAthleteTrackerSim& Sim, int32 Offset, bool& bFell, FString& Measure)
		{
			const TArray<FRecord> Records = Run(Sim, {{2.0, {0, 0, 0}}, {6.0, {1.5, 0, 0}}}, Offset, bFell);
			if (bFell) { Measure = TEXT("fell"); return false; }
			const TArray<double> Speed = Smooth(Forward(Records));
			const int32 Switch = FMath::RoundToInt(2.0 / Sim.ControlDt());
			double T90 = -1.0;
			for (int32 I = Switch; I < Speed.Num(); ++I)
			{
				if (Speed[I] >= 0.9 * 1.5) { T90 = (I - Switch) * Sim.ControlDt(); break; }
			}
			double Error = 0.0;
			const int32 Last = FMath::RoundToInt(2.0 / Sim.ControlDt());
			for (int32 I = Speed.Num() - Last; I < Speed.Num(); ++I) Error += FMath::Abs(Speed[I] - 1.5) / Last;
			Measure = FString::Printf(TEXT("t90 %.2f s, error %.3f"), T90, Error);
			return T90 >= 0.0 && T90 <= 3.0 && Error < 0.25;
		});
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteTrackerBrakeTest, "Athlete.Tracker.Acceptance.Brake", AthleteTestFlags)

bool FAthleteTrackerBrakeTest::RunTest(const FString& Parameters)
{
	return RunTrials(*this, TEXT("brake"), TEXT("no fall; below 0.15 m/s within 2.5 s of the stop command"),
		[](FAthleteTrackerSim& Sim, int32 Offset, bool& bFell, FString& Measure)
		{
			const TArray<FRecord> Records = Run(Sim, {{6.0, {1.5, 0, 0}}, {4.0, {0, 0, 0}}}, Offset, bFell);
			if (bFell) { Measure = TEXT("fell"); return false; }
			const TArray<double> Speed = Smooth(Forward(Records));
			const int32 Switch = FMath::RoundToInt(6.0 / Sim.ControlDt());
			const int32 Hold = FMath::RoundToInt(1.0 / Sim.ControlDt());
			const int32 Remaining = Speed.Num() - Switch;
			int32 StopIndex = -1;
			for (int32 I = 0; I < Remaining - Hold && StopIndex < 0; ++I)
			{
				bool bStill = true;
				for (int32 J = I; J < I + Hold && bStill; ++J) bStill = FMath::Abs(Speed[Switch + J]) < 0.15;
				if (bStill) StopIndex = I;
			}
			const double StopS = StopIndex < 0 ? -1.0 : StopIndex * Sim.ControlDt();
			Measure = FString::Printf(TEXT("before %.2f m/s, stop %.2f s"), Speed[Switch - 1], StopS);
			return StopS >= 0.0 && StopS <= 2.5;
		});
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteTrackerTurnTest, "Athlete.Tracker.Acceptance.Turn", AthleteTestFlags)

bool FAthleteTrackerTurnTest::RunTest(const FString& Parameters)
{
	return RunTrials(*this, TEXT("turn"), TEXT("no fall; heading change within 30% of the commanded 3.2 rad"),
		[](FAthleteTrackerSim& Sim, int32 Offset, bool& bFell, FString& Measure)
		{
			const TArray<FRecord> Records = Run(Sim, {{3.0, {1.0, 0, 0}}, {4.0, {1.0, 0, 0.8}}, {2.0, {1.0, 0, 0}}}, Offset, bFell);
			if (bFell) { Measure = TEXT("fell"); return false; }
			const double Change = HeadingChange(Records, FMath::RoundToInt(3.0 / Sim.ControlDt()), FMath::RoundToInt(7.0 / Sim.ControlDt()));
			Measure = FString::Printf(TEXT("%.2f rad"), Change);
			return FMath::Abs(Change - 3.2) <= 0.3 * 3.2;
		});
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteTrackerTurnOnSpotTest, "Athlete.Tracker.Acceptance.TurnOnSpot", AthleteTestFlags)

bool FAthleteTrackerTurnOnSpotTest::RunTest(const FString& Parameters)
{
	return RunTrials(*this, TEXT("turn_on_spot"), TEXT("no fall; heading change within 30% of the commanded 4.0 rad"),
		[](FAthleteTrackerSim& Sim, int32 Offset, bool& bFell, FString& Measure)
		{
			const TArray<FRecord> Records = Run(Sim, {{2.0, {0, 0, 0}}, {4.0, {0, 0, 1.0}}}, Offset, bFell);
			if (bFell) { Measure = TEXT("fell"); return false; }
			const int32 Switch = FMath::RoundToInt(2.0 / Sim.ControlDt());
			const double Change = HeadingChange(Records, Switch, FMath::RoundToInt(6.0 / Sim.ControlDt()));
			const double Drift = FMath::Sqrt(FMath::Square(Records.Last().X - Records[Switch].X) + FMath::Square(Records.Last().Y - Records[Switch].Y));
			Measure = FString::Printf(TEXT("%.2f rad, drift %.2f m"), Change, Drift);
			return FMath::Abs(Change - 4.0) <= 0.3 * 4.0;
		});
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteTrackerRunTest, "Athlete.Tracker.Acceptance.Run", AthleteTestFlags)

bool FAthleteTrackerRunTest::RunTest(const FString& Parameters)
{
	return RunTrials(*this, TEXT("run"), TEXT("no fall; mean speed over the last 2 s at least 2.0 m/s"),
		[](FAthleteTrackerSim& Sim, int32 Offset, bool& bFell, FString& Measure)
		{
			const TArray<FRecord> Records = Run(Sim, {{1.0, {0, 0, 0}}, {6.0, {2.5, 0, 0}}}, Offset, bFell);
			if (bFell) { Measure = TEXT("fell"); return false; }
			const TArray<double> Speed = Smooth(Forward(Records));
			const int32 Last = FMath::RoundToInt(2.0 / Sim.ControlDt());
			double Mean = 0.0;
			for (int32 I = Speed.Num() - Last; I < Speed.Num(); ++I) Mean += Speed[I] / Last;
			Measure = FString::Printf(TEXT("%.2f m/s"), Mean);
			return Mean >= 2.0;
		});
}

namespace AthleteTrackerAcceptance
{
	/**
	 * Episodes of 20 s: a 2 s stand, then a new random command every 3 s (stand 15%; forward 0..3 m/s;
	 * sideways +-0.5 m/s 30% of the time; turning +-1.2 rad/s half the time). Same distribution as Python,
	 * different samples (Unreal's random stream). Returns the falls; OutMeanError: forward-speed error over the
	 * second half of each command.
	 */
	int32 RunRandomCommands(FAutomationTestBase& Test, int32 Episodes, int32 Seed, double& OutMeanError);
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteTrackerRandomTest, "Athlete.Tracker.Acceptance.RandomCommands", AthleteTestFlags)

bool FAthleteTrackerRandomTest::RunTest(const FString& Parameters)
{
	// Milestone 4's goal is a RATE: at most 1 fall per 10 episodes (Docs/Milestone4_Locomotion.md, 9.6), and a
	// mean forward-speed error < 0.4 m/s. 200 episodes (~4 min) measure the rate; 10 are mostly luck (a
	// tracker falling in 4% of episodes fails a 10-episode test 6% of the time). Until 2026-10-07 this checked
	// "at most 1 fall" whatever the count, which with 40 episodes was four times stricter than the goal.
	// The football bar (at most 1 per 100) is reported as information: Athlete.Tracker.Study.FootballFallBar.
	constexpr int32 Episodes = 200;
	double MeanError = 0.0;
	const int32 Falls = RunRandomCommands(*this, Episodes, 3, MeanError);
	TestTrue(FString::Printf(TEXT("At most 1 fall per 10 episodes (%d falls in %d)"), Falls, Episodes), Falls >= 0 && Falls * 10 <= Episodes);
	TestTrue(TEXT("Mean forward-speed error < 0.4 m/s"), MeanError < 0.4);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteTrackerFootballBarStudy, "Athlete.Tracker.Study.FootballFallBar", AthleteTestFlags)

bool FAthleteTrackerFootballBarStudy::RunTest(const FString& Parameters)
{
	// A measurement against the stricter football bar: at most 1 fall per 100 random-command episodes
	// (about one fall per half hour of random play). 400 episodes on different samples from the
	// acceptance test. Reported, not asserted, until a tracker reaches it.
	constexpr int32 Episodes = 400;
	double MeanError = 0.0;
	const int32 Falls = RunRandomCommands(*this, Episodes, 11, MeanError);
	AddInfo(FString::Printf(TEXT("Football bar (<= 1 fall per 100 episodes): %d falls in %d = %.1f per 100 -> %s"),
		Falls, Episodes, 100.0 * Falls / Episodes, Falls * 100 <= Episodes ? TEXT("MET") : TEXT("not met")));
	return true;
}

int32 AthleteTrackerAcceptance::RunRandomCommands(FAutomationTestBase& Test, int32 Episodes, int32 Seed, double& OutMeanError)
{
	TSharedPtr<FAthleteTrackerSim> Sim = MakeSim(Test);
	if (!Sim) return -1;
	FRandomStream Random(Seed);
	int32 Falls = 0;
	double ErrorSum = 0.0;
	int32 ErrorCount = 0;
	int64 StepsRun = 0;
	const double Start = FPlatformTime::Seconds();
	TArray<FString> FallNotes;
	for (int32 Episode = 0; Episode < Episodes; ++Episode)
	{
		TArray<FSegment> Schedule = {{2.0, {0, 0, 0}}};
		for (int32 I = 0; I < 6; ++I)
		{
			const bool bStand = Random.FRand() < 0.15;
			FAthleteCommand Command;
			if (!bStand)
			{
				Command.Forward = Random.FRandRange(0.0, 3.0);
				Command.Sideways = Random.FRand() < 0.3 ? Random.FRandRange(-0.5, 0.5) : 0.0;
				Command.Turn = Random.FRand() < 0.5 ? Random.FRandRange(-1.2, 1.2) : 0.0;
			}
			Schedule.Add({3.0, Command});
		}
		bool bFell = false;
		const TArray<FRecord> Records = Run(*Sim, Schedule, TrialOffsets[Episode % 5], bFell);
		StepsRun += Records.Num();
		if (bFell)
		{
			++Falls;
			const double FallS = Records.Num() * Sim->ControlDt();
			const int32 Segment = FMath::Min(Schedule.Num() - 1, FallS < 2.0 ? 0 : 1 + FMath::FloorToInt((FallS - 2.0) / 3.0));
			const FAthleteCommand& Now = Schedule[Segment].Command;
			const FAthleteCommand& Before = Schedule[FMath::Max(0, Segment - 1)].Command;
			const double Into = FallS - (Segment == 0 ? 0.0 : 2.0 + 3.0 * (Segment - 1));
			FallNotes.Add(FString::Printf(TEXT("ep %d at %.1f s, %.1f s into (%.2f, %+.2f, %+.2f) after (%.2f, %+.2f, %+.2f)"),
				Episode, FallS, Into, Now.Forward, Now.Sideways, Now.Turn, Before.Forward, Before.Sideways, Before.Turn));
		}
		// Speed error over the second half of each command.
		const TArray<double> Speed = Smooth(Forward(Records));
		double SegmentStart = 0.0, Error = 0.0;
		int32 Count = 0;
		for (const FSegment& Segment : Schedule)
		{
			for (int32 I = FMath::RoundToInt((SegmentStart + 0.5 * Segment.Seconds) / Sim->ControlDt() + 1);
				 I < FMath::Min(Speed.Num(), FMath::RoundToInt((SegmentStart + Segment.Seconds) / Sim->ControlDt())); ++I)
			{
				Error += FMath::Abs(Speed[I] - Segment.Command.Forward);
				++Count;
			}
			SegmentStart += Segment.Seconds;
		}
		if (Count > 0)
		{
			ErrorSum += Error / Count;
			++ErrorCount;
		}
	}
	const double MeanError = ErrorCount > 0 ? ErrorSum / ErrorCount : 0.0;
	const double Wall = FPlatformTime::Seconds() - Start;
	Test.AddInfo(FString::Printf(TEXT("random: %d episodes, %d falls (%s); forward-speed error %.3f m/s; %.1f s wall for %.0f s simulated (%.2f ms per 10 ms step)"),
		Episodes, Falls, FallNotes.Num() ? *FString::Join(FallNotes, TEXT(", ")) : TEXT("none"), MeanError, Wall,
		StepsRun * Sim->ControlDt(), 1000.0 * Wall / FMath::Max<int64>(StepsRun, 1)));
	OutMeanError = MeanError;
	return Falls;
}

#endif

#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/FileHelper.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteTrackerPythonSchedulesStudy, "Athlete.Tracker.Study.PythonRandomSchedules", AthleteTestFlags)

bool FAthleteTrackerPythonSchedulesStudy::RunTest(const FString& Parameters)
{
	// A measurement: the random-command episodes of locomotion_tests.py exactly (its numpy schedules,
	// exported to the bundle folder as python_random_schedules.json), so the C++ fall count compares
	// directly with Python's (chunk 55: 1 fall in 100).
	FString Text;
	if (!FFileHelper::LoadFileToString(Text, *FPaths::Combine(FAthleteTrackerBundle::DefaultDirectory(), TEXT("python_random_schedules.json"))))
	{
		AddWarning(TEXT("No python_random_schedules.json in the bundle folder: nothing to compare"));
		return true;
	}
	TSharedPtr<FJsonObject> Root;
	FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text), Root);
	TSharedPtr<FAthleteTrackerSim> Sim = MakeSim(*this);
	if (!Sim || !Root) return false;
	int32 Falls = 0, Episode = 0;
	TArray<FString> Notes;
	for (const TSharedPtr<FJsonValue>& EpisodeValue : Root->GetArrayField(TEXT("episodes")))
	{
		TArray<FSegment> Schedule;
		for (const TSharedPtr<FJsonValue>& Segment : EpisodeValue->AsArray())
		{
			const TArray<TSharedPtr<FJsonValue>>& Pair = Segment->AsArray();
			const TArray<TSharedPtr<FJsonValue>>& C = Pair[1]->AsArray();
			Schedule.Add({Pair[0]->AsNumber(), {C[0]->AsNumber(), C[1]->AsNumber(), C[2]->AsNumber()}});
		}
		bool bFell = false;
		const TArray<FRecord> Records = Run(*Sim, Schedule, TrialOffsets[Episode % 5], bFell);
		if (bFell)
		{
			++Falls;
			Notes.Add(FString::Printf(TEXT("episode %d at %.1f s"), Episode, Records.Num() * Sim->ControlDt()));
		}
		++Episode;
	}
	AddInfo(FString::Printf(TEXT("Python's %d random schedules in C++: %d falls (%s)"), Episode, Falls,
		Notes.Num() ? *FString::Join(Notes, TEXT(", ")) : TEXT("none")));
	return true;
}

#endif
