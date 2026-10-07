// Project ATHLETE

#include "AthleteTestFlags.h"
#include "AthleteTrackerBundle.h"
#include "AthleteTrackerPolicy.h"
#include "AthleteTrackerSim.h"

THIRD_PARTY_INCLUDES_START
#include <mujoco/mujoco.h>
THIRD_PARTY_INCLUDES_END

#if WITH_DEV_AUTOMATION_TESTS

// The C++ learned athlete against a session recorded from the Python test driver (export_unreal_bundle.py):
// same command schedule, same start. These say WHERE the port departs from Python, step by step.

namespace AthleteTrackerParity
{
	TSharedPtr<const FAthleteTrackerBundle> LoadBundle(FAutomationTestBase& Test)
	{
		FString Error;
		TSharedPtr<const FAthleteTrackerBundle> Bundle = FAthleteTrackerBundle::Load(FAthleteTrackerBundle::DefaultDirectory(), Error);
		if (!Bundle.IsValid())
		{
			Test.AddError(Error);
		}
		return Bundle;
	}

	/** Row Index of a 2-D float64 golden array. */
	TConstArrayView<double> Row(const FAthleteTrackerBundle& Bundle, const TCHAR* Name, int32 Index)
	{
		const FAthleteTrackerArray* Array = Bundle.FindTyped(Name, TEXT("float64"));
		check(Array);
		const int64 Width = Array->Dim(1);
		return TConstArrayView<double>(Array->Data<double>() + Index * Width, static_cast<int32>(Width));
	}

	double MaxAbsDiff(TConstArrayView<double> A, TConstArrayView<double> B, int32* OutWorst = nullptr)
	{
		double Max = 0.0;
		for (int32 I = 0; I < FMath::Min(A.Num(), B.Num()); ++I)
		{
			const double D = FMath::Abs(A[I] - B[I]);
			if (D > Max)
			{
				Max = D;
				if (OutWorst) *OutWorst = I;
			}
		}
		return Max;
	}

	/** The golden schedule expanded to one command per control step. */
	TArray<FAthleteCommand> Commands(const FAthleteTrackerBundle& Bundle, double Dt)
	{
		TArray<FAthleteCommand> Out;
		for (const TSharedPtr<FJsonValue>& Entry : Bundle.Constants().GetObjectField(TEXT("golden"))->GetArrayField(TEXT("schedule")))
		{
			const TArray<TSharedPtr<FJsonValue>>& Pair = Entry->AsArray();
			const TArray<TSharedPtr<FJsonValue>>& C = Pair[1]->AsArray();
			const FAthleteCommand Command{C[0]->AsNumber(), C[1]->AsNumber(), C[2]->AsNumber()};
			for (int32 I = 0; I < FMath::RoundToInt(Pair[0]->AsNumber() / Dt); ++I)
			{
				Out.Add(Command);
			}
		}
		return Out;
	}
}

using namespace AthleteTrackerParity;

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteTrackerPolicyParityTest, "Athlete.Tracker.Parity.Policy", AthleteTestFlags)

bool FAthleteTrackerPolicyParityTest::RunTest(const FString& Parameters)
{
	TSharedPtr<const FAthleteTrackerBundle> Bundle = LoadBundle(*this);
	if (!Bundle) return false;
	FAthleteTrackerPolicy Policy;
	FString Error;
	if (!Policy.Initialize(*Bundle, Error))
	{
		AddError(Error);
		return false;
	}
	const int32 Steps = Bundle->FindTyped(TEXT("golden_action"), TEXT("float64"))->Dim(0);
	TArray<double> Action;
	Action.SetNum(Policy.ActionDim());
	double Worst = 0.0;
	for (int32 K = 0; K < Steps; ++K)
	{
		Policy.Act(Row(*Bundle, TEXT("golden_obs"), K), Action);
		Worst = FMath::Max(Worst, MaxAbsDiff(Action, Row(*Bundle, TEXT("golden_action"), K)));
	}
	AddInfo(FString::Printf(TEXT("Policy on %d golden observations: largest action difference %.2e"), Steps, Worst));
	TestTrue(TEXT("Actions match Python (float32 network: < 1e-4)"), Worst < 1e-4);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteTrackerResetParityTest, "Athlete.Tracker.Parity.Reset", AthleteTestFlags)

bool FAthleteTrackerResetParityTest::RunTest(const FString& Parameters)
{
	TSharedPtr<const FAthleteTrackerBundle> Bundle = LoadBundle(*this);
	if (!Bundle) return false;
	FAthleteTrackerSim Sim;
	FString Error;
	if (!Sim.Initialize(Bundle, Error))
	{
		AddError(Error);
		return false;
	}
	const TArray<FAthleteCommand> Commands = AthleteTrackerParity::Commands(*Bundle, Sim.ControlDt());
	Sim.Reset(Commands[0]);

	const FAthleteTrackerArray* Initial = Bundle->FindTyped(TEXT("golden_initial_qpos"), TEXT("float64"));
	const TConstArrayView<double> Qpos(Sim.Data()->qpos, Sim.Model()->nq);
	TestTrue(TEXT("Initial pose matches"), MaxAbsDiff(Qpos, TConstArrayView<double>(Initial->Data<double>(), Initial->Num())) < 1e-5);

	const int32 Rows = Bundle->FindTyped(TEXT("golden_reset_ref_qpos"), TEXT("float64"))->Dim(0);
	double WorstRow = 0.0;
	for (int32 R = 0; R < Rows; ++R)
	{
		TArray<double> RefQ, RefV;
		Sim.ReferenceRow(R, RefQ, RefV);
		WorstRow = FMath::Max(WorstRow, MaxAbsDiff(RefQ, Row(*Bundle, TEXT("golden_reset_ref_qpos"), R)));
		WorstRow = FMath::Max(WorstRow, MaxAbsDiff(RefV, Row(*Bundle, TEXT("golden_reset_ref_qvel"), R)));
	}
	int32 WorstIndex = -1;
	const double WorstObs = MaxAbsDiff(Sim.Observation(), Row(*Bundle, TEXT("golden_obs"), 0), &WorstIndex);
	AddInfo(FString::Printf(TEXT("Reset: reference rows differ by %.2e at most; observation by %.2e (index %d)"), WorstRow, WorstObs, WorstIndex));
	TestTrue(TEXT("Queued reference rows match"), WorstRow < 1e-4);
	TestTrue(TEXT("Observation at reset matches"), WorstObs < 1e-4);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteTrackerSessionParityTest, "Athlete.Tracker.Parity.Session", AthleteTestFlags)

bool FAthleteTrackerSessionParityTest::RunTest(const FString& Parameters)
{
	TSharedPtr<const FAthleteTrackerBundle> Bundle = LoadBundle(*this);
	if (!Bundle) return false;
	const FAthleteTrackerArray* Frames = Bundle->FindTyped(TEXT("golden_frame"), TEXT("int32"));

	// Two runs: the golden actions forced (physics, matcher and observation alone), then the C++ policy.
	// The closed loop is chaotic (contacts): in Python alone, 1e-6 of action noise moves the athlete 0.18 m
	// off its path within 0.5 s and ~0.3 m by 5 s (2026-10-05). So step-by-step agreement is checked only
	// briefly; for the whole session, the same behaviour: no fall, ending in about the same place.
	for (const bool bForced : {true, false})
	{
		FAthleteTrackerSim Sim;
		FString Error;
		if (!Sim.Initialize(Bundle, Error))
		{
			AddError(Error);
			return false;
		}
		const TArray<FAthleteCommand> Commands = AthleteTrackerParity::Commands(*Bundle, Sim.ControlDt());
		Sim.Reset(Commands[0]);
		const int32 Steps = FMath::Min(Commands.Num(), static_cast<int32>(Frames->Dim(0)));
		int32 FirstFrameMismatch = -1, FirstObsMismatch = -1;
		double WorstCommand = 0.0, WorstObsEarly = 0.0, WorstQposEarly = 0.0, WorstQpos = 0.0;
		bool bFell = false;
		for (int32 K = 0; K < Steps && !bFell; ++K)
		{
			int32 ObsIndex = -1;
			const double ObsDiff = MaxAbsDiff(Sim.Observation(), Row(*Bundle, TEXT("golden_obs"), K), &ObsIndex);
			if (FirstObsMismatch < 0 && ObsDiff > 1e-3)
			{
				FirstObsMismatch = K;
				AddInfo(FString::Printf(TEXT("%s: observation first differs by > 1e-3 at step %d (index %d, %.2e)"),
					bForced ? TEXT("Forced") : TEXT("Policy"), K, ObsIndex, ObsDiff));
			}
			if (K < (bForced ? 50 : 5)) WorstObsEarly = FMath::Max(WorstObsEarly, ObsDiff);
			bFell = Sim.Step(Commands[K], bForced ? Row(*Bundle, TEXT("golden_action"), K) : TConstArrayView<double>());
			const FAthleteCommand& Shaped = Sim.ShapedCommand();
			const TArray<double> ShapedValues = {Shaped.Forward, Shaped.Sideways, Shaped.Turn};
			WorstCommand = FMath::Max(WorstCommand, MaxAbsDiff(ShapedValues, Row(*Bundle, TEXT("golden_command"), K)));
			if (FirstFrameMismatch < 0 && Sim.Matcher().Frame() != Frames->Data<int32>()[K])
			{
				FirstFrameMismatch = K;
			}
			const double QposDiff = MaxAbsDiff(TConstArrayView<double>(Sim.Data()->qpos, Sim.Model()->nq), Row(*Bundle, TEXT("golden_qpos"), K));
			if (K < (bForced ? 50 : 5)) WorstQposEarly = FMath::Max(WorstQposEarly, QposDiff);
			WorstQpos = FMath::Max(WorstQpos, QposDiff);
		}
		const TCHAR* Mode = bForced ? TEXT("Forced golden actions") : TEXT("C++ policy");
		const TConstArrayView<double> GoldenEnd = Row(*Bundle, TEXT("golden_qpos"), Steps - 1);
		const double EndDistance = FMath::Sqrt(FMath::Square(Sim.Data()->qpos[0] - GoldenEnd[0]) + FMath::Square(Sim.Data()->qpos[1] - GoldenEnd[1]));
		const double YawGolden = FMath::Atan2(2 * (GoldenEnd[4] * GoldenEnd[5] + GoldenEnd[3] * GoldenEnd[6]), 1 - 2 * (GoldenEnd[5] * GoldenEnd[5] + GoldenEnd[6] * GoldenEnd[6]));
		const double EndYaw = FMath::Abs(FMath::UnwindRadians(Sim.HeadingRad() - YawGolden));
		AddInfo(FString::Printf(TEXT("%s, %d steps: command diff %.1e; first matcher-frame mismatch at step %d; early obs diff %.1e, "
			"early qpos diff %.1e, whole-session qpos diff %.1e; end: %.2f m and %.2f rad from Python's; fell: %s"),
			Mode, Steps, WorstCommand, FirstFrameMismatch, WorstObsEarly, WorstQposEarly, WorstQpos, EndDistance, EndYaw, bFell ? TEXT("yes") : TEXT("no")));
		TestTrue(FString::Printf(TEXT("%s: commands shaped identically"), Mode), WorstCommand < 1e-9);
		TestTrue(FString::Printf(TEXT("%s: observations match at first"), Mode), WorstObsEarly < 1e-3);
		TestTrue(FString::Printf(TEXT("%s: the body follows the same path at first"), Mode), WorstQposEarly < 1e-3);
		if (!bForced)
		{
			// Forced actions are open loop: they can't correct the small drift, so only the policy run must stay up.
			TestFalse(TEXT("C++ policy: no fall (the Python session didn't fall)"), bFell);
			TestTrue(TEXT("C++ policy: ends within 1 m and 0.5 rad of Python's athlete"), EndDistance < 1.0 && EndYaw < 0.5);
		}
	}
	return true;
}

#endif
