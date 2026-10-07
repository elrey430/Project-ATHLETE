// Project ATHLETE

#include "AthleteTrackerPolicy.h"

#include "AthleteTrackerBundle.h"

namespace
{
	bool CopyFloats(const FAthleteTrackerBundle& Bundle, const TCHAR* Name, TArray<float>& Out, FString& OutError)
	{
		const FAthleteTrackerArray* Array = Bundle.FindTyped(Name, TEXT("float32"));
		if (!Array)
		{
			OutError = FString::Printf(TEXT("Bundle has no float32 array %s"), Name);
			return false;
		}
		Out.SetNumUninitialized(static_cast<int32>(Array->Num()));
		FMemory::Memcpy(Out.GetData(), Array->Data<float>(), Array->Bytes.Num());
		return true;
	}

	/** Out = tanh?(In * W + B), W stored (inputs, outputs) row-major as Flax keeps it. */
	void Dense(const float* In, int32 NumIn, const TArray<float>& W, const TArray<float>& B, float* Out, bool bTanh)
	{
		const int32 NumOut = B.Num();
		FMemory::Memcpy(Out, B.GetData(), NumOut * sizeof(float));
		for (int32 I = 0; I < NumIn; ++I)
		{
			const float X = In[I];
			const float* Row = W.GetData() + static_cast<int64>(I) * NumOut;
			for (int32 J = 0; J < NumOut; ++J)
			{
				Out[J] += X * Row[J];
			}
		}
		if (bTanh)
		{
			for (int32 J = 0; J < NumOut; ++J)
			{
				Out[J] = FMath::Tanh(Out[J]);
			}
		}
	}
}

bool FAthleteTrackerPolicy::Initialize(const FAthleteTrackerBundle& Bundle, FString& OutError)
{
	TArray<float> Var;
	if (!CopyFloats(Bundle, TEXT("policy_obs_mean"), Mean, OutError) || !CopyFloats(Bundle, TEXT("policy_obs_var"), Var, OutError)
		|| !CopyFloats(Bundle, TEXT("policy_w0"), W0, OutError) || !CopyFloats(Bundle, TEXT("policy_b0"), B0, OutError)
		|| !CopyFloats(Bundle, TEXT("policy_w1"), W1, OutError) || !CopyFloats(Bundle, TEXT("policy_b1"), B1, OutError)
		|| !CopyFloats(Bundle, TEXT("policy_w2"), W2, OutError) || !CopyFloats(Bundle, TEXT("policy_b2"), B2, OutError))
	{
		return false;
	}
	if (W0.Num() != Mean.Num() * B0.Num() || W1.Num() != B0.Num() * B1.Num() || W2.Num() != B1.Num() * B2.Num() || Var.Num() != Mean.Num())
	{
		OutError = TEXT("Policy layer sizes don't match");
		return false;
	}
	// LocoMuJoCo's RunningMeanStd: (x - mean) / sqrt(var + 1e-8).
	InvStd.SetNumUninitialized(Var.Num());
	for (int32 I = 0; I < Var.Num(); ++I)
	{
		InvStd[I] = 1.0f / FMath::Sqrt(Var[I] + 1e-8f);
	}
	Input.SetNumUninitialized(Mean.Num());
	Hidden0.SetNumUninitialized(B0.Num());
	Hidden1.SetNumUninitialized(B1.Num());
	return true;
}

void FAthleteTrackerPolicy::Act(TConstArrayView<double> Observation, TArrayView<double> OutAction) const
{
	check(Observation.Num() == Mean.Num() && OutAction.Num() == B2.Num());
	for (int32 I = 0; I < Mean.Num(); ++I)
	{
		Input[I] = (static_cast<float>(Observation[I]) - Mean[I]) * InvStd[I];
	}
	Dense(Input.GetData(), Input.Num(), W0, B0, Hidden0.GetData(), true);
	Dense(Hidden0.GetData(), Hidden0.Num(), W1, B1, Hidden1.GetData(), true);
	TArray<float, TInlineAllocator<64>> Out;
	Out.SetNumUninitialized(B2.Num());
	Dense(Hidden1.GetData(), Hidden1.Num(), W2, B2, Out.GetData(), false);
	for (int32 J = 0; J < Out.Num(); ++J)
	{
		OutAction[J] = Out[J];
	}
}
