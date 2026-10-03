// Project ATHLETE

#pragma once

#include "AthleteStandingTestHelpers.h"

#if WITH_DEV_AUTOMATION_TESTS

/** One entry of a movement script: from TimeS on, the athlete wants this. */
struct FAthleteIntentKey
{
	double TimeS = 0.0;
	FAthleteMovementIntent Intent;

	static FAthleteIntentKey Velocity(double TimeS, double XMps, double YMps)
	{
		FAthleteIntentKey Key;
		Key.TimeS = TimeS;
		Key.Intent.DesiredVelocityMps = FVector2D(XMps, YMps);
		return Key;
	}
};

/** How to run one walking trial (the athlete starts standing, facing +X, at the origin). */
struct FAthleteWalkingTrial
{
	FAthleteMorphology Morphology;
	FAthleteMotorSkill Skill;
	double DurationS = 8.0;
	TArray<FAthleteIntentKey> Script;
	/** Speeds are averaged over this window (s from the start). */
	double MeasureFromS = 0.0;
	double MeasureToS = 0.0;
	/** Record a trace line every TraceEveryFrames frames (0 = no trace). */
	int32 TraceEveryFrames = 0;
};

/** Outcome of one walking trial. */
struct FAthleteWalkingResult
{
	bool bFell = false;
	double FellAtS = -1.0;
	FVector StartComM = FVector::ZeroVector;
	FVector FinalComM = FVector::ZeroVector;
	FVector2D MeanVelocityMps = FVector2D::ZeroVector; // over the measure window
	double MaxSpeedMps = 0.0;
	int32 Steps = 0;
	double FinalHeadingYawDeg = 0.0;
	double FinalSpeedMps = 0.0;
	bool bSteppingAtEnd = false;
	double MeanTickMs = 0.0;
	TArray<FVector> ComPath; // every frame
	TArray<FString> Trace;

	/** One landed step: where the gait aimed the foot and where it came down (ground points under the foot's center). */
	struct FStep
	{
		double TimeS = 0.0;
		FVector2D TargetM = FVector2D::ZeroVector;
		FVector2D LandedM = FVector2D::ZeroVector;
		/** Landed minus target in the heading frame: X ahead (+) / short (-), Y to the right (+). */
		FVector2D ErrorM = FVector2D::ZeroVector;
	};
	TArray<FStep> StepLog; // steps landed before any fall

	/** Mean and worst |landing error| (m), in the heading frame: forward and sideways separately. */
	FVector2D MeanAbsLandingErrorM() const
	{
		FVector2D Sum = FVector2D::ZeroVector;
		for (const FStep& Step : StepLog) { Sum += FVector2D(FMath::Abs(Step.ErrorM.X), FMath::Abs(Step.ErrorM.Y)); }
		return StepLog.IsEmpty() ? FVector2D::ZeroVector : Sum / StepLog.Num();
	}
	FVector2D MaxAbsLandingErrorM() const
	{
		FVector2D Max = FVector2D::ZeroVector;
		for (const FStep& Step : StepLog) { Max = FVector2D(FMath::Max(Max.X, FMath::Abs(Step.ErrorM.X)), FMath::Max(Max.Y, FMath::Abs(Step.ErrorM.Y))); }
		return Max;
	}
};

inline bool RunWalkingTrial(FAutomationTestBase& Test, const FAthleteWalkingTrial& Trial, FAthleteWalkingResult& Out)
{
	FAthletePhysicsTestScene Scene;
	if (!Scene.Initialize(Test, /*bWithFloor=*/true, Trial.Morphology))
	{
		return false;
	}
	const FAthleteStrengthProfile Strength = FAthleteStrengthProfile::MakeGeneralPopulationMaleBaseline(Trial.Morphology.AgeYears, Trial.Morphology.StatureM, Trial.Morphology.BodyMassKg);
	UAthleteMotorComponent* Motor = Scene.AddMotor(Strength, Trial.Skill, /*bBalance=*/true);

	Out.StartComM = Scene.Body->GetCenterOfMassM();
	if (Trial.TraceEveryFrames > 0)
	{
		for (const EAthleteJoint Joint : { EAthleteJoint::HipLeft, EAthleteJoint::KneeLeft, EAthleteJoint::AnkleLeft })
		{
			const FAthleteJointImpedance& Impedance = Motor->GetJointImpedance(Joint);
			Out.Trace.Add(FString::Printf(TEXT("IMPEDANCE %s: stiffness (%.0f %.0f %.0f) N*m/rad, damping (%.1f %.1f %.1f) N*m*s/rad, limit %.0f N*m, inertia %.2f kg*m^2"),
				*UEnum::GetValueAsString(Joint), Impedance.AxisStiffnessNmPerRad.X, Impedance.AxisStiffnessNmPerRad.Y, Impedance.AxisStiffnessNmPerRad.Z,
				Impedance.AxisDampingNmsPerRad.X, Impedance.AxisDampingNmsPerRad.Y, Impedance.AxisDampingNmsPerRad.Z, Impedance.TorqueLimitNm, Impedance.EffectiveInertiaKgM2));
		}
	}
	if (Trial.TraceEveryFrames > 0)
	{
		const FAthleteBodyModel& Model = Scene.Body->GetBodyModel();
		const double AnkleX = Model.GetJointCenterM(EAthleteJoint::AnkleLeft).X;
		const FAthleteSegmentCollisionShape& FootShape = Scene.Body->GetCollisionShape(EAthleteSegment::FootLeft);
		Out.Trace.Add(FString::Printf(TEXT("FOOT relative to ankle (m, forward): collision center %+.3f (half length %.3f), center of mass %+.3f"),
			FootShape.CenterM.X - AnkleX, FootShape.BoxHalfExtentsM.X, Model.GetSegment(EAthleteSegment::FootLeft).CenterOfMassM.X - AnkleX));
	}
	const double FallenHeightM = 0.8 * Out.StartComM.Z;
	const double Dt = FAthletePhysicsTestScene::FrameDeltaSeconds;
	const int32 Frames = FMath::RoundToInt(Trial.DurationS / Dt);
	double TotalMs = 0.0;
	FVector MeasureStart = Out.StartComM;
	bool bMeasuring = false;
	for (int32 Frame = 0; Frame < Frames; ++Frame)
	{
		const double Time = Frame * Dt;
		for (const FAthleteIntentKey& Key : Trial.Script)
		{
			if (FMath::IsNearlyEqual(Key.TimeS, Time, 0.5 * Dt) || (Frame == 0 && Key.TimeS <= 0.0))
			{
				Motor->SetMovementIntent(Key.Intent);
			}
		}
		if (!bMeasuring && Trial.MeasureToS > Trial.MeasureFromS && Time >= Trial.MeasureFromS)
		{
			bMeasuring = true;
			MeasureStart = Scene.Body->GetCenterOfMassM();
		}
		// The target in force going into this frame, and which leg is swinging, to score a landing.
		const FAthleteGaitState GaitBefore = Motor->GetGaitState();
		TotalMs += Scene.Simulate(Dt);

		const FAthleteGaitState& GaitAfter = Motor->GetGaitState();
		if (GaitAfter.StepCount > GaitBefore.StepCount && !Out.bFell)
		{
			const EAthleteSegment LandedFoot = GaitBefore.SwingLeg == 0 ? EAthleteSegment::FootLeft : EAthleteSegment::FootRight;
			// The point the gait aims: the foot's collision-box center (1.5 cm ahead of its center of mass).
			const UPrimitiveComponent* FootBody = Scene.Body->GetSegmentBody(LandedFoot);
			const FVector ShapeFromComM = Scene.Body->GetCollisionShape(LandedFoot).CenterM - Scene.Body->GetBodyModel().GetSegment(LandedFoot).CenterOfMassM;
			const FVector FootCenterM = AthleteUnits::UnrealToMeters(FootBody->GetComponentTransform().TransformPosition(
				FootBody->GetComponentTransform().InverseTransformPosition(FootBody->GetBodyInstance()->GetCOMPosition()) + AthleteUnits::MetersToUnreal(ShapeFromComM)));
			FAthleteWalkingResult::FStep Step;
			Step.TimeS = Time + Dt;
			Step.TargetM = FVector2D(GaitBefore.FootTargetM.X, GaitBefore.FootTargetM.Y);
			Step.LandedM = FVector2D(FootCenterM.X, FootCenterM.Y);
			const FVector Error = FRotator(0.0, GaitBefore.HeadingYawDeg, 0.0).UnrotateVector(FVector(Step.LandedM - Step.TargetM, 0.0));
			Step.ErrorM = FVector2D(Error.X, Error.Y);
			Out.StepLog.Add(Step);
		}

		const FVector Com = Scene.Body->GetCenterOfMassM();
		const FVector Velocity = Scene.Body->GetLinearMomentumKgMps() / Scene.Body->GetTotalMassKg();
		Out.ComPath.Add(Com);
		Out.MaxSpeedMps = FMath::Max(Out.MaxSpeedMps, Velocity.Size2D());
		if (bMeasuring && FMath::IsNearlyEqual(Time + Dt, Trial.MeasureToS, 0.5 * Dt))
		{
			const FVector Travel = Com - MeasureStart;
			Out.MeanVelocityMps = FVector2D(Travel.X, Travel.Y) / (Trial.MeasureToS - Trial.MeasureFromS);
		}
		if (!Out.bFell && (Com.Z < FallenHeightM || Motor->GetMotorState() == EAthleteMotorState::Fallen))
		{
			Out.bFell = true;
			Out.FellAtS = Time + Dt;
		}

		if (Trial.TraceEveryFrames > 0 && Frame % Trial.TraceEveryFrames == 0)
		{
			const FAthleteGaitState& Gait = Motor->GetGaitState();
			const double SoleL = Scene.Body->GetSegmentLowestPointM(EAthleteSegment::FootLeft);
			const double SoleR = Scene.Body->GetSegmentLowestPointM(EAthleteSegment::FootRight);
			const int32 Swing = Gait.SwingLeg;
			const EAthleteJoint SwingAnkle = Swing == 0 ? EAthleteJoint::AnkleLeft : EAthleteJoint::AnkleRight;
			const EAthleteJoint StanceAnkle = Swing == 0 ? EAthleteJoint::AnkleRight : EAthleteJoint::AnkleLeft;
			const EAthleteJoint StanceHip = Swing == 0 ? EAthleteJoint::HipRight : EAthleteJoint::HipLeft;
			const EAthleteJoint SwingKnee = Swing == 0 ? EAthleteJoint::KneeLeft : EAthleteJoint::KneeRight;
			const EAthleteJoint SwingHip = Swing == 0 ? EAthleteJoint::HipLeft : EAthleteJoint::HipRight;
			const EAthleteSegment SwingFoot = Swing == 0 ? EAthleteSegment::FootLeft : EAthleteSegment::FootRight;
			const FVector AnkleNow = Scene.Body->GetJointCenterWorldM(SwingAnkle);
			Out.Trace.Add(FString::Printf(TEXT("t=%.2f CoM (%.2f %.2f %.3f) v (%+.2f %+.2f) | %s%s sw %s ph %.2f n %d | plan (%+.2f %+.2f) hd %.0f | cap (%.2f %.2f) plan (%.2f %.2f) cop (%+.3f %+.3f) tgt (%.2f %.2f) | soles L %.3f R %.3f | swing ankle z tgt %.3f act %.3f x tgt %.2f act %.2f y tgt %.2f act %.2f | knee %.0f foot pitch %+.0f | pelvis roll %+.1f pitch %+.1f | swing hip tq (%+.0f %+.0f %+.0f) err (%+.1f %+.1f %+.1f) knee err %+.1f | stance ankle tq (%+.0f %+.0f %+.0f) | stance hip tq (%+.0f %+.0f %+.0f) err (%+.1f %+.1f %+.1f)"),
				Time + Dt, Com.X, Com.Y, Com.Z, Velocity.X, Velocity.Y,
				Gait.bShifting ? TEXT("SHIFT") : TEXT(""), Gait.bStepping ? TEXT("STEP") : TEXT("stand"), Swing == 0 ? TEXT("L") : TEXT("R"), Gait.PhaseFraction, Gait.StepCount,
				Gait.PlannedVelocityMps.X, Gait.PlannedVelocityMps.Y, Gait.HeadingYawDeg,
				Gait.CapturePointM.X, Gait.CapturePointM.Y, Gait.PlannedCapturePointM.X, Gait.PlannedCapturePointM.Y, Gait.PressureOffsetM.X, Gait.PressureOffsetM.Y, Gait.FootTargetM.X, Gait.FootTargetM.Y, SoleL, SoleR,
				Gait.SwingAnkleTargetM.Z, AnkleNow.Z, Gait.SwingAnkleTargetM.X, AnkleNow.X, Gait.SwingAnkleTargetM.Y, AnkleNow.Y,
				Scene.Body->GetJointAngles(SwingKnee).SwingSagittalDeg, -Scene.Body->GetSegmentBody(SwingFoot)->GetComponentRotation().Pitch,
				Scene.Body->GetSegmentBody(EAthleteSegment::LowerTrunk)->GetComponentRotation().Roll, -Scene.Body->GetSegmentBody(EAthleteSegment::LowerTrunk)->GetComponentRotation().Pitch,
				Motor->GetJointTorqueNm(SwingHip).X, Motor->GetJointTorqueNm(SwingHip).Y, Motor->GetJointTorqueNm(SwingHip).Z,
				Motor->GetJointErrorDeg(SwingHip).X, Motor->GetJointErrorDeg(SwingHip).Y, Motor->GetJointErrorDeg(SwingHip).Z, Motor->GetJointErrorDeg(SwingKnee).Y,
				Motor->GetJointTorqueNm(StanceAnkle).X, Motor->GetJointTorqueNm(StanceAnkle).Y, Motor->GetJointTorqueNm(StanceAnkle).Z,
				Motor->GetJointTorqueNm(StanceHip).X, Motor->GetJointTorqueNm(StanceHip).Y, Motor->GetJointTorqueNm(StanceHip).Z,
				Motor->GetJointErrorDeg(StanceHip).X, Motor->GetJointErrorDeg(StanceHip).Y, Motor->GetJointErrorDeg(StanceHip).Z));
		}
	}
	Out.FinalComM = Scene.Body->GetCenterOfMassM();
	Out.FinalSpeedMps = (Scene.Body->GetLinearMomentumKgMps() / Scene.Body->GetTotalMassKg()).Size2D();
	Out.Steps = Motor->GetGaitState().StepCount;
	Out.FinalHeadingYawDeg = Motor->GetGaitState().HeadingYawDeg;
	Out.bSteppingAtEnd = Motor->GetGaitState().bStepping;
	Out.MeanTickMs = TotalMs / Frames;
	return true;
}

/**
 * The same task run several times, each trial starting at a slightly different moment (so from a
 * slightly different state). Near a stability edge one run is a noisy sample; judge by how many fell,
 * the median, and landing errors pooled over every step.
 */
struct FAthleteWalkingRobustness
{
	int32 Trials = 0;
	int32 Falls = 0;
	TArray<int32> StepCounts;       // per trial
	TArray<double> FinalDriftM;     // horizontal distance of the CoM from its start, per trial
	TArray<double> MeanSpeedMps;    // forward speed over the measure window, trials that stayed up
	FAthleteWalkingResult Pooled;   // every trial's landings

	int32 MinSteps() const { return StepCounts.IsEmpty() ? 0 : FMath::Min(StepCounts); }
	int32 MedianSteps() const
	{
		TArray<int32> Sorted = StepCounts;
		Sorted.Sort();
		return Sorted.IsEmpty() ? 0 : Sorted[Sorted.Num() / 2];
	}
	double MaxDriftM() const { return FinalDriftM.IsEmpty() ? 0.0 : FMath::Max(FinalDriftM); }
	double AverageSpeedMps() const
	{
		double Sum = 0.0;
		for (const double Speed : MeanSpeedMps) { Sum += Speed; }
		return MeanSpeedMps.IsEmpty() ? 0.0 : Sum / MeanSpeedMps.Num();
	}
	FString Summary() const
	{
		return FString::Printf(TEXT("fell in %d of %d; steps median %d (min %d); drift max %.2f m; speed %.2f m/s; landing over %d steps: mean |fwd| %.3f |side| %.3f m, worst |fwd| %.3f |side| %.3f m"),
			Falls, Trials, MedianSteps(), MinSteps(), MaxDriftM(), AverageSpeedMps(), Pooled.StepLog.Num(),
			Pooled.MeanAbsLandingErrorM().X, Pooled.MeanAbsLandingErrorM().Y, Pooled.MaxAbsLandingErrorM().X, Pooled.MaxAbsLandingErrorM().Y);
	}
};

/** Stepping in place (SpeedMps = 0) or walking forward at SpeedMps, Trials times. The measure window is the last part of each trial, after the start-up. */
inline bool RunWalkingRobustness(FAutomationTestBase& Test, const FAthleteMotorSkill& Skill, double SpeedMps, int32 Trials, double DurationS, FAthleteWalkingRobustness& Out)
{
	constexpr double FirstStartS = 1.0;
	constexpr double StartSpacingS = 0.07; // spread over most of a step
	constexpr double SettleS = 4.0;        // speeds are measured from this long after the start
	Out = FAthleteWalkingRobustness();
	Out.Trials = Trials;
	for (int32 Index = 0; Index < Trials; ++Index)
	{
		FAthleteWalkingTrial Trial;
		Trial.Skill = Skill;
		Trial.DurationS = DurationS;
		const double StartS = FirstStartS + StartSpacingS * Index;
		FAthleteIntentKey Key = FAthleteIntentKey::Velocity(StartS, SpeedMps, 0.0);
		Key.Intent.bKeepStepping = true;
		Trial.Script.Add(Key);
		Trial.MeasureFromS = StartS + SettleS;
		Trial.MeasureToS = DurationS;
		FAthleteWalkingResult Result;
		if (!RunWalkingTrial(Test, Trial, Result))
		{
			return false;
		}
		Out.Falls += Result.bFell ? 1 : 0;
		Out.StepCounts.Add(Result.Steps);
		Out.FinalDriftM.Add(FVector::Dist2D(Result.FinalComM, Result.StartComM));
		if (!Result.bFell)
		{
			Out.MeanSpeedMps.Add(Result.MeanVelocityMps.X);
		}
		Out.Pooled.StepLog.Append(Result.StepLog);
	}
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
