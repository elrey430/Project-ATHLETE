// Project ATHLETE

#pragma once

#include "../Physics/AthletePhysicsTestHelpers.h"
#include "Components/ShapeComponent.h"

#if WITH_DEV_AUTOMATION_TESTS

/** How to run one standing trial. */
struct FAthleteStandingTrial
{
	FAthleteMorphology Morphology;
	FAthleteMotorSkill Skill;
	bool bBalance = true;
	double DurationS = 6.0;

	/**
	 * Push: total impulse (world N*s; zero = no push), spread evenly over PushDurationS starting at
	 * PushTimeS, applied at PushSegment's center of mass raised by PushHeightM. The default is a
	 * 100 ms push at the pelvis, like the waist-pull perturbations of balance research; a push high
	 * on the chest also spins the trunk (a different, harsher test).
	 */
	FVector PushImpulseNs = FVector::ZeroVector;
	double PushTimeS = 1.0;
	double PushDurationS = 0.1;
	EAthleteSegment PushSegment = EAthleteSegment::LowerTrunk;
	double PushHeightM = 0.0;

	/** Which way the athlete faces (yaw, degrees). Push directions are in HIS frame (+X = his forward). */
	double BodyYawDeg = 0.0;

	/** Solver iterations for the athlete's bodies (0 = engine default). */
	int32 PositionIterations = 0;
	int32 VelocityIterations = 0;

	/** Record a trace line every TraceEveryFrames frames (0 = no trace). */
	int32 TraceEveryFrames = 0;
};

/** Outcome of one standing trial. */
struct FAthleteStandingResult
{
	double InitialComHeightM = 0.0;
	double FinalComHeightM = 0.0;
	double MinComHeightM = 0.0;
	double MaxHorizontalComDriftM = 0.0; // from the initial CoM, over the whole trial
	double FinalHorizontalComDriftM = 0.0;
	double MaxFootSlideM = 0.0;          // how far either foot moved horizontally
	double FinalKineticEnergyJ = 0.0;
	double LateSwayRmsM = 0.0;          // RMS horizontal CoM motion over the second half of the trial
	double LateMaxKineticEnergyJ = 0.0; // peak kinetic energy over the last 2 s: does the body come to rest?
	double MeanTickMs = 0.0;
	int32 FramesWithSleepingSegment = 0; // Chaos put part of the body to sleep (frozen, not balancing)
	bool bMotorFallen = false;           // the motor system concluded he is down and stopped balancing
	TArray<FString> Trace;

	/** Upright at the end: CoM at least 90% of its starting height. */
	bool IsUpright() const { return FinalComHeightM >= 0.9 * InitialComHeightM; }
};

/** Stands an athlete on the floor with muscles (and optionally balance) on, optionally pushes him, and simulates. */
inline bool RunStandingTrial(FAutomationTestBase& Test, const FAthleteStandingTrial& Trial, FAthleteStandingResult& Out)
{
	FAthletePhysicsTestScene Scene;
	if (!Scene.Initialize(Test, /*bWithFloor=*/true, Trial.Morphology, FVector::ZeroVector, Trial.PositionIterations, Trial.VelocityIterations, true, FRotator(0.0, Trial.BodyYawDeg, 0.0)))
	{
		return false;
	}
	const FAthleteStrengthProfile Strength = FAthleteStrengthProfile::MakeGeneralPopulationMaleBaseline(Trial.Morphology.AgeYears, Trial.Morphology.StatureM, Trial.Morphology.BodyMassKg);
	UAthleteMotorComponent* Motor = Scene.AddMotor(Strength, Trial.Skill, Trial.bBalance);

	const FVector StartCom = Scene.Body->GetCenterOfMassM();
	const FVector StartLeftFoot = Scene.Body->GetSegmentBody(EAthleteSegment::FootLeft)->GetComponentLocation();
	const FVector StartRightFoot = Scene.Body->GetSegmentBody(EAthleteSegment::FootRight)->GetComponentLocation();
	Out.InitialComHeightM = StartCom.Z;
	Out.MinComHeightM = StartCom.Z;

	const int32 Frames = FMath::RoundToInt(Trial.DurationS / FAthletePhysicsTestScene::FrameDeltaSeconds);
	const int32 PushFrame = FMath::RoundToInt(Trial.PushTimeS / FAthletePhysicsTestScene::FrameDeltaSeconds);
	const int32 PushFrames = FMath::Max(1, FMath::RoundToInt(Trial.PushDurationS / FAthletePhysicsTestScene::FrameDeltaSeconds));
	double TotalMs = 0.0;
	TArray<FVector> LateCom;
	for (int32 Frame = 0; Frame < Frames; ++Frame)
	{
		if (Frame >= PushFrame && Frame < PushFrame + PushFrames && !Trial.PushImpulseNs.IsNearlyZero())
		{
			const FVector PointM = AthleteUnits::UnrealToMeters(Scene.Body->GetSegmentBody(Trial.PushSegment)->BodyInstance.GetCOMPosition()) + FVector(0, 0, Trial.PushHeightM);
			Scene.Body->AddImpulseAtPoint(Trial.PushSegment, FRotator(0.0, Trial.BodyYawDeg, 0.0).RotateVector(Trial.PushImpulseNs) / PushFrames, PointM);
		}
		TotalMs += Scene.Simulate(FAthletePhysicsTestScene::FrameDeltaSeconds);

		const FVector Com = Scene.Body->GetCenterOfMassM();
		Out.MinComHeightM = FMath::Min(Out.MinComHeightM, Com.Z);
		if (Frame >= Frames / 2)
		{
			LateCom.Add(Com);
		}
		if (Frame >= Frames - FMath::RoundToInt(2.0 / FAthletePhysicsTestScene::FrameDeltaSeconds))
		{
			Out.LateMaxKineticEnergyJ = FMath::Max(Out.LateMaxKineticEnergyJ, Scene.Body->GetKineticEnergyJ());
		}
		Out.MaxHorizontalComDriftM = FMath::Max(Out.MaxHorizontalComDriftM, FVector::Dist2D(Com, StartCom));
		const double LeftSlide = AthleteUnits::UnrealToMeters(FVector::Dist2D(Scene.Body->GetSegmentBody(EAthleteSegment::FootLeft)->GetComponentLocation(), StartLeftFoot));
		const double RightSlide = AthleteUnits::UnrealToMeters(FVector::Dist2D(Scene.Body->GetSegmentBody(EAthleteSegment::FootRight)->GetComponentLocation(), StartRightFoot));
		Out.MaxFootSlideM = FMath::Max3(Out.MaxFootSlideM, LeftSlide, RightSlide);
		for (int32 Segment = 0; Segment < static_cast<int32>(EAthleteSegment::Count); ++Segment)
		{
			if (!Scene.Body->GetSegmentBody(static_cast<EAthleteSegment>(Segment))->BodyInstance.IsInstanceAwake())
			{
				++Out.FramesWithSleepingSegment;
				break;
			}
		}

		if (Trial.TraceEveryFrames > 0 && Frame % Trial.TraceEveryFrames == 0)
		{
			const FAthleteBalanceCommand& Cmd = Motor->GetLastCommand();
			auto Pitch = [&Scene](EAthleteSegment Segment) { return -Scene.Body->GetSegmentBody(Segment)->GetComponentRotation().Pitch; }; // + = leaning forward
			auto TorqueY = [Motor](EAthleteJoint Joint) { return Motor->GetJointTorqueNm(Joint).Y; };
			auto Roll = [&Scene](EAthleteSegment Segment) { return Scene.Body->GetSegmentBody(Segment)->GetComponentRotation().Roll; };
			auto TorqueX = [Motor](EAthleteJoint Joint) { return Motor->GetJointTorqueNm(Joint).X; };
			Out.Trace.Add(FString::Printf(TEXT("      frontal: roll footL %+.1f footR %+.1f shankL %+.1f shankR %+.1f pelvis %+.1f thorax %+.1f | torqueX ankleL %+.0f ankleR %+.0f hipL %+.0f hipR %+.0f lumbar %+.0f"),
				Roll(EAthleteSegment::FootLeft), Roll(EAthleteSegment::FootRight), Roll(EAthleteSegment::ShankLeft), Roll(EAthleteSegment::ShankRight),
				Roll(EAthleteSegment::LowerTrunk), Roll(EAthleteSegment::UpperTrunk),
				TorqueX(EAthleteJoint::AnkleLeft), TorqueX(EAthleteJoint::AnkleRight), TorqueX(EAthleteJoint::HipLeft), TorqueX(EAthleteJoint::HipRight), TorqueX(EAthleteJoint::Lumbar)));
			Out.Trace.Add(FString::Printf(TEXT("t=%.2f CoM (%.3f, %.3f, %.3f) XCoM err %+.3f/%+.3f | cmd lean %+.1f/%+.1f hip %+.1f | pitch foot %+.1f shank %+.1f thigh %+.1f pelvis %+.1f thorax %+.1f head %+.1f | torqueY ankle %+.0f knee %+.0f hip %+.0f lumbar %+.0f"),
				(Frame + 1) * FAthletePhysicsTestScene::FrameDeltaSeconds, Com.X, Com.Y, Com.Z,
				Cmd.XcomErrorForwardM, Cmd.XcomErrorRightM, Cmd.LeanForwardDeg, Cmd.LeanRightDeg, Cmd.HipFlexionDeg,
				Pitch(EAthleteSegment::FootLeft), Pitch(EAthleteSegment::ShankLeft), Pitch(EAthleteSegment::ThighLeft), Pitch(EAthleteSegment::LowerTrunk),
				Pitch(EAthleteSegment::UpperTrunk), Pitch(EAthleteSegment::Head),
				TorqueY(EAthleteJoint::AnkleLeft), TorqueY(EAthleteJoint::KneeLeft), TorqueY(EAthleteJoint::HipLeft), TorqueY(EAthleteJoint::Lumbar)));
		}
	}
	const FVector EndCom = Scene.Body->GetCenterOfMassM();
	Out.FinalComHeightM = EndCom.Z;
	Out.FinalHorizontalComDriftM = FVector::Dist2D(EndCom, StartCom);
	Out.FinalKineticEnergyJ = Scene.Body->GetKineticEnergyJ();
	Out.bMotorFallen = Motor->GetMotorState() == EAthleteMotorState::Fallen;
	Out.MeanTickMs = TotalMs / Frames;

	// Sway: RMS distance of the horizontal CoM from its mean over the second half of the trial.
	if (LateCom.Num() > 0)
	{
		FVector Mean = FVector::ZeroVector;
		for (const FVector& Sample : LateCom) { Mean += Sample; }
		Mean /= LateCom.Num();
		double SumSquares = 0.0;
		for (const FVector& Sample : LateCom) { SumSquares += FVector::DistSquared2D(Sample, Mean); }
		Out.LateSwayRmsM = FMath::Sqrt(SumSquares / LateCom.Num());
	}
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
