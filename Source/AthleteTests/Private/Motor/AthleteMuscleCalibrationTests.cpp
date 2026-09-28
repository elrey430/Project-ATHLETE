// Project ATHLETE

#include "AthleteTestFlags.h"
#include "../Physics/AthletePhysicsTestHelpers.h"
#include "Articulation/AthleteJointSetup.h"
#include "Components/ShapeComponent.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	/** Gravity torque of the right arm about the shoulder when held straight out horizontally (N*m). */
	double HorizontalArmGravityTorqueNm(const FAthleteBodyModel& Model, double Gravity)
	{
		const FVector Shoulder = Model.GetJointCenterM(EAthleteJoint::ShoulderRight);
		double Torque = 0.0;
		for (const EAthleteSegment Segment : { EAthleteSegment::UpperArmRight, EAthleteSegment::ForearmRight, EAthleteSegment::HandRight })
		{
			const FAthleteBodySegment& Seg = Model.GetSegment(Segment);
			Torque += Seg.MassKg * Gravity * FVector::Dist(Seg.CenterOfMassM, Shoulder);
		}
		return Torque;
	}

	/**
	 * Holds the thorax fixed, commands 90 deg of right-shoulder flexion (arm straight forward), lets
	 * gravity act for 3 s, and returns how far the upper arm droops below horizontal (degrees).
	 */
	bool MeasureShoulderDroop(FAutomationTestBase& Test, const FAthleteStrengthProfile& Strength, const FAthleteMotorSkill& Skill,
		double& OutDroopDeg, double& OutStiffness, double& OutGravityTorque, double DurationS = 3.0, TArray<FString>* OutHistory = nullptr)
	{
		FAthletePhysicsTestScene Scene;
		if (!Scene.Initialize(Test, /*bWithFloor=*/false, FAthleteMorphology(), FVector(0, 0, 20)))
		{
			return false;
		}
		// A kinematic body is moved only by code, never by forces: the thorax becomes a fixed mount.
		Scene.Body->GetSegmentBody(EAthleteSegment::UpperTrunk)->SetSimulatePhysics(false);

		UAthleteMotorComponent* Motor = Scene.AddMotor(Strength, Skill, /*bBalance=*/false);
		FAthletePosture Posture;
		Posture.ChildRelativeToParent[AthleteJoints::ToIndex(EAthleteJoint::ShoulderRight)] =
			AthleteJointSetup::RotateToward(FVector(0, 0, -1), FVector(1, 0, 0), 90.0);
		Motor->ApplyPosture(Posture);

		auto Droop = [&Scene]()
		{
			const FVector ArmDirection = Scene.Body->GetSegmentBody(EAthleteSegment::UpperArmRight)->GetComponentQuat().RotateVector(FVector(0, 0, -1));
			return FMath::RadiansToDegrees(FMath::Asin(-ArmDirection.Z));
		};
		constexpr double SampleIntervalS = 0.5;
		for (double Elapsed = 0.0; Elapsed < DurationS - 1e-6; Elapsed += SampleIntervalS)
		{
			Scene.Simulate(SampleIntervalS);
			if (OutHistory)
			{
				OutHistory->Add(FString::Printf(TEXT("t=%.1f s: droop %.2f deg"), Elapsed + SampleIntervalS, Droop()));
			}
		}
		OutDroopDeg = Droop();
		OutStiffness = Motor->GetJointImpedance(EAthleteJoint::ShoulderRight).StiffnessNmPerRad;
		OutGravityTorque = HorizontalArmGravityTorqueNm(Scene.Body->GetBodyModel(), FMath::Abs(AthleteUnits::UnrealToMeters(Scene.GetWorld()->GetGravityZ())));
		return true;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteMuscleStiffnessCalibrationTest, "Athlete.Motor.Muscle.StiffnessIsRealTorque", AthleteTestFlags)

bool FAthleteMuscleStiffnessCalibrationTest::RunTest(const FString& Parameters)
{
	// Static equilibrium: k * droop = tau_gravity * cos(droop). If the muscle torque had the wrong
	// units, sign, or frame, or were applied to the wrong bodies, the droop would be far off.
	const FAthleteMorphology Morphology;
	const FAthleteStrengthProfile Strength = FAthleteStrengthProfile::MakeGeneralPopulationMaleBaseline(24.0, Morphology.StatureM, Morphology.BodyMassKg);
	double DroopDeg, Stiffness, GravityTorque;
	UTEST_TRUE(TEXT("Scenario ran"), MeasureShoulderDroop(*this, Strength, FAthleteMotorSkill(), DroopDeg, Stiffness, GravityTorque));

	// Solve k * d = T * cos(d) for d by fixed-point iteration.
	double Predicted = 0.0;
	for (int32 Iteration = 0; Iteration < 50; ++Iteration)
	{
		Predicted = GravityTorque * FMath::Cos(Predicted) / Stiffness;
	}
	const double PredictedDeg = FMath::RadiansToDegrees(Predicted);
	TestEqual(TEXT("Measured droop matches k*d = T*cos(d) (deg)"), DroopDeg, PredictedDeg, 0.15 * PredictedDeg);
	AddInfo(FString::Printf(TEXT("Shoulder stiffness %.2f N*m/rad, arm gravity torque %.2f N*m: predicted droop %.2f deg, measured %.2f deg"),
		Stiffness, GravityTorque, PredictedDeg, DroopDeg));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteMuscleTorqueLimitTest, "Athlete.Motor.Muscle.TorqueLimitIsStrength", AthleteTestFlags)

bool FAthleteMuscleTorqueLimitTest::RunTest(const FString& Parameters)
{
	// Make the shoulder too weak to hold the arm out (limit = half the arm's horizontal gravity
	// torque) and too stiff for the spring to matter, so the muscle runs at its limit the whole time.
	//
	// A muscle at its limit gives a constant torque and NO damping (the limit caps damping too), so
	// the arm swings about the equilibrium instead of settling (see "saturated muscles" in the
	// Milestone 3 doc). The test therefore uses angular-impulse balance, which holds regardless:
	// averaged over a long window the arm's angular momentum doesn't grow, so the time-average of
	// gravity's torque about the shoulder must equal the muscle's (constant) maximum torque.
	constexpr double WarmUpS = 2.0;
	constexpr double WindowS = 8.0;

	const FAthleteMorphology Morphology;
	FAthleteBodyModel Model;
	FAthleteBodyModel::Build(Morphology, Model);
	FAthleteStrengthProfile Weak = FAthleteStrengthProfile::MakeGeneralPopulationMaleBaseline(24.0, Morphology.StatureM, Morphology.BodyMassKg);
	const double LimitNm = 0.5 * HorizontalArmGravityTorqueNm(Model, 9.8);
	Weak.Actions[EAthleteJointAction::ShoulderAbduction].PeakTorqueNm = LimitNm;
	FAthleteMotorSkill Stiff;
	Stiff.MuscleToneGain = 5.0;

	FAthletePhysicsTestScene Scene;
	UTEST_TRUE(TEXT("Scene"), Scene.Initialize(*this, /*bWithFloor=*/false, Morphology, FVector(0, 0, 20)));
	Scene.Body->GetSegmentBody(EAthleteSegment::UpperTrunk)->SetSimulatePhysics(false);
	UAthleteMotorComponent* Motor = Scene.AddMotor(Weak, Stiff, /*bBalance=*/false);
	FAthletePosture Posture;
	Posture.ChildRelativeToParent[AthleteJoints::ToIndex(EAthleteJoint::ShoulderRight)] = AthleteJointSetup::RotateToward(FVector(0, 0, -1), FVector(1, 0, 0), 90.0);
	Motor->ApplyPosture(Posture);
	Scene.Simulate(WarmUpS);

	// Gravity torque about the shoulder's side-to-side axis, from the actual segment positions
	// (so elbow and wrist bending are accounted for).
	const double Gravity = FMath::Abs(AthleteUnits::UnrealToMeters(Scene.GetWorld()->GetGravityZ()));
	auto GravityTorqueNow = [&Scene, Gravity]()
	{
		const UShapeComponent* Trunk = Scene.Body->GetSegmentBody(EAthleteSegment::UpperTrunk);
		const FConstraintInstance* Shoulder = Scene.Body->GetJointConstraint(EAthleteJoint::ShoulderRight);
		const FVector ShoulderM = AthleteUnits::UnrealToMeters(Trunk->GetComponentTransform().TransformPosition(Shoulder->GetRefFrame(EConstraintFrame::Frame2).GetLocation()));
		double Torque = 0.0;
		for (const EAthleteSegment Segment : { EAthleteSegment::UpperArmRight, EAthleteSegment::ForearmRight, EAthleteSegment::HandRight })
		{
			const UShapeComponent* Body = Scene.Body->GetSegmentBody(Segment);
			const double Horizontal = AthleteUnits::UnrealToMeters(Body->BodyInstance.GetCOMPosition()).X - ShoulderM.X;
			Torque += Body->BodyInstance.GetBodyMass() * Gravity * Horizontal;
		}
		return Torque;
	};

	const int32 Frames = FMath::RoundToInt(WindowS / FAthletePhysicsTestScene::FrameDeltaSeconds);
	double TorqueSum = 0.0, MinDroop = 90.0, MaxDroop = -90.0;
	for (int32 Frame = 0; Frame < Frames; ++Frame)
	{
		Scene.Simulate(FAthletePhysicsTestScene::FrameDeltaSeconds);
		TorqueSum += GravityTorqueNow();
		const FVector Arm = Scene.Body->GetSegmentBody(EAthleteSegment::UpperArmRight)->GetComponentQuat().RotateVector(FVector(0, 0, -1));
		const double Droop = FMath::RadiansToDegrees(FMath::Asin(-Arm.Z));
		MinDroop = FMath::Min(MinDroop, Droop);
		MaxDroop = FMath::Max(MaxDroop, Droop);
	}
	const double MeanGravityTorque = TorqueSum / Frames;
	TestEqual(TEXT("Time-averaged gravity torque equals the strength limit (N*m)"), MeanGravityTorque, LimitNm, 0.05 * LimitNm);
	AddInfo(FString::Printf(TEXT("Strength limit %.3f N*m; time-averaged gravity torque %.3f N*m; arm swung between %.1f and %.1f deg below horizontal"),
		LimitNm, MeanGravityTorque, MinDroop, MaxDroop));
	// The muscle really ran at its limit: the torque it reported is the strength limit.
	TestEqual(TEXT("Reported shoulder torque equals the strength limit (N*m)"), Motor->GetJointTorqueNm(EAthleteJoint::ShoulderRight).Size(), LimitNm, 0.01 * LimitNm);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
