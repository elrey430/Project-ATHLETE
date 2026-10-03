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
	// Make the shoulder too weak to hold the arm out (isometric strength = half the arm's horizontal
	// gravity torque) and too stiff for the spring to matter, so the muscle runs at its limit.
	//
	// The arm drops. As it falls the shoulder muscle is STRETCHED, and a lengthening muscle resists
	// with more than its isometric strength (force-velocity), which brakes the fall. So the arm
	// settles where gravity's torque equals the isometric strength. (Before Milestone 4 there was no
	// force-velocity relation: a capped muscle lost its damping and the arm swung between 30 and 86
	// deg forever.)
	constexpr double WarmUpS = 6.0;
	constexpr double WindowS = 2.0;

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
	TestTrue(TEXT("Arm has settled (moves less than 2 deg over the last 2 s)"), MaxDroop - MinDroop < 2.0);
	TestEqual(TEXT("Settled where gravity's torque equals the isometric strength (N*m)"), MeanGravityTorque, LimitNm, 0.05 * LimitNm);
	AddInfo(FString::Printf(TEXT("Isometric strength %.3f N*m; gravity torque over the last 2 s %.3f N*m; arm between %.1f and %.1f deg below horizontal"),
		LimitNm, MeanGravityTorque, MinDroop, MaxDroop));
	// The muscle really ran at its limit: the torque it reported is the strength limit.
	TestEqual(TEXT("Reported shoulder torque equals the strength limit (N*m)"), Motor->GetJointTorqueNm(EAthleteJoint::ShoulderRight).Size(), LimitNm, 0.01 * LimitNm);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteForceVelocityCurveTest, "Athlete.Motor.Muscle.ForceVelocityCurve", AthleteTestFlags)

bool FAthleteForceVelocityCurveTest::RunTest(const FString& Parameters)
{
	using namespace AthleteMuscleModel;
	TestEqual(TEXT("Isometric: full strength"), ForceVelocityFactor(0.0), 1.0, 1e-12);
	TestEqual(TEXT("At maximum shortening speed: no torque"), ForceVelocityFactor(1.0), 0.0, 1e-12);
	TestEqual(TEXT("Half speed: (1 - 0.5) / (1 + 0.5 / k)"), ForceVelocityFactor(0.5), 0.5 / (1.0 + 0.5 / HillCurvature), 1e-12);
	TestEqual(TEXT("Stretched fast: eccentric plateau"), ForceVelocityFactor(-1.0), EccentricPlateau, 1e-12);
	TestEqual(TEXT("Beyond maximum speed: still zero"), ForceVelocityFactor(2.0), 0.0, 1e-12);

	// Monotonic: the faster it shortens, the weaker; the faster it's stretched, the stronger.
	bool bMonotonic = true;
	for (double X = -1.0; X < 1.0; X += 0.01)
	{
		bMonotonic &= ForceVelocityFactor(X + 0.01) < ForceVelocityFactor(X);
	}
	TestTrue(TEXT("Decreasing with shortening speed"), bMonotonic);
	const double Step = 1e-4;
	const double ShorteningSlope = (1.0 - ForceVelocityFactor(Step)) / Step;
	const double LengtheningSlope = (ForceVelocityFactor(-Step) - 1.0) / Step;
	TestTrue(TEXT("Steeper when lengthening than when shortening (Katz 1939)"), LengtheningSlope > 2.0 * ShorteningSlope);
	AddInfo(FString::Printf(TEXT("F/F0 at x = -1, -0.1, 0, 0.1, 0.3, 0.5, 1: %.3f %.3f %.3f %.3f %.3f %.3f %.3f; slopes at 0: shortening %.1f, lengthening %.1f"),
		ForceVelocityFactor(-1.0), ForceVelocityFactor(-0.1), ForceVelocityFactor(0.0), ForceVelocityFactor(0.1), ForceVelocityFactor(0.3), ForceVelocityFactor(0.5), ForceVelocityFactor(1.0),
		ShorteningSlope, LengtheningSlope));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteIsokineticConversionTest, "Athlete.Motor.Muscle.IsokineticStrengthConvertedToIsometric", AthleteTestFlags)

bool FAthleteIsokineticConversionTest::RunTest(const FString& Parameters)
{
	// Hip extension was measured moving at 60 deg/s (Harbo 2012). The muscle model's limit is the
	// isometric strength, so it must be the measured torque divided by the force-velocity factor
	// at the test speed. Knee extension was measured isometrically and passes through unchanged.
	const FAthleteMorphology Morphology;
	FAthleteBodyModel Model;
	UTEST_TRUE(TEXT("Model"), FAthleteBodyModel::Build(Morphology, Model));
	const FAthleteStrengthProfile Strength = FAthleteStrengthProfile::MakeGeneralPopulationMaleBaseline(Morphology.AgeYears, Morphology.StatureM, Morphology.BodyMassKg);
	const FAthleteJointActionStrength& Hip = *Strength.Find(EAthleteJointAction::HipExtension);
	const FAthleteJointActionStrength& Knee = *Strength.Find(EAthleteJointAction::KneeExtension);
	UTEST_TRUE(TEXT("Hip extension was measured isokinetically"), Hip.TestMode == EAthleteStrengthTestMode::Isokinetic && Hip.TestVelocityDegPerSec > 0.0);

	const FAthleteJointImpedance HipImpedance = AthleteMuscleModel::ComputeStandingImpedance(Model, EAthleteJoint::HipLeft, Strength, FAthleteMotorSkill(), 9.81);
	const FAthleteJointImpedance KneeImpedance = AthleteMuscleModel::ComputeStandingImpedance(Model, EAthleteJoint::KneeLeft, Strength, FAthleteMotorSkill(), 9.81);
	const double Expected = Hip.PeakTorqueNm / AthleteMuscleModel::ForceVelocityFactor(Hip.TestVelocityDegPerSec / Hip.MaxVelocityDegPerSec);
	TestEqual(TEXT("Hip isometric strength = measured / f(test speed) (N*m)"), HipImpedance.TorqueLimitNm, Expected, 1e-9);
	TestEqual(TEXT("Knee (measured isometric) unchanged (N*m)"), KneeImpedance.TorqueLimitNm, Knee.PeakTorqueNm, 1e-9);
	AddInfo(FString::Printf(TEXT("Hip extension: %.1f N*m at %.0f deg/s -> %.1f N*m isometric (max speed %.0f deg/s)"),
		Hip.PeakTorqueNm, Hip.TestVelocityDegPerSec, HipImpedance.TorqueLimitNm, Hip.MaxVelocityDegPerSec));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteMaxJointSpeedTest, "Athlete.Motor.Muscle.JointSpeedBoundedByForceVelocity", AthleteTestFlags)

bool FAthleteMaxJointSpeedTest::RunTest(const FString& Parameters)
{
	// Weightless arm, thorax held still, shoulder commanded to swing 150 deg with a very stiff
	// muscle (tone 10): it pushes at its strength limit for most of the swing. Without force-velocity,
	// a constant torque would keep accelerating the arm; with it, torque vanishes as the joint
	// approaches its maximum speed. Measured from the start of the swing until the arm arrives.
	// (At absurd tone, e.g. 50, the arm chain becomes numerically unstable after arriving: a limit
	// of explicit muscles at 240 Hz, far outside what an athlete uses.)
	const FAthleteMorphology Morphology;
	FAthletePhysicsTestScene Scene;
	UTEST_TRUE(TEXT("Scene"), Scene.Initialize(*this, /*bWithFloor=*/false, Morphology, FVector(0, 0, 20)));
	Scene.Body->SetGravityEnabled(false);
	Scene.Body->GetSegmentBody(EAthleteSegment::UpperTrunk)->SetSimulatePhysics(false);
	FAthleteMotorSkill Stiff;
	Stiff.MuscleToneGain = 10.0;
	UAthleteMotorComponent* Motor = Scene.AddMotor(FAthleteStrengthProfile::MakeGeneralPopulationMaleBaseline(Morphology.AgeYears, Morphology.StatureM, Morphology.BodyMassKg), Stiff, /*bBalance=*/false);
	FAthletePosture Posture;
	Posture.ChildRelativeToParent[AthleteJoints::ToIndex(EAthleteJoint::ShoulderRight)] = AthleteJointSetup::RotateToward(FVector(0, 0, -1), FVector(1, 0, 0), 150.0);
	Motor->ApplyPosture(Posture);

	const double MaxSpeed = Motor->GetJointImpedance(EAthleteJoint::ShoulderRight).MaxVelocityRadPerS;
	double PeakSpeed = 0.0;
	const UShapeComponent* Arm = Scene.Body->GetSegmentBody(EAthleteSegment::UpperArmRight);
	for (int32 Frame = 0; Frame < 120; ++Frame)
	{
		Scene.Simulate(FAthletePhysicsTestScene::FrameDeltaSeconds);
		PeakSpeed = FMath::Max(PeakSpeed, Arm->BodyInstance.GetUnrealWorldAngularVelocityInRadians().Size());
		if (Frame > 3 && Motor->GetJointErrorDeg(EAthleteJoint::ShoulderRight).Size() < 10.0)
		{
			break; // arrived
		}
	}
	TestTrue(TEXT("Never faster than the joint's maximum speed"), PeakSpeed <= MaxSpeed * 1.02);
	TestTrue(TEXT("Still fast: over half the maximum speed"), PeakSpeed > 0.5 * MaxSpeed);
	AddInfo(FString::Printf(TEXT("Shoulder maximum speed %.0f deg/s; arm peak %.0f deg/s"), FMath::RadiansToDegrees(MaxSpeed), FMath::RadiansToDegrees(PeakSpeed)));
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
