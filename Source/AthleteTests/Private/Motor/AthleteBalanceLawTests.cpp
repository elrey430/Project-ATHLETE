// Project ATHLETE

#include "AthleteTestFlags.h"
#include "Anatomy/AthleteBodyModel.h"
#include "Balance/AthleteBalanceController.h"
#include "Capability/AthleteMotorSkill.h"
#include "Muscle/AthleteMuscleModel.h"
#include "../Physics/AthletePhysicsTestHelpers.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	/** Standing sensing: CoM 1 m above the support center, plus an offset and a velocity. */
	FAthleteBalanceSensing MakeSensing(const FVector& ComOffsetM, const FVector& VelocityMps)
	{
		FAthleteBalanceSensing Sensing;
		Sensing.SupportCenterM = FVector::ZeroVector;
		Sensing.CenterOfMassM = FVector(0, 0, 1.0) + ComOffsetM;
		Sensing.CenterOfMassVelocityMps = VelocityMps;
		return Sensing;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteBalanceLawSignsTest, "Athlete.Motor.BalanceLaw.LeansAgainstTheFall", AthleteTestFlags)

bool FAthleteBalanceLawSignsTest::RunTest(const FString& Parameters)
{
	const FAthleteMotorSkill Skill;
	constexpr double G = 9.81;
	const FAthleteBalanceCommand Ahead = AthleteBalanceController::Compute(MakeSensing(FVector(0.03, 0, 0), FVector::ZeroVector), Skill, G);
	const FAthleteBalanceCommand Behind = AthleteBalanceController::Compute(MakeSensing(FVector(-0.03, 0, 0), FVector::ZeroVector), Skill, G);
	const FAthleteBalanceCommand ToRight = AthleteBalanceController::Compute(MakeSensing(FVector(0, 0.03, 0), FVector::ZeroVector), Skill, G);
	const FAthleteBalanceCommand MovingForward = AthleteBalanceController::Compute(MakeSensing(FVector::ZeroVector, FVector(0.1, 0, 0)), Skill, G);
	const FAthleteBalanceCommand Centered = AthleteBalanceController::Compute(MakeSensing(FVector::ZeroVector, FVector::ZeroVector), Skill, G);

	TestTrue(TEXT("CoM ahead of the feet: lean back"), Ahead.LeanForwardDeg < 0.0);
	TestTrue(TEXT("CoM behind the feet: lean forward"), Behind.LeanForwardDeg > 0.0);
	TestTrue(TEXT("CoM to the right: tilt left"), ToRight.LeanRightDeg < 0.0);
	TestTrue(TEXT("Moving forward over centered feet: lean back (velocity counts)"), MovingForward.LeanForwardDeg < 0.0);
	TestEqual(TEXT("Centered and still: no correction"), Centered.LeanForwardDeg, 0.0, 1e-9);
	TestEqual(TEXT("Small error: no hip strategy"), Ahead.HipFlexionDeg, 0.0, 1e-9);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteHipStrategyTest, "Athlete.Motor.BalanceLaw.HipStrategyBeyondTheFeet", AthleteTestFlags)

bool FAthleteHipStrategyTest::RunTest(const FString& Parameters)
{
	// Once the error exceeds what the feet can resist, the hips bend toward the fall (trunk forward
	// when falling forward), which drives the hips back.
	const FAthleteMotorSkill Skill;
	const FAthleteBalanceCommand FallingForward = AthleteBalanceController::Compute(MakeSensing(FVector(0.2, 0, 0), FVector::ZeroVector), Skill, 9.81);
	const FAthleteBalanceCommand FallingBack = AthleteBalanceController::Compute(MakeSensing(FVector(-0.2, 0, 0), FVector::ZeroVector), Skill, 9.81);
	TestTrue(TEXT("Falling forward: hips flex"), FallingForward.HipFlexionDeg > 0.0);
	TestTrue(TEXT("Falling back: hips extend"), FallingBack.HipFlexionDeg < 0.0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteReferenceTargetsTest, "Athlete.Motor.BalanceLaw.ReferenceBodyGetsReferenceTargets", AthleteTestFlags)

bool FAthleteReferenceTargetsTest::RunTest(const FString& Parameters)
{
	// Body in its reference pose, no correction wanted: every joint target is the reference angle.
	const FAthleteBalanceSensing Perceived; // all segments at their reference orientation
	const FAthletePosture Posture = AthleteBalanceController::SolveJointTargets(AthleteBalanceController::MakeSegmentTargets(FAthleteBalanceCommand()), Perceived, FAthleteFootSupport());
	for (int32 Index = 0; Index < AthleteJoints::NumJoints; ++Index)
	{
		TestTrue(FString::Printf(TEXT("%s target is the reference"), AthleteJoints::GetName(AthleteJoints::FromIndex(Index))),
			Posture.ChildRelativeToParent[Index].Equals(FQuat::Identity, 1e-9));
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteFootSupportLimitTest, "Athlete.Motor.BalanceLaw.AnklesAskOnlyWhatTheFeetCanTake", AthleteTestFlags)

bool FAthleteFootSupportLimitTest::RunTest(const FString& Parameters)
{
	// A command to lean the body 15 deg back from upright: the ankle target may turn the shank
	// back only by the angle whose torque the foot can take before it tips onto its toes.
	FAthleteFootSupport Feet;
	Feet.TippingTorqueToesNm = 70.0;
	Feet.TippingTorqueHeelNm = 20.0;
	Feet.TippingTorqueSideNm = 15.0;
	Feet.AnkleStiffnessNmPerRad = 600.0;
	FAthleteBalanceCommand LeanBack;
	LeanBack.LeanForwardDeg = -15.0;
	const FAthletePosture Posture = AthleteBalanceController::SolveJointTargets(AthleteBalanceController::MakeSegmentTargets(LeanBack), FAthleteBalanceSensing(), Feet);

	// Ankle target = (desired shank)^-1 * foot; with the foot flat that is the inverse of the shank's allowed turn.
	const double TurnDeg = FMath::RadiansToDegrees(Posture.ChildRelativeToParent[AthleteJoints::ToIndex(EAthleteJoint::AnkleLeft)].GetAngle());
	const double AllowedDeg = FMath::RadiansToDegrees(AthleteBalanceController::FootSupportMargin * Feet.TippingTorqueToesNm / Feet.AnkleStiffnessNmPerRad);
	TestEqual(TEXT("Ankle turns the shank back by exactly the foot's budget (deg)"), TurnDeg, AllowedDeg, 1e-6);
	AddInfo(FString::Printf(TEXT("Commanded 15.0 deg back; ankle asks for %.2f deg (budget %.0f%% of %.0f N*m at %.0f N*m/rad)"),
		TurnDeg, 100.0 * AthleteBalanceController::FootSupportMargin, Feet.TippingTorqueToesNm, Feet.AnkleStiffnessNmPerRad));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteStandingImpedanceTest, "Athlete.Motor.Muscle.StandingImpedance", AthleteTestFlags)

bool FAthleteStandingImpedanceTest::RunTest(const FString& Parameters)
{
	const FAthleteMorphology Morphology;
	FAthleteBodyModel Model;
	UTEST_TRUE(TEXT("Model"), FAthleteBodyModel::Build(Morphology, Model));
	const FAthleteStrengthProfile Strength = FAthleteStrengthProfile::MakeGeneralPopulationMaleBaseline(Morphology.AgeYears, Morphology.StatureM, Morphology.BodyMassKg);
	FAthleteMotorSkill Tone1, Tone2;
	Tone1.MuscleToneGain = 1.0;
	Tone2.MuscleToneGain = 2.0;
	constexpr double G = 9.81;

	const FAthleteJointImpedance Ankle1 = AthleteMuscleModel::ComputeStandingImpedance(Model, EAthleteJoint::AnkleLeft, Strength, Tone1, G);
	const FAthleteJointImpedance Ankle2 = AthleteMuscleModel::ComputeStandingImpedance(Model, EAthleteJoint::AnkleLeft, Strength, Tone2, G);
	const FAthleteJointImpedance Hip2 = AthleteMuscleModel::ComputeStandingImpedance(Model, EAthleteJoint::HipLeft, Strength, Tone2, G);

	// Each ankle carries half of everything above the feet: m * g * (height of that CoM above the ankle) / 2.
	double MassAboveFeet = 0.0;
	FVector Weighted = FVector::ZeroVector;
	for (int32 Index = 0; Index < AthleteSegments::NumSegments; ++Index)
	{
		const EAthleteSegment Segment = AthleteSegments::FromIndex(Index);
		if (Segment != EAthleteSegment::FootLeft && Segment != EAthleteSegment::FootRight)
		{
			MassAboveFeet += Model.GetSegment(Segment).MassKg;
			Weighted += Model.GetSegment(Segment).CenterOfMassM * Model.GetSegment(Segment).MassKg;
		}
	}
	const FVector AnkleCenter = Model.GetJointCenterM(EAthleteJoint::AnkleLeft);
	const double Lever = FVector::Dist(Weighted / MassAboveFeet, FVector(AnkleCenter.X, 0.0, AnkleCenter.Z));
	TestEqual(TEXT("Ankle gravity load = m g h / 2 (N*m/rad)"), Ankle1.GravityStiffnessNmPerRad, MassAboveFeet * G * Lever / 2.0, 1e-6);
	TestEqual(TEXT("Tone 1: stiffness equals the gravity load"), Ankle1.StiffnessNmPerRad, Ankle1.GravityStiffnessNmPerRad, 1e-9);
	TestEqual(TEXT("Tone 2: twice as stiff"), Ankle2.StiffnessNmPerRad, 2.0 * Ankle1.StiffnessNmPerRad, 1e-9);
	TestEqual(TEXT("Hip frontal axis carries the whole body's sideways sway (= ankle load)"), Hip2.AxisStiffnessNmPerRad.Z, Ankle2.StiffnessNmPerRad, 1e-9);
	TestEqual(TEXT("Hip sagittal axis carries the trunk above it"), Hip2.AxisStiffnessNmPerRad.Y, Hip2.StiffnessNmPerRad, 1e-9);
	TestTrue(TEXT("Torque limit is the measured plantarflexion strength"), Ankle2.TorqueLimitNm > 0.0 && Ankle2.LimitingAction == EAthleteJointAction::AnklePlantarFlexion);
	AddInfo(FString::Printf(TEXT("Ankle (each): load %.0f N*m/rad, stiffness %.0f, damping %.0f N*m*s/rad, limit %.0f N*m | hip: sagittal %.0f, frontal %.0f N*m/rad"),
		Ankle2.GravityStiffnessNmPerRad, Ankle2.StiffnessNmPerRad, Ankle2.DampingNmsPerRad, Ankle2.TorqueLimitNm, Hip2.AxisStiffnessNmPerRad.Y, Hip2.AxisStiffnessNmPerRad.Z));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteReferencePoseRelaxedTest, "Athlete.Motor.Muscle.ReferencePoseIsRelaxed", AthleteTestFlags)

bool FAthleteReferencePoseRelaxedTest::RunTest(const FString& Parameters)
{
	// Weightless body in its reference pose, muscles holding the reference pose: nothing to do.
	// Every muscle must read zero error and push nothing, and the body must stay still. A frame or
	// sign mistake in any joint, or a numerically unstable muscle, shows up here as motion.
	FAthletePhysicsTestScene Scene;
	UTEST_TRUE(TEXT("Scene"), Scene.Initialize(*this, false, FAthleteMorphology(), FVector(0, 0, 20)));
	Scene.Body->SetGravityEnabled(false);
	const FAthleteMorphology Morphology;
	UAthleteMotorComponent* Motor = Scene.AddMotor(FAthleteStrengthProfile::MakeGeneralPopulationMaleBaseline(Morphology.AgeYears, Morphology.StatureM, Morphology.BodyMassKg), FAthleteMotorSkill(), /*bBalance=*/false);
	Scene.Simulate(1.0);

	double WorstErrorDeg = 0.0, WorstTorqueNm = 0.0;
	for (int32 Index = 0; Index < AthleteJoints::NumJoints; ++Index)
	{
		const EAthleteJoint Joint = AthleteJoints::FromIndex(Index);
		WorstErrorDeg = FMath::Max(WorstErrorDeg, Motor->GetJointErrorDeg(Joint).Size());
		WorstTorqueNm = FMath::Max(WorstTorqueNm, Motor->GetJointTorqueNm(Joint).Size());
	}
	TestTrue(TEXT("Every joint within 0.5 deg of its target"), WorstErrorDeg < 0.5);
	TestTrue(TEXT("No muscle pushes more than 0.5 N*m"), WorstTorqueNm < 0.5);
	TestTrue(TEXT("Body still (kinetic energy under 1 mJ)"), Scene.Body->GetKineticEnergyJ() < 1e-3);
	AddInfo(FString::Printf(TEXT("After 1 s: worst joint error %.3f deg, worst muscle torque %.3f N*m, kinetic energy %.2e J"), WorstErrorDeg, WorstTorqueNm, Scene.Body->GetKineticEnergyJ()));
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
