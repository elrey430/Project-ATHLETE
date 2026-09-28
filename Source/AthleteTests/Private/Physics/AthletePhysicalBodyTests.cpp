// Project ATHLETE

#include "AthleteTestFlags.h"
#include "AthletePhysicsTestHelpers.h"
#include "Components/ShapeComponent.h"
#include "Misc/ScopeExit.h"
#include "PhysicsEngine/BodyInstance.h"
#include "PhysicsEngine/PhysicsSettings.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteBodyMassPropertiesTest, "Athlete.Physics.Body.MassPropertiesMatchModel", AthleteTestFlags)

bool FAthleteBodyMassPropertiesTest::RunTest(const FString& Parameters)
{
	// The physics engine must carry exactly the mass distribution AthleteBody computed:
	// per-segment mass, center of mass, and inertia, with no hidden inertia conditioning.
	FAthletePhysicsTestScene Scene;
	UTEST_TRUE(TEXT("Scene"), Scene.Initialize(*this, /*bWithFloor=*/false, FAthleteMorphology::FromImperial(6, 1, 225), FVector(0, 0, 10)));
	const FAthleteBodyModel& Model = Scene.Body->GetBodyModel();
	const FTransform BodyFrame = Scene.Body->GetComponentTransform();

	int32 MassErrors = 0, ComErrors = 0, InertiaErrors = 0, Conditioned = 0;
	double WorstInertiaError = 0.0;
	for (int32 Index = 0; Index < AthleteSegments::NumSegments; ++Index)
	{
		const EAthleteSegment Segment = AthleteSegments::FromIndex(Index);
		const FAthleteBodySegment& Expected = Model.GetSegment(Segment);
		const FBodyInstance& Instance = Scene.Body->GetSegmentBody(Segment)->BodyInstance;

		MassErrors += FMath::IsNearlyEqual(Instance.GetBodyMass(), Expected.MassKg, 1e-4 * Expected.MassKg) ? 0 : 1;

		const FVector ExpectedComCm = BodyFrame.TransformPosition(AthleteUnits::MetersToUnreal(Expected.CenterOfMassM));
		ComErrors += Instance.GetCOMPosition().Equals(ExpectedComCm, 0.01) ? 0 : 1; // 0.1 mm

		// World inertia about the segment's CoM vs de Leva's principal moments (body is unrotated,
		// so the expected world tensor is diagonal).
		const FMatrix I = Scene.Body->GetSegmentWorldInertiaKgM2(Segment);
		const FVector Target = Expected.PrincipalInertiaKgM2;
		const double Scale = Target.GetMax();
		double Error = 0.0;
		for (int32 Row = 0; Row < 3; ++Row)
		{
			for (int32 Col = 0; Col < 3; ++Col)
			{
				const double ExpectedValue = (Row == Col) ? Target[Row] : 0.0;
				Error = FMath::Max(Error, FMath::Abs(I.M[Row][Col] - ExpectedValue) / Scale);
			}
		}
		WorstInertiaError = FMath::Max(WorstInertiaError, Error);
		InertiaErrors += Error < 1e-3 ? 0 : 1; // 0.1% of the largest moment
		if (Error >= 1e-3)
		{
			AddInfo(FString::Printf(TEXT("  %s: target %s kg*m^2, engine principal %s kg*cm^2, mass-space rot %s, scale %s, world diag (%.5f %.5f %.5f)"),
				AthleteSegments::GetName(Segment), *Target.ToString(), *Instance.GetBodyInertiaTensor().ToString(),
				*Instance.GetMassSpaceLocal().GetRotation().Rotator().ToString(), *Instance.InertiaTensorScale.ToString(),
				I.M[0][0], I.M[1][1], I.M[2][2]));
		}

		Conditioned += Instance.IsInertiaConditioningEnabled() ? 1 : 0;
	}

	TestEqual(TEXT("Segment masses match the model"), MassErrors, 0);
	TestEqual(TEXT("Segment centers of mass match the model"), ComErrors, 0);
	TestEqual(TEXT("Segment inertia tensors match the model"), InertiaErrors, 0);
	TestEqual(TEXT("No body uses inertia conditioning"), Conditioned, 0);
	TestEqual(TEXT("Total mass (kg)"), Scene.Body->GetTotalMassKg(), Model.GetTotalMassKg(), 1e-3);
	TestTrue(TEXT("Whole-body CoM matches the model (1 mm)"),
		Scene.Body->GetCenterOfMassM().Equals(AthleteUnits::UnrealToMeters(BodyFrame.TransformPosition(AthleteUnits::MetersToUnreal(Model.GetCenterOfMassM()))), 1e-3));

	AddInfo(FString::Printf(TEXT("Worst per-segment inertia error: %.2e of largest moment. Ignored self-collision pairs: %d"),
		WorstInertiaError, Scene.Body->GetIgnoredCollisionPairs().Num()));
	for (const TPair<EAthleteSegment, EAthleteSegment>& Pair : Scene.Body->GetIgnoredCollisionPairs())
	{
		AddInfo(FString::Printf(TEXT("  ignores %s <-> %s"), AthleteSegments::GetName(Pair.Key), AthleteSegments::GetName(Pair.Value)));
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteBodyFreeFallTest, "Athlete.Physics.Body.FreeFallKeepsShapeAndFallsAtG", AthleteTestFlags)

bool FAthleteBodyFreeFallTest::RunTest(const FString& Parameters)
{
	// With nothing to push against, an articulated body falls as one piece: its CoM accelerates
	// at g, no joint opens, and no segment rotates relative to the others.
	constexpr double DurationS = 1.0;
	FAthletePhysicsTestScene Scene;
	UTEST_TRUE(TEXT("Scene"), Scene.Initialize(*this, /*bWithFloor=*/false, FAthleteMorphology(), FVector(0, 0, 100)));

	const FVector StartCom = Scene.Body->GetCenterOfMassM();
	TArray<FQuat> StartRotations;
	for (int32 Index = 0; Index < AthleteSegments::NumSegments; ++Index)
	{
		StartRotations.Add(Scene.Body->GetSegmentBody(AthleteSegments::FromIndex(Index))->GetComponentQuat());
	}

	Scene.Simulate(DurationS);

	const double GravityMps2 = AthleteUnits::UnrealToMeters(Scene.GetWorld()->GetGravityZ());
	const double ComVelocityZ = Scene.Body->GetLinearMomentumKgMps().Z / Scene.Body->GetTotalMassKg();
	TestEqual(TEXT("CoM velocity after 1 s equals g*t (m/s)"), ComVelocityZ, GravityMps2 * DurationS, FMath::Abs(GravityMps2) * 0.01);

	double MaxRotationDeg = 0.0;
	EAthleteSegment MostRotated = EAthleteSegment::LowerTrunk;
	for (int32 Index = 0; Index < AthleteSegments::NumSegments; ++Index)
	{
		const FQuat Now = Scene.Body->GetSegmentBody(AthleteSegments::FromIndex(Index))->GetComponentQuat();
		const double RotationDeg = FMath::RadiansToDegrees(Now.AngularDistance(StartRotations[Index]));
		if (RotationDeg > MaxRotationDeg)
		{
			MaxRotationDeg = RotationDeg;
			MostRotated = AthleteSegments::FromIndex(Index);
		}
	}
	const double SeparationMm = Scene.Body->GetMaxJointSeparationM() * 1000.0;
	TestTrue(*FString::Printf(TEXT("No segment rotates (max %.4f deg, %s)"), MaxRotationDeg, AthleteSegments::GetName(MostRotated)), MaxRotationDeg < 0.1);
	TestTrue(*FString::Printf(TEXT("Joints stay closed (max separation %.4f mm)"), SeparationMm), SeparationMm < 1.0);
	TestTrue(TEXT("Body actually fell"), Scene.Body->GetCenterOfMassM().Z < StartCom.Z - 4.0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteBodyImpulseTest, "Athlete.Physics.Body.ImpulseConservesMomentum", AthleteTestFlags)

bool FAthleteBodyImpulseTest::RunTest(const FString& Parameters)
{
	// In zero gravity, strike the chest with a known off-center impulse J at point P. Joints are
	// internal forces, so however the body flails:
	//   total linear momentum  = J
	//   CoM velocity           = J / M
	//   angular momentum (CoM) = (P - CoM) x J
	constexpr double DurationS = 0.5;
	const FVector ImpulseNs(60.0, 0.0, 0.0);

	FAthletePhysicsTestScene Scene;
	UTEST_TRUE(TEXT("Scene"), Scene.Initialize(*this, /*bWithFloor=*/false, FAthleteMorphology(), FVector(0, 0, 10)));
	Scene.Body->SetGravityEnabled(false);
	Scene.Simulate(FAthletePhysicsTestScene::FrameDeltaSeconds); // let physics state settle

	const FVector Com = Scene.Body->GetCenterOfMassM();
	const FVector HitPoint = AthleteUnits::UnrealToMeters(Scene.Body->GetSegmentBody(EAthleteSegment::UpperTrunk)->BodyInstance.GetCOMPosition()) + FVector(0, 0.05, 0.1);
	const FVector ExpectedAngular = (HitPoint - Com) ^ ImpulseNs;
	Scene.Body->AddImpulseAtPoint(EAthleteSegment::UpperTrunk, ImpulseNs, HitPoint);

	// Track angular momentum over time: an error present immediately points at how the impulse is
	// applied or measured; an error that grows points at the joint solver.
	TArray<FString> History;
	double Elapsed = 0.0;
	for (const double Checkpoint : { FAthletePhysicsTestScene::FrameDeltaSeconds * 1.0, 0.05, 0.1, 0.2, 0.3, DurationS })
	{
		Scene.Simulate(Checkpoint - Elapsed);
		Elapsed = Checkpoint;
		const FVector L = Scene.Body->GetAngularMomentumAboutComKgM2ps();
		History.Add(FString::Printf(TEXT("t=%.3f s: L=(%.2f, %.2f, %.2f), error %.2f%%"), Elapsed, L.X, L.Y, L.Z, 100.0 * (L - ExpectedAngular).Size() / ExpectedAngular.Size()));
	}
	for (const FString& Line : History)
	{
		AddInfo(Line);
	}

	const FVector Linear = Scene.Body->GetLinearMomentumKgMps();
	const FVector Angular = Scene.Body->GetAngularMomentumAboutComKgM2ps();
	const double Mass = Scene.Body->GetTotalMassKg();

	TestTrue(*FString::Printf(TEXT("Linear momentum equals the impulse (got %s)"), *Linear.ToString()), Linear.Equals(ImpulseNs, 0.01 * ImpulseNs.Size()));
	TestEqual(TEXT("CoM speed = J/M (m/s)"), Linear.X / Mass, ImpulseNs.X / Mass, 0.01 * ImpulseNs.X / Mass);
	const double AngularError = (Angular - ExpectedAngular).Size() / ExpectedAngular.Size();
	TestTrue(*FString::Printf(TEXT("Angular momentum conserved (error %.2f%%)"), AngularError * 100.0), AngularError < 0.05);

	AddInfo(FString::Printf(TEXT("J = %s N*s. Linear momentum %s. Angular expected %s, measured %s (error %.2f%%). Max joint separation %.3f mm."),
		*ImpulseNs.ToString(), *Linear.ToString(), *ExpectedAngular.ToString(), *Angular.ToString(), AngularError * 100.0, Scene.Body->GetMaxJointSeparationM() * 1000.0));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteSolverIterationStudy, "Athlete.Physics.Study.SolverIterationsVsAccuracy", AthleteTestFlags)

bool FAthleteSolverIterationStudy::RunTest(const FString& Parameters)
{
	// Measurement, not a pass/fail gate: how closely does the articulated body conserve angular
	// momentum after a chest impulse, as a function of Chaos solver iterations, and at what cost?
	struct FConfig { int32 Position; int32 Velocity; float SubstepS; int32 MaxSubsteps; bool bGyro; };
	const FConfig Configs[] =
	{
		{ 0, 0, 1.0f / 240.0f, 8, false }, // project settings, Chaos default (no gyroscopic torque)
		{ 0, 0, 1.0f / 240.0f, 8, true },  // project settings, ATHLETE default
		{ 32, 4, 1.0f / 240.0f, 8, true },
		{ 0, 0, 1.0f / 480.0f, 16, true }, // finer time steps
		{ 0, 0, 1.0f / 960.0f, 16, true },
	};
	const FVector ImpulseNs(60.0, 0.0, 0.0);

	// Physics substep settings are project-wide; change them for the study and restore after.
	UPhysicsSettings* PhysicsSettings = UPhysicsSettings::Get();
	const float SavedSubstep = PhysicsSettings->MaxSubstepDeltaTime;
	const int32 SavedMaxSubsteps = PhysicsSettings->MaxSubsteps;
	ON_SCOPE_EXIT
	{
		PhysicsSettings->MaxSubstepDeltaTime = SavedSubstep;
		PhysicsSettings->MaxSubsteps = SavedMaxSubsteps;
	};

	for (const FConfig& Config : Configs)
	{
		PhysicsSettings->MaxSubstepDeltaTime = Config.SubstepS;
		PhysicsSettings->MaxSubsteps = Config.MaxSubsteps;
		FAthletePhysicsTestScene Scene;
		UTEST_TRUE(TEXT("Scene"), Scene.Initialize(*this, false, FAthleteMorphology(), FVector(0, 0, 10), Config.Position, Config.Velocity, Config.bGyro));
		Scene.Body->SetGravityEnabled(false);
		Scene.Simulate(FAthletePhysicsTestScene::FrameDeltaSeconds);

		const FVector Com = Scene.Body->GetCenterOfMassM();
		const FVector HitPoint = AthleteUnits::UnrealToMeters(Scene.Body->GetSegmentBody(EAthleteSegment::UpperTrunk)->BodyInstance.GetCOMPosition()) + FVector(0, 0.05, 0.1);
		const FVector Expected = (HitPoint - Com) ^ ImpulseNs;
		Scene.Body->AddImpulseAtPoint(EAthleteSegment::UpperTrunk, ImpulseNs, HitPoint);

		const double FirstFrameMs = Scene.Simulate(FAthletePhysicsTestScene::FrameDeltaSeconds);
		const double FirstError = (Scene.Body->GetAngularMomentumAboutComKgM2ps() - Expected).Size() / Expected.Size();
		const double MeanMs = Scene.Simulate(0.5);
		const double FinalError = (Scene.Body->GetAngularMomentumAboutComKgM2ps() - Expected).Size() / Expected.Size();

		AddInfo(FString::Printf(TEXT("Substep 1/%.0f s, iterations pos %2d / vel %d (0 = default), gyroscopic %s: angular momentum error %.2f%% after 1 frame, %.2f%% after 0.5 s; joint separation %.3f mm; %.3f ms per frame"),
			1.0f / Config.SubstepS, Config.Position, Config.Velocity, Config.bGyro ? TEXT("on ") : TEXT("off"), FirstError * 100.0, FinalError * 100.0,
			Scene.Body->GetMaxJointSeparationM() * 1000.0, (FirstFrameMs + MeanMs * 30.0) / 31.0));
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteDropEnergyStudy, "Athlete.Physics.Study.DropImpactEnergy", AthleteTestFlags)

bool FAthleteDropEnergyStudy::RunTest(const FString& Parameters)
{
	// Measurement: drop a passive 5'9" 190 lb body from 1.5 m tilted 30 deg forward onto the floor.
	// Only gravity does work, and contacts/joints can only remove energy, so mechanical energy
	// (kinetic + potential) must never rise. Any rise above its running minimum is energy the
	// solver created. Report it against time step and iterations.
	struct FConfig { int32 Position; int32 Velocity; float SubstepS; int32 MaxSubsteps; };
	const FConfig Configs[] =
	{
		{ 0, 0, 1.0f / 240.0f, 8 },   // Chaos default iterations
		{ 16, 2, 1.0f / 240.0f, 8 },
		{ 32, 4, 1.0f / 240.0f, 8 },
		{ 64, 8, 1.0f / 240.0f, 8 },
		{ 0, 0, 1.0f / 960.0f, 16 },
	};
	// One drop is a single chaotic sample; several tilts show whether a setting helps consistently.
	const float TiltsDeg[] = { 15.0f, 30.0f, 45.0f };

	UPhysicsSettings* PhysicsSettings = UPhysicsSettings::Get();
	const float SavedSubstep = PhysicsSettings->MaxSubstepDeltaTime;
	const int32 SavedMaxSubsteps = PhysicsSettings->MaxSubsteps;
	ON_SCOPE_EXIT
	{
		PhysicsSettings->MaxSubstepDeltaTime = SavedSubstep;
		PhysicsSettings->MaxSubsteps = SavedMaxSubsteps;
	};

	for (const FConfig& Config : Configs)
	for (const float TiltDeg : TiltsDeg)
	{
		PhysicsSettings->MaxSubstepDeltaTime = Config.SubstepS;
		PhysicsSettings->MaxSubsteps = Config.MaxSubsteps;

		FAthletePhysicsTestScene Scene;
		UTEST_TRUE(TEXT("Scene"), Scene.Initialize(*this, true, FAthleteMorphology::FromImperial(5, 9, 190), FVector(0, 0, 1.5),
			Config.Position, Config.Velocity, true, FRotator(-TiltDeg, 0.0, 0.0)));

		const double Mass = Scene.Body->GetTotalMassKg();
		const double Gravity = FMath::Abs(AthleteUnits::UnrealToMeters(Scene.GetWorld()->GetGravityZ()));
		double MinEnergy = TNumericLimits<double>::Max();
		double MaxGain = 0.0, MaxSeparationMm = 0.0, LowestM = 0.0, MaxExcessDeg = 0.0;
		double TotalMs = 0.0;
		constexpr int32 Frames = 180; // 3 s
		for (int32 Frame = 0; Frame < Frames; ++Frame)
		{
			TotalMs += Scene.Simulate(FAthletePhysicsTestScene::FrameDeltaSeconds);
			const double Energy = Scene.Body->GetKineticEnergyJ() + Mass * Gravity * Scene.Body->GetCenterOfMassM().Z;
			MinEnergy = FMath::Min(MinEnergy, Energy);
			MaxGain = FMath::Max(MaxGain, Energy - MinEnergy);
			MaxSeparationMm = FMath::Max(MaxSeparationMm, Scene.Body->GetMaxJointSeparationM() * 1000.0);
			LowestM = FMath::Min(LowestM, Scene.Body->GetLowestPointM());
			for (int32 Index = 0; Index < AthleteJoints::NumJoints; ++Index)
			{
				MaxExcessDeg = FMath::Max(MaxExcessDeg, Scene.Body->GetJointAngles(AthleteJoints::FromIndex(Index)).LimitExcessDeg);
			}
		}
		AddInfo(FString::Printf(TEXT("Substep 1/%.0f s, iterations pos %2d / vel %d, tilt %2.0f deg: max energy created %6.1f J; worst joint separation %5.2f mm; deepest penetration %4.1f cm; worst limit excess %4.1f deg; %.3f ms per frame"),
			1.0f / Config.SubstepS, Config.Position, Config.Velocity, TiltDeg, MaxGain, MaxSeparationMm, -LowestM * 100.0, MaxExcessDeg, TotalMs / Frames));
	}
	return true;
}

namespace
{
	struct FCollapseResult
	{
		FVector ComM;
		FVector PelvisLocationCm;
		double KineticEnergyJ = 0.0;
		double LowestPointM = 0.0;
		double MaxSeparationMm = 0.0;
		double MaxLimitExcessDeg = 0.0;
		double MeanTickMs = 0.0;
		EAthleteSegment LowestSegment = EAthleteSegment::Count;
		EAthleteJoint WorstJoint = EAthleteJoint::Count;
		FAthleteJointAngles WorstJointAngles;
	};

	/** Releases the reference body standing on the floor and lets it collapse passively. */
	bool RunCollapse(FAutomationTestBase& Test, double DurationS, FCollapseResult& Out)
	{
		FAthletePhysicsTestScene Scene;
		if (!Scene.Initialize(Test, /*bWithFloor=*/true))
		{
			return false;
		}
		Out.MeanTickMs = Scene.Simulate(DurationS);
		Out.ComM = Scene.Body->GetCenterOfMassM();
		Out.PelvisLocationCm = Scene.Body->GetSegmentBody(EAthleteSegment::LowerTrunk)->GetComponentLocation();
		Out.KineticEnergyJ = Scene.Body->GetKineticEnergyJ();
		Out.LowestPointM = Scene.Body->GetLowestPointM(&Out.LowestSegment);
		Out.MaxSeparationMm = Scene.Body->GetMaxJointSeparationM() * 1000.0;

		// How far any joint ended up beyond its limit, decomposed as Chaos limits it (0 = within limits).
		for (int32 Index = 0; Index < AthleteJoints::NumJoints; ++Index)
		{
			const EAthleteJoint Joint = AthleteJoints::FromIndex(Index);
			const FAthleteJointAngles Angles = Scene.Body->GetJointAngles(Joint);
			if (Angles.LimitExcessDeg >= Out.MaxLimitExcessDeg)
			{
				Out.MaxLimitExcessDeg = Angles.LimitExcessDeg;
				Out.WorstJoint = Joint;
				Out.WorstJointAngles = Angles;
			}
		}
		return true;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteBodyCollapseTest, "Athlete.Physics.Body.PassiveCollapseSettlesOnFloor", AthleteTestFlags)

bool FAthleteBodyCollapseTest::RunTest(const FString& Parameters)
{
	// With no muscles yet, a standing body must crumple to the floor and come to rest there:
	// nothing sinks through the floor, joints hold together, limits are respected, energy dissipates.
	FCollapseResult Result;
	UTEST_TRUE(TEXT("Collapse ran"), RunCollapse(*this, 4.0, Result));

	TestTrue(*FString::Printf(TEXT("Nothing sinks into the floor (lowest %.2f cm)"), Result.LowestPointM * 100.0), Result.LowestPointM > -0.01);
	TestTrue(*FString::Printf(TEXT("Body came down (CoM %.3f m)"), Result.ComM.Z), Result.ComM.Z < 0.4);
	TestTrue(*FString::Printf(TEXT("Body at rest (kinetic energy %.3f J)"), Result.KineticEnergyJ), Result.KineticEnergyJ < 1.0);
	TestTrue(*FString::Printf(TEXT("Joints hold (max separation %.3f mm)"), Result.MaxSeparationMm), Result.MaxSeparationMm < 5.0);
	TestTrue(*FString::Printf(TEXT("Joint limits respected (max excess %.2f deg)"), Result.MaxLimitExcessDeg), Result.MaxLimitExcessDeg < 5.0);

	AddInfo(FString::Printf(TEXT("After 4 s: CoM height %.3f m, KE %.4f J, lowest point %.2f cm (%s), joint separation %.3f mm. Mean world tick %.3f ms (1 athlete, 60 Hz frames, 240 Hz substeps)."),
		Result.ComM.Z, Result.KineticEnergyJ, Result.LowestPointM * 100.0, AthleteSegments::GetName(Result.LowestSegment), Result.MaxSeparationMm, Result.MeanTickMs));
	if (Result.WorstJoint != EAthleteJoint::Count)
	{
		AddInfo(FString::Printf(TEXT("Most-loaded joint %s: swing sagittal %.1f, frontal %.1f, twist %.1f deg from range center; limit excess %.2f deg"),
			AthleteJoints::GetName(Result.WorstJoint), Result.WorstJointAngles.SwingSagittalDeg, Result.WorstJointAngles.SwingFrontalDeg,
			Result.WorstJointAngles.TwistDeg, Result.MaxLimitExcessDeg));
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteBodyDeterminismTest, "Athlete.Physics.Body.CollapseIsDeterministic", AthleteTestFlags)

bool FAthleteBodyDeterminismTest::RunTest(const FString& Parameters)
{
	// Same body, same start, same fixed time step: the collapse must end identically.
	// Reproducibility is a project requirement (principle 28); this detects if Chaos breaks it.
	FCollapseResult First, Second;
	UTEST_TRUE(TEXT("First run"), RunCollapse(*this, 2.0, First));
	UTEST_TRUE(TEXT("Second run"), RunCollapse(*this, 2.0, Second));

	const double DifferenceCm = FVector::Dist(First.PelvisLocationCm, Second.PelvisLocationCm);
	TestTrue(*FString::Printf(TEXT("Pelvis ends at the same place (difference %.6f cm)"), DifferenceCm), DifferenceCm < 1e-4);
	AddInfo(FString::Printf(TEXT("Pelvis difference between identical runs: %.8f cm"), DifferenceCm));
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
