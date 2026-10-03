// Project ATHLETE

#include "Locomotion/AthleteGaitController.h"
#include "Anatomy/AthleteBodyModel.h"
#include "Articulation/AthleteJointSetup.h"
#include "Capability/AthleteMotorSkill.h"

namespace
{
	/** Per leg (0 = left, 1 = right). */
	constexpr EAthleteJoint HipJoint[2] = { EAthleteJoint::HipLeft, EAthleteJoint::HipRight };
	constexpr EAthleteJoint KneeJoint[2] = { EAthleteJoint::KneeLeft, EAthleteJoint::KneeRight };
	constexpr EAthleteJoint AnkleJoint[2] = { EAthleteJoint::AnkleLeft, EAthleteJoint::AnkleRight };
	constexpr EAthleteSegment ThighSegment[2] = { EAthleteSegment::ThighLeft, EAthleteSegment::ThighRight };
	constexpr EAthleteSegment ShankSegment[2] = { EAthleteSegment::ShankLeft, EAthleteSegment::ShankRight };
	constexpr EAthleteSegment FootSegment[2] = { EAthleteSegment::FootLeft, EAthleteSegment::FootRight };

	/** The swing foot counts as down once its sole is this close to the ground (touch). */
	constexpr double TouchdownSoleHeightM = 0.015;
	/** A step can't end before this fraction of its time: a toe touching the ground before the foot has arrived over its target is a scuff, not a landing. */
	constexpr double EarliestTouchdownFraction = 0.85; // = SwingArrivalFraction: a touch before the foot arrives is a scuff
	/** If the swing foot still hasn't landed by this fraction of the step, it's put down anyway. */
	constexpr double LatestTouchdownFraction = 1.5;
	/** The swing foot arrives over its target at this fraction of the step, then presses down. */
	constexpr double SwingArrivalFraction = 0.85;
	/** Feet may not come closer side to side than this fraction of the standing stance width. */
	constexpr double MinStanceWidthFraction = 0.8;
	/** Longest step, as a fraction of leg length. */
	constexpr double MaxStepFractionOfLeg = 0.9;
	/** Below this CoM speed (and with no wish to move) the athlete stops stepping. */
	constexpr double StopSpeedMps = 0.15;
	/** Weight shift before the first step: done once the extrapolated CoM is this close (sideways) to the stance foot, or after MaxShiftTimeS. */
	constexpr double ShiftDoneLateralM = 0.01;
	constexpr double MaxShiftTimeS = 0.8;
	/** A landing counts once the swing foot has stayed down this long (a landing foot can touch and bounce). */
	constexpr double PlantedConfirmS = 0.03;
	/** Balance gain during the weight shift (quiet standing uses the skill's gentler AnkleStrategyGain). */
	constexpr double WeightShiftGain = 0.6;
	/**
	 * Stance ankle: pressure shift per unit of capture-point error. Above 1 the error shrinks between
	 * steps (xi' = omega0 (xi - p)); 1 would only stop it growing.
	 */
	constexpr double CenterOfPressureGain = 1.5;
	/** Once over its target, the swing ankle is pressed this far below standing height (reached by the step's end) until the sole touches. */
	constexpr double PressDownM = 0.02;
	/**
	 * Push-off (walking): the swing foot starts rolled PushOffFootDeg onto its toes, easing back by
	 * PushOffEndCarry of the swing, and while its sole is within PushOffContactHeightM of the ground
	 * the ankle pushes with up to the body's weight on the toes, tapering off. Full push at
	 * PushOffFullSpeedMps planned speed, proportionally less below it, none standing still.
	 */
	constexpr double PushOffFootDeg = 25.0;
	constexpr double PushOffEndCarry = 0.3;
	constexpr double PushOffContactHeightM = 0.02;
	constexpr double PushOffFullSpeedMps = 1.5;

	int32 Other(int32 Leg) { return 1 - Leg; }

	/**
	 * Sideways landing offset outside the capture point for a steady rhythm with the feet Width apart
	 * (LIPM: the capture point drifts away from the stance foot by exp(omega0 T) over a step, so
	 * landing b outside it and ending b inside the next foot repeats when b (1 + exp(omega0 T)) = Width).
	 */
	double LateralOffset(double Width, double Omega0, double StepTimeS) { return Width / (1.0 + FMath::Exp(Omega0 * StepTimeS)); }

	FVector Flat(const FVector& V) { return FVector(V.X, V.Y, 0.0); }

	double SmoothStep(double X)
	{
		X = FMath::Clamp(X, 0.0, 1.0);
		return X * X * (3.0 - 2.0 * X);
	}

	double YawDegOf(const FQuat& WorldRotation)
	{
		const FVector Forward = WorldRotation.GetForwardVector();
		return FMath::RadiansToDegrees(FMath::Atan2(Forward.Y, Forward.X));
	}
}

FAthleteLegGeometry FAthleteLegGeometry::FromModel(const FAthleteBodyModel& Model, double FootCenterXM, double FootHalfHeightM)
{
	const FVector Hip = Model.GetJointCenterM(EAthleteJoint::HipLeft);
	const FVector Knee = Model.GetJointCenterM(EAthleteJoint::KneeLeft);
	const FVector Ankle = Model.GetJointCenterM(EAthleteJoint::AnkleLeft);
	FAthleteLegGeometry Legs;
	Legs.ThighLengthM = FVector::Dist(Hip, Knee);
	Legs.ShankLengthM = FVector::Dist(Knee, Ankle);
	Legs.AnkleHeightM = Ankle.Z; // the model stands with its soles at Z = 0
	Legs.AnkleToFootCenterM = FootCenterXM - Ankle.X;
	Legs.StanceHalfWidthM = FMath::Abs(Ankle.Y);
	Legs.ThighMassKg = Model.GetSegment(EAthleteSegment::ThighLeft).MassKg;
	Legs.ShankMassKg = Model.GetSegment(EAthleteSegment::ShankLeft).MassKg;
	Legs.FootMassKg = Model.GetSegment(EAthleteSegment::FootLeft).MassKg;
	for (int32 Leg = 0; Leg < 2; ++Leg)
	{
		Legs.ThighComFromHipM[Leg] = Model.GetSegment(ThighSegment[Leg]).CenterOfMassM - Model.GetJointCenterM(HipJoint[Leg]);
		Legs.ShankComFromKneeM[Leg] = Model.GetSegment(ShankSegment[Leg]).CenterOfMassM - Model.GetJointCenterM(KneeJoint[Leg]);
		Legs.FootComFromAnkleM[Leg] = Model.GetSegment(FootSegment[Leg]).CenterOfMassM - Model.GetJointCenterM(AnkleJoint[Leg]);
	}
	Legs.ThighInertiaKgM2 = Model.GetSegment(EAthleteSegment::ThighLeft).PrincipalInertiaKgM2;
	Legs.ShankInertiaKgM2 = Model.GetSegment(EAthleteSegment::ShankLeft).PrincipalInertiaKgM2;
	Legs.FootInertiaKgM2 = Model.GetSegment(EAthleteSegment::FootLeft).PrincipalInertiaKgM2;
	return Legs;
}

void FAthleteGaitController::Initialize(const FAthleteLegGeometry& InLegs, double InGroundHeightM, double InBodyFrameYawDeg, double InitialHeadingYawDeg)
{
	Legs = InLegs;
	GroundHeightM = InGroundHeightM;
	BodyFrameYawDeg = InBodyFrameYawDeg;
	State = FAthleteGaitState();
	State.HeadingYawDeg = InitialHeadingYawDeg;
}

FQuat FAthleteGaitController::WorldYaw(double YawDeg) const
{
	return FQuat(FVector::UpVector, FMath::DegreesToRadians(YawDeg));
}

FQuat FAthleteGaitController::GetHeadingDeviation() const
{
	// Body frames are pure yaw, so the deviation is the yaw difference about the body's up axis.
	return WorldYaw(State.HeadingYawDeg - BodyFrameYawDeg);
}

void FAthleteGaitController::StartStep(int32 SwingLeg, const FAthleteBalanceSensing& Now)
{
	State.bStepping = true;
	State.SwingLeg = SwingLeg;
	State.PhaseFraction = 0.0;
	StepStartTimeS = Now.TimeS;
	LiftoffAnkleM = Now.JointCentersM[AthleteJoints::ToIndex(AnkleJoint[SwingLeg])];
	StanceAnkleM = Now.JointCentersM[AthleteJoints::ToIndex(AnkleJoint[Other(SwingLeg)])];
	State.PressureOffsetM = FVector2D(Legs.AnkleToFootCenterM, 0.0); // a new stance foot starts pressing mid-foot
}

bool FAthleteGaitController::Update(const FAthleteMovementIntent& Intent, const FAthleteBalanceSensing& ForPlanning, const FAthleteBalanceSensing& ForPosture,
	const FAthleteBalanceSensing& Now, const FAthleteMotorSkill& Skill, const FAthleteFootSupport& Feet, double GravityMps2, double DeltaTimeS, FAthletePosture& OutPosture)
{
	const double StepTimeS = Skill.StepDurationS;

	// Motor planning: the velocity aimed for moves toward the wish at a limited rate.
	const FVector2D Wish = Intent.WantsToMove() ? Intent.DesiredVelocityMps : FVector2D::ZeroVector;
	FVector2D Change = Wish - State.PlannedVelocityMps;
	const double MaxChange = Skill.PlanAccelerationMps2 * DeltaTimeS;
	if (Change.Size() > MaxChange)
	{
		Change *= MaxChange / Change.Size();
	}
	State.PlannedVelocityMps += Change;

	// Touchdown (the sole's touch is the fastest sense): the swing foot becomes the new stance foot
	// and the other foot lifts at once. (A double-support phase, the trailing leg holding its stance
	// targets while the weight moved across, was tried and did worse: Milestone 4 doc.)
	if (State.bStepping)
	{
		State.PhaseFraction = (Now.TimeS - StepStartTimeS) / StepTimeS;
		const int32 SwingNow = State.SwingLeg;
		// Landed = down, and still down a moment later: a landing foot can touch and bounce (measured:
		// back up 4 cm after the first touch, the other foot already lifted, the body dropped 6 cm).
		if (State.PhaseFraction < EarliestTouchdownFraction || Now.FootSoleHeightM[SwingNow] >= TouchdownSoleHeightM)
		{
			PlantedSinceS = -1.0;
		}
		else if (PlantedSinceS < 0.0)
		{
			PlantedSinceS = Now.TimeS;
		}
		const bool bTouched = PlantedSinceS >= 0.0 && Now.TimeS - PlantedSinceS >= PlantedConfirmS;
		if (bTouched || State.PhaseFraction >= LatestTouchdownFraction)
		{
			++State.StepCount;
			if (!Intent.WantsToStep() && State.PlannedVelocityMps.Size() < 0.05 && Flat(Now.CenterOfMassVelocityMps).Size() < StopSpeedMps)
			{
				State.bStepping = false;
				return false; // both feet down, slow enough: stand
			}
			StartStep(Other(SwingNow), Now);
			PlantedSinceS = -1.0;
		}
	}

	if (!State.bStepping && !State.bShifting)
	{
		if (!Intent.WantsToStep())
		{
			return false; // standing balance
		}
		// First step: swing the leg on the side he wants to go (sideways), else the right leg. Before
		// lifting it, shift the weight onto the other foot.
		const FVector Right = WorldYaw(State.HeadingYawDeg).GetRightVector();
		const double Sideways = FVector(Wish.X, Wish.Y, 0.0) | Right;
		State.SwingLeg = Sideways < -0.05 ? 0 : 1;
		State.bShifting = true;
		ShiftStartTimeS = Now.TimeS;
	}

	if (State.bShifting)
	{
		if (!Intent.WantsToStep())
		{
			State.bShifting = false;
			return false;
		}
		// Anticipatory postural adjustment before the first step: standing balance, aiming the
		// extrapolated center of mass at the stance foot instead of between the feet. Lifting a foot
		// that still carries weight would drop the body toward it. Not exactly over it: the
		// steady-rhythm offset toward the swing side, so that once the foot is up the body drifts
		// toward where it will land. (Standing balance assumes the feet where they stand at rest, so
		// between steps the double support below does this job instead.)
		const int32 StanceLeg = Other(State.SwingLeg);
		const FVector StanceAnkle = ForPlanning.JointCentersM[AthleteJoints::ToIndex(AnkleJoint[StanceLeg])];
		const double ComHeight = FMath::Max(ForPlanning.CenterOfMassM.Z - GroundHeightM, 0.3);
		const double TowardSwing = State.SwingLeg == 1 ? 1.0 : -1.0;
		FAthleteBalanceSensing OverStanceFoot = ForPlanning;
		OverStanceFoot.SupportCenterM = FVector(StanceAnkle.X, StanceAnkle.Y, GroundHeightM) + OverStanceFoot.ForwardAxis * Legs.AnkleToFootCenterM
			+ OverStanceFoot.RightAxis * TowardSwing * LateralOffset(Skill.StepWidthM, FMath::Sqrt(GravityMps2 / ComHeight), StepTimeS);
		// A deliberate shift: firmer than the quiet-standing gains (it has to move the body 9 cm, not hold it).
		FAthleteMotorSkill ShiftSkill = Skill;
		ShiftSkill.AnkleStrategyGain = WeightShiftGain;
		ShiftSkill.VelocityFeedbackGain = 1.0;
		const FAthleteBalanceCommand Command = AthleteBalanceController::Compute(OverStanceFoot, ShiftSkill, GravityMps2);
		const bool bOverStanceFoot = FMath::Abs(Command.XcomErrorRightM) < ShiftDoneLateralM;
		if (!bOverStanceFoot && Now.TimeS - ShiftStartTimeS < MaxShiftTimeS)
		{
			OutPosture = AthleteBalanceController::SolveJointTargets(AthleteBalanceController::MakeSegmentTargets(Command, GetHeadingDeviation()), ForPosture, Feet);
			return true;
		}
		State.bShifting = false;
		StartStep(State.SwingLeg, Now);
	}

	// Turn the heading toward where he wants to face, at his turning rate, only while stepping:
	// facing a new way takes steps.
	const bool bFaceTravel = Intent.bFaceMovementDirection;
	double DesiredYaw = State.HeadingYawDeg;
	if (bFaceTravel && State.PlannedVelocityMps.Size() > 0.2)
	{
		DesiredYaw = FMath::RadiansToDegrees(FMath::Atan2(State.PlannedVelocityMps.Y, State.PlannedVelocityMps.X));
	}
	else if (!bFaceTravel)
	{
		DesiredYaw = Intent.DesiredFacingYawDeg;
	}
	const double YawError = FMath::FindDeltaAngleDegrees(State.HeadingYawDeg, DesiredYaw);
	State.HeadingYawDeg += FMath::Clamp(YawError, -Skill.TurnRateDegPerS * DeltaTimeS, Skill.TurnRateDegPerS * DeltaTimeS);

	State.PhaseFraction = (Now.TimeS - StepStartTimeS) / StepTimeS;
	const int32 Swing = State.SwingLeg;
	const int32 Stance = Other(Swing);
	const double Phase = State.PhaseFraction;

	// --- Where to put the swing foot: the capture point predicted for touchdown, minus the offset
	// that carries the body on at the planned velocity, plus a sideways offset. ---
	const double ComHeight = FMath::Max(ForPlanning.CenterOfMassM.Z - GroundHeightM, 0.3);
	const double Omega0 = FMath::Sqrt(GravityMps2 / ComHeight);
	const FVector Com = Flat(ForPlanning.CenterOfMassM);
	const FVector ComVelocity = Flat(ForPlanning.CenterOfMassVelocityMps);
	const FVector CapturePoint = Com + ComVelocity / Omega0;
	const FVector StanceAnkle = Flat(StanceAnkleM);
	const FQuat HeadingWorld = WorldYaw(State.HeadingYawDeg);
	const FVector Forward = HeadingWorld.GetForwardVector();
	const FVector Right = HeadingWorld.GetRightVector();
	const double Side = Swing == 1 ? 1.0 : -1.0;
	// The body pivots over the stance foot's center of pressure, nominally mid-foot. (Feet were placed
	// by their centers while the body pivoted about the ankle, 5 cm further back: he walked forward
	// even when stepping in place.)
	const FVector StancePressure = StanceAnkle + Forward * Legs.AnkleToFootCenterM;
	// The perceived state is ReactionTimeS old: carry the capture point forward across that delay to
	// where it is now (LIPM over the stance foot's pressure point).
	const FVector PredictedCapturePoint = StancePressure + (CapturePoint - StancePressure) * FMath::Exp(Omega0 * Skill.ReactionTimeS);
	// Carrying on at the planned velocity: land the planned step length L = v T behind the capture
	// point divided by (exp(omega0 T) - 1). Over the next step the capture point then grows from that
	// offset to the offset plus L, exactly one step on.
	const FVector Planned(State.PlannedVelocityMps.X, State.PlannedVelocityMps.Y, 0.0);
	const double StepGrowth = FMath::Exp(Omega0 * StepTimeS);
	const FVector CarryOn = Planned * StepTimeS / (StepGrowth - 1.0);
	const double SideOffset = LateralOffset(Skill.StepWidthM, Omega0, StepTimeS);

	// Where the capture point should be on the planned walk, t into this step: it starts CarryOn
	// ahead and the steady-rhythm offset toward the swing side of the pressure point, and grows.
	auto PlannedCapturePointAt = [&](double SecondsIntoStep)
	{
		const double Growth = FMath::Exp(Omega0 * FMath::Clamp(SecondsIntoStep, 0.0, LatestTouchdownFraction * StepTimeS));
		return StancePressure + CarryOn * Growth + Right * Side * SideOffset * Growth;
	};
	const double SecondsIntoStep = Now.TimeS - StepStartTimeS;
	const FVector PlannedCapturePoint = PlannedCapturePointAt(SecondsIntoStep);

	// The foot goes where the capture point is now, re-evaluated every frame, so by touchdown it sits
	// where the capture point then is (Hof's constant-offset foot placement). Aiming instead at a
	// prediction for the moment of landing was tried two ways (the pendulum alone; the stance ankle
	// pulling the capture point onto its plan) and did worse overall (5-trial probes, Milestone 4).
	FVector FootTarget = PredictedCapturePoint - CarryOn + Right * Side * SideOffset;

	// --- Stance ankle (the ankle strategy of walking): shift the pressure along the foot to keep the
	// capture point on its planned path. Each step sets the capture point off from the pressure point
	// and it grows away by exp(omega0 t): landing 10 cm off costs 50 cm by the next touchdown. The
	// feet correct what the ankle can't; the ankle keeps it from growing between steps. ---
	const FVector PressureShift = CenterOfPressureGain * (PredictedCapturePoint - PlannedCapturePoint);
	// Within what the foot can take, with the whole body on it: the tipping torques are for half.
	FVector2D PressureOffset(Legs.AnkleToFootCenterM + (PressureShift | Forward), PressureShift | Right);
	if (Feet.LoadPerFootN > 0.0)
	{
		const double Margin = AthleteBalanceController::FootSupportMargin;
		PressureOffset.X = FMath::Clamp(PressureOffset.X, -Margin * Feet.TippingTorqueHeelNm / Feet.LoadPerFootN, Margin * Feet.TippingTorqueToesNm / Feet.LoadPerFootN);
		PressureOffset.Y = FMath::Clamp(PressureOffset.Y, -Margin * Feet.TippingTorqueSideNm / Feet.LoadPerFootN, Margin * Feet.TippingTorqueSideNm / Feet.LoadPerFootN);
	}
	State.PlannedCapturePointM = PlannedCapturePoint + FVector(0, 0, GroundHeightM);
	State.PressureOffsetM = PressureOffset;

	// Keep the feet apart side to side (no crossing) and the step within reach.
	const double MinSeparation = MinStanceWidthFraction * 2.0 * Legs.StanceHalfWidthM;
	const double Lateral = (FootTarget - StanceAnkle) | Right;
	if (Lateral * Side < MinSeparation)
	{
		FootTarget += Right * (Side * MinSeparation - Lateral);
	}
	const double MaxStep = MaxStepFractionOfLeg * (Legs.ThighLengthM + Legs.ShankLengthM);
	const FVector Step = FootTarget - StanceAnkle;
	if (Step.Size() > MaxStep)
	{
		FootTarget = StanceAnkle + Step.GetSafeNormal() * MaxStep;
	}
	State.CapturePointM = CapturePoint + FVector(0, 0, GroundHeightM);
	State.FootTargetM = FootTarget + FVector(0, 0, GroundHeightM);

	// --- The swing plan, as a function of the step's phase (for the current target, from the hip as
	// perceived): the ankle lifts, carries, sets down, then presses toward the ground until touch;
	// a two-link leg puts it there with the knee bending forward. Evaluated now for the targets, and
	// just before and after for the motion the internal model has to drive (see below). ---
	const FVector Hip = ForPosture.JointCentersM[AthleteJoints::ToIndex(HipJoint[Swing])];
	const FVector AnkleTargetFlat = FootTarget - Forward * Legs.AnkleToFootCenterM;
	// Push-off: walking, the trailing foot doesn't just lift. It rolls onto its toes and pushes
	// (plantarflexion) while the toes are still down, which launches the leg forward before the foot
	// clears the floor. More push the faster he means to go; none stepping in place.
	const double PushOffShare = FMath::Clamp(Planned.Size() / PushOffFullSpeedMps, 0.0, 1.0);
	auto SwingFootPitchDeg = [&Skill, PushOffShare](double CarryAt)
	{
		const double ToesDown = PushOffFootDeg * PushOffShare * FMath::Max(1.0 - CarryAt / PushOffEndCarry, 0.0);
		return Skill.SwingToeUpDeg * FMath::Sin(PI * CarryAt) - ToesDown;
	};
	struct FPlannedLeg
	{
		FVector Ankle, Knee, LegDirection, ThighDirection, ShankDirection;
		double Reach = 0.0;
		double Carry = 0.0;
		FQuat ThighWorld, ShankWorld, FootWorld;
	};
	auto PlanLegAt = [&](double PhaseAt)
	{
		FPlannedLeg Plan;
		Plan.Carry = FMath::Clamp(PhaseAt / SwingArrivalFraction, 0.0, 1.0);
		Plan.Ankle = FMath::Lerp(Flat(LiftoffAnkleM), AnkleTargetFlat, SmoothStep(Plan.Carry));
		const double Lift = Skill.SwingHeightM * FMath::Sin(PI * Plan.Carry);
		const double PressDown = PressDownM * FMath::Clamp((PhaseAt - SwingArrivalFraction) / (1.0 - SwingArrivalFraction), 0.0, 1.0);
		Plan.Ankle.Z = GroundHeightM + Legs.AnkleHeightM + Lift - PressDown;

		const double L1 = Legs.ThighLengthM;
		const double L2 = Legs.ShankLengthM;
		const FVector HipToAnkle = Plan.Ankle - Hip;
		Plan.Reach = FMath::Clamp(HipToAnkle.Size(), 0.3 * (L1 + L2), 0.995 * (L1 + L2));
		Plan.LegDirection = HipToAnkle.GetSafeNormal();
		const double CosHipAngle = FMath::Clamp((L1 * L1 + Plan.Reach * Plan.Reach - L2 * L2) / (2.0 * L1 * Plan.Reach), -1.0, 1.0);
		const double HipAngle = FMath::Acos(CosHipAngle);
		const FVector KneeForward = (Forward - (Forward | Plan.LegDirection) * Plan.LegDirection).GetSafeNormal();
		Plan.ThighDirection = (Plan.LegDirection * FMath::Cos(HipAngle) + KneeForward * FMath::Sin(HipAngle)).GetSafeNormal();
		Plan.Knee = Hip + Plan.ThighDirection * L1;
		Plan.ShankDirection = (Hip + Plan.LegDirection * Plan.Reach - Plan.Knee).GetSafeNormal();
		// World orientations: segments hang along -Z in the reference pose, facing the heading; the
		// foot faces the heading, rolls onto its toes to push off (walking), then toes up to clear,
		// flat for landing.
		Plan.ThighWorld = FQuat::FindBetweenNormals(FVector(0, 0, -1), Plan.ThighDirection) * HeadingWorld;
		Plan.ShankWorld = FQuat::FindBetweenNormals(FVector(0, 0, -1), Plan.ShankDirection) * HeadingWorld;
		Plan.FootWorld = HeadingWorld * AthleteJointSetup::RotateToward(FVector(1, 0, 0), FVector(0, 0, 1), SwingFootPitchDeg(Plan.Carry));
		return Plan;
	};
	const FPlannedLeg SwingPlan = PlanLegAt(Phase);
	const double Carry = SwingPlan.Carry;
	const FVector AnkleTarget = SwingPlan.Ankle;
	State.SwingAnkleTargetM = AnkleTarget;
	const double Reach = SwingPlan.Reach;
	const FVector LegDirection = SwingPlan.LegDirection;
	const FVector ThighDirection = SwingPlan.ThighDirection;
	const FVector Knee = SwingPlan.Knee;
	const FVector ShankDirection = SwingPlan.ShankDirection;

	// World orientation of a leg segment pointing along Direction, facing the heading, as a
	// deviation from the reference pose (segments hang along -Z in the reference pose).
	const FQuat BodyFrameInverse = WorldYaw(BodyFrameYawDeg).Inverse();
	auto LegSegmentDeviation = [&](const FVector& Direction)
	{
		const FQuat World = FQuat::FindBetweenNormals(FVector(0, 0, -1), Direction) * HeadingWorld;
		return BodyFrameInverse * World;
	};
	const FQuat DesiredSwingThigh = LegSegmentDeviation(ThighDirection);
	const FQuat DesiredSwingShank = LegSegmentDeviation(ShankDirection);
	// Swing foot: facing the heading, toes up mid-swing, flat for landing.
	const double ToeUpDeg = SwingFootPitchDeg(Carry);
	const FQuat DesiredSwingFoot = BodyFrameInverse * HeadingWorld * AthleteJointSetup::RotateToward(FVector(1, 0, 0), FVector(0, 0, 1), ToeUpDeg);
	// Stance foot: flat, keeping the yaw it landed with.
	const FQuat StanceFootWorld = ForPosture.BodyFrameRotation * ForPosture.GetSegmentDeviation(FootSegment[Stance]);
	const double StanceFootYawDeg = YawDegOf(StanceFootWorld);
	const FQuat DesiredStanceFoot = BodyFrameInverse * WorldYaw(StanceFootYawDeg);
	// Stance shank: free to tilt over the foot (the body pivots like an inverted pendulum), but its
	// twist about its own axis held to the planted foot's heading (the foot grips the ground; the leg
	// must not spin over it). Its current tilt, with the foot's yaw.
	const FVector StanceShankAxis = (ForPosture.SegmentRotations[static_cast<int32>(ShankSegment[Stance])].RotateVector(FVector(0, 0, -1))).GetSafeNormal();
	const FQuat HeldStanceShank = BodyFrameInverse * FQuat::FindBetweenNormals(FVector(0, 0, -1), StanceShankAxis) * WorldYaw(StanceFootYawDeg);

	// --- Joint targets. Spine, neck and arms: the standing posture rules, facing the heading. ---
	const FQuat Heading = GetHeadingDeviation();
	const FAthleteSegmentTargets Upright = AthleteBalanceController::MakeSegmentTargets(FAthleteBalanceCommand(), Heading);
	OutPosture = AthleteBalanceController::SolveJointTargets(Upright, ForPosture, FAthleteFootSupport());
	const FQuat DesiredPelvis = Upright.Deviation[static_cast<int32>(EAthleteSegment::LowerTrunk)];
	auto Perceived = [&ForPosture](EAthleteSegment Segment) { return ForPosture.GetSegmentDeviation(Segment); };
	auto Set = [&OutPosture](EAthleteJoint Joint, const FQuat& ChildRelativeToParent, double StiffnessScale)
	{
		OutPosture.ChildRelativeToParent[AthleteJoints::ToIndex(Joint)] = ChildRelativeToParent;
		OutPosture.StiffnessScale[AthleteJoints::ToIndex(Joint)] = StiffnessScale;
	};

	// One leg now carries the whole body: twice the load of standing on two, so twice the activation.
	constexpr double SingleSupportScale = 2.0;
	// The stance hip also has to hold the pelvis against the swing leg's pull (the reaction to swinging
	// the leg pitches and turns the pelvis): it must clearly out-stiffen the swing hip. The hip
	// extensors and abductors are among the strongest muscles in the body.
	constexpr double StanceHipScale = 4.0;
	// Stance leg: pelvis upright and facing the heading over the thigh as it is (this is where the
	// turn happens), a nearly straight knee, a foot kept flat under the pivoting shank.
	Set(HipJoint[Stance], DesiredPelvis.Inverse() * Perceived(ThighSegment[Stance]), StanceHipScale);
	Set(KneeJoint[Stance], AthleteJointSetup::RotateToward(FVector(0, 0, -1), FVector(-1, 0, 0), Skill.StanceKneeFlexionDeg), SingleSupportScale);
	Set(AnkleJoint[Stance], HeldStanceShank.Inverse() * DesiredStanceFoot, SingleSupportScale);
	// The ankle puts the pressure where the capture point needs it, by activation: the torque that
	// holds the foot against the whole body's weight pressing at that point (muscle on the foot =
	// -(lever x load)). Done by the reflex spring instead (turning its target) it fell short: the
	// light foot caps the ankle's usable stiffness in a substep (measured: 40% of the torque asked).
	const FVector PressureLever = Forward * PressureOffset.X + Right * PressureOffset.Y;
	const FVector Load(0.0, 0.0, 2.0 * Feet.LoadPerFootN);
	OutPosture.FeedforwardTorqueNm[AthleteJoints::ToIndex(AnkleJoint[Stance])] = -FVector::CrossProduct(PressureLever, Load);

	// Swing leg: in JOINT space, from where the pelvis SHOULD be (level, facing the heading), not from
	// where it is. Holding the swing thigh at an orientation in space relative to the actual pelvis
	// makes the swing hip push the pelvis further whichever way it drifts (its reaction torque), and
	// at swing activation it overpowers the stance hip: the pelvis spun and tipped. With joint-space
	// targets the stance hip alone holds the pelvis, and a pelvis error just shifts the foot a little.
	// (SIMBICON, Yin et al. 2007, solves the same problem by subtracting the swing hip torque from
	// the stance hip.)
	Set(HipJoint[Swing], DesiredPelvis.Inverse() * DesiredSwingThigh, Skill.SwingStiffnessScale);
	Set(KneeJoint[Swing], DesiredSwingThigh.Inverse() * DesiredSwingShank, Skill.SwingStiffnessScale);
	Set(AnkleJoint[Swing], DesiredSwingShank.Inverse() * DesiredSwingFoot, 1.0);
	// The swing is a planned movement, not a posture: its joints are commanded the motion itself.
	// Without that, their own damping held the foot back (measured: the ankle 0.2-0.45 m behind its
	// target late in the swing), every step landed short of the capture point, and he ran forward.
	for (const EAthleteJoint Joint : { HipJoint[Swing], KneeJoint[Swing], AnkleJoint[Swing] })
	{
		OutPosture.bPlannedMotion[AthleteJoints::ToIndex(Joint)] = true;
	}
	// Swinging the leg forward pulls the pelvis into a forward tilt just as hard (measured: the pelvis
	// pitched 5-13 deg while the swing hip lagged 10 deg). The stance hip takes up that reaction,
	// passing it down the stance leg to the ground, so the pelvis only feels its own control.
	OutPosture.CancelReactionOf[AthleteJoints::ToIndex(HipJoint[Stance])] = AthleteJoints::ToIndex(HipJoint[Swing]);

	// The swing leg is moved by anticipation (an internal model of the limb), not by lagging until the
	// reflex stiffness catches up: each swing joint also produces the torque the PLANNED motion takes,
	// against gravity and the leg's own inertia (inverse dynamics, foot to hip). The reflex stiffness
	// then only corrects what the model gets wrong. Measured without it: the swing hip, which has to
	// swing the whole leg, fell behind the knee, and stepping in place every foot landed 7-30 cm
	// ahead of its target (and the misses walked him backward).
	{
		const FQuat ThighWorld = SwingPlan.ThighWorld;
		const FQuat ShankWorld = SwingPlan.ShankWorld;
		const FQuat FootWorld = SwingPlan.FootWorld;

		// The plan's own spin and angular acceleration: the same trajectory, for the same target, a
		// moment either side (central differences). Not frame-to-frame differences of the targets:
		// those also carry every correction of the target as the capture point moves, which the
		// reflexes handle; differentiated twice they were noise the hip obeyed (measured: walking,
		// the swing foot shot up and hung there until the step timed out).
		const double PhaseStep = DeltaTimeS > 0.0 ? DeltaTimeS / StepTimeS : 0.0;
		const FPlannedLeg Before = PlanLegAt(FMath::Max(Phase - PhaseStep, 0.0));
		const FPlannedLeg After = PlanLegAt(Phase + PhaseStep);
		const double TimeBefore = (Phase - FMath::Max(Phase - PhaseStep, 0.0)) * StepTimeS;
		const double TimeAfter = PhaseStep * StepTimeS;
		auto SpinBetween = [](const FQuat& From, const FQuat& To, double Seconds)
		{
			if (Seconds <= 0.0)
			{
				return FVector::ZeroVector;
			}
			FQuat Turn = To * From.Inverse();
			Turn.EnforceShortestArcWith(FQuat::Identity);
			return Turn.ToRotationVector() / Seconds;
		};
		FVector Spin[3];
		FVector Alpha[3];
		const FQuat FromBefore[3] = { Before.ThighWorld, Before.ShankWorld, Before.FootWorld };
		const FQuat AtNow[3] = { ThighWorld, ShankWorld, FootWorld };
		const FQuat ToAfter[3] = { After.ThighWorld, After.ShankWorld, After.FootWorld };
		for (int32 Segment = 0; Segment < 3; ++Segment)
		{
			const FVector SpinIn = SpinBetween(FromBefore[Segment], AtNow[Segment], TimeBefore);
			const FVector SpinOut = SpinBetween(AtNow[Segment], ToAfter[Segment], TimeAfter);
			Spin[Segment] = TimeBefore > 0.0 ? 0.5 * (SpinIn + SpinOut) : SpinOut;
			Alpha[Segment] = TimeBefore > 0.0 ? (SpinOut - SpinIn) / (0.5 * (TimeBefore + TimeAfter)) : FVector::ZeroVector;
		}

		// Planned positions (the hip as perceived; its own acceleration is small next to the leg's).
		const FVector HipAt = Hip;
		const FVector KneeAt = Knee;
		const FVector AnkleAt = Hip + LegDirection * Reach;
		const FVector ThighCom = ThighWorld.RotateVector(Legs.ThighComFromHipM[Swing]);  // from the hip
		const FVector ShankCom = ShankWorld.RotateVector(Legs.ShankComFromKneeM[Swing]); // from the knee
		const FVector FootCom = FootWorld.RotateVector(Legs.FootComFromAnkleM[Swing]);   // from the ankle
		auto Accelerate = [](const FVector& W, const FVector& A, const FVector& R) { return (A ^ R) + (W ^ (W ^ R)); };
		const FVector KneeAcceleration = Accelerate(Spin[0], Alpha[0], KneeAt - HipAt);
		const FVector AnkleAcceleration = KneeAcceleration + Accelerate(Spin[1], Alpha[1], AnkleAt - KneeAt);
		const FVector ThighComAcceleration = Accelerate(Spin[0], Alpha[0], ThighCom);
		const FVector ShankComAcceleration = KneeAcceleration + Accelerate(Spin[1], Alpha[1], ShankCom);
		const FVector FootComAcceleration = AnkleAcceleration + Accelerate(Spin[2], Alpha[2], FootCom);

		// Newton-Euler, foot to hip: the torque each joint must put on everything below it.
		const FVector Gravity(0.0, 0.0, -GravityMps2);
		auto InertiaTimes = [](const FQuat& World, const FVector& Principal, const FVector& V) { return World.RotateVector(Principal * World.UnrotateVector(V)); };
		auto SpinTorque = [&](const FQuat& World, const FVector& Principal, const FVector& W, const FVector& A)
		{
			return InertiaTimes(World, Principal, A) + (W ^ InertiaTimes(World, Principal, W));
		};
		const FVector FootForce = Legs.FootMassKg * (FootComAcceleration - Gravity);
		const FVector AnkleTorque = SpinTorque(FootWorld, Legs.FootInertiaKgM2, Spin[2], Alpha[2]) + (FootCom ^ FootForce);
		const FVector ShankOwnForce = Legs.ShankMassKg * (ShankComAcceleration - Gravity);
		const FVector KneeTorque = SpinTorque(ShankWorld, Legs.ShankInertiaKgM2, Spin[1], Alpha[1]) + (ShankCom ^ ShankOwnForce)
			+ ((AnkleAt - KneeAt) ^ FootForce) + AnkleTorque;
		const FVector ThighOwnForce = Legs.ThighMassKg * (ThighComAcceleration - Gravity);
		const FVector HipTorque = SpinTorque(ThighWorld, Legs.ThighInertiaKgM2, Spin[0], Alpha[0]) + (ThighCom ^ ThighOwnForce)
			+ ((KneeAt - HipAt) ^ (ShankOwnForce + FootForce)) + KneeTorque;
		OutPosture.FeedforwardTorqueNm[AthleteJoints::ToIndex(AnkleJoint[Swing])] = AnkleTorque;
		OutPosture.FeedforwardTorqueNm[AthleteJoints::ToIndex(KneeJoint[Swing])] = KneeTorque;
		OutPosture.FeedforwardTorqueNm[AthleteJoints::ToIndex(HipJoint[Swing])] = HipTorque;

		// Push-off: while the toes are still on the ground early in the swing, the ankle pushes them
		// down (plantarflexion), within what the forefoot can take. The ground pushes back up the leg.
		if (PushOffShare > 0.0 && Carry < PushOffEndCarry && Now.FootSoleHeightM[Swing] < PushOffContactHeightM && Feet.LoadPerFootN > 0.0)
		{
			const double ToeLeverM = AthleteBalanceController::FootSupportMargin * Feet.TippingTorqueToesNm / Feet.LoadPerFootN;
			const double PushN = PushOffShare * 2.0 * Feet.LoadPerFootN * (1.0 - Carry / PushOffEndCarry);
			OutPosture.FeedforwardTorqueNm[AthleteJoints::ToIndex(AnkleJoint[Swing])] -= FVector::CrossProduct(Forward * ToeLeverM, FVector(0.0, 0.0, PushN));
		}
	}

	return true;
}
