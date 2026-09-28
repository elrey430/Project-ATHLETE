// Project ATHLETE

#include "Balance/AthleteBalanceController.h"
#include "Articulation/AthleteJointSetup.h"
#include "Capability/AthleteMotorSkill.h"

namespace
{
	/** Hip strategy engages once the XCoM error exceeds this fraction of the half foot length. */
	constexpr double HipStrategyOnsetFraction = 0.5;

	/** Command limits, in degrees, kept inside normal ranges of motion. */
	constexpr double MaxLeanForwardDeg = 15.0;  // ankle dorsiflexion range is 20 deg
	constexpr double MaxLeanBackwardDeg = 15.0; // plantarflexion is 50 deg, but leaning back far is useless
	constexpr double MaxLeanSidewaysDeg = 10.0;
	constexpr double MaxHipFlexionDeg = 60.0;
	constexpr double MaxHipExtensionDeg = 20.0;


	const FVector BodyForward(1, 0, 0);
	const FVector BodyRight(0, 1, 0);
	const FVector BodyUp(0, 0, 1);
}

FAthleteBalanceCommand AthleteBalanceController::Compute(const FAthleteBalanceSensing& Sensing, const FAthleteMotorSkill& Skill, double GravityMps2)
{
	FAthleteBalanceCommand Command;

	const double Height = FMath::Max(Sensing.CenterOfMassM.Z - Sensing.SupportCenterM.Z, 0.1);
	const double Omega0 = FMath::Sqrt(GravityMps2 / Height);
	const FVector Xcom = Sensing.CenterOfMassM + Skill.VelocityFeedbackGain * Sensing.CenterOfMassVelocityMps / Omega0;
	const FVector Error = Xcom - Sensing.SupportCenterM;
	const double ErrorForward = Error | Sensing.ForwardAxis;
	const double ErrorRight = Error | Sensing.RightAxis;
	Command.XcomErrorForwardM = ErrorForward;
	Command.XcomErrorRightM = ErrorRight;

	// Ankle strategy: lean against the error, by the angle the error subtends at the ankles.
	const double LeanForward = -Skill.AnkleStrategyGain * FMath::RadiansToDegrees(FMath::Atan2(ErrorForward, Height));
	const double LeanRight = -Skill.AnkleStrategyGain * FMath::RadiansToDegrees(FMath::Atan2(ErrorRight, Height));
	Command.LeanForwardDeg = FMath::Clamp(LeanForward, -MaxLeanBackwardDeg, MaxLeanForwardDeg);
	Command.LeanRightDeg = FMath::Clamp(LeanRight, -MaxLeanSidewaysDeg, MaxLeanSidewaysDeg);

	// Hip strategy: bend toward the fall by the angle of the excess beyond what the feet can hold.
	const double Onset = HipStrategyOnsetFraction * Sensing.FootHalfLengthM;
	const double Excess = FMath::Max(FMath::Abs(ErrorForward) - Onset, 0.0) * FMath::Sign(ErrorForward);
	const double HipFlexion = Skill.HipStrategyGain * FMath::RadiansToDegrees(FMath::Atan2(Excess, Height));
	Command.HipFlexionDeg = FMath::Clamp(HipFlexion, -MaxHipExtensionDeg, MaxHipFlexionDeg);
	return Command;
}

FAthleteSegmentTargets AthleteBalanceController::MakeSegmentTargets(const FAthleteBalanceCommand& Command)
{
	const FQuat Lean = AthleteJointSetup::RotateToward(BodyUp, BodyForward, Command.LeanForwardDeg);
	// Pelvis toward +Right means each leg's top moves right: the leg tilts from vertical toward +Right.
	const FQuat SideTilt = AthleteJointSetup::RotateToward(BodyUp, BodyRight, Command.LeanRightDeg);
	const FQuat HipPitch = AthleteJointSetup::RotateToward(BodyUp, BodyForward, Command.HipFlexionDeg);
	const FQuat Legs = SideTilt * Lean;
	const FQuat Trunk = HipPitch * Lean;

	FAthleteSegmentTargets Targets;
	for (int32 Index = 0; Index < AthleteSegments::NumSegments; ++Index)
	{
		switch (AthleteSegments::GetKind(static_cast<EAthleteSegment>(Index)))
		{
		case EAthleteSegmentKind::Thigh:
		case EAthleteSegmentKind::Shank:
			Targets.Deviation[Index] = Legs;
			break;
		case EAthleteSegmentKind::Foot:
			Targets.Deviation[Index] = FQuat::Identity; // flat on the ground (the feet are the base, not controlled here)
			break;
		default:
			Targets.Deviation[Index] = Trunk; // pelvis, abdomen, thorax, head (and arms, which hold joint angles)
			break;
		}
	}
	return Targets;
}

FAthletePosture AthleteBalanceController::SolveJointTargets(const FAthleteSegmentTargets& Targets, const FAthleteBalanceSensing& Perceived, const FAthleteFootSupport& Feet)
{
	// Where the shank may be sent, relative to where it is: its sagittal turn is capped at the angle
	// whose ankle torque (stiffness x angle) the foot can still take. Leaning the shank BACK takes
	// plantarflexor torque (pressure toward the toes), leaning it FORWARD dorsiflexor torque (heel).
	auto LimitShank = [&Feet, &Perceived](const FQuat& Desired, EAthleteSegment Shank)
	{
		if (Feet.AnkleStiffnessNmPerRad <= 0.0)
		{
			return Desired;
		}
		const FQuat Current = Perceived.GetSegmentDeviation(Shank);
		FVector Turn = (Desired * Current.Inverse()).GetShortestArcWith(FQuat::Identity).ToRotationVector(); // + about Y = forward
		const double MaxBack = FootSupportMargin * Feet.TippingTorqueToesNm / Feet.AnkleStiffnessNmPerRad;
		const double MaxForward = FootSupportMargin * Feet.TippingTorqueHeelNm / Feet.AnkleStiffnessNmPerRad;
		const double MaxSideways = FootSupportMargin * Feet.TippingTorqueSideNm / Feet.AnkleStiffnessNmPerRad;
		Turn.Y = FMath::Clamp(Turn.Y, -MaxBack, MaxForward);
		Turn.X = FMath::Clamp(Turn.X, -MaxSideways, MaxSideways); // about the forward axis: sideways tilt
		return FQuat::MakeFromRotationVector(Turn) * Current;
	};

	FAthletePosture Posture;
	for (int32 Index = 0; Index < AthleteJoints::NumJoints; ++Index)
	{
		const EAthleteJoint Joint = AthleteJoints::FromIndex(Index);
		const EAthleteSegment Parent = AthleteJoints::GetParentSegment(Joint);
		const EAthleteSegment Child = AthleteJoints::GetChildSegment(Joint);
		const FQuat& DesiredParent = Targets.Deviation[static_cast<int32>(Parent)];
		const FQuat& DesiredChild = Targets.Deviation[static_cast<int32>(Child)];

		// Child relative to parent = Parent^-1 * Child (orientations as deviations from reference).
		switch (AthleteJoints::GetKind(Joint))
		{
		case EAthleteJointKind::Ankle:
			// The foot is the base; the muscle turns the shank, within what the foot can take.
			Posture.ChildRelativeToParent[Index] = LimitShank(DesiredParent, Parent).Inverse() * Perceived.GetSegmentDeviation(Child);
			break;
		case EAthleteJointKind::Hip:
		{
			// Sagittal: like the other leg joints, turn the pelvis relative to the thigh as it is.
			// Frontal: with both feet down, legs, pelvis, and ground form a closed loop, and sideways
			// balance works by the hips shifting load from one foot to the other (hip load/unload
			// mechanism, Winter 1995). That needs the hips to hold the thigh-to-pelvis ANGLE, so the
			// frontal part of the target is the desired joint angle, not "accept the thigh's roll".
			const FQuat FromActual = DesiredParent.Inverse() * Perceived.GetSegmentDeviation(Child);
			const FQuat FromDesired = DesiredParent.Inverse() * DesiredChild;
			FVector Correction = (FromActual * FromDesired.Inverse()).GetShortestArcWith(FQuat::Identity).ToRotationVector();
			Correction.X = 0.0; // about the forward axis: sideways
			Posture.ChildRelativeToParent[Index] = FQuat::MakeFromRotationVector(Correction) * FromDesired;
			break;
		}
		case EAthleteJointKind::Knee:
			// Support comes from the ground: the child (foot, shank, thigh) is the base, as it is;
			// the muscle turns the parent (shank, thigh, pelvis) to where it should be.
			Posture.ChildRelativeToParent[Index] = DesiredParent.Inverse() * Perceived.GetSegmentDeviation(Child);
			break;
		case EAthleteJointKind::Lumbar:
		case EAthleteJointKind::Thoracic:
		case EAthleteJointKind::Neck:
			// From the pelvis up: the parent is the base, as it is; turn the child where it should be.
			Posture.ChildRelativeToParent[Index] = Perceived.GetSegmentDeviation(Parent).Inverse() * DesiredChild;
			break;
		default:
			Posture.ChildRelativeToParent[Index] = FQuat::Identity; // arms hold the reference joint angles
			break;
		}
	}
	return Posture;
}
