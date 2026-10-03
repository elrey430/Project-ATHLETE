// Project ATHLETE

#include "Muscle/AthleteMuscleModel.h"
#include "Anatomy/AthleteBodyModel.h"
#include "Capability/AthleteMotorSkill.h"

namespace
{
	/** Segments whose weight this joint carries in upright standing, and how many joints share it. */
	void GetSupportedSegments(EAthleteJoint Joint, TArray<EAthleteSegment>& OutSegments, int32& OutSharedBy)
	{
		OutSegments.Reset();
		OutSharedBy = 1;

		auto AllExcept = [&OutSegments](std::initializer_list<EAthleteSegment> Excluded)
		{
			for (int32 Index = 0; Index < AthleteSegments::NumSegments; ++Index)
			{
				const EAthleteSegment Segment = AthleteSegments::FromIndex(Index);
				bool bExcluded = false;
				for (const EAthleteSegment Excl : Excluded)
				{
					bExcluded |= (Excl == Segment);
				}
				if (!bExcluded)
				{
					OutSegments.Add(Segment);
				}
			}
		};

		using ES = EAthleteSegment;
		const bool bLeft = AthleteJoints::GetSide(Joint) == EAthleteBodySide::Left;
		switch (AthleteJoints::GetKind(Joint))
		{
		// Standing: each leg joint supports everything above it, shared by both legs.
		case EAthleteJointKind::Ankle: AllExcept({ ES::FootLeft, ES::FootRight }); OutSharedBy = 2; break;
		case EAthleteJointKind::Knee:  AllExcept({ ES::FootLeft, ES::FootRight, ES::ShankLeft, ES::ShankRight }); OutSharedBy = 2; break;
		case EAthleteJointKind::Hip:   AllExcept({ ES::FootLeft, ES::FootRight, ES::ShankLeft, ES::ShankRight, ES::ThighLeft, ES::ThighRight }); OutSharedBy = 2; break;
		// Trunk and neck support what sits above them.
		case EAthleteJointKind::Lumbar:
			OutSegments = { ES::MiddleTrunk, ES::UpperTrunk, ES::Head, ES::UpperArmLeft, ES::ForearmLeft, ES::HandLeft, ES::UpperArmRight, ES::ForearmRight, ES::HandRight };
			break;
		case EAthleteJointKind::Thoracic:
			OutSegments = { ES::UpperTrunk, ES::Head, ES::UpperArmLeft, ES::ForearmLeft, ES::HandLeft, ES::UpperArmRight, ES::ForearmRight, ES::HandRight };
			break;
		case EAthleteJointKind::Neck: OutSegments = { ES::Head }; break;
		// Arm joints carry the part of the arm beyond them.
		case EAthleteJointKind::Shoulder:
			OutSegments = bLeft ? TArray<ES>{ ES::UpperArmLeft, ES::ForearmLeft, ES::HandLeft } : TArray<ES>{ ES::UpperArmRight, ES::ForearmRight, ES::HandRight };
			break;
		case EAthleteJointKind::Elbow:
			OutSegments = bLeft ? TArray<ES>{ ES::ForearmLeft, ES::HandLeft } : TArray<ES>{ ES::ForearmRight, ES::HandRight };
			break;
		case EAthleteJointKind::Wrist:
			OutSegments = { bLeft ? ES::HandLeft : ES::HandRight };
			break;
		default: checkNoEntry(); break;
		}
	}

	/** Midpoint of a joint and its mirror (leg joints act as a pair in standing). */
	FVector PairedJointCenter(const FAthleteBodyModel& Model, EAthleteJoint Joint, int32 SharedBy)
	{
		const FVector Center = Model.GetJointCenterM(Joint);
		if (SharedBy < 2)
		{
			return Center;
		}
		return FVector(Center.X, 0.0, Center.Z); // the pair's midpoint lies on the midline
	}
}

EAthleteJointAction AthleteMuscleModel::GetLimitingAction(EAthleteJoint Joint)
{
	// The action that resists gravity (or holds the limb) in the standing posture.
	switch (AthleteJoints::GetKind(Joint))
	{
	case EAthleteJointKind::Ankle:    return EAthleteJointAction::AnklePlantarFlexion;
	case EAthleteJointKind::Knee:     return EAthleteJointAction::KneeExtension;
	case EAthleteJointKind::Hip:      return EAthleteJointAction::HipExtension;
	case EAthleteJointKind::Lumbar:
	case EAthleteJointKind::Thoracic: return EAthleteJointAction::TrunkExtension;
	case EAthleteJointKind::Neck:     return EAthleteJointAction::NeckExtension;
	case EAthleteJointKind::Shoulder: return EAthleteJointAction::ShoulderAbduction;
	case EAthleteJointKind::Elbow:    return EAthleteJointAction::ElbowFlexion;
	case EAthleteJointKind::Wrist:    return EAthleteJointAction::WristFlexion;
	default:                          checkNoEntry(); return EAthleteJointAction::Count;
	}
}

FAthleteJointImpedance AthleteMuscleModel::ComputeStandingImpedance(const FAthleteBodyModel& Model, EAthleteJoint Joint,
	const FAthleteStrengthProfile& Strength, const FAthleteMotorSkill& Skill, double GravityMps2)
{
	TArray<EAthleteSegment> Supported;
	int32 SharedBy = 1;
	GetSupportedSegments(Joint, Supported, SharedBy);

	// Mass, center of mass, and inertia (about the side-to-side axis through the joint) of the
	// supported set. Sagittal (Y) inertia is used: forward/backward sway dominates standing.
	const FVector Pivot = PairedJointCenter(Model, Joint, SharedBy);
	double Mass = 0.0;
	FVector WeightedCom = FVector::ZeroVector;
	double InertiaAboutPivot = 0.0;
	for (const EAthleteSegment Segment : Supported)
	{
		const FAthleteBodySegment& Seg = Model.GetSegment(Segment);
		Mass += Seg.MassKg;
		WeightedCom += Seg.CenterOfMassM * Seg.MassKg;
		const FVector R = Seg.CenterOfMassM - Pivot;
		InertiaAboutPivot += Seg.PrincipalInertiaKgM2.Y + Seg.MassKg * (R.X * R.X + R.Z * R.Z);
	}
	const FVector Com = WeightedCom / Mass;
	const double Lever = FVector::Dist(Com, Pivot);

	FAthleteJointImpedance Impedance;
	Impedance.Joint = Joint;
	Impedance.GravityStiffnessNmPerRad = Mass * GravityMps2 * Lever / SharedBy;
	Impedance.EffectiveInertiaKgM2 = InertiaAboutPivot / SharedBy;
	Impedance.StiffnessNmPerRad = Skill.MuscleToneGain * Impedance.GravityStiffnessNmPerRad;
	Impedance.DampingNmsPerRad = 2.0 * Skill.DampingRatio * FMath::Sqrt(Impedance.StiffnessNmPerRad * Impedance.EffectiveInertiaKgM2);
	Impedance.AxisStiffnessNmPerRad = FVector(Impedance.StiffnessNmPerRad);
	Impedance.AxisDampingNmsPerRad = FVector(Impedance.DampingNmsPerRad);

	if (AthleteJoints::GetKind(Joint) == EAthleteJointKind::Hip)
	{
		// Frontal axis: the whole body above the ankles swaying sideways about the feet, shared by
		// both hips (see header). That is exactly the ankles' standing load.
		const EAthleteJoint Ankle = AthleteJoints::GetSide(Joint) == EAthleteBodySide::Left ? EAthleteJoint::AnkleLeft : EAthleteJoint::AnkleRight;
		const FAthleteJointImpedance Sway = ComputeStandingImpedance(Model, Ankle, Strength, Skill, GravityMps2);
		Impedance.AxisStiffnessNmPerRad.Z = Sway.StiffnessNmPerRad;
		Impedance.AxisDampingNmsPerRad.Z = Sway.DampingNmsPerRad;
	}

	Impedance.LimitingAction = GetLimitingAction(Joint);
	const FAthleteJointActionStrength* Limit = Strength.Find(Impedance.LimitingAction);
	const double MaxVelocityDegPerSec = (Limit && Limit->MaxVelocityDegPerSec > 0.0) ? Limit->MaxVelocityDegPerSec : AthleteStrength::GetEstimatedMaxVelocityDegPerSec(Impedance.LimitingAction);
	Impedance.MaxVelocityRadPerS = FMath::DegreesToRadians(MaxVelocityDegPerSec);
	// A strength measured while the joint moved (isokinetic) understates the isometric strength:
	// divide out the force-velocity factor at the test speed.
	const double TestFactor = (Limit && Limit->TestMode == EAthleteStrengthTestMode::Isokinetic)
		? ForceVelocityFactor(Limit->TestVelocityDegPerSec / MaxVelocityDegPerSec) : 1.0;
	Impedance.TorqueLimitNm = (Limit && TestFactor > 0.0) ? Limit->PeakTorqueNm / TestFactor : 0.0;
	return Impedance;
}

double AthleteMuscleModel::ForceVelocityFactor(double ShorteningFraction)
{
	if (ShorteningFraction >= 0.0)
	{
		// Hill's hyperbola, normalized: F/F0 = (1 - v/vmax) / (1 + v/(k vmax)).
		const double X = FMath::Min(ShorteningFraction, 1.0);
		return (1.0 - X) / (1.0 + X / HillCurvature);
	}
	// Lengthening: a mirrored hyperbola, from 1 at rest to the eccentric plateau at y = 1.
	// Slope at zero is 0.5 * (1 + 6 / k) = 12.5 vs 1 + 1 / k = 5 when shortening.
	const double Y = FMath::Min(-ShorteningFraction, 1.0);
	return EccentricPlateau - (EccentricPlateau - 1.0) * (1.0 - Y) / (1.0 + 6.0 * Y / HillCurvature);
}
