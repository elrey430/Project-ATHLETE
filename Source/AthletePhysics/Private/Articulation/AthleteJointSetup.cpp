// Project ATHLETE

#include "Articulation/AthleteJointSetup.h"
#include "Anatomy/AthleteBodyModel.h"

namespace
{
	const FVector Forward(1, 0, 0);
	const FVector Up(0, 0, 1);

	/** Anatomical directions for a joint, in the reference pose. */
	struct FJointDirections
	{
		FVector LongAxis;        // child segment's long axis, from the joint outward
		FVector Flexion;         // direction the long-axis tip moves under flexion (ankle: dorsiflexion)
		FVector Abduction;       // direction the tip moves under abduction / lateral flexion
		FVector AxialReference;  // a vector perpendicular to the long axis...
		FVector InternalRotation;// ...and the direction it turns under internal rotation (ankle: inversion)
	};

	FJointDirections GetDirections(EAthleteJoint Joint)
	{
		// Lateral = away from the midline. Left side is -Y. Midline joints use +Y by convention;
		// their ranges are symmetric, so the choice has no effect.
		const EAthleteBodySide Side = AthleteJoints::GetSide(Joint);
		const FVector Lateral(0, Side == EAthleteBodySide::Left ? -1.0 : 1.0, 0);
		const FVector Medial = -Lateral;

		FJointDirections D;
		switch (AthleteJoints::GetKind(Joint))
		{
		case EAthleteJointKind::Lumbar:
		case EAthleteJointKind::Thoracic:
		case EAthleteJointKind::Neck:
			// Child segment points up; flexion bends it forward.
			D = { Up, Forward, Lateral, Forward, Medial };
			break;
		case EAthleteJointKind::Knee:
			// Knee flexion swings the shank BACKWARD (heel toward buttock).
			D = { -Up, -Forward, Lateral, Forward, Medial };
			break;
		case EAthleteJointKind::Ankle:
			// Foot points forward. Dorsiflexion lifts the toes. Toe-out turns them laterally.
			// Inversion turns the sole (-Z) toward the midline.
			D = { Forward, Up, Lateral, -Up, Medial };
			break;
		default:
			// Shoulder, elbow, wrist, hip: child segment hangs down; flexion swings it forward.
			// Internal rotation turns the front of the limb toward the midline (forearm: pronation).
			D = { -Up, Forward, Lateral, Forward, Medial };
			break;
		}
		return D;
	}
}

FQuat AthleteJointSetup::RotateToward(const FVector& From, const FVector& To, double AngleDeg)
{
	if (FMath::IsNearlyZero(AngleDeg))
	{
		return FQuat::Identity;
	}
	// Rotating about (From x To) by a positive angle turns From toward To (right-hand rule).
	return FQuat((From ^ To).GetSafeNormal(), FMath::DegreesToRadians(AngleDeg));
}

FQuat AthleteJointSetup::ComputeDriveTarget(const FAthleteJointSetup& Setup, const FQuat& ChildRelativeToParent)
{
	// GetParentFrame() = NeutralRotation * JointFrame = J * N_local, and the child frame is J.
	return Setup.GetParentFrame().Inverse() * ChildRelativeToParent * Setup.JointFrame;
}


FAthleteJointSetup AthleteJointSetup::Compute(const FAthleteBodyModel& Model, EAthleteJoint Joint, const FAthleteJointRangeOfMotion& Range)
{
	const FJointDirections D = GetDirections(Joint);

	FAthleteJointSetup Setup;
	Setup.Joint = Joint;
	Setup.CenterM = Model.GetJointCenterM(Joint);
	Setup.LongAxis = D.LongAxis;
	Setup.FlexionDirection = D.Flexion;
	Setup.AbductionDirection = D.Abduction;
	Setup.AxialReference = D.AxialReference;
	Setup.InternalRotationDirection = D.InternalRotation;

	// Joint frame: X = long axis, Y = flexion rotation axis, Z = X x Y (right-handed).
	const FVector AxisX = D.LongAxis;
	const FVector AxisY = (D.LongAxis ^ D.Flexion).GetSafeNormal();
	const FVector AxisZ = AxisX ^ AxisY;
	Setup.JointFrame = FMatrix(AxisX, AxisY, AxisZ, FVector::ZeroVector).ToQuat();

	// Center of each range: positive = toward flexion / abduction / internal rotation.
	const double SagittalCenter = 0.5 * (Range.FlexionDeg - Range.ExtensionDeg);
	const double FrontalCenter = 0.5 * (Range.AbductionDeg - Range.AdductionDeg);
	const double AxialCenter = 0.5 * (Range.InternalRotationDeg - Range.ExternalRotationDeg);

	// Build the neutral in the joint's own frame the same way Chaos decomposes rotations: one pure
	// SWING (axis in the joint's YZ plane) times one pure TWIST (about X). Composing two separate
	// swings instead would leak a small twist, putting a joint with LOCKED twist (e.g. the wrist)
	// outside its own limit in the reference pose.
	//
	// Signs, in joint-local axes: +rotation about Y moves the long axis toward the flexion
	// direction (by construction of Y). +rotation about Z moves it toward joint +Y, and +rotation
	// about X turns the axial reference toward (X x reference); flip each to match anatomy.
	const double FrontalSign = FMath::Sign(AxisY | D.Abduction);
	const double TwistSign = FMath::Sign((AxisX ^ D.AxialReference) | D.InternalRotation);
	const FVector SwingVectorDeg(0.0, SagittalCenter, FrontalSign * FrontalCenter);
	const double SwingAngleDeg = SwingVectorDeg.Size();
	const FQuat LocalSwing = SwingAngleDeg > 0.0 ? FQuat(SwingVectorDeg / SwingAngleDeg, FMath::DegreesToRadians(SwingAngleDeg)) : FQuat::Identity;
	const FQuat LocalTwist(FVector::XAxisVector, FMath::DegreesToRadians(TwistSign * AxialCenter));
	const FQuat LocalNeutral = LocalSwing * LocalTwist;

	// Express the neutral in the body frame: Neutral_body = J * N_local * J^-1, so that
	// GetParentFrame() = Neutral_body * J = J * N_local.
	Setup.NeutralRotation = Setup.JointFrame * LocalNeutral * Setup.JointFrame.Inverse();

	Setup.SwingSagittalHalfRangeDeg = 0.5 * (Range.FlexionDeg + Range.ExtensionDeg);
	Setup.SwingFrontalHalfRangeDeg = 0.5 * (Range.AbductionDeg + Range.AdductionDeg);
	Setup.TwistHalfRangeDeg = 0.5 * (Range.InternalRotationDeg + Range.ExternalRotationDeg);
	return Setup;
}
