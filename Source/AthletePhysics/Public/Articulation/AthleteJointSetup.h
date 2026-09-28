// Project ATHLETE

#pragma once

#include "CoreMinimal.h"
#include "Anatomy/AthleteJoints.h"
#include "Anatomy/AthleteMobility.h"

class FAthleteBodyModel;

/**
 * Everything needed to build one anatomical joint as a physics constraint, computed in the
 * reference pose (body frame: X forward, Y right, Z up; meters).
 *
 * Joint frame axes (Unreal constraint convention):
 *   X = twist axis    = the child segment's long axis, pointing away from the joint
 *   Y = swing-2 axis  = the side-to-side axis: rotation about it is flexion/extension
 *   Z = swing-1 axis  = the remaining axis: rotation about it is abduction/adduction
 *                       (front-to-back axis for vertical segments; vertical for the foot)
 *
 * Why a "neutral" rotation: Unreal limits are symmetric (+/- half-range about a center), but
 * anatomy is not (hip: 120 deg flexion, 30 deg extension). The parent's constraint frame is
 * therefore rotated to the MIDDLE of each anatomical range. In the reference pose the joint then
 * sits offset from that center (e.g. hip: 45 deg toward extension), and the symmetric +/-75 deg
 * limit covers exactly -30..+120 deg of flexion.
 *
 * Approximation: combining the three plane centers into one rotation (sagittal, then frontal,
 * then axial) and using an elliptical swing cone is exact for single-plane motion and
 * approximate for combined motion. Good enough for anatomical limits; revisit if limits start
 * to shape results (e.g. shoulder in tackling).
 */
struct FAthleteJointSetup
{
	EAthleteJoint Joint = EAthleteJoint::Count;

	/** Joint center (body frame, meters). */
	FVector CenterM = FVector::ZeroVector;

	/** Orientation of the joint frame in the reference pose (child side). */
	FQuat JointFrame = FQuat::Identity;

	/** Rotation (body frame) from the reference pose to the center of the range of motion. */
	FQuat NeutralRotation = FQuat::Identity;

	/** Symmetric half-ranges about the neutral rotation, in degrees. 0 = locked. */
	double SwingSagittalHalfRangeDeg = 0.0; // Unreal Swing2 (about Y)
	double SwingFrontalHalfRangeDeg = 0.0;  // Unreal Swing1 (about Z)
	double TwistHalfRangeDeg = 0.0;         // Unreal Twist (about X)

	/** Anatomical directions used to build the frames (body frame, unit vectors). Exposed for tests and debug drawing. */
	FVector LongAxis = FVector::ZeroVector;      // twist axis
	FVector FlexionDirection = FVector::ZeroVector; // where the long axis tip moves under positive sagittal motion
	FVector AbductionDirection = FVector::ZeroVector; // where the tip moves under positive frontal motion

	/** The parent-side constraint frame orientation: NeutralRotation * JointFrame. */
	FQuat GetParentFrame() const { return NeutralRotation * JointFrame; }
};

namespace AthleteJointSetup
{
	ATHLETEPHYSICS_API FAthleteJointSetup Compute(const FAthleteBodyModel& Model, EAthleteJoint Joint, const FAthleteJointRangeOfMotion& Range);

	/** Rotation that turns From toward To (perpendicular unit vectors) by AngleDeg. */
	ATHLETEPHYSICS_API FQuat RotateToward(const FVector& From, const FVector& To, double AngleDeg);
}
