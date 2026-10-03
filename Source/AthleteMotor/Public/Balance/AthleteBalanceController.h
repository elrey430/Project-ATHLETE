// Project ATHLETE

#pragma once

#include "CoreMinimal.h"
#include "Anatomy/AthleteJoints.h"
#include "Anatomy/AthleteSegments.h"
#include "Containers/StaticArray.h"

struct FAthleteMotorSkill;

/** What the neural controller knows about the body (world frame, meters, as SENSED, so possibly delayed). */
struct FAthleteBalanceSensing
{
	FAthleteBalanceSensing()
	{
		for (FQuat& Rotation : SegmentRotations) { Rotation = FQuat::Identity; }
		for (FVector& Center : JointCentersM) { Center = FVector::ZeroVector; }
		FootSoleHeightM[0] = FootSoleHeightM[1] = 0.0;
	}

	double TimeS = 0.0;
	FVector CenterOfMassM = FVector::ZeroVector;
	FVector CenterOfMassVelocityMps = FVector::ZeroVector;
	/** Middle of the two feet's contact areas, on the ground. */
	FVector SupportCenterM = FVector::ZeroVector;
	/** The athlete's horizontal forward and right directions. */
	FVector ForwardAxis = FVector::XAxisVector;
	FVector RightAxis = FVector::YAxisVector;
	/** Half of the foot's heel-to-toe length: how far the pressure point can move fore/aft. */
	double FootHalfLengthM = 0.13;

	/** Orientation of the body frame (every segment's orientation in the reference pose). */
	FQuat BodyFrameRotation = FQuat::Identity;
	/** Each segment's world orientation (vestibular + proprioceptive sense of where the body parts are). */
	TStaticArray<FQuat, AthleteSegments::NumSegments> SegmentRotations;
	/** Each joint's center in the world (proprioception: where the limbs are). */
	TStaticArray<FVector, AthleteJoints::NumJoints> JointCentersM;
	/** Lowest point of each foot above the ground [left, right] (m): touch and pressure on the soles. */
	double FootSoleHeightM[2];
	/**
	 * The heading the athlete is holding, as a rotation from the reference-pose heading, in
	 * body-frame axes (a pure yaw). Standing and walking hold the body facing this way.
	 */
	FQuat HeadingDeviation = FQuat::Identity;

	/** A segment's orientation relative to its reference-pose orientation, in body-frame axes. */
	FQuat GetSegmentDeviation(EAthleteSegment Segment) const
	{
		return BodyFrameRotation.Inverse() * SegmentRotations[static_cast<int32>(Segment)];
	}
};

/** Posture command, as whole-body angles relative to the reference standing pose. */
struct FAthleteBalanceCommand
{
	/** Whole body leans about the ankles; + = forward (degrees). */
	double LeanForwardDeg = 0.0;
	/** Legs tilt so the pelvis shifts sideways; + = pelvis toward the athlete's right (degrees). */
	double LeanRightDeg = 0.0;
	/** Trunk pitches relative to the legs at the hips; + = forward (hip flexion) (degrees). */
	double HipFlexionDeg = 0.0;

	/** Diagnostic: extrapolated center of mass relative to the support center (forward, right; m). */
	double XcomErrorForwardM = 0.0;
	double XcomErrorRightM = 0.0;
};

/** Desired orientation of each segment IN SPACE, relative to its reference-pose orientation, in body-frame axes (identity = reference). */
struct FAthleteSegmentTargets
{
	FAthleteSegmentTargets() { for (FQuat& Q : Deviation) { Q = FQuat::Identity; } }
	/** The heading these targets face (pure yaw, body-frame axes): see FAthleteBalanceSensing::HeadingDeviation. */
	FQuat Heading = FQuat::Identity;
	TStaticArray<FQuat, AthleteSegments::NumSegments> Deviation;
};

/**
 * What the feet can take. A foot pushes on the ground only between its heel and its toes, so an
 * ankle torque beyond (load on the foot) x (lever from the ankle to the toes or heel) tips the foot
 * instead of turning the body. The athlete knows his own feet and does not ask for more.
 */
struct FAthleteFootSupport
{
	/** Plantarflexion torque that puts all of one foot's load on its toes (N*m). */
	double TippingTorqueToesNm = 0.0;
	/** Dorsiflexion torque that puts all of one foot's load on its heel (N*m). */
	double TippingTorqueHeelNm = 0.0;
	/** Inversion/eversion torque that puts all of one foot's load on one side edge (the nearer edge; N*m). */
	double TippingTorqueSideNm = 0.0;
	/** The ankle muscles' stiffness (N*m/rad): turns a torque budget into an angle budget. 0 = no limit. */
	double AnkleStiffnessNmPerRad = 0.0;
	/** The load each foot carries standing on both (half the body weight, N); the tipping torques are for this load. */
	double LoadPerFootN = 0.0;
};

/** Muscle targets: each joint's child-relative-to-parent rotation (body frame, identity = reference pose). */
struct FAthletePosture
{
	FAthletePosture()
	{
		for (FQuat& Q : ChildRelativeToParent) { Q = FQuat::Identity; }
		for (double& Scale : StiffnessScale) { Scale = 1.0; }
		for (bool& bPlanned : bPlannedMotion) { bPlanned = false; }
		for (int32& Partner : CancelReactionOf) { Partner = INDEX_NONE; }
		for (FVector& Torque : FeedforwardTorqueNm) { Torque = FVector::ZeroVector; }
	}
	TStaticArray<FQuat, AthleteJoints::NumJoints> ChildRelativeToParent;
	/**
	 * Muscle activation per joint, as a multiple of its standing stiffness (damping scales with the
	 * square root, keeping the damping ratio). Moving a limb quickly takes more activation than
	 * holding a posture. Strength and force-velocity limits still apply.
	 */
	TStaticArray<double, AthleteJoints::NumJoints> StiffnessScale;
	/**
	 * True where the target is a planned movement (a swinging leg) rather than a posture to hold:
	 * the joint is then also commanded the target's velocity, so it follows the plan instead of
	 * being dragged behind it by its own damping (see FAthleteMuscleCommand::TargetSpin).
	 */
	TStaticArray<bool, AthleteJoints::NumJoints> bPlannedMotion;
	/** Per joint: index of another joint whose reaction on their shared parent this one takes up (INDEX_NONE = none; see FAthleteMuscleCommand). */
	TStaticArray<int32, AthleteJoints::NumJoints> CancelReactionOf;
	/** Per joint: torque by activation alone (world frame, N*m, on the child; see FAthleteMuscleCommand::FeedforwardTorque). */
	TStaticArray<FVector, AthleteJoints::NumJoints> FeedforwardTorqueNm;
};

namespace AthleteBalanceController
{
	/** Use at most this fraction of a foot's tipping torque, keeping the pressure point off the very edge. */
	inline constexpr double FootSupportMargin = 0.8;
}

namespace AthleteBalanceController
{
	/**
	 * Upright balance: ankle strategy plus hip strategy, driven by the extrapolated center of
	 * mass (XCoM, Hof et al. 2005): xi = CoM + v / omega0, omega0 = sqrt(g / h). A body stays
	 * balanced while xi stays over its base of support.
	 *  - Ankle strategy: lean the whole body AGAINST the XCoM error (lean back if it is ahead).
	 *  - Hip strategy: once the error exceeds what the feet can resist, bend at the hips toward the
	 *    fall (trunk forward when falling forward), which drives the hips and CoM back.
	 */
	ATHLETEMOTOR_API FAthleteBalanceCommand Compute(const FAthleteBalanceSensing& Sensing, const FAthleteMotorSkill& Skill, double GravityMps2);

	/**
	 * Where each body part should be in space for a command: shanks and thighs lean with the body
	 * (and tilt sideways), pelvis, trunk, and head lean with the body plus the hip pitch.
	 */
	ATHLETEMOTOR_API FAthleteSegmentTargets MakeSegmentTargets(const FAthleteBalanceCommand& Command, const FQuat& Heading = FQuat::Identity);

	/**
	 * Turns desired segment orientations into joint targets, using where the body parts actually are
	 * (as perceived). This is what makes posture control stable.
	 *
	 * Holding each joint at a fixed ANGLE is not enough to keep a stack of segments upright: when
	 * every joint gives a little in the same direction, the top of the stack moves by the sum, and
	 * gravity's pull grows faster than any one joint's stiffness resists (the stiffness matrix
	 * must beat the gravity matrix, and a diagonal one sized per joint doesn't). People keep their
	 * trunk and head upright IN SPACE (vestibular and proprioceptive feedback), so each joint's
	 * target is recomputed from the actual orientation of the segment it pushes against:
	 *  - Legs (support from the ground up): ankle, knee, hip turn the shank, thigh, pelvis toward
	 *    their desired orientation relative to the foot, shank, thigh as they actually are.
	 *  - Spine and neck (from the pelvis up): lumbar, thoracic, neck turn the abdomen, thorax, head
	 *    toward their desired orientation relative to the segment below as it actually is.
	 *  - Arms hang from the shoulders and hold the reference joint angles.
	 * Each joint then only has to carry its own share of the load, which tone > 1 does.
	 *
	 * The ankles' correction is limited to what the feet can take (see FAthleteFootSupport): asking
	 * an ankle to turn the shank further than that would only lift a heel or the toes.
	 */
	ATHLETEMOTOR_API FAthletePosture SolveJointTargets(const FAthleteSegmentTargets& Targets, const FAthleteBalanceSensing& Perceived, const FAthleteFootSupport& Feet);
}
