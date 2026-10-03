// Project ATHLETE

#pragma once

#include "CoreMinimal.h"
#include "Anatomy/AthleteBodyModel.h"
#include "Anatomy/AthleteMobility.h"
#include "Capability/AthleteStrength.h"

/**
 * The athlete, described for MuJoCo: the physics engine being evaluated after Milestone 4 (see
 * Docs/Milestone4_Locomotion.md §8).
 *
 * Writes an MJCF model (MuJoCo's XML format) of the SAME athlete the Unreal simulation builds: de Leva
 * segment masses, centers of mass and inertias; the collision shapes; the anatomical joints with their
 * ranges of motion; and one torque actuator per joint axis, limited to the isometric strength the
 * muscle model uses. The model is generated from the athlete definition, never edited by hand.
 *
 * Differences from the Chaos body, each forced by how the engines work:
 *  - Coordinates: Unreal's body frame is left-handed (X forward, Y right, Z up); MuJoCo's is
 *    right-handed. Positions and directions flip Y (X forward, Y LEFT, Z up), and every joint axis is
 *    rebuilt there from anatomical directions, so +flexion is still flexion.
 *  - Joints: each anatomical joint becomes up to three hinges, applied sagittal, then frontal, then
 *    axial (locked planes are left out), each with its own asymmetric range. Chaos uses one ball joint
 *    with a swing cone and twist about the center of the range. The two agree for motion in one plane
 *    and differ slightly for combined motion.
 *  - Muscles: torque actuators limited to isometric strength. Force-velocity (Hill) and impedance
 *    (stiffness, damping) are the controller's job; their constants travel with the model as custom
 *    data (each actuator's maximum joint speed, the Hill curvature, the eccentric plateau).
 */

/** One hinge of an anatomical joint, in MuJoCo coordinates. */
struct FAthleteMjcfHinge
{
	EAthleteJoint Joint = EAthleteJoint::Count;
	/** "flex" (sagittal: + = flexion; ankle: dorsiflexion), "abd" (frontal: + = abduction / lateral flexion; ankle: toe-out), "rot" (axial: + = internal rotation, pronation; ankle: inversion). */
	const TCHAR* Plane = TEXT("");
	FString Name;
	/** Unit rotation axis in the child body's frame (MuJoCo coordinates; in the reference pose all body frames are aligned with the world). */
	FVector Axis = FVector::ZeroVector;
	double LowerRad = 0.0;
	double UpperRad = 0.0;
	/** Isometric strength of the muscles crossing the joint (N*m). */
	double TorqueLimitNm = 0.0;
	/** Joint speed at which those muscles can no longer produce torque while shortening (rad/s). */
	double MaxVelocityRadPerS = 0.0;
};

struct FAthleteMjcfOptions
{
	FString ModelName = TEXT("athlete");
	/** 500 Hz: MuJoCo's usual humanoid step. The engine spike measures what the athlete needs. */
	double TimestepS = 0.002;
	/** Unreal's default gravity (980 cm/s^2), so both engines simulate the same world. */
	double GravityMps2 = 9.8;
	/** Unreal's default physical material friction; the athlete bodies and the test floor use it. */
	double FrictionCoefficient = 0.7;
	/**
	 * MuJoCo contacts and joint limits are SOFT: stiff springs solved with the dynamics. Its defaults
	 * (0.02 s time constant) let a collapsing body sink 4 cm into the floor and joints go 20-44 degrees
	 * past their anatomical limits, and energy stored that way comes back out as a rebound. Chaos limits
	 * are hard (bone and ligament stops), so these are made as stiff as MuJoCo allows: a time constant
	 * of twice the time step (measured: limits within ~2-4 degrees, the floor within ~5 mm).
	 * 0 = twice TimestepS.
	 */
	double ConstraintTimeConstantS = 0.0;
	bool bIncludeFloor = true;
};

namespace AthleteMjcfExport
{
	/** A position or direction in the body frame (Unreal) expressed in MuJoCo coordinates (Y flipped). */
	inline FVector ToMuJoCo(const FVector& BodyFrame) { return FVector(BodyFrame.X, -BodyFrame.Y, BodyFrame.Z); }

	/** The hinges the anatomical joints become, in joint order, then sagittal / frontal / axial. */
	ATHLETEMOTOR_API TArray<FAthleteMjcfHinge> ComputeHinges(const FAthleteBodyModel& Model, const FAthleteMobilityProfile& Mobility, const FAthleteStrengthProfile& Strength);

	/**
	 * The MJCF document. IgnoredCollisionPairs: non-adjacent segments that must not collide because they
	 * touch in the reference pose (UAthletePhysicalBodyComponent::GetIgnoredCollisionPairs). Jointed
	 * segments never collide in MuJoCo (parent-child filtering).
	 */
	ATHLETEMOTOR_API FString Export(const FAthleteBodyModel& Model, const FAthleteMobilityProfile& Mobility, const FAthleteStrengthProfile& Strength,
		TConstArrayView<TPair<EAthleteSegment, EAthleteSegment>> IgnoredCollisionPairs, const FAthleteMjcfOptions& Options = FAthleteMjcfOptions());
}
