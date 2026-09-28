// Project ATHLETE

#pragma once

#include "CoreMinimal.h"
#include "Anatomy/AthleteBodyModel.h"
#include "Anatomy/AthleteMobility.h"
#include "Articulation/AthleteCollisionGeometry.h"
#include "Articulation/AthleteJointSetup.h"
#include "Components/SceneComponent.h"
#include "PhysicsEngine/ConstraintInstance.h" // full type needed: owned through TUniquePtr
#include "AthletePhysicalBodyComponent.generated.h"

class UAthleteDefinition;
class UShapeComponent;

/**
 * A joint's current rotation relative to the CENTER of its range of motion, decomposed the way
 * Chaos applies limits: swing (an elliptical cone about the Y and Z axes) then twist (about X).
 */
struct FAthleteJointAngles
{
	double SwingSagittalDeg = 0.0; // swing component about the joint's Y axis (flexion/extension)
	double SwingFrontalDeg = 0.0;  // swing component about the joint's Z axis (abduction/adduction)
	double TwistDeg = 0.0;         // twist about the joint's X axis (long axis)

	/** How far outside the joint's limits this rotation is (0 = within limits). */
	double LimitExcessDeg = 0.0;
};

/**
 * An athlete's physical body in the simulation: 16 Chaos rigid bodies joined by 15 anatomical
 * joints, built from an athlete definition. This is the "articulated body" in the ATHLETE
 * pipeline; the motor system (Milestone 3+) will act on it only through forces and torques.
 *
 * Fidelity rules this component enforces (see Docs/Milestone2_PhysicalHumanoid.md):
 *  - Each body's mass, center of mass, and inertia are set to FAthleteBodyModel's values and
 *    verified after creation. Collision shapes only decide contact.
 *  - Chaos stabilization fudges that silently alter physics are OFF: inertia conditioning
 *    (bodies), mass conditioning and projection (joints). No damping.
 *  - Joints are ball joints with hard anatomical limits from the athlete's mobility profile.
 *
 * The body frame (origin on the floor between the ankles, X forward) is this component's
 * transform at build time. Build it with scale 1.
 *
 * An Actor Component is a reusable piece of behavior attached to an Actor. This one creates the
 * segment bodies as extra components owned by the same actor.
 */
UCLASS(ClassGroup = (Athlete), meta = (BlueprintSpawnableComponent))
class ATHLETEPHYSICS_API UAthletePhysicalBodyComponent : public USceneComponent
{
	GENERATED_BODY()

public:
	UAthletePhysicalBodyComponent();

	/** The athlete whose body to build. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Athlete")
	TObjectPtr<UAthleteDefinition> Athlete;

	/** Build automatically when play begins. Turn off to build manually (e.g. in tests). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Athlete")
	bool bBuildOnBeginPlay = true;

	/**
	 * Chaos solver iterations for this body's segments (0 = project default). More iterations =
	 * joints and contacts closer to converged (more accurate dynamics) at higher CPU cost.
	 * See the solver-iteration study in Docs/Milestone2_PhysicalHumanoid.md.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Athlete|Solver", meta = (ClampMin = "0", ClampMax = "255"))
	int32 PositionIterations = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Athlete|Solver", meta = (ClampMin = "0", ClampMax = "255"))
	int32 VelocityIterations = 0;

	/**
	 * Gyroscopic torque: the term that makes a spinning body with unequal principal inertias
	 * change its angular velocity so its angular MOMENTUM stays constant. Chaos leaves it off by
	 * default; without it, spinning limbs and tumbling bodies drift from correct rigid-body
	 * motion. On by default here because it is physics, not an effect.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Athlete|Solver")
	bool bGyroscopicTorque = true;

	/** Builds the body from Athlete. Returns false (and logs why) on failure. */
	bool BuildBody();

	/** Builds the body from explicit data (no asset needed). */
	bool BuildBodyFrom(const FAthleteBodyModel& InModel, const FAthleteMobilityProfile& InMobility);

	void DestroyBody();
	bool IsBuilt() const { return SegmentBodies.Num() == AthleteSegments::NumSegments; }

	// ---- Access ----
	const FAthleteBodyModel& GetBodyModel() const { return Model; }
	UShapeComponent* GetSegmentBody(EAthleteSegment Segment) const;
	FConstraintInstance* GetJointConstraint(EAthleteJoint Joint) const;
	const FAthleteJointSetup& GetJointSetup(EAthleteJoint Joint) const { return JointSetups[AthleteJoints::ToIndex(Joint)]; }
	const FAthleteSegmentCollisionShape& GetCollisionShape(EAthleteSegment Segment) const { return CollisionShapes[AthleteSegments::ToIndex(Segment)]; }

	/** Non-adjacent segment pairs whose collision is ignored because they overlap in the reference pose. */
	const TArray<TPair<EAthleteSegment, EAthleteSegment>>& GetIgnoredCollisionPairs() const { return IgnoredCollisionPairs; }

	// ---- Measurement (world frame, SI units) ----
	double GetTotalMassKg() const;
	FVector GetCenterOfMassM() const;
	FVector GetLinearMomentumKgMps() const;
	FVector GetAngularMomentumAboutComKgM2ps() const;
	double GetKineticEnergyJ() const;
	/** Largest distance between the two sides of any joint (0 = joints perfectly intact). */
	double GetMaxJointSeparationM() const;
	/** Lowest point of any collision shape, computed from the exact shape geometry (world Z, meters). */
	double GetLowestPointM(EAthleteSegment* OutLowestSegment = nullptr) const;
	/** Current joint rotation relative to its range-of-motion center, and any limit violation. */
	FAthleteJointAngles GetJointAngles(EAthleteJoint Joint) const;
	/** Segment's world inertia tensor about its own center of mass, from the physics engine (kg*m^2). */
	FMatrix GetSegmentWorldInertiaKgM2(EAthleteSegment Segment) const;

	// ---- Forces ----
	/** Applies a linear impulse (N*s) at a world point (meters) on one segment. */
	void AddImpulseAtPoint(EAthleteSegment Segment, const FVector& ImpulseNs, const FVector& WorldPointM);
	void SetGravityEnabled(bool bEnabled);

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
	UShapeComponent* CreateSegmentBody(EAthleteSegment Segment);
	void ApplyMassProperties(UShapeComponent* Body, EAthleteSegment Segment);
	void CreateJoint(EAthleteJoint Joint);
	void IgnoreOverlappingPairs();

	/** One shape component per segment, indexed by EAthleteSegment. UPROPERTY keeps them alive (garbage collection). */
	UPROPERTY(Transient)
	TArray<TObjectPtr<UShapeComponent>> SegmentBodies;

	/** Constraint instances must not move in memory once initialized, hence heap allocation. */
	TArray<TUniquePtr<FConstraintInstance>> Joints;

	TArray<FAthleteJointSetup> JointSetups;
	TArray<FAthleteSegmentCollisionShape> CollisionShapes;
	TArray<TPair<EAthleteSegment, EAthleteSegment>> IgnoredCollisionPairs;
	FAthleteBodyModel Model;
};
