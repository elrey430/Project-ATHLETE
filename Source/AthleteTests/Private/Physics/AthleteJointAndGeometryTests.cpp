// Project ATHLETE

#include "AthleteTestFlags.h"
#include "Anatomy/AthleteBodyModel.h"
#include "Anatomy/AthleteMobility.h"
#include "Articulation/AthleteCollisionGeometry.h"
#include "Articulation/AthleteJointSetup.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	FAthleteBodyModel BuildReference()
	{
		FAthleteBodyModel Model;
		FAthleteBodyModel::Build(FAthleteMorphology(), Model);
		return Model;
	}

	FAthleteJointSetup SetupFor(const FAthleteBodyModel& Model, EAthleteJoint Joint)
	{
		return AthleteJointSetup::Compute(Model, Joint, FAthleteMobilityProfile().Get(Joint));
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteJointTopologyTest, "Athlete.Physics.Joints.TopologyIsATree", AthleteTestFlags)

bool FAthleteJointTopologyTest::RunTest(const FString& Parameters)
{
	// Every segment except the root (lower trunk) hangs from exactly one joint, and each joint
	// center lies exactly on the endpoint the two segments share.
	const FAthleteBodyModel Model = BuildReference();
	int32 Problems = 0;
	for (int32 Index = 0; Index < AthleteSegments::NumSegments; ++Index)
	{
		const EAthleteSegment Segment = AthleteSegments::FromIndex(Index);
		EAthleteJoint Joint;
		const bool bHasParent = AthleteJoints::FindJointToParent(Segment, Joint);
		Problems += (bHasParent == (Segment != EAthleteSegment::LowerTrunk)) ? 0 : 1;
	}
	TestEqual(TEXT("Only the lower trunk has no parent joint"), Problems, 0);

	int32 Misplaced = 0;
	for (int32 Index = 0; Index < AthleteJoints::NumJoints; ++Index)
	{
		const EAthleteJoint Joint = AthleteJoints::FromIndex(Index);
		const FVector Center = Model.GetJointCenterM(Joint);
		const FAthleteBodySegment& Parent = Model.GetSegment(AthleteJoints::GetParentSegment(Joint));
		const FAthleteBodySegment& Child = Model.GetSegment(AthleteJoints::GetChildSegment(Joint));
		const EAthleteJointKind Kind = AthleteJoints::GetKind(Joint);
		// Shoulders and hips attach at the sides of the trunk, not at its (midline) endpoints;
		// the ankle sits above the foot, not at the heel or toe.
		const bool bOnParent = Center.Equals(Parent.OriginM, 1e-9) || Center.Equals(Parent.EndM, 1e-9)
			|| Kind == EAthleteJointKind::Shoulder || Kind == EAthleteJointKind::Hip;
		const bool bOnChild = Center.Equals(Child.OriginM, 1e-9) || Center.Equals(Child.EndM, 1e-9) || Kind == EAthleteJointKind::Ankle;
		Misplaced += (bOnParent && bOnChild) ? 0 : 1;
	}
	TestEqual(TEXT("Joint centers sit on the shared segment endpoints"), Misplaced, 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteJointFrameTest, "Athlete.Physics.Joints.FramesPointAnatomically", AthleteTestFlags)

bool FAthleteJointFrameTest::RunTest(const FString& Parameters)
{
	const FAthleteBodyModel Model = BuildReference();
	constexpr double Tolerance = 1e-9;

	// RotateToward really turns From toward To.
	TestTrue(TEXT("RotateToward(X, Y, 90) maps X to Y"),
		AthleteJointSetup::RotateToward(FVector::XAxisVector, FVector::YAxisVector, 90.0).RotateVector(FVector::XAxisVector).Equals(FVector::YAxisVector, 1e-12));

	// Joint frame X axis is the child's long axis.
	const FAthleteJointSetup Hip = SetupFor(Model, EAthleteJoint::HipRight);
	TestTrue(TEXT("Hip twist axis points down the thigh"), Hip.JointFrame.RotateVector(FVector::XAxisVector).Equals(FVector(0, 0, -1), Tolerance));

	// Hip: 120 flexion / 30 extension -> neutral is 45 deg of flexion: thigh tip swings FORWARD.
	// (The neutral also includes 7.5 deg of abduction, which shifts the sagittal projection a
	// fraction of a degree, hence the tolerance.)
	const FVector HipNeutralAxis = Hip.NeutralRotation.RotateVector(Hip.LongAxis);
	TestEqual(TEXT("Hip neutral: 45 deg forward of vertical"), FMath::RadiansToDegrees(FMath::Atan2(HipNeutralAxis.X, -HipNeutralAxis.Z)), 45.0, 0.5);
	TestEqual(TEXT("Hip sagittal half-range"), Hip.SwingSagittalHalfRangeDeg, 75.0, Tolerance);

	// Knee: 135 flexion swings the shank BACKWARD.
	const FAthleteJointSetup Knee = SetupFor(Model, EAthleteJoint::KneeLeft);
	TestTrue(TEXT("Knee neutral swings the shank backward"), Knee.NeutralRotation.RotateVector(Knee.LongAxis).X < -0.5);
	TestEqual(TEXT("Knee does not abduct (locked)"), Knee.SwingFrontalHalfRangeDeg, 0.0, Tolerance);

	// Abduction is mirrored: left hip abducts toward -Y, right toward +Y (45 abd / 30 add -> 7.5 deg center).
	const FAthleteJointSetup LeftHip = SetupFor(Model, EAthleteJoint::HipLeft);
	TestTrue(TEXT("Left hip neutral leans the thigh outward (-Y)"), LeftHip.NeutralRotation.RotateVector(LeftHip.LongAxis).Y < 0.0);
	TestTrue(TEXT("Right hip neutral leans the thigh outward (+Y)"), Hip.NeutralRotation.RotateVector(Hip.LongAxis).Y > 0.0);

	// Ankle: 20 dorsi / 50 plantar -> neutral is 15 deg of plantarflexion: toes point DOWN.
	const FAthleteJointSetup Ankle = SetupFor(Model, EAthleteJoint::AnkleRight);
	TestTrue(TEXT("Ankle twist axis points along the foot"), Ankle.JointFrame.RotateVector(FVector::XAxisVector).Equals(FVector(1, 0, 0), Tolerance));
	TestTrue(TEXT("Ankle neutral points the toes down"), Ankle.NeutralRotation.RotateVector(Ankle.LongAxis).Z < 0.0);

	// Every joint frame is a proper rotation.
	int32 NotOrthonormal = 0;
	for (int32 Index = 0; Index < AthleteJoints::NumJoints; ++Index)
	{
		const FQuat Frame = SetupFor(Model, AthleteJoints::FromIndex(Index)).JointFrame;
		NotOrthonormal += FMath::IsNearlyEqual(Frame.Size(), 1.0, 1e-9) ? 0 : 1;
	}
	TestEqual(TEXT("All joint frames are unit rotations"), NotOrthonormal, 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteCollisionGeometryTest, "Athlete.Physics.Geometry.ShapesMatchSegmentMass", AthleteTestFlags)

bool FAthleteCollisionGeometryTest::RunTest(const FString& Parameters)
{
	// Shapes are sized so that volume x sizing density = segment mass: a heavier athlete is bigger.
	const FAthleteBodyModel Model = BuildReference();
	int32 Mismatches = 0;
	for (int32 Index = 0; Index < AthleteSegments::NumSegments; ++Index)
	{
		const EAthleteSegment Segment = AthleteSegments::FromIndex(Index);
		const FAthleteSegmentCollisionShape Shape = AthleteCollisionGeometry::ComputeShape(Model, Segment);
		// Trunk boxes are sized from an elliptical cross-section (breadth x depth), so their sizing
		// volume is the inscribed elliptical cylinder: pi/4 of the box.
		const bool bTrunk = AthleteSegments::GetRegion(Segment) == EAthleteBodyRegion::Trunk;
		const double SizingVolume = Shape.GetVolumeM3() * (bTrunk ? UE_DOUBLE_PI / 4.0 : 1.0);
		const double ShapeMass = SizingVolume * AthleteCollisionGeometry::ShapeSizingDensityKgPerM3;
		Mismatches += FMath::IsNearlyEqual(ShapeMass, Model.GetSegment(Segment).MassKg, 1e-6 * Model.GetSegment(Segment).MassKg) ? 0 : 1;
	}
	TestEqual(TEXT("Every shape's volume matches its segment's mass"), Mismatches, 0);

	// Feet rest exactly on the floor.
	const FAthleteSegmentCollisionShape Foot = AthleteCollisionGeometry::ComputeShape(Model, EAthleteSegment::FootLeft);
	TestEqual(TEXT("Foot sole at floor level (m)"), Foot.CenterM.Z - Foot.BoxHalfExtentsM.Z, 0.0, 1e-9);

	// Reference-male trunk breadth should be human-scale (chest breadth is roughly 0.3 m).
	const FAthleteSegmentCollisionShape Thorax = AthleteCollisionGeometry::ComputeShape(Model, EAthleteSegment::UpperTrunk);
	const double Breadth = 2.0 * Thorax.BoxHalfExtentsM.Y;
	TestTrue(TEXT("Thorax breadth between 0.25 and 0.36 m"), Breadth > 0.25 && Breadth < 0.36);
	AddInfo(FString::Printf(TEXT("Reference male: thorax %.3f m wide x %.3f m deep; thigh radius %.3f m; head radius %.3f m"),
		Breadth, 2.0 * Thorax.BoxHalfExtentsM.X,
		AthleteCollisionGeometry::ComputeShape(Model, EAthleteSegment::ThighLeft).CapsuleRadiusM,
		AthleteCollisionGeometry::ComputeShape(Model, EAthleteSegment::Head).CapsuleRadiusM));
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
