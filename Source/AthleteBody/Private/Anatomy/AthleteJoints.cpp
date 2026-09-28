// Project ATHLETE

#include "Anatomy/AthleteJoints.h"

namespace
{
	struct FJointTopology
	{
		EAthleteJoint Joint;
		EAthleteJointKind Kind;
		EAthleteSegment Parent;
		EAthleteSegment Child;
		const TCHAR* Name;
	};

	using ES = EAthleteSegment;
	using EK = EAthleteJointKind;
	const FJointTopology GTopology[] =
	{
		{ EAthleteJoint::Lumbar,        EK::Lumbar,   ES::LowerTrunk,  ES::MiddleTrunk,   TEXT("Lumbar") },
		{ EAthleteJoint::Thoracic,      EK::Thoracic, ES::MiddleTrunk, ES::UpperTrunk,    TEXT("Thoracic") },
		{ EAthleteJoint::Neck,          EK::Neck,     ES::UpperTrunk,  ES::Head,          TEXT("Neck") },
		{ EAthleteJoint::ShoulderLeft,  EK::Shoulder, ES::UpperTrunk,  ES::UpperArmLeft,  TEXT("ShoulderLeft") },
		{ EAthleteJoint::ElbowLeft,     EK::Elbow,    ES::UpperArmLeft, ES::ForearmLeft,  TEXT("ElbowLeft") },
		{ EAthleteJoint::WristLeft,     EK::Wrist,    ES::ForearmLeft, ES::HandLeft,      TEXT("WristLeft") },
		{ EAthleteJoint::ShoulderRight, EK::Shoulder, ES::UpperTrunk,  ES::UpperArmRight, TEXT("ShoulderRight") },
		{ EAthleteJoint::ElbowRight,    EK::Elbow,    ES::UpperArmRight, ES::ForearmRight, TEXT("ElbowRight") },
		{ EAthleteJoint::WristRight,    EK::Wrist,    ES::ForearmRight, ES::HandRight,    TEXT("WristRight") },
		{ EAthleteJoint::HipLeft,       EK::Hip,      ES::LowerTrunk,  ES::ThighLeft,     TEXT("HipLeft") },
		{ EAthleteJoint::KneeLeft,      EK::Knee,     ES::ThighLeft,   ES::ShankLeft,     TEXT("KneeLeft") },
		{ EAthleteJoint::AnkleLeft,     EK::Ankle,    ES::ShankLeft,   ES::FootLeft,      TEXT("AnkleLeft") },
		{ EAthleteJoint::HipRight,      EK::Hip,      ES::LowerTrunk,  ES::ThighRight,    TEXT("HipRight") },
		{ EAthleteJoint::KneeRight,     EK::Knee,     ES::ThighRight,  ES::ShankRight,    TEXT("KneeRight") },
		{ EAthleteJoint::AnkleRight,    EK::Ankle,    ES::ShankRight,  ES::FootRight,     TEXT("AnkleRight") },
	};
	static_assert(UE_ARRAY_COUNT(GTopology) == AthleteJoints::NumJoints, "Every joint needs a topology row.");

	const FJointTopology& Row(EAthleteJoint Joint)
	{
		const FJointTopology& Entry = GTopology[AthleteJoints::ToIndex(Joint)];
		check(Entry.Joint == Joint);
		return Entry;
	}
}

EAthleteJointKind AthleteJoints::GetKind(EAthleteJoint Joint) { return Row(Joint).Kind; }
EAthleteSegment AthleteJoints::GetParentSegment(EAthleteJoint Joint) { return Row(Joint).Parent; }
EAthleteSegment AthleteJoints::GetChildSegment(EAthleteJoint Joint) { return Row(Joint).Child; }
const TCHAR* AthleteJoints::GetName(EAthleteJoint Joint) { return Row(Joint).Name; }

EAthleteBodySide AthleteJoints::GetSide(EAthleteJoint Joint)
{
	return AthleteSegments::GetSide(Row(Joint).Child);
}

bool AthleteJoints::FindJointToParent(EAthleteSegment Segment, EAthleteJoint& OutJoint)
{
	for (const FJointTopology& Entry : GTopology)
	{
		if (Entry.Child == Segment)
		{
			OutJoint = Entry.Joint;
			return true;
		}
	}
	return false;
}
