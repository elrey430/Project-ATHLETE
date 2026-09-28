// Project ATHLETE

#pragma once

#include "CoreMinimal.h"
#include "Anatomy/AthleteSegments.h"
#include "AthleteJoints.generated.h"

/**
 * The 15 joints connecting the 16 body segments into a tree rooted at the lower trunk (pelvis).
 * Each joint joins a parent segment (closer to the pelvis) to a child segment.
 */
UENUM(BlueprintType)
enum class EAthleteJoint : uint8
{
	Lumbar,			// lower trunk -> middle trunk, at omphalion level
	Thoracic,		// middle trunk -> upper trunk, at xiphion level
	Neck,			// upper trunk -> head, at cervicale (C7)
	ShoulderLeft,
	ElbowLeft,
	WristLeft,
	ShoulderRight,
	ElbowRight,
	WristRight,
	HipLeft,
	KneeLeft,
	AnkleLeft,
	HipRight,
	KneeRight,
	AnkleRight,

	Count UMETA(Hidden)
};

enum class EAthleteJointKind : uint8
{
	Lumbar,
	Thoracic,
	Neck,
	Shoulder,
	Elbow,
	Wrist,
	Hip,
	Knee,
	Ankle,

	Count
};

namespace AthleteJoints
{
	inline constexpr int32 NumJoints = static_cast<int32>(EAthleteJoint::Count);

	ATHLETEBODY_API EAthleteJointKind GetKind(EAthleteJoint Joint);
	ATHLETEBODY_API EAthleteSegment GetParentSegment(EAthleteJoint Joint);
	ATHLETEBODY_API EAthleteSegment GetChildSegment(EAthleteJoint Joint);
	ATHLETEBODY_API EAthleteBodySide GetSide(EAthleteJoint Joint);
	ATHLETEBODY_API const TCHAR* GetName(EAthleteJoint Joint);

	/** The joint whose child is this segment; false for the root (lower trunk). */
	ATHLETEBODY_API bool FindJointToParent(EAthleteSegment Segment, EAthleteJoint& OutJoint);

	inline EAthleteJoint FromIndex(int32 Index) { return static_cast<EAthleteJoint>(Index); }
	inline int32 ToIndex(EAthleteJoint Joint) { return static_cast<int32>(Joint); }
}
