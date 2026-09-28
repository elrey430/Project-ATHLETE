// Project ATHLETE

#pragma once

#include "CoreMinimal.h"
#include "AthleteSegments.generated.h"

/**
 * The rigid segments an ATHLETE body is divided into.
 *
 * The division follows de Leva (1996), so every segment has published inertial parameters.
 * Order is root-first (lower trunk = pelvis region), which is the order Milestone 2's
 * articulated body will be assembled in. Fingers and toes are not separate segments:
 * the hand and foot are single rigid bodies for now.
 *
 * UENUM makes the enum visible to Unreal's reflection system, so it can be used in
 * UPROPERTYs, shown in the editor, and used from Blueprints.
 */
UENUM(BlueprintType)
enum class EAthleteSegment : uint8
{
	LowerTrunk,		// omphalion (navel) to mid-hip joint centers: the pelvis region
	MiddleTrunk,	// xiphion (bottom of sternum) to omphalion: abdomen / lumbar region
	UpperTrunk,		// cervicale (C7) to xiphion: thorax
	Head,			// vertex (top of head) to cervicale: head and neck
	UpperArmLeft,
	ForearmLeft,
	HandLeft,
	UpperArmRight,
	ForearmRight,
	HandRight,
	ThighLeft,
	ShankLeft,
	FootLeft,
	ThighRight,
	ShankRight,
	FootRight,

	Count UMETA(Hidden)
};

/** Segment type, independent of side. Left and right segments of the same kind share reference data. */
enum class EAthleteSegmentKind : uint8
{
	Head,
	UpperTrunk,
	MiddleTrunk,
	LowerTrunk,
	UpperArm,
	Forearm,
	Hand,
	Thigh,
	Shank,
	Foot,

	Count
};

/** Body regions used for authoring mass distribution (e.g. "this athlete carries more mass in his legs"). */
UENUM(BlueprintType)
enum class EAthleteBodyRegion : uint8
{
	HeadNeck,
	Trunk,
	Arms,
	Legs,

	Count UMETA(Hidden)
};

enum class EAthleteBodySide : uint8
{
	Center,
	Left,
	Right,
};

namespace AthleteSegments
{
	inline constexpr int32 NumSegments = static_cast<int32>(EAthleteSegment::Count);

	ATHLETEBODY_API EAthleteSegmentKind GetKind(EAthleteSegment Segment);
	ATHLETEBODY_API EAthleteBodyRegion GetRegion(EAthleteSegment Segment);
	ATHLETEBODY_API EAthleteBodySide GetSide(EAthleteSegment Segment);

	/** The same segment on the other side of the body (center segments return themselves). */
	ATHLETEBODY_API EAthleteSegment GetMirror(EAthleteSegment Segment);

	/** Stable, CSV-safe name, e.g. "ThighLeft". */
	ATHLETEBODY_API const TCHAR* GetName(EAthleteSegment Segment);

	inline EAthleteSegment FromIndex(int32 Index) { return static_cast<EAthleteSegment>(Index); }
	inline int32 ToIndex(EAthleteSegment Segment) { return static_cast<int32>(Segment); }
}
