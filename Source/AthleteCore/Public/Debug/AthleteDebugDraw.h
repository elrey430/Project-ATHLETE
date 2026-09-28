// Project ATHLETE

#pragma once

#include "CoreMinimal.h"

class UWorld;

/**
 * Debug visualization channels. Each channel is toggled independently with a console variable
 * (open the console with the ` / ~ key while playing):
 *
 *   athlete.Debug.Kinematics 1      velocity / acceleration arrows
 *   athlete.Debug.CenterOfMass 1    center-of-mass markers
 *   athlete.Debug.Anatomy 1         body segments and segment centers of mass (on by default)
 *   athlete.Debug.Balance 1         balance: support center, extrapolated center of mass (XCoM), muscle torques
 *
 * Add a channel only when a real system needs it (forces, contacts, perception, intent...).
 * All drawing compiles to nothing in Shipping builds.
 */
enum class EAthleteDebugChannel : uint8
{
	Kinematics,
	CenterOfMass,
	Anatomy,
	Balance,

	Count
};

namespace AthleteDebug
{
	/** True if the channel's console variable is non-zero (always false in Shipping). */
	ATHLETECORE_API bool IsChannelEnabled(EAthleteDebugChannel Channel);

	/** Arrow length in Unreal cm drawn per 1 m/s of velocity. Console: athlete.Debug.VelocityArrowScale */
	ATHLETECORE_API float GetVelocityArrowScale();

	/** Draws an arrow for one frame. Positions in Unreal world units (cm). */
	ATHLETECORE_API void DrawArrow(const UWorld* World, EAthleteDebugChannel Channel, const FVector& Start, const FVector& End, const FColor& Color);

	/** Draws a point marker for one frame. Position in Unreal world units (cm). */
	ATHLETECORE_API void DrawPoint(const UWorld* World, EAthleteDebugChannel Channel, const FVector& Location, const FColor& Color);

	/** Draws a line for one frame. Positions in Unreal world units (cm). */
	ATHLETECORE_API void DrawLine(const UWorld* World, EAthleteDebugChannel Channel, const FVector& Start, const FVector& End, const FColor& Color);

	/** Draws a capsule whose axis runs from Start to End, for one frame. Unreal units (cm). */
	ATHLETECORE_API void DrawCapsuleBetween(const UWorld* World, EAthleteDebugChannel Channel, const FVector& Start, const FVector& End, float Radius, const FColor& Color);

	/** Draws a wireframe sphere for one frame. Unreal units (cm). */
	ATHLETECORE_API void DrawSphere(const UWorld* World, EAthleteDebugChannel Channel, const FVector& Center, float Radius, const FColor& Color);
}
