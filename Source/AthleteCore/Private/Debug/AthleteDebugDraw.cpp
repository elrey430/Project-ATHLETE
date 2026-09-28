// Project ATHLETE

#include "Debug/AthleteDebugDraw.h"
#include "DrawDebugHelpers.h"
#include "HAL/IConsoleManager.h"

namespace
{
	// Visual-only tuning; these never feed back into the simulation.
	constexpr float ArrowHeadSize = 15.0f;
	constexpr float LineThickness = 2.0f;
	constexpr float PointSize = 12.0f;

	TAutoConsoleVariable<bool> CVarDebugKinematics(
		TEXT("athlete.Debug.Kinematics"), false,
		TEXT("Draw ATHLETE velocity/acceleration debug arrows."));

	TAutoConsoleVariable<bool> CVarDebugCenterOfMass(
		TEXT("athlete.Debug.CenterOfMass"), false,
		TEXT("Draw ATHLETE center-of-mass markers."));

	TAutoConsoleVariable<float> CVarVelocityArrowScale(
		TEXT("athlete.Debug.VelocityArrowScale"), 10.0f,
		TEXT("Debug arrow length in cm per 1 m/s of velocity."));
}

bool AthleteDebug::IsChannelEnabled(EAthleteDebugChannel Channel)
{
#if ENABLE_DRAW_DEBUG
	switch (Channel)
	{
	case EAthleteDebugChannel::Kinematics:   return CVarDebugKinematics.GetValueOnGameThread();
	case EAthleteDebugChannel::CenterOfMass: return CVarDebugCenterOfMass.GetValueOnGameThread();
	default:                                 return false;
	}
#else
	return false;
#endif
}

float AthleteDebug::GetVelocityArrowScale()
{
	return CVarVelocityArrowScale.GetValueOnGameThread();
}

void AthleteDebug::DrawArrow(const UWorld* World, EAthleteDebugChannel Channel, const FVector& Start, const FVector& End, const FColor& Color)
{
#if ENABLE_DRAW_DEBUG
	if (World && IsChannelEnabled(Channel))
	{
		DrawDebugDirectionalArrow(World, Start, End, ArrowHeadSize, Color, /*bPersistentLines=*/false, /*LifeTime=*/-1.0f, /*DepthPriority=*/0, LineThickness);
	}
#endif
}

void AthleteDebug::DrawPoint(const UWorld* World, EAthleteDebugChannel Channel, const FVector& Location, const FColor& Color)
{
#if ENABLE_DRAW_DEBUG
	if (World && IsChannelEnabled(Channel))
	{
		DrawDebugPoint(World, Location, PointSize, Color, /*bPersistentLines=*/false, /*LifeTime=*/-1.0f, /*DepthPriority=*/0);
	}
#endif
}
