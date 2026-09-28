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

	TAutoConsoleVariable<bool> CVarDebugAnatomy(
		TEXT("athlete.Debug.Anatomy"), true,
		TEXT("Draw ATHLETE body segments and segment centers of mass (lab body previews)."));

	TAutoConsoleVariable<bool> CVarDebugBalance(
		TEXT("athlete.Debug.Balance"), false,
		TEXT("Draw ATHLETE balance state: support center, extrapolated center of mass, muscle torques."));

	constexpr int32 SphereSegments = 12;

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
	case EAthleteDebugChannel::Anatomy:      return CVarDebugAnatomy.GetValueOnGameThread();
	case EAthleteDebugChannel::Balance:      return CVarDebugBalance.GetValueOnGameThread();
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

void AthleteDebug::DrawLine(const UWorld* World, EAthleteDebugChannel Channel, const FVector& Start, const FVector& End, const FColor& Color)
{
#if ENABLE_DRAW_DEBUG
	if (World && IsChannelEnabled(Channel))
	{
		DrawDebugLine(World, Start, End, Color, /*bPersistentLines=*/false, /*LifeTime=*/-1.0f, /*DepthPriority=*/0, LineThickness);
	}
#endif
}

void AthleteDebug::DrawCapsuleBetween(const UWorld* World, EAthleteDebugChannel Channel, const FVector& Start, const FVector& End, float Radius, const FColor& Color)
{
#if ENABLE_DRAW_DEBUG
	if (World && IsChannelEnabled(Channel))
	{
		// DrawDebugCapsule takes a center, a half-height measured to the tips (including the
		// hemispherical caps), and a rotation whose Z axis is the capsule axis.
		const FVector Axis = End - Start;
		const float Length = Axis.Size();
		const FQuat Rotation = FRotationMatrix::MakeFromZ(Axis.GetSafeNormal()).ToQuat();
		DrawDebugCapsule(World, (Start + End) * 0.5, Length * 0.5f + Radius, Radius, Rotation, Color, /*bPersistentLines=*/false, /*LifeTime=*/-1.0f);
	}
#endif
}

void AthleteDebug::DrawSphere(const UWorld* World, EAthleteDebugChannel Channel, const FVector& Center, float Radius, const FColor& Color)
{
#if ENABLE_DRAW_DEBUG
	if (World && IsChannelEnabled(Channel))
	{
		DrawDebugSphere(World, Center, Radius, SphereSegments, Color, /*bPersistentLines=*/false, /*LifeTime=*/-1.0f);
	}
#endif
}
