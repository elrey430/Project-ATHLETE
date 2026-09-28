// Project ATHLETE

#pragma once

#include "CoreMinimal.h"

/**
 * A symmetric 3x3 inertia tensor in kg*m^2.
 *
 * Rotational inertia is to rotation what mass is to straight-line motion: the larger it is
 * about an axis, the more torque is needed to start or stop turning about that axis. It is
 * the physical quantity behind "smaller athletes change direction more easily", so ATHLETE
 * computes it rather than assuming it.
 *
 * Stored as tensor components (not "products of inertia"): XY = -sum(m*x*y), etc.
 * Axes are the body frame: X forward, Y right, Z up (Unreal's axes, in meters).
 */
struct ATHLETEBODY_API FAthleteInertiaTensor
{
	double XX = 0.0;
	double YY = 0.0;
	double ZZ = 0.0;
	double XY = 0.0;
	double XZ = 0.0;
	double YZ = 0.0;

	static FAthleteInertiaTensor Diagonal(double InXX, double InYY, double InZZ)
	{
		FAthleteInertiaTensor Tensor;
		Tensor.XX = InXX;
		Tensor.YY = InYY;
		Tensor.ZZ = InZZ;
		return Tensor;
	}

	/**
	 * Parallel-axis (Steiner) term: the extra inertia a mass M contributes about a point when its
	 * own center of mass is displaced by Offset from that point.  I = M * (|d|^2 * Identity - d d^T)
	 */
	static FAthleteInertiaTensor PointMass(double Mass, const FVector& Offset)
	{
		FAthleteInertiaTensor Tensor;
		Tensor.XX = Mass * (Offset.Y * Offset.Y + Offset.Z * Offset.Z);
		Tensor.YY = Mass * (Offset.X * Offset.X + Offset.Z * Offset.Z);
		Tensor.ZZ = Mass * (Offset.X * Offset.X + Offset.Y * Offset.Y);
		Tensor.XY = -Mass * Offset.X * Offset.Y;
		Tensor.XZ = -Mass * Offset.X * Offset.Z;
		Tensor.YZ = -Mass * Offset.Y * Offset.Z;
		return Tensor;
	}

	FAthleteInertiaTensor& operator+=(const FAthleteInertiaTensor& Other)
	{
		XX += Other.XX; YY += Other.YY; ZZ += Other.ZZ;
		XY += Other.XY; XZ += Other.XZ; YZ += Other.YZ;
		return *this;
	}

	FAthleteInertiaTensor operator+(const FAthleteInertiaTensor& Other) const
	{
		FAthleteInertiaTensor Result = *this;
		Result += Other;
		return Result;
	}

	FAthleteInertiaTensor operator*(double Scale) const
	{
		FAthleteInertiaTensor Result;
		Result.XX = XX * Scale; Result.YY = YY * Scale; Result.ZZ = ZZ * Scale;
		Result.XY = XY * Scale; Result.XZ = XZ * Scale; Result.YZ = YZ * Scale;
		return Result;
	}

	/** Moment of inertia about a unit-length axis through the reference point: a^T I a. */
	double AboutAxis(const FVector& UnitAxis) const
	{
		const FVector& A = UnitAxis;
		return XX * A.X * A.X + YY * A.Y * A.Y + ZZ * A.Z * A.Z
			+ 2.0 * (XY * A.X * A.Y + XZ * A.X * A.Z + YZ * A.Y * A.Z);
	}

	bool Equals(const FAthleteInertiaTensor& Other, double Tolerance) const
	{
		return FMath::IsNearlyEqual(XX, Other.XX, Tolerance) && FMath::IsNearlyEqual(YY, Other.YY, Tolerance)
			&& FMath::IsNearlyEqual(ZZ, Other.ZZ, Tolerance) && FMath::IsNearlyEqual(XY, Other.XY, Tolerance)
			&& FMath::IsNearlyEqual(XZ, Other.XZ, Tolerance) && FMath::IsNearlyEqual(YZ, Other.YZ, Tolerance);
	}
};
