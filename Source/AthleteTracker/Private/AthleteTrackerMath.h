// Project ATHLETE

#pragma once

#include "CoreMinimal.h"

/**
 * Rotation helpers with the Python side's conventions: MuJoCo quaternions (w, x, y, z), row-major 3x3
 * matrices, and SciPy's rotation-vector conversions (angle in [0, pi], same small-angle series).
 */
namespace AthleteTrackerMath
{
	/** Res = A * B (apply B, then A). Res may not alias A or B. */
	inline void MulQuat(const double* A, const double* B, double* Res)
	{
		Res[0] = A[0] * B[0] - A[1] * B[1] - A[2] * B[2] - A[3] * B[3];
		Res[1] = A[0] * B[1] + A[1] * B[0] + A[2] * B[3] - A[3] * B[2];
		Res[2] = A[0] * B[2] - A[1] * B[3] + A[2] * B[0] + A[3] * B[1];
		Res[3] = A[0] * B[3] + A[1] * B[2] - A[2] * B[1] + A[3] * B[0];
	}

	inline void NormalizeQuat(double* Q)
	{
		const double Norm = FMath::Sqrt(Q[0] * Q[0] + Q[1] * Q[1] + Q[2] * Q[2] + Q[3] * Q[3]);
		if (Norm > 0.0)
		{
			for (int32 K = 0; K < 4; ++K)
			{
				Q[K] /= Norm;
			}
		}
	}

	/** SciPy Rotation.from_rotvec. */
	inline void RotvecToQuat(const double* V, double* Q)
	{
		const double Angle = FMath::Sqrt(V[0] * V[0] + V[1] * V[1] + V[2] * V[2]);
		const double Scale = Angle <= 1e-3 ? 0.5 - Angle * Angle / 48.0 + Angle * Angle * Angle * Angle / 3840.0
			: FMath::Sin(0.5 * Angle) / Angle;
		Q[0] = FMath::Cos(0.5 * Angle);
		Q[1] = V[0] * Scale;
		Q[2] = V[1] * Scale;
		Q[3] = V[2] * Scale;
	}

	/** SciPy Rotation.as_rotvec (normalizes first, as SciPy stores unit quaternions). */
	inline void QuatToRotvec(const double* InQ, double* V)
	{
		double Q[4] = {InQ[0], InQ[1], InQ[2], InQ[3]};
		NormalizeQuat(Q);
		if (Q[0] < 0.0)
		{
			for (int32 K = 0; K < 4; ++K)
			{
				Q[K] = -Q[K];
			}
		}
		const double Angle = 2.0 * FMath::Atan2(FMath::Sqrt(Q[1] * Q[1] + Q[2] * Q[2] + Q[3] * Q[3]), Q[0]);
		const double Scale = Angle <= 1e-3 ? 2.0 + Angle * Angle / 12.0 + 7.0 * Angle * Angle * Angle * Angle / 2880.0
			: Angle / FMath::Sin(0.5 * Angle);
		V[0] = Q[1] * Scale;
		V[1] = Q[2] * Scale;
		V[2] = Q[3] * Scale;
	}

	/** Row-major rotation matrix of a quaternion (as MuJoCo's xmat). */
	inline void QuatToMat(const double* Q, double* R)
	{
		const double W = Q[0], X = Q[1], Y = Q[2], Z = Q[3];
		R[0] = 1 - 2 * (Y * Y + Z * Z); R[1] = 2 * (X * Y - W * Z);     R[2] = 2 * (X * Z + W * Y);
		R[3] = 2 * (X * Y + W * Z);     R[4] = 1 - 2 * (X * X + Z * Z); R[5] = 2 * (Y * Z - W * X);
		R[6] = 2 * (X * Z - W * Y);     R[7] = 2 * (Y * Z + W * X);     R[8] = 1 - 2 * (X * X + Y * Y);
	}

	/** Res = R^T V (world vector into the frame R). */
	inline void MulMatTVec(const double* R, const double* V, double* Res)
	{
		Res[0] = R[0] * V[0] + R[3] * V[1] + R[6] * V[2];
		Res[1] = R[1] * V[0] + R[4] * V[1] + R[7] * V[2];
		Res[2] = R[2] * V[0] + R[5] * V[1] + R[8] * V[2];
	}

	/** Yaw of the pelvis's forward (x) axis. */
	inline double Heading(const double* Q)
	{
		return FMath::Atan2(2 * (Q[1] * Q[2] + Q[0] * Q[3]), 1 - 2 * (Q[2] * Q[2] + Q[3] * Q[3]));
	}

	/** Angle in [-pi, pi) (as (a + pi) % 2pi - pi in Python). */
	inline double Wrap(double Angle)
	{
		double Shifted = FMath::Fmod(Angle + UE_DOUBLE_PI, 2.0 * UE_DOUBLE_PI);
		if (Shifted < 0.0)
		{
			Shifted += 2.0 * UE_DOUBLE_PI;
		}
		return Shifted - UE_DOUBLE_PI;
	}

	inline void Rotate2D(const double* V, double Angle, double* Res)
	{
		const double C = FMath::Cos(Angle), S = FMath::Sin(Angle);
		const double X = V[0], Y = V[1];
		Res[0] = C * X - S * Y;
		Res[1] = S * X + C * Y;
	}
}
