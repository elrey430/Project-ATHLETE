// Project ATHLETE

#pragma once

#include "CoreMinimal.h"

class FAthleteTrackerBundle;

/** A controller command in the athlete's own frame: forward m/s, sideways m/s (left +), turning rad/s (left +). */
struct FAthleteCommand
{
	double Forward = 0.0;
	double Sideways = 0.0;
	double Turn = 0.0;

	double operator[](int32 I) const { return I == 0 ? Forward : I == 1 ? Sideways : Turn; }
};

/**
 * The controller layer (port of Scripts/MuJoCo/controller.py, CommandShaper): the player's command, rate
 * limited to what an athlete can do (acceleration, deceleration, sideways and turning changes).
 */
class ATHLETETRACKER_API FAthleteCommandShaper
{
public:
	void Configure(double InForwardAcceleration, double InForwardDeceleration, double InSidewaysAcceleration, double InTurnAcceleration);
	void Reset(const FAthleteCommand& InCommand = FAthleteCommand()) { Command = InCommand; }
	const FAthleteCommand& Step(const FAthleteCommand& Wanted, double Dt);
	const FAthleteCommand& Current() const { return Command; }

private:
	FAthleteCommand Command;
	double ForwardAcceleration = 3.5, ForwardDeceleration = 4.0, SidewaysAcceleration = 2.5, TurnAcceleration = 4.0;
};

/**
 * The motion-matching database (port of motion_matching.MotionDatabase): playback quantities and normalized
 * features per frame, as exported. Views into the bundle's arrays (no copies).
 */
class ATHLETETRACKER_API FAthleteMotionDatabase
{
public:
	bool Initialize(const FAthleteTrackerBundle& Bundle, FString& OutError);

	int32 NumFrames = 0, NumJoints = 0, NumFeatures = 0, StartFrame = 0;
	const float* Joints = nullptr;          // (N, J)
	const float* JointVelocity = nullptr;   // (N, J)
	const float* Height = nullptr;          // (N)
	const float* PelvisVelocity = nullptr;  // (N, 3): heading frame xy, world z
	const float* TiltWxyz = nullptr;        // (N, 4): pelvis orientation relative to its heading
	const float* YawRate = nullptr;         // (N)
	const float* AngularVelocity = nullptr; // (N, 3): pelvis frame
	const float* Lowest = nullptr;          // (N): lowest sole point in the mocap frame
	const float* Features = nullptr;        // (N, F)
	const uint8* Valid = nullptr;           // (N)
	const int32* Ranges = nullptr;          // (R, 2): [start, end) of each take
	const int32* RangeOf = nullptr;         // (N)
	const double* FeatureMean = nullptr;    // (F)
	const double* FeatureScale = nullptr;   // (F)
	int32 FuturePosition[2] = {0, 0}, FutureFacing[2] = {0, 0};
	TArray<int32> FutureFrames;

	int32 RangeEnd(int32 Frame) const { return Ranges[2 * RangeOf[Frame] + 1]; }

	/** Squared distance of a frame's features to the query. */
	float Cost(int32 Frame, const float* Query) const;

	/** Best valid frame for the query, skipping Near's take from IgnoreBehind before to IgnoreSurrounding after it (Near < 0: none). */
	int32 Search(const float* Query, int32 Near, int32 IgnoreBehind, int32 IgnoreSurrounding, float& OutCost) const;
};

/**
 * The kinematic character (port of motion_matching.MotionMatcher): plays the database, jumping to the frame
 * that best fits where the controller wants to go, with inertialization blending and a minimum hold after
 * each jump. Produces the reference pose (MuJoCo qpos/qvel) the tracking policy follows.
 */
class ATHLETETRACKER_API FAthleteMotionMatcher
{
public:
	/** LowestSole(qpos) -> lowest sole point of the athlete in that pose (world z), on the contact model. */
	bool Initialize(const FAthleteTrackerBundle& Bundle, TFunction<double(const double*)> InLowestSole, FString& OutError);

	/** Restart at Frame (or the start clip), at the origin facing +x. Writes the reference (nq, nv). */
	void Reset(int32 Frame, double* OutQpos, double* OutQvel);

	/** One 10 ms frame toward the command. Writes the reference (nq, nv). */
	void Step(const FAthleteCommand& Command, double* OutQpos, double* OutQvel);

	const FAthleteMotionDatabase& Database() const { return Db; }
	int32 Frame() const { return CurrentFrame; }
	int32 Jumps() const { return NumJumps; }

	/** The reference's ground position and heading: the test driver's yaw tether turns them. */
	FVector2D Position = FVector2D::ZeroVector;
	double Facing = 0.0;

private:
	struct FPose
	{
		const float* Joints;
		const float* JointVelocity;
		double Height, VerticalVelocity, TiltWxyz[4], Velocity[2], YawRate;
	};
	FPose Pose(int32 Frame) const;
	void Query(const FAthleteCommand& Command, TArray<float>& OutQuery) const;
	void Jump(int32 Frame);
	void Reference(const FPose& P, double* OutQpos, double* OutQvel) const;

	FAthleteMotionDatabase Db;
	TFunction<double(const double*)> LowestSole;
	int32 NumQpos = 0, NumQvel = 0;
	int32 SearchInterval = 10, IgnoreSurrounding = 20, IgnoreBehind = 100, MinJumpInterval = 20;
	double JumpMargin = 2.0, CommandChangeSearch = 0.25, BlendHalflife = 0.1, ControlHalflife = 0.25, Dt = 0.01;

	int32 CurrentFrame = 0, NumJumps = 0, SinceSearch = 0, SinceJump = 0;
	bool bHasSearchedCommand = false;
	FAthleteCommand SearchedCommand;
	TArray<double> Offset, OffsetRate; // joints, height, tilt rotvec (3), velocity xy (2), yaw rate
	double ControlVelocity[2] = {0, 0}, ControlYawRate = 0.0;
	mutable TArray<double> PoseJoints, PoseJointVelocity;
};
