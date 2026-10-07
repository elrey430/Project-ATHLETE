// Project ATHLETE

#include "AthleteMotionMatcher.h"

#include "AthleteTrackerBundle.h"
#include "AthleteTrackerMath.h"

using namespace AthleteTrackerMath;

// ----------------------------------------------------------------------------------------------------------------
// Command shaper

void FAthleteCommandShaper::Configure(double InForwardAcceleration, double InForwardDeceleration, double InSidewaysAcceleration, double InTurnAcceleration)
{
	ForwardAcceleration = InForwardAcceleration;
	ForwardDeceleration = InForwardDeceleration;
	SidewaysAcceleration = InSidewaysAcceleration;
	TurnAcceleration = InTurnAcceleration;
}

const FAthleteCommand& FAthleteCommandShaper::Step(const FAthleteCommand& Wanted, double Dt)
{
	// Forward speed: speeding up (away from zero) is limited by acceleration, slowing down by deceleration.
	const double SignReference = Command.Forward != 0.0 ? Command.Forward : Wanted.Forward;
	const bool bSpeedingUp = FMath::Abs(Wanted.Forward) > FMath::Abs(Command.Forward) && FMath::Sign(Wanted.Forward) == FMath::Sign(SignReference);
	const double Limit = (bSpeedingUp ? ForwardAcceleration : ForwardDeceleration) * Dt;
	Command.Forward += FMath::Clamp(Wanted.Forward - Command.Forward, -Limit, Limit);
	Command.Sideways += FMath::Clamp(Wanted.Sideways - Command.Sideways, -SidewaysAcceleration * Dt, SidewaysAcceleration * Dt);
	Command.Turn += FMath::Clamp(Wanted.Turn - Command.Turn, -TurnAcceleration * Dt, TurnAcceleration * Dt);
	return Command;
}

// ----------------------------------------------------------------------------------------------------------------
// Database

namespace
{
	template <typename T>
	const T* View(const FAthleteTrackerBundle& Bundle, const TCHAR* Name, const TCHAR* Dtype, int64 ExpectedNum, FString& OutError)
	{
		const FAthleteTrackerArray* Array = Bundle.FindTyped(Name, Dtype);
		if (!Array || (ExpectedNum >= 0 && Array->Num() != ExpectedNum))
		{
			OutError = FString::Printf(TEXT("Bundle array %s missing or wrong size"), Name);
			return nullptr;
		}
		return Array->Data<T>();
	}
}

bool FAthleteMotionDatabase::Initialize(const FAthleteTrackerBundle& Bundle, FString& OutError)
{
	const FAthleteTrackerArray* JointsArray = Bundle.FindTyped(TEXT("db_joints"), TEXT("float32"));
	const FAthleteTrackerArray* FeaturesArray = Bundle.FindTyped(TEXT("db_features"), TEXT("float32"));
	if (!JointsArray || !FeaturesArray)
	{
		OutError = TEXT("Bundle has no motion database");
		return false;
	}
	NumFrames = static_cast<int32>(JointsArray->Dim(0));
	NumJoints = static_cast<int32>(JointsArray->Dim(1));
	NumFeatures = static_cast<int32>(FeaturesArray->Dim(1));
	const int64 N = NumFrames;
	Joints = JointsArray->Data<float>();
	Features = FeaturesArray->Data<float>();
	JointVelocity = View<float>(Bundle, TEXT("db_joint_velocity"), TEXT("float32"), N * NumJoints, OutError);
	Height = View<float>(Bundle, TEXT("db_height"), TEXT("float32"), N, OutError);
	PelvisVelocity = View<float>(Bundle, TEXT("db_pelvis_velocity"), TEXT("float32"), N * 3, OutError);
	TiltWxyz = View<float>(Bundle, TEXT("db_tilt_wxyz"), TEXT("float32"), N * 4, OutError);
	YawRate = View<float>(Bundle, TEXT("db_yaw_rate"), TEXT("float32"), N, OutError);
	AngularVelocity = View<float>(Bundle, TEXT("db_angular_velocity"), TEXT("float32"), N * 3, OutError);
	Lowest = View<float>(Bundle, TEXT("db_lowest"), TEXT("float32"), N, OutError);
	Valid = View<uint8>(Bundle, TEXT("db_valid"), TEXT("uint8"), N, OutError);
	Ranges = View<int32>(Bundle, TEXT("db_ranges"), TEXT("int32"), -1, OutError);
	RangeOf = View<int32>(Bundle, TEXT("db_range_of"), TEXT("int32"), N, OutError);
	FeatureMean = View<double>(Bundle, TEXT("db_feature_mean"), TEXT("float64"), NumFeatures, OutError);
	FeatureScale = View<double>(Bundle, TEXT("db_feature_scale"), TEXT("float64"), NumFeatures, OutError);
	if (!JointVelocity || !Height || !PelvisVelocity || !TiltWxyz || !YawRate || !AngularVelocity || !Lowest || !Valid || !Ranges
		|| !RangeOf || !FeatureMean || !FeatureScale)
	{
		return false;
	}

	const FJsonObject& Matcher = *Bundle.Constants().GetObjectField(TEXT("matcher"));
	StartFrame = Matcher.GetIntegerField(TEXT("start_frame"));
	FutureFrames = AthleteTrackerJson::IntArray(Matcher, TEXT("future_frames"));
	const FJsonObject& Slices = *Matcher.GetObjectField(TEXT("slices"));
	const TArray<int32> Position = AthleteTrackerJson::IntArray(Slices, TEXT("future_position"));
	const TArray<int32> Facing = AthleteTrackerJson::IntArray(Slices, TEXT("future_facing"));
	FuturePosition[0] = Position[0];
	FuturePosition[1] = Position[1];
	FutureFacing[0] = Facing[0];
	FutureFacing[1] = Facing[1];
	return true;
}

float FAthleteMotionDatabase::Cost(int32 Frame, const float* Query) const
{
	const float* Row = Features + static_cast<int64>(Frame) * NumFeatures;
	float Sum = 0.0f;
	for (int32 K = 0; K < NumFeatures; ++K)
	{
		const float D = Row[K] - Query[K];
		Sum += D * D;
	}
	return Sum;
}

int32 FAthleteMotionDatabase::Search(const float* Query, int32 Near, int32 IgnoreBehind, int32 IgnoreSurrounding, float& OutCost) const
{
	const int32 NearRange = Near >= 0 ? RangeOf[Near] : -1;
	int32 Best = INDEX_NONE;
	float BestCost = TNumericLimits<float>::Max();
	for (int32 Frame = 0; Frame < NumFrames; ++Frame)
	{
		if (!Valid[Frame] || (RangeOf[Frame] == NearRange && Frame >= Near - IgnoreBehind && Frame <= Near + IgnoreSurrounding))
		{
			continue;
		}
		const float C = Cost(Frame, Query);
		if (C < BestCost)
		{
			BestCost = C;
			Best = Frame;
		}
	}
	OutCost = BestCost;
	return Best;
}

// ----------------------------------------------------------------------------------------------------------------
// Matcher

bool FAthleteMotionMatcher::Initialize(const FAthleteTrackerBundle& Bundle, TFunction<double(const double*)> InLowestSole, FString& OutError)
{
	if (!Db.Initialize(Bundle, OutError))
	{
		return false;
	}
	LowestSole = MoveTemp(InLowestSole);
	const FJsonObject& Matcher = *Bundle.Constants().GetObjectField(TEXT("matcher"));
	SearchInterval = Matcher.GetIntegerField(TEXT("search_interval_frames"));
	IgnoreSurrounding = Matcher.GetIntegerField(TEXT("ignore_surrounding_frames"));
	IgnoreBehind = Matcher.GetIntegerField(TEXT("ignore_behind_frames"));
	MinJumpInterval = Matcher.GetIntegerField(TEXT("min_jump_interval_frames"));
	JumpMargin = Matcher.GetNumberField(TEXT("jump_margin"));
	CommandChangeSearch = Matcher.GetNumberField(TEXT("command_change_search"));
	BlendHalflife = Matcher.GetNumberField(TEXT("blend_halflife_s"));
	ControlHalflife = Matcher.GetNumberField(TEXT("control_halflife_s"));
	Dt = 1.0 / Matcher.GetNumberField(TEXT("frequency"));
	NumQpos = Db.NumJoints + 7;
	NumQvel = Db.NumJoints + 6;
	Offset.SetNumZeroed(Db.NumJoints + 7);
	OffsetRate.SetNumZeroed(Db.NumJoints + 7);
	PoseJoints.SetNumZeroed(Db.NumJoints);
	PoseJointVelocity.SetNumZeroed(Db.NumJoints);
	return true;
}

FAthleteMotionMatcher::FPose FAthleteMotionMatcher::Pose(int32 Frame) const
{
	const int32 N = Db.NumJoints;
	const float* DbJoints = Db.Joints + static_cast<int64>(Frame) * N;
	const float* DbJointVelocity = Db.JointVelocity + static_cast<int64>(Frame) * N;
	for (int32 J = 0; J < N; ++J)
	{
		PoseJoints[J] = DbJoints[J] + Offset[J];
		PoseJointVelocity[J] = DbJointVelocity[J] + OffsetRate[J];
	}
	FPose P;
	P.Joints = nullptr;
	P.JointVelocity = nullptr;
	P.Height = Db.Height[Frame] + Offset[N];
	P.VerticalVelocity = Db.PelvisVelocity[3 * Frame + 2] + OffsetRate[N];
	double OffsetQuat[4], DbTilt[4];
	RotvecToQuat(&Offset[N + 1], OffsetQuat);
	for (int32 K = 0; K < 4; ++K)
	{
		DbTilt[K] = Db.TiltWxyz[4 * Frame + K];
	}
	NormalizeQuat(DbTilt); // stored as float32; SciPy keeps unit quaternions
	MulQuat(OffsetQuat, DbTilt, P.TiltWxyz);
	P.Velocity[0] = Db.PelvisVelocity[3 * Frame] + Offset[N + 4];
	P.Velocity[1] = Db.PelvisVelocity[3 * Frame + 1] + Offset[N + 5];
	P.YawRate = Db.YawRate[Frame] + Offset[N + 6];
	return P;
}

void FAthleteMotionMatcher::Reset(int32 Frame, double* OutQpos, double* OutQvel)
{
	CurrentFrame = Frame >= 0 ? Frame : Db.StartFrame;
	Position = FVector2D::ZeroVector;
	Facing = 0.0;
	for (int32 I = 0; I < Offset.Num(); ++I)
	{
		Offset[I] = 0.0;
		OffsetRate[I] = 0.0;
	}
	SinceSearch = SearchInterval;
	NumJumps = 0;
	bHasSearchedCommand = false;
	SinceJump = MinJumpInterval;
	// The controller's own movement state (heading frame) approaches the command; the desired trajectory
	// starts from it, not from the animation.
	ControlVelocity[0] = Db.PelvisVelocity[3 * CurrentFrame];
	ControlVelocity[1] = Db.PelvisVelocity[3 * CurrentFrame + 1];
	ControlYawRate = Db.YawRate[CurrentFrame];
	Reference(Pose(CurrentFrame), OutQpos, OutQvel);
}

void FAthleteMotionMatcher::Query(const FAthleteCommand& Command, TArray<float>& OutQuery) const
{
	double Velocity[2] = {ControlVelocity[0], ControlVelocity[1]};
	double Rate = ControlYawRate;
	const double Lag = FMath::Exp(-FMath::Loge(2.0) * Dt / ControlHalflife);
	double PositionAhead[2] = {0.0, 0.0};
	double Turned = 0.0;
	TArray<double, TInlineAllocator<16>> FuturePositionValues, FutureFacingValues;
	const int32 MaxAhead = FMath::Max(Db.FutureFrames);
	for (int32 I = 1; I <= MaxAhead; ++I)
	{
		Velocity[0] = Command.Forward + (Velocity[0] - Command.Forward) * Lag;
		Velocity[1] = Command.Sideways + (Velocity[1] - Command.Sideways) * Lag;
		Rate = Command.Turn + (Rate - Command.Turn) * Lag;
		Turned += Rate * Dt;
		double Moved[2];
		Rotate2D(Velocity, Turned, Moved);
		PositionAhead[0] += Moved[0] * Dt;
		PositionAhead[1] += Moved[1] * Dt;
		if (Db.FutureFrames.Contains(I))
		{
			FuturePositionValues.Append({PositionAhead[0], PositionAhead[1]});
			FutureFacingValues.Append({FMath::Cos(Turned), FMath::Sin(Turned)});
		}
	}
	OutQuery.SetNumUninitialized(Db.NumFeatures);
	FMemory::Memcpy(OutQuery.GetData(), Db.Features + static_cast<int64>(CurrentFrame) * Db.NumFeatures, Db.NumFeatures * sizeof(float));
	for (int32 K = 0; K < FuturePositionValues.Num(); ++K)
	{
		const int32 F = Db.FuturePosition[0] + K;
		OutQuery[F] = static_cast<float>((FuturePositionValues[K] - Db.FeatureMean[F]) / Db.FeatureScale[F]);
	}
	for (int32 K = 0; K < FutureFacingValues.Num(); ++K)
	{
		const int32 F = Db.FutureFacing[0] + K;
		OutQuery[F] = static_cast<float>((FutureFacingValues[K] - Db.FeatureMean[F]) / Db.FeatureScale[F]);
	}
}

void FAthleteMotionMatcher::Jump(int32 Frame)
{
	// The current pose's difference from the new frame becomes the decaying offset (inertialization).
	const int32 N = Db.NumJoints;
	const FPose Old = Pose(CurrentFrame);
	TArray<double> X, V;
	X.SetNumZeroed(Offset.Num());
	V.SetNumZeroed(Offset.Num());
	for (int32 J = 0; J < N; ++J)
	{
		X[J] = PoseJoints[J] - Db.Joints[static_cast<int64>(Frame) * N + J];
		V[J] = PoseJointVelocity[J] - Db.JointVelocity[static_cast<int64>(Frame) * N + J];
	}
	X[N] = Old.Height - Db.Height[Frame];
	V[N] = Old.VerticalVelocity - Db.PelvisVelocity[3 * Frame + 2];
	double NewTiltInverse[4], Difference[4];
	for (int32 K = 0; K < 4; ++K)
	{
		NewTiltInverse[K] = (K == 0 ? 1.0 : -1.0) * Db.TiltWxyz[4 * Frame + K];
	}
	NormalizeQuat(NewTiltInverse);
	MulQuat(Old.TiltWxyz, NewTiltInverse, Difference);
	QuatToRotvec(Difference, &X[N + 1]);
	X[N + 4] = Old.Velocity[0] - Db.PelvisVelocity[3 * Frame];
	X[N + 5] = Old.Velocity[1] - Db.PelvisVelocity[3 * Frame + 1];
	X[N + 6] = Old.YawRate - Db.YawRate[Frame];
	Offset = MoveTemp(X);
	OffsetRate = MoveTemp(V);
	CurrentFrame = Frame;
	++NumJumps;
}

void FAthleteMotionMatcher::Step(const FAthleteCommand& Command, double* OutQpos, double* OutQvel)
{
	// Only a real change of intent searches early (a smoothly ramping command would search every frame).
	bool bChanged = !bHasSearchedCommand;
	for (int32 K = 0; K < 3 && !bChanged; ++K)
	{
		bChanged = FMath::Abs(Command[K] - SearchedCommand[K]) > CommandChangeSearch;
	}
	const double Lag = FMath::Exp(-FMath::Loge(2.0) * Dt / ControlHalflife);
	ControlVelocity[0] = Command.Forward + (ControlVelocity[0] - Command.Forward) * Lag;
	ControlVelocity[1] = Command.Sideways + (ControlVelocity[1] - Command.Sideways) * Lag;
	ControlYawRate = Command.Turn + (ControlYawRate - Command.Turn) * Lag;

	int32 Next = CurrentFrame + 1;
	const bool bAtEnd = Next >= Db.RangeEnd(CurrentFrame);
	++SinceSearch;
	++SinceJump;
	const bool bMayJump = SinceJump >= MinJumpInterval;
	if (bAtEnd || ((bChanged || SinceSearch >= SearchInterval) && bMayJump))
	{
		SinceSearch = 0;
		SearchedCommand = Command;
		bHasSearchedCommand = true;
		// Jump only if a frame elsewhere beats playing on; never back into the moment just played.
		TArray<float> QueryValues;
		Query(Command, QueryValues);
		float BestCost = 0.0f;
		const int32 Best = Db.Search(QueryValues.GetData(), bAtEnd ? -1 : Next, IgnoreBehind, IgnoreSurrounding, BestCost);
		if (bAtEnd || !Db.Valid[Next] || BestCost < Db.Cost(Next, QueryValues.GetData()) - JumpMargin)
		{
			Jump(Best);
			Next = Best;
			SinceJump = 0;
		}
	}
	CurrentFrame = Next;

	// Critically damped decay of the offsets (Holden 2020).
	const double Y = 2.0 * FMath::Loge(2.0) / (BlendHalflife + 1e-5);
	const double E = FMath::Exp(-Y * Dt);
	for (int32 I = 0; I < Offset.Num(); ++I)
	{
		const double J1 = OffsetRate[I] + Offset[I] * Y;
		Offset[I] = E * (Offset[I] + J1 * Dt);
		OffsetRate[I] = E * (OffsetRate[I] - J1 * Y * Dt);
	}

	const FPose P = Pose(CurrentFrame);
	Facing += P.YawRate * Dt;
	double Moved[2];
	Rotate2D(P.Velocity, Facing, Moved);
	Position.X += Moved[0] * Dt;
	Position.Y += Moved[1] * Dt;
	Reference(P, OutQpos, OutQvel);
}

void FAthleteMotionMatcher::Reference(const FPose& P, double* OutQpos, double* OutQvel) const
{
	const int32 N = Db.NumJoints;
	OutQpos[0] = Position.X;
	OutQpos[1] = Position.Y;
	OutQpos[2] = P.Height;
	const double Yaw[4] = {FMath::Cos(0.5 * Facing), 0.0, 0.0, FMath::Sin(0.5 * Facing)};
	MulQuat(Yaw, P.TiltWxyz, OutQpos + 3);
	for (int32 J = 0; J < N; ++J)
	{
		OutQpos[7 + J] = PoseJoints[J];
	}
	// Ground: blending leg angles moves the feet through or above the floor. Keep the lowest sole point where
	// the mocap frame has it (on the floor, or in the air while running).
	OutQpos[2] -= LowestSole(OutQpos) - Db.Lowest[CurrentFrame];
	Rotate2D(P.Velocity, Facing, OutQvel);
	OutQvel[2] = P.VerticalVelocity;
	for (int32 K = 0; K < 3; ++K)
	{
		OutQvel[3 + K] = Db.AngularVelocity[3 * CurrentFrame + K];
	}
	for (int32 J = 0; J < N; ++J)
	{
		OutQvel[6 + J] = PoseJointVelocity[J];
	}
}
