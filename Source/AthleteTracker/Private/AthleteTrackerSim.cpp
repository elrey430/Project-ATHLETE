// Project ATHLETE

#include "AthleteTrackerSim.h"

#include "AthleteTrackerBundle.h"
#include "AthleteTrackerMath.h"

THIRD_PARTY_INCLUDES_START
#include <mujoco/mujoco.h>
THIRD_PARTY_INCLUDES_END

using namespace AthleteTrackerMath;

namespace
{
	/** R^T (0, 0, -1): the direction of gravity in the frame of a quaternion (its tilt). */
	void GravityInFrame(const double* Q, double* Out)
	{
		double R[9];
		QuatToMat(Q, R);
		const double Down[3] = {0.0, 0.0, -1.0};
		MulMatTVec(R, Down, Out);
	}
}

FAthleteTrackerSim::FAthleteTrackerSim() = default;

FAthleteTrackerSim::~FAthleteTrackerSim()
{
	if (GroundData) mj_deleteData(GroundData);
	if (RefData) mj_deleteData(RefData);
	if (Sim) mj_deleteData(Sim);
	if (M) mj_deleteModel(M);
}

bool FAthleteTrackerSim::Initialize(TSharedPtr<const FAthleteTrackerBundle> InBundle, FString& OutError)
{
	Bundle = InBundle;
	if (!Bundle.IsValid())
	{
		OutError = TEXT("No bundle");
		return false;
	}
	M = Bundle->LoadModel(OutError);
	if (!M)
	{
		return false;
	}
	Sim = mj_makeData(M);
	RefData = mj_makeData(M);
	GroundData = mj_makeData(M);
	Nq = M->nq;
	Nv = M->nv;
	Nu = M->nu;

	const FJsonObject& C = Bundle->Constants();
	ControlDtS = C.GetNumberField(TEXT("control_dt"));
	Substeps = C.GetIntegerField(TEXT("substeps"));
	Lead = C.GetIntegerField(TEXT("lead"));
	MaxReferenceYawLead = C.GetNumberField(TEXT("max_reference_yaw_lead_rad"));
	HingeQposAdr = AthleteTrackerJson::IntArray(C, TEXT("hinge_qpos_adr"));
	HingeDofAdr = AthleteTrackerJson::IntArray(C, TEXT("hinge_dof_adr"));
	LookaheadSteps = AthleteTrackerJson::IntArray(C, TEXT("lookahead_steps"));
	SiteIds = AthleteTrackerJson::IntArray(C, TEXT("mimic_site_ids"));
	SoleGeoms = AthleteTrackerJson::IntArray(C, TEXT("sole_geom_ids"));
	const TArray<double> Healthy = AthleteTrackerJson::NumberArray(C, TEXT("root_height_healthy_range"));
	HealthyLow = Healthy[0];
	HealthyHigh = Healthy[1];
	const FJsonObject& ShaperConstants = *C.GetObjectField(TEXT("shaper"));
	Shaper.Configure(ShaperConstants.GetNumberField(TEXT("forward_acceleration")), ShaperConstants.GetNumberField(TEXT("forward_deceleration")),
		ShaperConstants.GetNumberField(TEXT("sideways_acceleration")), ShaperConstants.GetNumberField(TEXT("turn_acceleration")));
	for (const int32 Site : SiteIds)
	{
		SiteBodies.Add(M->site_bodyid[Site]);
		SiteRoots.Add(M->body_rootid[M->site_bodyid[Site]]);
	}
	PelvisBody = M->jnt_bodyid[0];

	const FAthleteTrackerArray* Mean = Bundle->FindTyped(TEXT("ctrl_mean"), TEXT("float64"));
	const FAthleteTrackerArray* Delta = Bundle->FindTyped(TEXT("ctrl_delta"), TEXT("float64"));
	if (!Mean || !Delta || Mean->Num() != Nu || Delta->Num() != Nu)
	{
		OutError = TEXT("Bundle control scaling doesn't match the model's motors");
		return false;
	}
	CtrlMean = TArray<double>(Mean->Data<double>(), Nu);
	CtrlDelta = TArray<double>(Delta->Data<double>(), Nu);

	if (!Policy.Initialize(*Bundle, OutError))
	{
		return false;
	}
	if (Policy.ObservationDim() != C.GetIntegerField(TEXT("obs_dim")) || Policy.ActionDim() != Nu)
	{
		OutError = TEXT("Policy sizes don't match the bundle");
		return false;
	}
	if (!Matcher_.Initialize(*Bundle, [this](const double* Qpos) { return LowestSole(Qpos); }, OutError))
	{
		return false;
	}
	if (Matcher_.Database().NumJoints + 7 != Nq)
	{
		OutError = TEXT("Motion database joints don't match the model");
		return false;
	}
	RingSize = FMath::RoundUpToPowerOfTwo(Lead + FMath::Max(LookaheadSteps) + 64);
	Rows.SetNumZeroed(RingSize * (Nq + Nv));
	Obs.SetNumZeroed(Policy.ObservationDim());
	Action.SetNumZeroed(Nu);
	Reset();
	return true;
}

double* FAthleteTrackerSim::RowQpos(int64 Row) { return Rows.GetData() + (Row & (RingSize - 1)) * (Nq + Nv); }
double* FAthleteTrackerSim::RowQvel(int64 Row) { return RowQpos(Row) + Nq; }
const double* FAthleteTrackerSim::RowQpos(int64 Row) const { return Rows.GetData() + (Row & (RingSize - 1)) * (Nq + Nv); }
const double* FAthleteTrackerSim::RowQvel(int64 Row) const { return RowQpos(Row) + Nq; }

void FAthleteTrackerSim::ReferenceRow(int64 Row, TArray<double>& OutQpos, TArray<double>& OutQvel) const
{
	OutQpos = TArray<double>(RowQpos(Row), Nq);
	OutQvel = TArray<double>(RowQvel(Row), Nv);
}

double FAthleteTrackerSim::LowestSole(const double* Qpos)
{
	// athlete_loco.env.lowest_sole_point: the lowest end of the heel and ball-of-foot capsules.
	FMemory::Memcpy(GroundData->qpos, Qpos, Nq * sizeof(double));
	mj_kinematics(M, GroundData);
	double Lowest = TNumericLimits<double>::Max();
	for (const int32 G : SoleGeoms)
	{
		const double AxisZ = GroundData->geom_xmat[9 * G + 8];
		const double Center = GroundData->geom_xpos[3 * G + 2];
		const double HalfLength = M->geom_size[3 * G + 1];
		const double Radius = M->geom_size[3 * G];
		Lowest = FMath::Min(Lowest, FMath::Min(Center + AxisZ * HalfLength, Center - AxisZ * HalfLength) - Radius);
	}
	return Lowest;
}

void FAthleteTrackerSim::Reset(const FAthleteCommand& Command, int32 StartOffsetFrames)
{
	Shaper.Reset();
	Now = 0;
	bFallen = false;
	// The athlete starts in the reference's first pose, and the policy sees Lead frames of its future.
	Matcher_.Reset(Matcher_.Database().StartFrame + StartOffsetFrames, RowQpos(0), RowQvel(0));
	for (int32 Row = 1; Row <= Lead; ++Row)
	{
		Matcher_.Step(Shaper.Step(Command, ControlDtS), RowQpos(Row), RowQvel(Row));
	}
	// As LocoMuJoCo's reset: forward at the model's default pose, then the start state written in WITHOUT
	// another forward. So the first observation's site terms (and the solver's warm start) come from the
	// default pose; the first physics step computes everything for the real pose. Kept for parity: the
	// policy was tested exactly like this.
	// LocoMuJoCo's set_sim_state_from_traj_data also copies the trajectory's body poses (xpos, xquat) and
	// velocities (cvel), but not sites or centres of mass.
	mj_resetData(M, Sim);
	mj_forward(M, Sim);
	FMemory::Memcpy(RefData->qpos, RowQpos(0), Nq * sizeof(double));
	FMemory::Memcpy(RefData->qvel, RowQvel(0), Nv * sizeof(double));
	mj_kinematics(M, RefData);
	mj_comPos(M, RefData);
	mj_comVel(M, RefData);
	FMemory::Memcpy(Sim->xpos, RefData->xpos, 3 * M->nbody * sizeof(double));
	FMemory::Memcpy(Sim->xquat, RefData->xquat, 4 * M->nbody * sizeof(double));
	FMemory::Memcpy(Sim->cvel, RefData->cvel, 6 * M->nbody * sizeof(double));
	FMemory::Memcpy(Sim->qpos, RowQpos(0), Nq * sizeof(double));
	FMemory::Memcpy(Sim->qvel, RowQvel(0), Nv * sizeof(double));
	for (int32 I = 0; I < Action.Num(); ++I)
	{
		Action[I] = 0.0;
	}
	BuildObservation();
}

void FAthleteTrackerSim::TurnQueuedRows(int64 NowRow, double Angle)
{
	// Turns the queued reference (rows after NowRow, and the matcher) by Angle about the current one.
	const double PivotX = RowQpos(NowRow)[0], PivotY = RowQpos(NowRow)[1];
	const double Spin[4] = {FMath::Cos(0.5 * Angle), 0.0, 0.0, FMath::Sin(0.5 * Angle)};
	for (int32 K = 1; K <= Lead; ++K)
	{
		double* Q = RowQpos(NowRow + K);
		double* V = RowQvel(NowRow + K);
		const double Relative[2] = {Q[0] - PivotX, Q[1] - PivotY};
		double Turned[2];
		Rotate2D(Relative, Angle, Turned);
		Q[0] = PivotX + Turned[0];
		Q[1] = PivotY + Turned[1];
		const double Old[4] = {Q[3], Q[4], Q[5], Q[6]};
		MulQuat(Spin, Old, Q + 3);
		const double OldV[2] = {V[0], V[1]};
		Rotate2D(OldV, Angle, V);
	}
	const double Relative[2] = {Matcher_.Position.X - PivotX, Matcher_.Position.Y - PivotY};
	double Turned[2];
	Rotate2D(Relative, Angle, Turned);
	Matcher_.Position = FVector2D(PivotX + Turned[0], PivotY + Turned[1]);
	Matcher_.Facing += Angle;
}

bool FAthleteTrackerSim::Step(const FAthleteCommand& Command, TConstArrayView<double> ForcedAction)
{
	// Keep the reference facing near the athlete (its heading is observed relative to the athlete's).
	if (Lead > 0)
	{
		const double Error = Wrap(Heading(RowQpos(Now) + 3) - Heading(Sim->qpos + 3));
		if (FMath::Abs(Error) > MaxReferenceYawLead)
		{
			TurnQueuedRows(Now, -(Error - FMath::Sign(Error) * MaxReferenceYawLead));
		}
	}
	const FAthleteCommand& Shaped = Shaper.Step(Command, ControlDtS);
	Matcher_.Step(Shaped, RowQpos(Now + 1 + Lead), RowQvel(Now + 1 + Lead));

	if (ForcedAction.Num() == Nu)
	{
		FMemory::Memcpy(Action.GetData(), ForcedAction.GetData(), Nu * sizeof(double));
	}
	else
	{
		Policy.Act(Obs, Action);
	}
	for (int32 I = 0; I < Nu; ++I)
	{
		Sim->ctrl[I] = Action[I] * CtrlDelta[I] + CtrlMean[I];
	}
	for (int32 I = 0; I < Substeps; ++I)
	{
		mj_step(M, Sim);
	}
	++Now;
	BuildObservation();
	bFallen = bFallen || Sim->qpos[2] < HealthyLow || Sim->qpos[2] > HealthyHigh;
	return bFallen;
}

void FAthleteTrackerSim::SiteQuantities(const mjData* D, double* OutRpos, double* OutRangles, double* OutRvel) const
{
	// Relative to the first site (the upper body): positions and velocities in its frame, orientations as
	// rotation vectors of the relative rotation (LocoMuJoCo's calculate_relative_site_quatities, with
	// athlete_loco/invariant.py's frame corrections).
	const int32 NumSites = SiteIds.Num();
	const double* Frame = D->site_xmat + 9 * SiteIds[0];
	double Velocity[64][6];
	check(NumSites <= 64);
	for (int32 I = 0; I < NumSites; ++I)
	{
		const int32 Site = SiteIds[I];
		const double* Cvel = D->cvel + 6 * SiteBodies[I];           // [angular, linear at the root's subtree CoM]
		const double* Com = D->subtree_com + 3 * SiteRoots[I];
		const double* P = D->site_xpos + 3 * Site;
		const double R[3] = {P[0] - Com[0], P[1] - Com[1], P[2] - Com[2]};
		Velocity[I][0] = Cvel[0];
		Velocity[I][1] = Cvel[1];
		Velocity[I][2] = Cvel[2];
		// linear - r x angular
		Velocity[I][3] = Cvel[3] - (R[1] * Cvel[2] - R[2] * Cvel[1]);
		Velocity[I][4] = Cvel[4] - (R[2] * Cvel[0] - R[0] * Cvel[2]);
		Velocity[I][5] = Cvel[5] - (R[0] * Cvel[1] - R[1] * Cvel[0]);
	}
	const double* P0 = D->site_xpos + 3 * SiteIds[0];
	for (int32 I = 1; I < NumSites; ++I)
	{
		const int32 Site = SiteIds[I];
		const double* P = D->site_xpos + 3 * Site;
		const double Delta[3] = {P[0] - P0[0], P[1] - P0[1], P[2] - P0[2]};
		MulMatTVec(Frame, Delta, OutRpos + 3 * (I - 1));

		// Relative rotation Frame^T * R_i as a rotation vector.
		const double* Ri = D->site_xmat + 9 * Site;
		double Relative[9];
		for (int32 Row = 0; Row < 3; ++Row)
		{
			for (int32 Col = 0; Col < 3; ++Col)
			{
				Relative[3 * Row + Col] = Frame[Row] * Ri[Col] + Frame[3 + Row] * Ri[3 + Col] + Frame[6 + Row] * Ri[6 + Col];
			}
		}
		double Q[4];
		mju_mat2Quat(Q, Relative);
		QuatToRotvec(Q, OutRangles + 3 * (I - 1));

		const double DeltaAngular[3] = {Velocity[I][0] - Velocity[0][0], Velocity[I][1] - Velocity[0][1], Velocity[I][2] - Velocity[0][2]};
		const double DeltaLinear[3] = {Velocity[I][3] - Velocity[0][3], Velocity[I][4] - Velocity[0][4], Velocity[I][5] - Velocity[0][5]};
		MulMatTVec(Frame, DeltaAngular, OutRvel + 6 * (I - 1));
		MulMatTVec(Frame, DeltaLinear, OutRvel + 6 * (I - 1) + 3);
	}
}

void FAthleteTrackerSim::BuildObservation()
{
	double* O = Obs.GetData();
	int32 K = 0;
	const double* Q = Sim->qpos;
	const double* V = Sim->qvel;

	// Environment part (invariant_observation_spec): pelvis height, tilt, joint angles, pelvis angular
	// velocity (pelvis frame), joint velocities.
	O[K++] = Q[2];
	GravityInFrame(Q + 3, O + K);
	K += 3;
	for (const int32 Adr : HingeQposAdr) O[K++] = Q[Adr];
	for (int32 I = 3; I < 6; ++I) O[K++] = V[I];
	for (const int32 Adr : HingeDofAdr) O[K++] = V[Adr];

	// Goal (GoalTrackInvariant): the current reference row with its kinematics.
	const double* RefQ = RowQpos(Now);
	const double* RefV = RowQvel(Now);
	FMemory::Memcpy(RefData->qpos, RefQ, Nq * sizeof(double));
	FMemory::Memcpy(RefData->qvel, RefV, Nv * sizeof(double));
	mj_kinematics(M, RefData);
	mj_comPos(M, RefData);
	mj_comVel(M, RefData);

	const int32 NumRelative = SiteIds.Num() - 1;
	SiteQuantities(Sim, O + K, O + K + 3 * NumRelative, O + K + 6 * NumRelative);
	K += 12 * NumRelative;
	for (int32 I = 7; I < Nq; ++I) O[K++] = RefQ[I];
	for (int32 I = 6; I < Nv; ++I) O[K++] = RefV[I];
	O[K++] = RefQ[2];
	GravityInFrame(RefQ + 3, O + K);
	K += 3;
	double RefR[9];
	QuatToMat(RefQ + 3, RefR);
	MulMatTVec(RefR, RefV, O + K);
	K += 3;
	for (int32 I = 3; I < 6; ++I) O[K++] = RefV[I];
	SiteQuantities(RefData, O + K, O + K + 3 * NumRelative, O + K + 6 * NumRelative);
	K += 12 * NumRelative;

	const double YawSim = Heading(Q + 3), YawRef = Heading(RefQ + 3);
	const double Cs = FMath::Cos(YawSim), Sn = FMath::Sin(YawSim);
	const double Dx = RefQ[0] - Q[0], Dy = RefQ[1] - Q[1];
	const double Turn = YawRef - YawSim;
	O[K++] = FMath::Cos(Turn);
	O[K++] = FMath::Sin(Turn);
	O[K++] = Cs * Dx + Sn * Dy;
	O[K++] = -Sn * Dx + Cs * Dy;
	O[K++] = RefQ[2] - Q[2];
	double SimR[9];
	QuatToMat(Q + 3, SimR);
	MulMatTVec(SimR, V, O + K);
	K += 3;

	// Lookahead (lookahead.future_features): future reference frames relative to the current one.
	const double Yaw0 = YawRef;
	const double C0 = FMath::Cos(Yaw0), S0 = FMath::Sin(Yaw0);
	for (const int32 Ahead : LookaheadSteps)
	{
		const double* LaterQ = RowQpos(Now + Ahead);
		const double* LaterV = RowQvel(Now + Ahead);
		const double Yaw = Heading(LaterQ + 3);
		const double Ddx = LaterQ[0] - RefQ[0], Ddy = LaterQ[1] - RefQ[1];
		O[K++] = C0 * Ddx + S0 * Ddy;
		O[K++] = -S0 * Ddx + C0 * Ddy;
		O[K++] = FMath::Cos(Yaw - Yaw0);
		O[K++] = FMath::Sin(Yaw - Yaw0);
		O[K++] = LaterQ[2];
		const double W = LaterQ[3], X = LaterQ[4], Y = LaterQ[5], Z = LaterQ[6];
		O[K++] = -(2 * (X * Z - W * Y));
		O[K++] = -(2 * (Y * Z + W * X));
		O[K++] = -(1 - 2 * (X * X + Y * Y));
		const double C = FMath::Cos(Yaw), S = FMath::Sin(Yaw);
		O[K++] = C * LaterV[0] + S * LaterV[1];
		O[K++] = -S * LaterV[0] + C * LaterV[1];
		O[K++] = LaterV[2];
		for (int32 I = 7; I < Nq; ++I) O[K++] = LaterQ[I];
	}
	check(K == Obs.Num());
}

FVector FAthleteTrackerSim::PelvisPositionM() const
{
	return FVector(Sim->qpos[0], Sim->qpos[1], Sim->qpos[2]);
}

double FAthleteTrackerSim::HeadingRad() const
{
	return Heading(Sim->qpos + 3);
}

double FAthleteTrackerSim::ForwardSpeedMps() const
{
	const double Yaw = HeadingRad();
	return Sim->qvel[0] * FMath::Cos(Yaw) + Sim->qvel[1] * FMath::Sin(Yaw);
}

double FAthleteTrackerSim::SidewaysSpeedMps() const
{
	const double Yaw = HeadingRad();
	return -Sim->qvel[0] * FMath::Sin(Yaw) + Sim->qvel[1] * FMath::Cos(Yaw);
}

void FAthleteTrackerSim::SetPelvisForce(const FVector& ForceN)
{
	double* Wrench = Sim->xfrc_applied + 6 * PelvisBody;
	Wrench[0] = ForceN.X;
	Wrench[1] = ForceN.Y;
	Wrench[2] = ForceN.Z;
	Wrench[3] = Wrench[4] = Wrench[5] = 0.0;
}
