// Project ATHLETE

#pragma once

#include "CoreMinimal.h"
#include "AthleteMotionMatcher.h"
#include "AthleteTrackerPolicy.h"

class FAthleteTrackerBundle;
struct mjModel_;
struct mjData_;
typedef struct mjModel_ mjModel;
typedef struct mjData_ mjData;

/**
 * The learned athlete, one 10 ms control step at a time (port of Scripts/MuJoCo/locomotion_tests.Driver with
 * the heading-invariant observation of athlete_loco/invariant.py):
 *   controller command -> command shaper -> motion matcher (runs `Lead` frames ahead, so the policy sees the
 *   reference's near future) -> observation -> policy -> motor commands -> MuJoCo (5 x 2 ms).
 * The reference's heading is tethered to the athlete's (within 0.35 rad), as in the tests.
 * Units are MuJoCo's: metres, radians, Z up, right-handed.
 */
class ATHLETETRACKER_API FAthleteTrackerSim
{
public:
	FAthleteTrackerSim();
	~FAthleteTrackerSim();
	FAthleteTrackerSim(const FAthleteTrackerSim&) = delete;
	FAthleteTrackerSim& operator=(const FAthleteTrackerSim&) = delete;

	bool Initialize(TSharedPtr<const FAthleteTrackerBundle> InBundle, FString& OutError);

	/** Standing start in the start clip (StartOffsetFrames into it), at the origin facing +x. */
	void Reset(const FAthleteCommand& Command = FAthleteCommand(), int32 StartOffsetFrames = 0);

	/**
	 * One control step toward the command. Returns true once the athlete has fallen.
	 * ForcedAction (tests only): use these policy outputs instead of the policy's.
	 */
	bool Step(const FAthleteCommand& Command, TConstArrayView<double> ForcedAction = TConstArrayView<double>());

	bool HasFallen() const { return bFallen; }
	double ControlDt() const { return ControlDtS; }
	int64 StepCount() const { return Now; }

	const mjModel* Model() const { return M; }
	const mjData* Data() const { return Sim; }
	const FAthleteMotionMatcher& Matcher() const { return Matcher_; }
	const FAthleteCommand& ShapedCommand() const { return Shaper.Current(); }
	TConstArrayView<double> Observation() const { return Obs; }
	TConstArrayView<double> LastAction() const { return Action; }

	/** The reference written for control step Row (qpos, qvel). */
	void ReferenceRow(int64 Row, TArray<double>& OutQpos, TArray<double>& OutQvel) const;

	/** Pelvis (free joint) state. */
	FVector PelvisPositionM() const;
	double HeadingRad() const;
	/** Pelvis velocity along the heading, and to its left. */
	double ForwardSpeedMps() const;
	double SidewaysSpeedMps() const;

	/** A horizontal force on the pelvis (world, N) until changed: pushes for tests and gameplay. */
	void SetPelvisForce(const FVector& ForceN);

private:
	double* RowQpos(int64 Row);
	double* RowQvel(int64 Row);
	const double* RowQpos(int64 Row) const;
	const double* RowQvel(int64 Row) const;
	void TurnQueuedRows(int64 NowRow, double Angle);
	void BuildObservation();
	void SiteQuantities(const mjData* D, double* OutRpos, double* OutRangles, double* OutRvel) const;
	double LowestSole(const double* Qpos);

	TSharedPtr<const FAthleteTrackerBundle> Bundle;
	mjModel* M = nullptr;
	mjData* Sim = nullptr;
	mjData* RefData = nullptr;
	mjData* GroundData = nullptr;

	FAthleteTrackerPolicy Policy;
	FAthleteMotionMatcher Matcher_;
	FAthleteCommandShaper Shaper;

	int32 Nq = 0, Nv = 0, Nu = 0, Substeps = 5, Lead = 30, RingSize = 512, PelvisBody = 1;
	double ControlDtS = 0.01, MaxReferenceYawLead = 0.35, HealthyLow = 0.0, HealthyHigh = 10.0;
	TArray<int32> HingeQposAdr, HingeDofAdr, LookaheadSteps, SiteIds, SiteBodies, SiteRoots, SoleGeoms;
	TArray<double> CtrlMean, CtrlDelta;
	TArray<double> Rows; // RingSize x (nq + nv)
	TArray<double> Obs, Action;
	int64 Now = 0;
	bool bFallen = false;
};
