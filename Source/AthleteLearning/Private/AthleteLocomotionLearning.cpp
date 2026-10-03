// Project ATHLETE

#include "AthleteLocomotionLearning.h"
#include "AthleteLearning.h"
#include "AthleteMotorComponent.h"
#include "Anatomy/AthleteJoints.h"
#include "Articulation/AthletePhysicalBodyComponent.h"
#include "Balance/AthleteBalanceController.h"
#include "Components/ShapeComponent.h"
#include "LearningAgentsActions.h"
#include "LearningAgentsObservations.h"
#include "Units/AthleteUnits.h"

namespace
{
	/** Falling below this fraction of his standing center-of-mass height ends the episode. */
	constexpr double FallenComHeightFraction = 0.7;
	/** Straying this far sideways out of his lane ends the episode (lanes are 3 m apart). */
	constexpr double MaxLaneDeviationM = 1.2;
	/** Share of episodes that ask him to stand still; the rest ask for a walk between these speeds. */
	constexpr double StandStillShare = 0.25;
	constexpr double MinWishedSpeedMps = 0.2;
	constexpr double MaxWishedSpeedMps = 1.2;
	/** Reward terms (per decision): being up, matching the wished velocity, keeping the pelvis upright. */
	constexpr double AliveReward = 1.0;
	constexpr double VelocityReward = 1.0;
	constexpr double VelocityToleranceSquared = 0.1; // exp(-|error|^2 / this): 0.3 m/s off earns ~40%
	constexpr double UprightReward = 0.5;
	/** Actions beyond this (in units of ActionScaleRad) are clipped: the policy's samples are unbounded. */
	constexpr double MaxAction = 1.5;
	/** Muscle activation (multiple of standing tone) for the leg joints the policy drives; 1 elsewhere. */
	constexpr double LegActivation = 2.0;
	/** Spins are observed in these units (rad/s x this) to keep them near unit scale. */
	constexpr double SpinObservationScale = 0.1;

	/** The athlete's own frame: the pelvis's heading, level. */
	FQuat HeadingOf(const UAthletePhysicalBodyComponent& Body)
	{
		const FVector Forward = Body.GetSegmentBody(EAthleteSegment::LowerTrunk)->GetComponentQuat().GetForwardVector();
		return FQuat(FVector::UpVector, FMath::Atan2(Forward.Y, Forward.X));
	}
}

UAthleteLearningAgentComponent* UAthleteLocomotionInteractor::AgentOf(int32 AgentId) const
{
	return Cast<UAthleteLearningAgentComponent>(GetAgent(AgentId, UAthleteLearningAgentComponent::StaticClass()));
}

UAthleteLearningAgentComponent* UAthleteLocomotionEnvironment::AgentOf(int32 AgentId) const
{
	return Cast<UAthleteLearningAgentComponent>(GetAgent(AgentId, UAthleteLearningAgentComponent::StaticClass()));
}

int32 AthleteLocomotionLearning::GetObservationSize()
{
	return ObservationRootValues + AthleteSegments::NumSegments * ObservationValuesPerSegment;
}

int32 AthleteLocomotionLearning::GetActionSize()
{
	return AthleteJoints::NumJoints * ActionValuesPerJoint;
}

// ----- Agent -----

void UAthleteLearningAgentComponent::Setup(UAthletePhysicalBodyComponent* InBody, UAthleteMotorComponent* InMotor, int32 Seed)
{
	Body = InBody;
	Motor = InMotor;
	Random.Initialize(Seed);
	GroundHeightM = Body->GetLowestPointM(); // built standing on the ground
	StandingComHeightM = Body->GetCenterOfMassM().Z - GroundHeightM;
	LaneOriginM = AthleteUnits::UnrealToMeters(Body->GetComponentLocation());
	LaneForward = Body->GetComponentQuat().GetForwardVector().GetSafeNormal2D();
}

void UAthleteLearningAgentComponent::StartEpisode()
{
	Body->ResetToReferencePose();
	Motor->ApplyPosture(FAthletePosture());
	const bool bStandStill = Random.FRand() < StandStillShare;
	WishedVelocityMps = bStandStill ? FVector::ZeroVector : LaneForward * Random.FRandRange(MinWishedSpeedMps, MaxWishedSpeedMps);
	EpisodeReturn = 0.0;
	EpisodeSteps = 0;
	EpisodeSpeedErrorSum = 0.0;
	bEpisodeFell = false;
}

FVector UAthleteLearningAgentComponent::GetComVelocityMps() const
{
	return Body->GetLinearMomentumKgMps() / Body->GetTotalMassKg();
}

bool UAthleteLearningAgentComponent::HasFallen() const
{
	return Body->GetCenterOfMassM().Z - GroundHeightM < FallenComHeightFraction * StandingComHeightM;
}

bool UAthleteLearningAgentComponent::HasLeftLane() const
{
	const FVector Offset = Body->GetCenterOfMassM() - LaneOriginM;
	const FVector Sideways = FVector::CrossProduct(FVector::UpVector, LaneForward);
	return FMath::Abs(FVector::DotProduct(Offset, Sideways)) > MaxLaneDeviationM;
}

// ----- Interactor: what he senses and what he decides -----

void UAthleteLocomotionInteractor::SpecifyAgentObservation_Implementation(FLearningAgentsObservationSchemaElement& OutObservationSchemaElement, ULearningAgentsObservationSchema* InObservationSchema)
{
	OutObservationSchemaElement = ULearningAgentsObservations::SpecifyContinuousObservation(InObservationSchema, AthleteLocomotionLearning::GetObservationSize());
}

void UAthleteLocomotionInteractor::GatherAgentObservation_Implementation(FLearningAgentsObservationObjectElement& OutObservationObjectElement, ULearningAgentsObservationObject* InObservationObject, const int32 AgentId)
{
	const UAthleteLearningAgentComponent* Agent = AgentOf(AgentId);
	TArray<float> Values;
	Values.Reserve(AthleteLocomotionLearning::GetObservationSize());
	if (Agent)
	{
		// Everything in his own frame (heading, level), so the same situation looks the same whichever
		// way he faces: positions relative to his center of mass, velocities relative to its velocity.
		const UAthletePhysicalBodyComponent& Body = *Agent->Body;
		const FQuat Heading = HeadingOf(Body);
		const FVector ComM = Body.GetCenterOfMassM();
		const FVector ComVelocity = Agent->GetComVelocityMps();
		auto Add = [&Values](const FVector& V) { Values.Add(V.X); Values.Add(V.Y); Values.Add(V.Z); };

		Values.Add(ComM.Z - Agent->GroundHeightM);
		Add(Heading.UnrotateVector(ComVelocity));
		Values.Add(FMath::Min(Body.GetSegmentLowestPointM(EAthleteSegment::FootLeft) - Agent->GroundHeightM, 0.5));
		Values.Add(FMath::Min(Body.GetSegmentLowestPointM(EAthleteSegment::FootRight) - Agent->GroundHeightM, 0.5));
		const FVector Wish = Heading.UnrotateVector(Agent->WishedVelocityMps);
		Values.Add(Wish.X);
		Values.Add(Wish.Y);

		for (int32 Index = 0; Index < AthleteSegments::NumSegments; ++Index)
		{
			const UShapeComponent* Segment = Body.GetSegmentBody(AthleteSegments::FromIndex(Index));
			const FQuat Orientation = Heading.Inverse() * Segment->GetComponentQuat();
			Add(Heading.UnrotateVector(AthleteUnits::UnrealToMeters(Segment->GetBodyInstance()->GetCOMPosition()) - ComM));
			Add(Orientation.GetForwardVector());
			Add(Orientation.GetUpVector());
			Add(Heading.UnrotateVector(AthleteUnits::UnrealToMeters(Segment->GetPhysicsLinearVelocity()) - ComVelocity));
			Add(Heading.UnrotateVector(Segment->GetPhysicsAngularVelocityInRadians()) * SpinObservationScale);
		}
	}
	Values.SetNumZeroed(AthleteLocomotionLearning::GetObservationSize());
	OutObservationObjectElement = ULearningAgentsObservations::MakeContinuousObservation(InObservationObject, Values);
}

void UAthleteLocomotionInteractor::SpecifyAgentAction_Implementation(FLearningAgentsActionSchemaElement& OutActionSchemaElement, ULearningAgentsActionSchema* InActionSchema)
{
	OutActionSchemaElement = ULearningAgentsActions::SpecifyContinuousAction(InActionSchema, AthleteLocomotionLearning::GetActionSize());
}

void UAthleteLocomotionInteractor::PerformAgentAction_Implementation(const ULearningAgentsActionObject* InActionObject, const FLearningAgentsActionObjectElement& InActionObjectElement, const int32 AgentId)
{
	UAthleteLearningAgentComponent* Agent = AgentOf(AgentId);
	TArray<float> Values;
	if (!Agent || !ULearningAgentsActions::GetContinuousAction(Values, InActionObject, InActionObjectElement) || Values.Num() != AthleteLocomotionLearning::GetActionSize())
	{
		return;
	}
	// Each joint's target: a rotation from the reference pose (body frame). The muscles do the rest.
	FAthletePosture Posture;
	for (int32 Joint = 0; Joint < AthleteJoints::NumJoints; ++Joint)
	{
		FVector Turn(Values[3 * Joint], Values[3 * Joint + 1], Values[3 * Joint + 2]);
		Turn = Turn.BoundToCube(MaxAction) * AthleteLocomotionLearning::ActionScaleRad;
		Posture.ChildRelativeToParent[Joint] = FQuat::MakeFromRotationVector(Turn);
		const EAthleteJointKind Kind = AthleteJoints::GetKind(AthleteJoints::FromIndex(Joint));
		const bool bLeg = Kind == EAthleteJointKind::Hip || Kind == EAthleteJointKind::Knee || Kind == EAthleteJointKind::Ankle;
		Posture.StiffnessScale[Joint] = bLeg ? LegActivation : 1.0;
	}
	Agent->Motor->ApplyPosture(Posture);
}

// ----- Environment: reward, episode end, restart -----

void UAthleteLocomotionEnvironment::GatherAgentReward_Implementation(float& OutReward, const int32 AgentId)
{
	OutReward = 0.0f;
	UAthleteLearningAgentComponent* Agent = AgentOf(AgentId);
	if (!Agent || Agent->HasFallen())
	{
		return; // on the ground: nothing
	}
	const FVector Velocity = Agent->GetComVelocityMps();
	const double SpeedError = (FVector(Velocity.X, Velocity.Y, 0.0) - Agent->WishedVelocityMps).Size();
	const double Upright = FMath::Max(Agent->Body->GetSegmentBody(EAthleteSegment::LowerTrunk)->GetComponentQuat().GetUpVector().Z, 0.0);
	const double Reward = AliveReward + VelocityReward * FMath::Exp(-FMath::Square(SpeedError) / VelocityToleranceSquared) + UprightReward * Upright;
	OutReward = static_cast<float>(Reward);
	Agent->EpisodeReturn += Reward;
	Agent->EpisodeSpeedErrorSum += SpeedError;
	++Agent->EpisodeSteps;
}

void UAthleteLocomotionEnvironment::GatherAgentCompletion_Implementation(ELearningAgentsCompletion& OutCompletion, const int32 AgentId)
{
	OutCompletion = ELearningAgentsCompletion::Running;
	UAthleteLearningAgentComponent* Agent = AgentOf(AgentId);
	if (Agent && (Agent->HasFallen() || Agent->HasLeftLane()))
	{
		Agent->bEpisodeFell = Agent->HasFallen();
		OutCompletion = ELearningAgentsCompletion::Termination;
	}
}

void UAthleteLocomotionEnvironment::ResetAgentEpisode_Implementation(const int32 AgentId)
{
	UAthleteLearningAgentComponent* Agent = AgentOf(AgentId);
	if (!Agent)
	{
		return;
	}
	if (Agent->EpisodeSteps > 0)
	{
		FAthleteLearningEpisode Episode;
		Episode.DurationS = Agent->EpisodeSteps / AthleteLocomotionLearning::DecisionHz;
		Episode.Return = Agent->EpisodeReturn;
		Episode.MeanSpeedErrorMps = Agent->EpisodeSpeedErrorSum / Agent->EpisodeSteps;
		Episode.WishedSpeedMps = Agent->WishedVelocityMps.Size();
		Episode.bFell = Agent->bEpisodeFell;
		FinishedEpisodes.Add(Episode);
	}
	Agent->StartEpisode();
}

TArray<FAthleteLearningEpisode> UAthleteLocomotionEnvironment::TakeFinishedEpisodes()
{
	return MoveTemp(FinishedEpisodes);
}
