// Project ATHLETE

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "LearningAgentsInteractor.h"
#include "LearningAgentsTrainingEnvironment.h"
#include "Math/RandomStream.h"
#include "AthleteLocomotionLearning.generated.h"

class UAthletePhysicalBodyComponent;
class UAthleteMotorComponent;

/**
 * Learned locomotion (Milestone 4 spike).
 *
 * A policy network decides, DecisionHz times a second, a target orientation for every joint. The
 * athlete's muscles (UAthleteMotorComponent: reflex stiffness and damping, strength, force-velocity)
 * turn those targets into torques inside the physics step, exactly as for the hand-built
 * controllers. Nothing is applied to the body from outside: the network only chooses what the
 * muscles aim for. Rewarded for staying up and moving at the velocity he wants.
 *
 * Not yet in the spike (see the Milestone 4 doc): sensing delays, energy cost, turning.
 */
namespace AthleteLocomotionLearning
{
	/** Decisions per second (the muscles hold each decision's targets between decisions). */
	inline constexpr double DecisionHz = 30.0;
	/** Per segment: position, orientation (two axes), velocity, spin. */
	inline constexpr int32 ObservationValuesPerSegment = 15;
	/** Root: CoM height, CoM velocity (3), sole heights (2), wished velocity (2). */
	inline constexpr int32 ObservationRootValues = 8;
	/** A rotation vector per joint (body frame, from the reference pose). */
	inline constexpr int32 ActionValuesPerJoint = 3;
	/** An action of 1 turns a joint's target this far from the reference pose (rad). */
	inline constexpr double ActionScaleRad = 1.0;

	ATHLETELEARNING_API int32 GetObservationSize();
	ATHLETELEARNING_API int32 GetActionSize();
}

/** Outcome of one finished episode, for training telemetry. */
struct FAthleteLearningEpisode
{
	double DurationS = 0.0;
	double Return = 0.0;
	double MeanSpeedErrorMps = 0.0;
	double WishedSpeedMps = 0.0;
	bool bFell = false;
};

/**
 * One learning athlete: the body and muscles the policy drives, and this episode's task.
 * Added to the Learning Agents manager as the agent object.
 */
UCLASS(ClassGroup = (Athlete))
class ATHLETELEARNING_API UAthleteLearningAgentComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UPROPERTY()
	TObjectPtr<UAthletePhysicalBodyComponent> Body;

	UPROPERTY()
	TObjectPtr<UAthleteMotorComponent> Motor;

	/** The ground the body stands on (m) and how high its center of mass stands above it. */
	double GroundHeightM = 0.0;
	double StandingComHeightM = 0.0;

	/** Where his lane runs: the body's spawn point and the direction he's asked to walk (world). */
	FVector LaneOriginM = FVector::ZeroVector;
	FVector LaneForward = FVector::XAxisVector;

	/** This episode's wish: velocity over the ground (world, m/s). */
	FVector WishedVelocityMps = FVector::ZeroVector;

	// Episode telemetry.
	double EpisodeReturn = 0.0;
	int32 EpisodeSteps = 0;
	double EpisodeSpeedErrorSum = 0.0;
	bool bEpisodeFell = false;

	FRandomStream Random;

	/** Call once the body and muscles exist. */
	void Setup(UAthletePhysicalBodyComponent* InBody, UAthleteMotorComponent* InMotor, int32 Seed);

	/** Back to standing in the reference pose, with a new wish. */
	void StartEpisode();

	/** Horizontal CoM velocity (world, m/s). */
	FVector GetComVelocityMps() const;
	bool HasFallen() const;
	bool HasLeftLane() const;
};

/** What the policy sees and does. */
UCLASS()
class ATHLETELEARNING_API UAthleteLocomotionInteractor : public ULearningAgentsInteractor
{
	GENERATED_BODY()

public:
	virtual void SpecifyAgentObservation_Implementation(FLearningAgentsObservationSchemaElement& OutObservationSchemaElement, ULearningAgentsObservationSchema* InObservationSchema) override;
	virtual void GatherAgentObservation_Implementation(FLearningAgentsObservationObjectElement& OutObservationObjectElement, ULearningAgentsObservationObject* InObservationObject, const int32 AgentId) override;
	virtual void SpecifyAgentAction_Implementation(FLearningAgentsActionSchemaElement& OutActionSchemaElement, ULearningAgentsActionSchema* InActionSchema) override;
	virtual void PerformAgentAction_Implementation(const ULearningAgentsActionObject* InActionObject, const FLearningAgentsActionObjectElement& InActionObjectElement, const int32 AgentId) override;

private:
	UAthleteLearningAgentComponent* AgentOf(int32 AgentId) const;
};

/** How well he's doing: the reward, when an episode ends, and how it restarts. */
UCLASS()
class ATHLETELEARNING_API UAthleteLocomotionEnvironment : public ULearningAgentsTrainingEnvironment
{
	GENERATED_BODY()

public:
	virtual void GatherAgentReward_Implementation(float& OutReward, const int32 AgentId) override;
	virtual void GatherAgentCompletion_Implementation(ELearningAgentsCompletion& OutCompletion, const int32 AgentId) override;
	virtual void ResetAgentEpisode_Implementation(const int32 AgentId) override;

	/** Episodes finished since the last call (telemetry). */
	TArray<FAthleteLearningEpisode> TakeFinishedEpisodes();

private:
	UAthleteLearningAgentComponent* AgentOf(int32 AgentId) const;

private:
	TArray<FAthleteLearningEpisode> FinishedEpisodes;
};
