// Project ATHLETE
// Milestone 4 spike: can the athlete learn to walk (reinforcement learning, Learning Agents PPO)?
//
// Trains a policy that drives the athlete's muscles (see AthleteLocomotionLearning.h) with many
// athletes in parallel lanes, and reports learning progress and throughput as it goes. Not part of
// the normal suite (the name doesn't contain "Athlete"); run it on purpose, e.g. a 3-minute smoke run:
//   powershell -ExecutionPolicy Bypass -File Scripts\RunTests.ps1 -Filter ProjectLearn.Spike.Walk.Smoke -ShowInfo
// Needs the trainer's Python environment (Scripts\SetupLearningPython.ps1). Progress is also logged
// live (LogAthleteLearning) to Saved\Logs.

#include "AthleteTestFlags.h"
#include "../Physics/AthletePhysicsTestHelpers.h"
#include "AthleteLearning.h"
#include "AthleteLocomotionLearning.h"
#include "Capability/AthleteStrength.h"
#include "LearningAgentsCommunicator.h"
#include "LearningAgentsCritic.h"
#include "LearningAgentsManager.h"
#include "LearningAgentsPPOTrainer.h"
#include "LearningAgentsPolicy.h"
#include "LearningAgentsNeuralNetwork.h"
#include "Misc/Paths.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	constexpr double LaneSpacingM = 3.0;
	constexpr double EpisodeSeconds = 10.0;
	constexpr double ReportEverySeconds = 30.0;

	/** Rolling statistics over the episodes finished in one report window. */
	struct FEpisodeWindow
	{
		int32 Count = 0;
		int32 Falls = 0;
		double DurationSum = 0.0;
		double ReturnSum = 0.0;
		int32 WalkCount = 0;
		double WalkSpeedErrorSum = 0.0;
		int32 StandCount = 0;
		double StandDurationSum = 0.0;

		void Add(const FAthleteLearningEpisode& Episode)
		{
			++Count;
			Falls += Episode.bFell ? 1 : 0;
			DurationSum += Episode.DurationS;
			ReturnSum += Episode.Return;
			if (Episode.WishedSpeedMps > 0.0)
			{
				++WalkCount;
				WalkSpeedErrorSum += Episode.MeanSpeedErrorMps;
			}
			else
			{
				++StandCount;
				StandDurationSum += Episode.DurationS;
			}
		}
	};
}

IMPLEMENT_COMPLEX_AUTOMATION_TEST(FAthleteLearnWalkSpike, "ProjectLearn.Spike.Walk", AthleteTestFlags)

// Command: "<athletes> <minutes> [memory: gru|none]" (default none: the full body state is observed, no sensing delay yet).
void FAthleteLearnWalkSpike::GetTests(TArray<FString>& OutBeautifiedNames, TArray<FString>& OutTestCommands) const
{
	OutBeautifiedNames.Add(TEXT("Smoke")); OutTestCommands.Add(TEXT("16 3"));
	OutBeautifiedNames.Add(TEXT("Smoke32")); OutTestCommands.Add(TEXT("32 3"));
	OutBeautifiedNames.Add(TEXT("Smoke32Gru")); OutTestCommands.Add(TEXT("32 3 gru"));
	OutBeautifiedNames.Add(TEXT("Train60")); OutTestCommands.Add(TEXT("32 60"));
	OutBeautifiedNames.Add(TEXT("Train180")); OutTestCommands.Add(TEXT("32 180"));
}

bool FAthleteLearnWalkSpike::RunTest(const FString& Parameters)
{
	TArray<FString> Parts;
	Parameters.ParseIntoArrayWS(Parts);
	const int32 Count = FCString::Atoi(*Parts[0]);
	const double Minutes = FCString::Atod(*Parts[1]);
	const bool bMemory = Parts.Num() > 2 && Parts[2] == TEXT("gru");

	// The athletes, each in his own lane along +X, centered on the 100 m floor. (Lanes starting at
	// the center ran off its edge past 16 athletes: those fell into space.)
	auto LaneStart = [Count](int32 Index) { return FVector(0.0, LaneSpacingM * (Index - 0.5 * (Count - 1)), 0.0); };
	FAthletePhysicsTestScene Scene;
	const FAthleteMorphology Morphology;
	if (!Scene.Initialize(*this, /*bWithFloor=*/true, Morphology, LaneStart(0)))
	{
		return false;
	}
	const FAthleteStrengthProfile Strength = FAthleteStrengthProfile::MakeGeneralPopulationMaleBaseline(Morphology.AgeYears, Morphology.StatureM, Morphology.BodyMassKg);
	TArray<UAthletePhysicalBodyComponent*> Bodies = { Scene.Body };
	for (int32 Index = 1; Index < Count; ++Index)
	{
		UAthletePhysicalBodyComponent* Body = Scene.AddBody(*this, LaneStart(Index));
		if (!Body)
		{
			return false;
		}
		Bodies.Add(Body);
	}
	TArray<UObject*> Agents;
	for (int32 Index = 0; Index < Bodies.Num(); ++Index)
	{
		// Muscles on, the hand-built balance and gait off: the policy is in charge.
		UAthleteMotorComponent* Motor = FAthletePhysicsTestScene::AddMotorTo(Bodies[Index], Strength, FAthleteMotorSkill(), /*bBalance=*/false);
		UAthleteLearningAgentComponent* Agent = NewObject<UAthleteLearningAgentComponent>(Bodies[Index]->GetOwner(), TEXT("LearningAgent"));
		Agent->RegisterComponent();
		Agent->Setup(Bodies[Index], Motor, 1000 + Index);
		Agents.Add(Agent);
	}

	// Learning Agents: manager, interactor, environment, policy and critic networks, PPO trainer.
	AActor* LearningActor = Scene.GetWorld()->SpawnActor<AActor>();
	ULearningAgentsManager* Manager = NewObject<ULearningAgentsManager>(LearningActor, TEXT("LearningAgentsManager"));
	// Register first, THEN size it: registering fills the free-id list for the current size, and
	// SetMaxAgentNum adds the ids above the previous size. The other order lists ids 1..N-1 twice,
	// and agents got duplicate ids (0 1 1 2 2 ...): one athlete's fall ended another's episode.
	Manager->RegisterComponent();
	Manager->SetMaxAgentNum(Count);

	ULearningAgentsInteractor* Interactor = ULearningAgentsInteractor::MakeInteractor(Manager, UAthleteLocomotionInteractor::StaticClass(), TEXT("Interactor"));
	ULearningAgentsTrainingEnvironment* Environment = ULearningAgentsTrainingEnvironment::MakeTrainingEnvironment(Manager, UAthleteLocomotionEnvironment::StaticClass(), TEXT("Environment"));
	UAthleteLocomotionEnvironment* LocomotionEnvironment = Cast<UAthleteLocomotionEnvironment>(Environment);

	FLearningAgentsPolicySettings PolicySettings;
	PolicySettings.HiddenLayerNum = 3;
	PolicySettings.HiddenLayerSize = 256;
	PolicySettings.MemoryCell = bMemory ? ELearningAgentsMemoryCell::LearningAgentsGRU : ELearningAgentsMemoryCell::NoMemoryCell;
	ULearningAgentsPolicy* Policy = ULearningAgentsPolicy::MakePolicy(Manager, Interactor, ULearningAgentsPolicy::StaticClass(), TEXT("Policy"),
		nullptr, nullptr, nullptr, true, true, true, PolicySettings, 1234);

	FLearningAgentsCriticSettings CriticSettings;
	CriticSettings.HiddenLayerNum = 3;
	CriticSettings.HiddenLayerSize = 256;
	ULearningAgentsCritic* Critic = ULearningAgentsCritic::MakeCritic(Manager, Interactor, Policy, ULearningAgentsCritic::StaticClass(), TEXT("Critic"),
		nullptr, true, CriticSettings, 1234);

	FLearningAgentsTrainerProcessSettings ProcessSettings;
	ProcessSettings.TaskName = TEXT("AthleteWalkSpike");
	ProcessSettings.TrainerFileName = TEXT("learning_core.train_ppo"); // the default is behavior cloning
	const FLearningAgentsCommunicator Communicator = ULearningAgentsCommunicatorLibrary::MakeSharedMemoryTrainingProcess(ProcessSettings);

	FLearningAgentsPPOTrainerSettings TrainerSettings;
	TrainerSettings.MaxEpisodeStepNum = FMath::RoundToInt(EpisodeSeconds * AthleteLocomotionLearning::DecisionHz);
	ULearningAgentsPPOTrainer* Trainer = ULearningAgentsPPOTrainer::MakePPOTrainer(Manager, Interactor, Environment, Policy, Critic, Communicator,
		ULearningAgentsPPOTrainer::StaticClass(), TEXT("PPOTrainer"), TrainerSettings);
	UTEST_NOT_NULL(TEXT("Trainer"), Trainer);

	TArray<int32> AgentIds;
	Manager->AddAgents(AgentIds, Agents);

	FLearningAgentsPPOTrainingSettings TrainingSettings;
	TrainingSettings.Device = ELearningAgentsTrainingDevice::CPU; // AMD GPU: no CUDA
	// We tick the world ourselves at a fixed step; leave the engine's frame settings alone.
	FLearningAgentsTrainingGameSettings GameSettings;
	GameSettings.bUseFixedTimeStep = false;
	GameSettings.bSetMaxPhysicsStepToFixedTimeStep = false;
	GameSettings.bDisableMaxFPS = false;
	GameSettings.bDisableVSync = false;

	// Train: a decision, then the frames it holds for, until time is up.
	const int32 FramesPerDecision = FMath::Max(1, FMath::RoundToInt(1.0 / (AthleteLocomotionLearning::DecisionHz * FAthletePhysicsTestScene::FrameDeltaSeconds)));
	const double StartS = FPlatformTime::Seconds();
	double LastReportS = StartS;
	double SimulateSeconds = 0.0;
	double TrainerSeconds = 0.0; // RunTraining: observing, the policy, rewards, and waiting on the trainer when it updates
	int64 Decisions = 0;
	int64 DecisionsAtLastReport = 0;
	int64 TotalEpisodes = 0;
	UE_LOG(LogAthleteLearning, Display, TEXT("Spike: %d athletes, %.0f min, %d observations, %d actions, %d frames per decision."),
		Count, Minutes, AthleteLocomotionLearning::GetObservationSize(), AthleteLocomotionLearning::GetActionSize(), FramesPerDecision);
	while (FPlatformTime::Seconds() - StartS < Minutes * 60.0)
	{
		const double TrainerStart = FPlatformTime::Seconds();
		Trainer->RunTraining(TrainingSettings, GameSettings, /*bResetAgentsOnBegin=*/true, /*bResetAgentsOnUpdate=*/true);
		TrainerSeconds += FPlatformTime::Seconds() - TrainerStart;
		if (Trainer->HasTrainingFailed() || !Trainer->IsTraining())
		{
			AddError(TEXT("Training failed or stopped (see LogLearning in the log)."));
			break;
		}
		const double SimStart = FPlatformTime::Seconds();
		for (int32 Frame = 0; Frame < FramesPerDecision; ++Frame)
		{
			Scene.TestWorld.TickTestWorld(FAthletePhysicsTestScene::FrameDeltaSeconds);
		}
		SimulateSeconds += FPlatformTime::Seconds() - SimStart;
		++Decisions;

		const double NowS = FPlatformTime::Seconds();
		if (NowS - LastReportS >= ReportEverySeconds)
		{
			FEpisodeWindow Window;
			for (const FAthleteLearningEpisode& Episode : LocomotionEnvironment->TakeFinishedEpisodes())
			{
				Window.Add(Episode);
			}
			TotalEpisodes += Window.Count;
			const double Elapsed = NowS - StartS;
			const double StepsPerSecond = double(Decisions - DecisionsAtLastReport) * Count / (NowS - LastReportS);
			const FString Line = FString::Printf(
				TEXT("LEARN t=%5.1f min | steps %lld (%.0f/s, %.1fx real time, physics %.0f%%, learning %.0f%% of wall) | episodes %d: length %.2f s, return %.1f, fell %.0f%% | walking speed error %.2f m/s | standing held %.2f s"),
				Elapsed / 60.0, Decisions * Count, StepsPerSecond, StepsPerSecond / AthleteLocomotionLearning::DecisionHz, 100.0 * SimulateSeconds / Elapsed, 100.0 * TrainerSeconds / Elapsed,
				Window.Count, Window.Count ? Window.DurationSum / Window.Count : 0.0, Window.Count ? Window.ReturnSum / Window.Count : 0.0,
				Window.Count ? 100.0 * Window.Falls / Window.Count : 0.0,
				Window.WalkCount ? Window.WalkSpeedErrorSum / Window.WalkCount : 0.0, Window.StandCount ? Window.StandDurationSum / Window.StandCount : 0.0);
			UE_LOG(LogAthleteLearning, Display, TEXT("%s"), *Line);
			AddInfo(Line);
			// Keep the latest networks: a run that stops early still leaves what it learned.
			const FString SnapshotDir = FPaths::ProjectSavedDir() / TEXT("Learning") / FString::Printf(TEXT("AthleteWalkSpike_%d_%s"), Count, bMemory ? TEXT("gru") : TEXT("mlp"));
			Policy->GetPolicyNetworkAsset()->SaveNetworkToSnapshot(FFilePath{ SnapshotDir / TEXT("Policy.bin") });
			Critic->GetCriticNetworkAsset()->SaveNetworkToSnapshot(FFilePath{ SnapshotDir / TEXT("Critic.bin") });
			LastReportS = NowS;
			DecisionsAtLastReport = Decisions;
		}
	}
	Trainer->EndTraining();
	AddInfo(FString::Printf(TEXT("SPIKE done: %lld decisions x %d athletes, %lld episodes."), Decisions, Count, TotalEpisodes));
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
