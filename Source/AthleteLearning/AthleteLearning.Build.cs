// Project ATHLETE

using UnrealBuildTool;

// AthleteLearning: learned motor control (reinforcement learning with Epic's Learning Agents).
// A policy network drives the athlete's muscles: it sets joint targets, and the same muscle model
// as every other controller (strength, force-velocity, reflex stiffness) turns them into torques.
// The physics still decides every outcome. Milestone 4 spike: can the athlete learn to walk?
public class AthleteLearning : ModuleRules
{
	public AthleteLearning(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core",
			"CoreUObject",
			"Engine",
			"AthleteCore",
			"AthleteBody",
			"AthletePhysics",
			"AthleteMotor",
			"LearningAgents",
			"LearningAgentsTraining",
			"LearningTraining", // LearningCore's training layer: the trainer headers include it
		});

		PrivateDependencyModuleNames.AddRange(new string[]
		{
			"Learning",
		});
	}
}
