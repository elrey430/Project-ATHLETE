// Project ATHLETE

using UnrealBuildTool;

// AthleteTests: automated tests. A DeveloperTool module, so it is compiled into the
// editor and development builds but never shipped to players.
public class AthleteTests : ModuleRules
{
	public AthleteTests(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PrivateDependencyModuleNames.AddRange(new string[]
		{
			"Core",
			"CoreUObject",
			"Engine",
			"AthleteCore",
			"AthleteBody",
			"AthletePhysics",
			"AthleteMotor",
			"ProjectAthlete",
			"AthleteLearning",
			"LearningAgents",
			"LearningAgentsTraining",
			"LearningTraining", // LearningCore's training layer: the trainer headers include it
			"XmlParser", // reads back the MuJoCo model export
		});
	}
}
