// Project ATHLETE

using UnrealBuildTool;

// ProjectAthlete: the primary game module. For now it only holds the AthleteLab
// scaffolding (game mode, lab instruments). Simulation systems live in their own modules.
public class ProjectAthlete : ModuleRules
{
	public ProjectAthlete(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core",
			"CoreUObject",
			"Engine",
			"AthleteCore",
			"AthleteBody",
		});
	}
}
