// Project ATHLETE

using UnrealBuildTool;

// AthleteBody: the athlete's physical definition, meaning morphology, segment inertial model,
// and physical capability data. Pure data and math; no actors, no physics simulation yet.
// Milestone 2 builds the articulated physics body from what this module computes.
public class AthleteBody : ModuleRules
{
	public AthleteBody(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core",
			"CoreUObject",
			"Engine",
			"AthleteCore",
		});
	}
}
