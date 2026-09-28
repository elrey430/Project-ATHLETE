// Project ATHLETE

using UnrealBuildTool;

// AthleteCore: simulation foundation shared by every other ATHLETE module.
// Units, deterministic randomness, telemetry, debug drawing, simulation settings.
// It must never depend on gameplay, football, or presentation modules.
public class AthleteCore : ModuleRules
{
	public AthleteCore(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core",
			"CoreUObject",
			"Engine",
			"DeveloperSettings",
		});
	}
}
