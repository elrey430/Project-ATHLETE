// Project ATHLETE

using UnrealBuildTool;

// AthleteTracker: the learned locomotion controller in Unreal (Milestone 4's tracking policy).
// The policy was trained in MuJoCo, so the athlete's body runs in MuJoCo here too (ThirdParty/MuJoCo);
// Unreal provides input, camera and rendering. The pieces are C++ ports of the Python test driver
// (Scripts/MuJoCo: controller.py, motion_matching.py, locomotion_tests.Driver, athlete_loco/invariant.py),
// fed by the bundle Scripts/MuJoCo/export_unreal_bundle.py writes, and checked against its golden session.
public class AthleteTracker : ModuleRules
{
	public AthleteTracker(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core",
			"CoreUObject",
			"Engine",
			"MuJoCo",
		});

		PrivateDependencyModuleNames.AddRange(new string[]
		{
			"Json",
			"Projects",
			"InputCore",
			"EnhancedInput",
		});
	}
}
