// Project ATHLETE

using UnrealBuildTool;

// AthleteMotor: the athlete's motor system. Muscles (joint impedance within strength limits) and
// the neural controllers that command them (balance now; locomotion and contact skills later).
// It moves the body ONLY through joint torques. It never pushes or positions the body directly.
public class AthleteMotor : ModuleRules
{
	public AthleteMotor(ReadOnlyTargetRules Target) : base(Target)
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
			"Chaos", // muscles run inside the Chaos solver (sim callback)
		});

		PrivateDependencyModuleNames.AddRange(new string[]
		{
			"PhysicsCore",
		});
	}
}
