// Project ATHLETE

using UnrealBuildTool;

// AthletePhysics: turns an athlete's computed body (AthleteBody) into a simulated articulated
// rigid body in Chaos: one rigid body per segment, anatomical joints, collision, and measurement.
// Passive only; the motor system that actively controls this body arrives in Milestone 3.
public class AthletePhysics : ModuleRules
{
	public AthletePhysics(ReadOnlyTargetRules Target) : base(Target)
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

		PrivateDependencyModuleNames.AddRange(new string[]
		{
			"PhysicsCore",
			"Chaos",
		});
	}
}
