// Project ATHLETE

using UnrealBuildTool;
using System.Collections.Generic;

// Describes the standalone game executable (what players would eventually run).
public class ProjectAthleteTarget : TargetRules
{
	public ProjectAthleteTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Game;
		DefaultBuildSettings = BuildSettingsVersion.V7;
		IncludeOrderVersion = EngineIncludeOrderVersion.Unreal5_8;
		ExtraModuleNames.Add("ProjectAthlete");
	}
}
