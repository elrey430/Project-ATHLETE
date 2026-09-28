// Project ATHLETE

using UnrealBuildTool;
using System.Collections.Generic;

// Describes the editor build of the project (what we develop and run tests in).
public class ProjectAthleteEditorTarget : TargetRules
{
	public ProjectAthleteEditorTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Editor;
		DefaultBuildSettings = BuildSettingsVersion.V7;
		IncludeOrderVersion = EngineIncludeOrderVersion.Unreal5_8;
		ExtraModuleNames.Add("ProjectAthlete");
	}
}
