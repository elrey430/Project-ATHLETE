// Project ATHLETE

using System.IO;
using UnrealBuildTool;

// MuJoCo (Apache-2.0, Google DeepMind): the physics engine the learned athlete was trained in. Version 3.5.0,
// the exact build the tracking policy was trained and tested with. The files come from the LocoMuJoCo Python
// environment: run Scripts/SetupMuJoCoUnreal.ps1 once (headers, DLL, licenses, and a generated import library).
public class MuJoCo : ModuleRules
{
	public MuJoCo(ReadOnlyTargetRules Target) : base(Target)
	{
		Type = ModuleType.External;

		if (Target.Platform != UnrealTargetPlatform.Win64)
		{
			throw new BuildException("MuJoCo is only set up for Win64 (Scripts/SetupMuJoCoUnreal.ps1).");
		}
		string Dll = Path.Combine(ModuleDirectory, "bin", "Win64", "mujoco.dll");
		string ImportLib = Path.Combine(ModuleDirectory, "lib", "Win64", "mujoco.lib");
		if (!File.Exists(Dll) || !File.Exists(ImportLib))
		{
			throw new BuildException("MuJoCo is not set up for Unreal: run Scripts/SetupMuJoCoUnreal.ps1.");
		}

		PublicSystemIncludePaths.Add(Path.Combine(ModuleDirectory, "include"));
		PublicAdditionalLibraries.Add(ImportLib);
		// Loaded explicitly by AthleteTracker at startup (from the project's Binaries folder).
		PublicDelayLoadDLLs.Add("mujoco.dll");
		RuntimeDependencies.Add("$(BinaryOutputDir)/mujoco.dll", Dll);
	}
}
