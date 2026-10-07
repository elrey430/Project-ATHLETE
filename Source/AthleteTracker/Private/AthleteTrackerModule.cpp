// Project ATHLETE

#include "HAL/PlatformProcess.h"
#include "Misc/Paths.h"
#include "Modules/ModuleManager.h"

THIRD_PARTY_INCLUDES_START
#include <mujoco/mujoco.h>
THIRD_PARTY_INCLUDES_END

DEFINE_LOG_CATEGORY_STATIC(LogAthleteTracker, Log, All);

/** Loads MuJoCo (delay-loaded; copied next to the project's binaries by the MuJoCo module) at startup. */
class FAthleteTrackerModule : public IModuleInterface
{
public:
	virtual void StartupModule() override
	{
		const FString Dll = FPaths::ConvertRelativePathToFull(FPaths::Combine(FPaths::ProjectDir(), TEXT("Binaries"), TEXT("Win64"), TEXT("mujoco.dll")));
		MuJoCoHandle = FPlatformProcess::GetDllHandle(*Dll);
		if (!MuJoCoHandle)
		{
			UE_LOG(LogAthleteTracker, Error, TEXT("Could not load %s (run Scripts/SetupMuJoCoUnreal.ps1 and rebuild)"), *Dll);
			return;
		}
		if (mj_version() != mjVERSION_HEADER)
		{
			UE_LOG(LogAthleteTracker, Error, TEXT("MuJoCo DLL version %d doesn't match the headers (%d)"), mj_version(), mjVERSION_HEADER);
		}
	}

	virtual void ShutdownModule() override
	{
		if (MuJoCoHandle)
		{
			FPlatformProcess::FreeDllHandle(MuJoCoHandle);
			MuJoCoHandle = nullptr;
		}
	}

private:
	void* MuJoCoHandle = nullptr;
};

IMPLEMENT_MODULE(FAthleteTrackerModule, AthleteTracker)
