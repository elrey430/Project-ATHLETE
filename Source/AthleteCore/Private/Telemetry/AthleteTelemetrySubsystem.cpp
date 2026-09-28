// Project ATHLETE

#include "Telemetry/AthleteTelemetrySubsystem.h"
#include "AthleteCore.h"
#include "Simulation/AthleteSimulationSettings.h"
#include "Simulation/AthleteSimulationSubsystem.h"
#include "Units/AthleteUnits.h"
#include "Engine/World.h"
#include "HAL/FileManager.h"
#include "HAL/IConsoleManager.h"
#include "Misc/App.h"
#include "Misc/CommandLine.h"
#include "Misc/DateTime.h"
#include "Misc/EngineVersion.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "PhysicsEngine/PhysicsSettings.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"

namespace
{
	FAutoConsoleCommandWithWorld GFlushTelemetryCommand(
		TEXT("athlete.Telemetry.Flush"),
		TEXT("Writes all ATHLETE telemetry tables for the current world to Saved/Telemetry now."),
		FConsoleCommandWithWorldDelegate::CreateLambda([](UWorld* World)
		{
			if (UAthleteTelemetrySubsystem* Telemetry = World ? World->GetSubsystem<UAthleteTelemetrySubsystem>() : nullptr)
			{
				Telemetry->FlushToDisk();
			}
			else
			{
				UE_LOG(LogAthleteTelemetry, Warning, TEXT("No telemetry subsystem in this world (telemetry runs only in PIE/game worlds)."));
			}
		}));
}

void UAthleteTelemetrySubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	// Guarantees the simulation subsystem (and therefore the master seed) exists before we record it.
	Collection.InitializeDependency<UAthleteSimulationSubsystem>();
	Super::Initialize(Collection);

	bEnabled = UAthleteSimulationSettings::Get()->bTelemetryEnabled;

	// Timestamp + world name, e.g. "20260927-142501_AthleteLab". Sorts chronologically in Explorer.
	SessionId = FString::Printf(TEXT("%s_%s"), *FDateTime::Now().ToString(TEXT("%Y%m%d-%H%M%S")), *GetWorld()->GetName());

	if (bEnabled)
	{
		RecordStandardMetadata();
	}
}

void UAthleteTelemetrySubsystem::Deinitialize()
{
	FlushToDisk();
	Tables.Reset();
	Super::Deinitialize();
}

bool UAthleteTelemetrySubsystem::DoesSupportWorldType(const EWorldType::Type WorldType) const
{
	return WorldType == EWorldType::Game || WorldType == EWorldType::PIE;
}

TSharedPtr<FAthleteTelemetryTable> UAthleteTelemetrySubsystem::CreateTable(FName TableName, TArray<FName> Columns)
{
	if (!bEnabled)
	{
		return nullptr;
	}

	FName UniqueName = TableName;
	for (int32 Suffix = 2; Tables.ContainsByPredicate([&UniqueName](const TSharedRef<FAthleteTelemetryTable>& Table) { return Table->GetName() == UniqueName; }); ++Suffix)
	{
		UniqueName = FName(*FString::Printf(TEXT("%s_%d"), *TableName.ToString(), Suffix));
	}

	TSharedRef<FAthleteTelemetryTable> Table = MakeShared<FAthleteTelemetryTable>(UniqueName, MoveTemp(Columns));
	Tables.Add(Table);
	return Table;
}

void UAthleteTelemetrySubsystem::SetSessionMetadata(const FString& Key, const FString& Value)
{
	ensureMsgf(FAthleteTelemetryTable::IsCsvSafe(Key), TEXT("Metadata key '%s' is not CSV-safe."), *Key);

	for (TPair<FString, FString>& Pair : SessionMetadata)
	{
		if (Pair.Key == Key)
		{
			Pair.Value = Value;
			return;
		}
	}
	SessionMetadata.Emplace(Key, Value);
}

void UAthleteTelemetrySubsystem::RecordStandardMetadata()
{
	const UWorld* World = GetWorld();
	const UPhysicsSettings* Physics = UPhysicsSettings::Get();

	SetSessionMetadata(TEXT("session_id"), SessionId);
	SetSessionMetadata(TEXT("engine_version"), FEngineVersion::Current().ToString());
	SetSessionMetadata(TEXT("world"), World->GetName());
	SetSessionMetadata(TEXT("master_seed"), LexToString(World->GetSubsystem<UAthleteSimulationSubsystem>()->GetMasterSeed()));
	SetSessionMetadata(TEXT("gravity_z_mps2"), LexToString(AthleteUnits::UnrealToMeters(World->GetGravityZ())));
	SetSessionMetadata(TEXT("physics_substepping"), Physics->bSubstepping ? TEXT("true") : TEXT("false"));
	SetSessionMetadata(TEXT("physics_max_substep_dt_s"), LexToString(Physics->MaxSubstepDeltaTime));
	SetSessionMetadata(TEXT("physics_max_substeps"), LexToString(Physics->MaxSubsteps));
	SetSessionMetadata(TEXT("physics_async"), Physics->bTickPhysicsAsync ? TEXT("true") : TEXT("false"));
	// Same rule the engine uses (UEngine::UpdateTimeAndHandleMaxTickRate): -benchmark also forces a fixed frame step.
	const bool bFixedFrameStep = FApp::IsBenchmarking() || FApp::UseFixedTimeStep();
	SetSessionMetadata(TEXT("fixed_frame_timestep_s"), bFixedFrameStep ? LexToString(FApp::GetFixedDeltaTime()) : TEXT("variable"));
	SetSessionMetadata(TEXT("command_line"), FString(FCommandLine::Get()).Replace(TEXT(","), TEXT(";")).Replace(TEXT("\""), TEXT("'")));
}

FString UAthleteTelemetrySubsystem::FlushToDisk()
{
	TRACE_CPUPROFILER_EVENT_SCOPE(AthleteTelemetry_FlushToDisk);

	if (!bEnabled || Tables.IsEmpty())
	{
		return FString();
	}

	const FString SessionDirectory = FPaths::Combine(FPaths::ProjectSavedDir(), UAthleteSimulationSettings::Get()->TelemetryFolderName, SessionId);
	IFileManager::Get().MakeDirectory(*SessionDirectory, /*Tree=*/true);

	TStringBuilder<1024> MetadataCsv;
	MetadataCsv << TEXT("key,value\n");
	for (const TPair<FString, FString>& Pair : SessionMetadata)
	{
		MetadataCsv << Pair.Key << TEXT(',') << Pair.Value << TEXT('\n');
	}
	FFileHelper::SaveStringToFile(MetadataCsv.ToView(), *FPaths::Combine(SessionDirectory, TEXT("session.csv")));

	int32 FilesWritten = 0;
	for (const TSharedRef<FAthleteTelemetryTable>& Table : Tables)
	{
		const FString FilePath = FPaths::Combine(SessionDirectory, Table->GetName().ToString() + TEXT(".csv"));
		if (FFileHelper::SaveStringToFile(Table->ToCsv(), *FilePath))
		{
			++FilesWritten;
		}
		else
		{
			UE_LOG(LogAthleteTelemetry, Error, TEXT("Failed to write telemetry file '%s'."), *FilePath);
		}
	}

	const FString FullPath = FPaths::ConvertRelativePathToFull(SessionDirectory);
	UE_LOG(LogAthleteTelemetry, Log, TEXT("Wrote %d telemetry table(s) to %s"), FilesWritten, *FullPath);
	return FullPath;
}
