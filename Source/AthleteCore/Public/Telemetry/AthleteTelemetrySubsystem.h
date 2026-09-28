// Project ATHLETE

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "Telemetry/AthleteTelemetryTable.h"
#include "AthleteTelemetrySubsystem.generated.h"

/**
 * Collects telemetry tables for one world ("session") and writes them to CSV.
 *
 * Output layout:
 *   <Project>/Saved/<TelemetryFolderName>/<SessionId>/session.csv   (key,value metadata: seed, engine, physics settings...)
 *   <Project>/Saved/<TelemetryFolderName>/<SessionId>/<Table>.csv   (one file per table)
 *
 * Files are written when the world is torn down (end of PIE, game exit, test world destroyed)
 * or on demand with the console command:  athlete.Telemetry.Flush
 *
 * Telemetry is observation only. Nothing in the simulation may read from it.
 */
UCLASS()
class ATHLETECORE_API UAthleteTelemetrySubsystem : public UWorldSubsystem
{
	GENERATED_BODY()

public:
	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Deinitialize() override;

	/** True when project settings allow telemetry recording. */
	bool IsEnabled() const { return bEnabled; }

	/**
	 * Creates a table owned by this session. Returns null when telemetry is disabled,
	 * so callers must null-check (and can skip computing samples entirely).
	 * Table names must be unique within a session; a duplicate name gets a numeric suffix.
	 */
	TSharedPtr<FAthleteTelemetryTable> CreateTable(FName TableName, TArray<FName> Columns);

	/** Adds or replaces a key/value pair written to session.csv. */
	void SetSessionMetadata(const FString& Key, const FString& Value);

	/**
	 * Writes all tables and metadata to disk (overwriting previous flushes of this session).
	 * Returns the session directory, or an empty string if nothing was written.
	 */
	FString FlushToDisk();

	const FString& GetSessionId() const { return SessionId; }

protected:
	virtual bool DoesSupportWorldType(const EWorldType::Type WorldType) const override;

private:
	void RecordStandardMetadata();

	bool bEnabled = false;
	FString SessionId;

	TArray<TSharedRef<FAthleteTelemetryTable>> Tables;

	// Insertion-ordered key/value pairs so session.csv is stable and diffable.
	TArray<TPair<FString, FString>> SessionMetadata;
};
