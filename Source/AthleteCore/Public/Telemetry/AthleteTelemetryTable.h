// Project ATHLETE

#pragma once

#include "CoreMinimal.h"

/**
 * An in-memory table of numeric samples with a fixed column schema, exportable to CSV.
 *
 * This is deliberately a plain C++ class (not a UObject): it has no dependency on worlds,
 * actors, or the editor, so it can be unit-tested in isolation and reused anywhere.
 *
 * Conventions:
 *  - Values are SI units. Column names end with their unit, e.g. "vel_z_mps", "force_N".
 *  - Column 0 is usually "time_s" (simulation time, not wall-clock time).
 *  - Game-thread only for now.
 */
class ATHLETECORE_API FAthleteTelemetryTable
{
public:
	FAthleteTelemetryTable(FName InName, TArray<FName> InColumns);

	FName GetName() const { return Name; }
	const TArray<FName>& GetColumns() const { return Columns; }
	int32 NumColumns() const { return Columns.Num(); }
	int32 NumRows() const { return Columns.Num() > 0 ? Values.Num() / Columns.Num() : 0; }

	/** Index of a column, or INDEX_NONE. */
	int32 FindColumn(FName ColumnName) const { return Columns.IndexOfByKey(ColumnName); }

	/**
	 * Appends one row. Values must be in column order and match the column count.
	 * Returns false (and records nothing) on a size mismatch.
	 */
	bool AddRow(TConstArrayView<double> RowValues);

	double GetValue(int32 Row, int32 Column) const;

	/** Header line plus one line per row. Uses '.' as the decimal separator regardless of locale. */
	FString ToCsv() const;

	/** Returns true if Text can be written into a CSV cell without quoting. */
	static bool IsCsvSafe(const FString& Text);

	void Reset() { Values.Reset(); }

private:
	FName Name;
	TArray<FName> Columns;

	/** Row-major storage: row R, column C lives at R * NumColumns() + C. One allocation, cache-friendly. */
	TArray<double> Values;
};
