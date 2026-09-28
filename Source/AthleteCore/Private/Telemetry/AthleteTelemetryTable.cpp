// Project ATHLETE

#include "Telemetry/AthleteTelemetryTable.h"
#include "AthleteCore.h"

FAthleteTelemetryTable::FAthleteTelemetryTable(FName InName, TArray<FName> InColumns)
	: Name(InName)
	, Columns(MoveTemp(InColumns))
{
	ensureMsgf(Columns.Num() > 0, TEXT("Telemetry table '%s' has no columns."), *Name.ToString());
	for (const FName& Column : Columns)
	{
		ensureMsgf(IsCsvSafe(Column.ToString()), TEXT("Telemetry column '%s' contains characters that break CSV."), *Column.ToString());
	}
}

bool FAthleteTelemetryTable::AddRow(TConstArrayView<double> RowValues)
{
	if (RowValues.Num() != Columns.Num())
	{
		UE_LOG(LogAthleteTelemetry, Warning, TEXT("Table '%s': row has %d values but the table has %d columns. Row dropped."),
			*Name.ToString(), RowValues.Num(), Columns.Num());
		return false;
	}
	Values.Append(RowValues.GetData(), RowValues.Num());
	return true;
}

double FAthleteTelemetryTable::GetValue(int32 Row, int32 Column) const
{
	check(Row >= 0 && Row < NumRows());
	check(Column >= 0 && Column < NumColumns());
	return Values[Row * Columns.Num() + Column];
}

FString FAthleteTelemetryTable::ToCsv() const
{
	TStringBuilder<4096> Builder;

	for (int32 Column = 0; Column < Columns.Num(); ++Column)
	{
		if (Column > 0)
		{
			Builder << TEXT(',');
		}
		Builder << Columns[Column];
	}
	Builder << TEXT('\n');

	const int32 RowCount = NumRows();
	for (int32 Row = 0; Row < RowCount; ++Row)
	{
		for (int32 Column = 0; Column < Columns.Num(); ++Column)
		{
			if (Column > 0)
			{
				Builder << TEXT(',');
			}
			// %.9g keeps ~9 significant digits: enough to resolve micrometers over a 100 m field
			// while keeping files readable. Unreal's Printf always uses '.' for decimals.
			Builder.Appendf(TEXT("%.9g"), Values[Row * Columns.Num() + Column]);
		}
		Builder << TEXT('\n');
	}

	return FString(Builder.ToView());
}

bool FAthleteTelemetryTable::IsCsvSafe(const FString& Text)
{
	int32 Unused = 0;
	return !Text.IsEmpty()
		&& !Text.FindChar(TEXT(','), Unused)
		&& !Text.FindChar(TEXT('"'), Unused)
		&& !Text.FindChar(TEXT('\n'), Unused)
		&& !Text.FindChar(TEXT('\r'), Unused);
}
