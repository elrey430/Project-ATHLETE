// Project ATHLETE

#include "AthleteTestFlags.h"
#include "Telemetry/AthleteTelemetryTable.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAthleteTelemetryTableTest, "Athlete.Core.Telemetry.TableRowsAndCsv", AthleteTestFlags)

bool FAthleteTelemetryTableTest::RunTest(const FString& Parameters)
{
	FAthleteTelemetryTable Table(TEXT("Example"), { TEXT("time_s"), TEXT("pos_x_m"), TEXT("vel_x_mps") });

	UTEST_EQUAL(TEXT("Column count"), Table.NumColumns(), 3);
	UTEST_EQUAL(TEXT("Starts empty"), Table.NumRows(), 0);

	TestTrue(TEXT("Valid row accepted"), Table.AddRow({ 0.0, 1.5, -2.25 }));
	TestTrue(TEXT("Valid row accepted"), Table.AddRow({ 0.5, 0.125, 3.0 }));

	// A malformed row must be rejected rather than silently corrupting the table layout.
	AddExpectedMessage(TEXT("row has 2 values but the table has 3 columns"), EAutomationExpectedMessageFlags::Contains, 1);
	TestFalse(TEXT("Short row rejected"), Table.AddRow({ 1.0, 2.0 }));

	UTEST_EQUAL(TEXT("Row count"), Table.NumRows(), 2);
	TestEqual(TEXT("Value lookup"), Table.GetValue(1, Table.FindColumn(TEXT("pos_x_m"))), 0.125);
	TestEqual(TEXT("Missing column"), Table.FindColumn(TEXT("nope")), static_cast<int32>(INDEX_NONE));

	const FString ExpectedCsv =
		TEXT("time_s,pos_x_m,vel_x_mps\n")
		TEXT("0,1.5,-2.25\n")
		TEXT("0.5,0.125,3\n");
	TestEqual(TEXT("CSV output"), Table.ToCsv(), ExpectedCsv);

	TestTrue(TEXT("Plain name is CSV-safe"), FAthleteTelemetryTable::IsCsvSafe(TEXT("speed_mps")));
	TestFalse(TEXT("Comma is not CSV-safe"), FAthleteTelemetryTable::IsCsvSafe(TEXT("a,b")));

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
