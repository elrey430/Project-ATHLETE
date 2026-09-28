// Project ATHLETE

#pragma once

#include "Misc/AutomationTest.h"

/**
 * Standard flags for ATHLETE tests.
 *  - EditorContext: runs inside the editor process (Session Frontend, Scripts/RunTests.ps1).
 *  - CommandletContext: runs inside commandlets, which is how Visual Studio's Test Explorer
 *    discovers and runs tests (through the VisualStudioTools plugin's VSTestAdapter commandlet).
 *  - ProductFilter: marks these as project tests (vs. Epic's engine tests).
 */
inline constexpr EAutomationTestFlags AthleteTestFlags =
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::CommandletContext | EAutomationTestFlags::ProductFilter;
