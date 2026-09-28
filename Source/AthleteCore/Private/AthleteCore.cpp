// Project ATHLETE

#include "AthleteCore.h"
#include "Modules/ModuleManager.h"

DEFINE_LOG_CATEGORY(LogAthlete);
DEFINE_LOG_CATEGORY(LogAthleteTelemetry);

// A plain (non-primary) module with no custom startup/shutdown logic.
IMPLEMENT_MODULE(FDefaultModuleImpl, AthleteCore);
