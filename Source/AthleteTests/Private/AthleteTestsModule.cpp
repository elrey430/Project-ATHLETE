// Project ATHLETE

#include "Modules/ModuleManager.h"

// Tests register themselves through static objects created by IMPLEMENT_*_AUTOMATION_TEST,
// so this module needs no startup code.
IMPLEMENT_MODULE(FDefaultModuleImpl, AthleteTests);
