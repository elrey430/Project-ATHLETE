# Project ATHLETE

A physics-first American football simulation in Unreal Engine 5.8.

> The athlete does not play an animation that says what happened. The athlete attempts to
> perform an action, and the simulation discovers what happens.

**Current state:** Milestone 0 (project foundation). There's no athlete and no football yet.

## Requirements

- Unreal Engine 5.8 at `C:\Program Files\Epic Games\UE_5.8` (override with env var `ATHLETE_UE_ROOT`)
- Visual Studio 2026 with the *Game development with C++* and *Desktop development with C++* workloads
- Git + Git LFS

## Daily commands

Run these from the project root in PowerShell. Close the editor before building from the command line.

| Task | Command |
|---|---|
| Compile | `powershell -ExecutionPolicy Bypass -File Scripts\Build.ps1` |
| Run all tests | `powershell -ExecutionPolicy Bypass -File Scripts\RunTests.ps1` |
| Run some tests | `... Scripts\RunTests.ps1 -Filter Athlete.Core` |
| Reproducible lab run | `... Scripts\RunLab.ps1 -Seconds 6 -Seed 42` |
| Regenerate VS solution | `... Scripts\GenerateProjectFiles.ps1` (after adding or removing C++ files) |
| Rebuild the lab level | delete `Content/Lab/Maps/AthleteLab`, then `... Scripts\BuildLab.ps1` |

Open the editor by double-clicking `ProjectAthlete.uproject`. Open the code with `ProjectAthlete.sln`.

In Visual Studio, tests also appear in **Test > Test Explorer** under `Athlete` (via the vendored
Visual Studio Integration Tool plugin in `Plugins/VisualStudioTools`; see its `VENDORED.md`).

## Where things go

- Telemetry CSVs: `Saved/Telemetry/<timestamp>_<world>/` (`session.csv` = seed, engine, physics settings)
- Test report: `Saved/Automation/Reports/index.json`
- Logs: `Saved/Logs/`

## Debug console (press ` while playing)

| Command | Effect |
|---|---|
| `athlete.Debug.Kinematics 1` | velocity arrows |
| `athlete.Debug.CenterOfMass 1` | center-of-mass markers |
| `athlete.Debug.VelocityArrowScale 10` | cm of arrow per m/s |
| `athlete.Telemetry.Flush` | write telemetry to disk now |

See [Docs/Architecture.md](Docs/Architecture.md) for the design.
