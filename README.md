# Project ATHLETE

A physics-first American football simulation in Unreal Engine 5.8.

> The athlete does not play an animation that says what happened. The athlete attempts to
> perform an action, and the simulation discovers what happens.

**Current state:** Milestone 4 ([locomotion](Docs/Milestone4_Locomotion.md)).
Athletes have computed bodies ([Milestone 1](Docs/Milestone1_AthleteBody.md)) that simulate as articulated
Chaos rigid bodies ([Milestone 2](Docs/Milestone2_PhysicalHumanoid.md)) and stand by their own muscles
([Milestone 3](Docs/Milestone3_StandingBalance.md)). Muscles now have force-velocity limits, and physics runs
at 480 Hz. A hand-built gait steps in place and walks slowly (~0.3 m/s); faster walking falls within a few steps.
A reinforcement-learning spike (Learning Agents) trains end to end. Milestone 4 concluded that human-like
motion should be learned from motion capture, in a faster physics engine than Chaos: a physics-engine spike
is next ([costs](Docs/Research_LearnedMotionCosts.md)). No football yet.

## Athletes

Athletes are Data Assets in `Content/Athletes` (`DA_Athlete_Reference`, `DA_Athlete_A`, `DA_Athlete_B`).
Double-click one to edit height, mass, proportions, mass distribution, and strength. The **Fill
Strength From General Population Baseline** button recomputes strength from age, height, and mass.
The AthleteLab level shows each sample athlete's body next to the start line, updating live as you edit.
Press Play (or Simulate) to watch three passive-body experiments 3 m past the start line: A dropped from
1.5 m, the reference male collapsing, and B pushed in the chest. Each writes `Ragdoll_*.csv` telemetry.
8 m past the start line, five standing experiments (Milestone 3) balance on their own muscles:
quiet standing, pelvis pushes they recover from, one they don't, and one with no neural control.
They write `Standing_*.csv`. An athlete's balance parameters are under **Capability > Motor Skill** in the Data Asset.

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
| Run some tests | `... Scripts\RunTests.ps1 -Filter Athlete.Core` (add `-ShowInfo` to see measurements of passing tests) |
| Performance benchmark | `... Scripts\RunTests.ps1 -Filter ProjectPerf -ShowInfo` (not part of the normal suite) |
| Learning spike | `... Scripts\SetupLearningPython.ps1` once, then `-Filter ProjectLearn.Spike.Walk.Smoke32` |
| Reproducible lab run | `... Scripts\RunLab.ps1 -Seconds 6 -Seed 42` |
| Regenerate VS solution | `... Scripts\GenerateProjectFiles.ps1` (after adding or removing C++ files) |
| Regenerate athletes + lab level | `... Scripts\BuildLab.ps1 -Rebuild` (replaces the generated level) |

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
| `athlete.Debug.Anatomy 0` / `1` | hide/show athlete body previews (on by default) |
| `athlete.Debug.Balance 1` | balance: center of the feet (yellow), extrapolated center of mass (cyan) |
| `athlete.Debug.VelocityArrowScale 10` | cm of arrow per m/s |
| `athlete.Telemetry.Flush` | write telemetry to disk now |

See [Docs/Architecture.md](Docs/Architecture.md) for the design.
