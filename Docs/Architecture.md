# ATHLETE Architecture

## Module map

Unreal code is organized into **modules**. Each module is a folder under `Source/` with a
`*.Build.cs` file, compiles to its own DLL in the editor, and declares its dependencies explicitly.
Dependencies point in one direction only:

```
AthleteTests (DeveloperTool; never shipped)
    |
    v
ProjectAthlete (primary game module: AthleteLab scaffolding)
    |
    v
AthletePhysics (articulated Chaos body: 16 rigid bodies, 15 anatomical joints)
    |
    v
AthleteBody (morphology, segment inertial model, joints, mobility, strength, athlete Data Assets)
    |
    v
AthleteCore (simulation foundation)
    |
    v
Unreal Engine (Core, CoreUObject, Engine, DeveloperSettings)
```

| Module | Contains | Must never depend on |
|---|---|---|
| `AthleteCore` | units, deterministic RNG, telemetry, debug draw, simulation settings/seed | anything football, gameplay, or presentation |
| `AthleteBody` | `FAthleteMorphology`, `FAthleteBodyModel`, joints, `FAthleteMobilityProfile`, `FAthleteStrengthProfile`, `UAthleteDefinition`, anthropometric reference data ([Milestone 1](Milestone1_AthleteBody.md)). Pure data: no physics engine | gameplay, football, presentation, physics engine |
| `AthletePhysics` | `UAthletePhysicalBodyComponent`, collision geometry, joint setup ([Milestone 2](Milestone2_PhysicalHumanoid.md)) | gameplay, football, presentation |
| `ProjectAthlete` | `AAthleteLabGameMode`, `AAthleteLabPhysicsProbe`, `AAthleteLabBodyPreview`, `AAthleteLabRagdoll` | presentation systems |
| `AthleteTests` | automation tests | (it's allowed to depend on everything it tests) |

Planned modules are added **only when their milestone starts**. No empty placeholders:

| Planned module | Milestone | Pipeline stage |
|---|---|---|
| `AthleteMotor` | 3-4, 7 | balance, locomotion, contact motor skills (Physical Athlete Controller) |
| `FootballBall` | 10 | ball physics |
| `AthletePerception` / `AthleteCognition` | 11 | perception, belief state, decisions, intent |
| `FootballRules` | 11+ | downs, scoring, possession: separate from biomechanics |
| `AthletePresentation` | later | animation polish, cameras, audio. Reads the simulation, never writes to it |

## Plugins

| Plugin | Source | Why | Scope |
|---|---|---|---|
| `PythonScriptPlugin` | engine | generates lab content: athlete assets (`create_sample_athletes.py`) and the AthleteLab level (`build_athlete_lab.py`) | editor only |
| `VisualStudioTools` | vendored at `Plugins/VisualStudioTools` (Microsoft, MIT) | Blueprint references in VS, and Test Explorer discovery and running | editor only |

Tests must carry `CommandletContext` (see `AthleteTestFlags`), because VS discovers and runs them through a commandlet.

## Source layout and naming

```
Source/<Module>/Public/<Area>/Foo.h      headers other modules may include
Source/<Module>/Private/<Area>/Foo.cpp   implementation
```

- Unreal prefixes: `U` = UObject, `A` = Actor, `F` = plain struct/class, `E` = enum, `I` = interface, `T` = template.
- Project prefix: `Athlete` (e.g. `UAthleteTelemetrySubsystem`).
- Test names: `Athlete.<Module>.<Area>.<Behavior>`, e.g. `Athlete.Core.Random.SameSeedSameSequence`.
- Telemetry columns: snake_case plus an SI unit suffix: `vel_z_mps`, `force_N`, `time_s`.

## Units

Simulation math is in **SI** (m, kg, s, N). Unreal is in **cm**. Football data entry is imperial.
Convert at the boundary with `AthleteUnits` (`Source/AthleteCore/Public/Units/AthleteUnits.h`).

## Reproducibility

- `UAthleteSimulationSubsystem` owns the per-world master seed (Project Settings, or `-AthleteSeed=N`).
- Consumers get independent streams: `MakeRandomStream(TEXT("Athlete3.MotorNoise"))`.
  Adding a consumer never changes another consumer's numbers.
- `FAthleteRandomStream` is PCG32, pinned by a test to the published reference output.
- `Scripts/RunLab.ps1` runs with a fixed frame step (`-benchmark -fps=60`). Two runs with the same seed
  produced byte-identical telemetry at Milestone 0.
- Randomness models natural variance only. It never decides outcomes.

## Physics timestep

`Config/DefaultEngine.ini`: Chaos substepping at a maximum of 1/240 s, up to 8 substeps (1/30 s).
Frames slower than 30 FPS slow simulated time down instead of taking an unstable large step.

**Open decision (Milestone 3):** the active-ragdoll controller must apply joint torques inside
*every* physics step, not once per rendered frame. Options are per-substep callbacks or Chaos async
physics with a truly fixed step (`bTickPhysicsAsync`). Decide with measurements when the controller exists.

**Open decision (Milestone 6), top technical risk:** at 240 Hz, violent impacts make the solver create
energy (up to ~330 J in a 1.5 m drop). About 1 kHz removes it, at roughly 2 ms per athlete-frame. See
[Milestone 2 §8–9](Milestone2_PhysicalHumanoid.md).

## Physics fidelity rules

Chaos stabilization defaults that silently change physics are disabled on athlete bodies: inertia
conditioning, joint mass conditioning, joint projection, the 3600°/s spin clamp, and the linear joint
solver. Gyroscopic torque is enabled. Mass properties are written directly to Chaos. Any new physics
object in ATHLETE should get the same audit.

## Telemetry and debug

- `FAthleteTelemetryTable`: plain C++ in-memory table, then CSV (unit-tested, no engine dependencies).
- `UAthleteTelemetrySubsystem`: one session per world. Writes CSVs on world teardown or `athlete.Telemetry.Flush`.
- `AthleteDebug::DrawArrow/DrawPoint`: console-variable-gated channels, compiled out of Shipping builds.
- Telemetry and debug drawing are **observation only**. Simulation code never reads them.

## Known technical debt

| Item | Why it's acceptable now | Revisit |
|---|---|---|
| Telemetry kept fully in memory, rewritten on every flush | lab sessions are seconds long | 22-athlete scale tests (Milestone 12) |
| Telemetry is game-thread only | nothing samples off-thread yet | async physics decision |
| Probe acceleration is a finite difference at frame rate | fine for validating gravity | athlete telemetry should sample inside the physics step |
