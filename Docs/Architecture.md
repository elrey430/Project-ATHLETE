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
AthleteLearning (learned control spike: Learning Agents interactor and environment)
    |
    v
AthleteMotor (muscles inside every physics substep; balance, posture and gait control)
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
| `AthleteMotor` | `UAthleteMotorComponent`, `FAthleteMuscleSimCallback` (Chaos sim callback), muscle impedance model with force-velocity limits, balance/posture controller ([Milestone 3](Milestone3_StandingBalance.md)), gait controller ([Milestone 4](Milestone4_Locomotion.md)), MuJoCo model export (`AthleteMjcfExport`, for the physics-engine spike). Moves the body only through internal joint torques | gameplay, football, presentation |
| `AthleteLearning` | `UAthleteLearningAgentComponent`, `UAthleteLocomotionInteractor`, `UAthleteLocomotionEnvironment`: a policy sets joint targets that the same muscles turn into torques ([Milestone 4 §6](Milestone4_Locomotion.md)). A spike, pending the physics-engine decision | gameplay, football, presentation |
| `ProjectAthlete` | `AAthleteLabGameMode`, `AAthleteLabPhysicsProbe`, `AAthleteLabBodyPreview`, `AAthleteLabRagdoll`, `AAthleteLabStanding` | presentation systems |
| `AthleteTests` | automation tests | (it's allowed to depend on everything it tests) |

Planned modules are added **only when their milestone starts**. No empty placeholders:

| Planned module | Milestone | Pipeline stage |
|---|---|---|
| contact skills | 7 | extend `AthleteMotor`; to be re-planned around learned skills after the physics-engine spike |
| `FootballBall` | 10 | ball physics |
| `AthletePerception` / `AthleteCognition` | 11 | perception, belief state, decisions, intent |
| `FootballRules` | 11+ | downs, scoring, possession: separate from biomechanics |
| `AthletePresentation` | later | animation polish, cameras, audio. Reads the simulation, never writes to it |

## Plugins

| Plugin | Source | Why | Scope |
|---|---|---|---|
| `PythonScriptPlugin` | engine | generates lab content: athlete assets (`create_sample_athletes.py`) and the AthleteLab level (`build_athlete_lab.py`) | editor only |
| `VisualStudioTools` | vendored at `Plugins/VisualStudioTools` (Microsoft, MIT) | Blueprint references in VS, and Test Explorer discovery and running | editor only |
| `LearningAgents` | engine (experimental) | reinforcement learning (PPO) for the learned control spike. Its Python trainer needs PyTorch: `Scripts/SetupLearningPython.ps1` installs a CPU-only build, and the engine's automatic CUDA install is switched off (`bRunPipInstallOnStartup=False`) | runtime + training |

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

`Config/DefaultEngine.ini`: Chaos substepping at a maximum of 1/480 s, up to 16 substeps (1/30 s).
Frames slower than 30 FPS slow simulated time down instead of taking an unstable large step.

**Decided (Milestone 4): 480 Hz.** At 240 Hz the light foot let the ankle muscles almost no damping, and
standing sway after a push never settled (11–12 mm RMS; 3.4 mm at 480 Hz). Cost: 13.2 ms of physics per
frame for 22 standing athletes, against 8.1 ms at 240 Hz ([Milestone 4 §4.2](Milestone4_Locomotion.md)).

**Decided (Milestone 3): muscles run inside every physics substep.** `FAthleteMuscleSimCallback` is a
Chaos sim callback (`TSimCallbackObject`, pre-simulate). Chaos calls it on every internal step and gives
each substep the frame's input. Each frame the game thread sends the muscle targets and impedances; the
physics thread computes and applies the torques. The neural loops (balance, posture) still run once per
frame, which is physiologically slow enough, but their delays are quantized to frames (see debt below).
Explicit muscle torques need numerical caps per substep ([Milestone 3 §5.4](Milestone3_StandingBalance.md)).
Per-joint gains apply along the joint's **anatomical** axes (`FAthleteMuscleCommand::AxisFrameLocal`), not
its constraint frame, which is turned by the range-of-motion neutral (a Milestone 4 bug fix).

**Open decision, top technical risk: the physics engine itself.** Violent impacts make Chaos create
energy below ~1 kHz ([Milestone 2 §8–9](Milestone2_PhysicalHumanoid.md)), muscles need 480 Hz, and Chaos
has no GPU path for training learned motion. Milestone 4 concluded to spike MuJoCo (primary) and NVIDIA
PhysX articulations (fallback) as the on-field physics, with Unreal staying authoritative for the game
([Milestone 4 §7–8](Milestone4_Locomotion.md)).

## Physics fidelity rules

Chaos stabilization defaults that silently change physics are disabled on athlete bodies: inertia
conditioning, joint mass conditioning, joint projection, the 3600°/s spin clamp, and the linear joint
solver. Gyroscopic torque is enabled. Mass properties are written directly to Chaos. **Sleeping is
disabled** on athlete bodies: a sleeping body is frozen, which faked standing stability in Milestone 3.
Chaos joint drives are not used for muscles (their stiffness depends on the bodies' mass ratio); muscles
are explicit torques. Any new physics object in ATHLETE should get the same audit.

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
| Neural loops run per frame; their delays are whole frames | tests and lab use a fixed 60 Hz step | move them into the sim callback before scale tests (Milestone 12) |
| Muscle torques computed one athlete after another on the physics thread | 4.1 ms of 13.2 ms for 22 athletes | batch and parallelise; or moot after the engine decision |
| Estimated maximum joint speeds for force-velocity | no better data found yet | when data is found |
| Hand-built gait limited to slow walking; accel/brake/turn unvalidated | kept as a baseline; learned skills are the plan | physics-engine spike |
| Learning spike has no sensing delay; Learning Agents is experimental | a feasibility spike | if kept after the engine decision |
| 16-segment skeleton (no toes, few spine joints) | enough for standing and walking | with the engine decision |
