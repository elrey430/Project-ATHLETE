# The learned athlete in Unreal

Built 2026-10-05. Milestone 4's tracking policy (run 10, chunk 55: all six M4 tests passed in Python),
playable in Unreal.

![The learned athlete walking in the lab](images/learned_athlete_unreal.png)

## How it works

The policy learned MuJoCo's physics: its contacts, solver, joint limits and 2 ms step. Driving the Chaos
athlete (Milestones 1-3) with it would fail, and retraining in Chaos isn't practical. So **the athlete's
body runs in MuJoCo inside Unreal**. Unreal provides input, camera, rendering and the game around it.
The Chaos athlete is untouched.

```
controller (keyboard/gamepad) -> command (forward m/s, sideways m/s, turn rad/s)
  -> command shaper (human acceleration limits)
  -> motion matcher (picks mocap frames; runs 30 frames ahead, so the policy sees 0.1-0.3 s of future)
  -> observation (653 values, heading-invariant)
  -> policy (653 -> 512 -> 256 -> 37, tanh)
  -> 37 joint motors -> MuJoCo (5 x 2 ms per 10 ms control step)
  -> body drawn from MuJoCo's geometry every frame
```

| Piece | Where | Ported from |
|---|---|---|
| MuJoCo 3.5.0 (C library) | `Source/ThirdParty/MuJoCo` | the exact version the policy was trained with (from the Python package) |
| Bundle loader | `AthleteTrackerBundle` | the export (`Scripts/MuJoCo/export_unreal_bundle.py`) |
| Policy | `AthleteTrackerPolicy` | LocoMuJoCo's PPO actor |
| Command shaper, motion matcher | `AthleteMotionMatcher` | `controller.py`, `motion_matching.py` |
| Simulation loop, observation | `AthleteTrackerSim` | `locomotion_tests.Driver`, `athlete_loco/invariant.py`, `lookahead.py` |
| Playable pawn, game mode | `AthleteTrackerPawn`, `AthleteTrackerGameMode` | new |

Units inside are MuJoCo's (metres, Z up, right-handed). The pawn converts to Unreal: cm, with Y mirrored,
as the MJCF exporter does.

## Setup (once per machine)

1. LocoMuJoCo Python environment (`Scripts/SetupLocoMuJoCoPython.ps1`), which also provides MuJoCo.
2. `Scripts/SetupMuJoCoUnreal.ps1`: copies MuJoCo's headers, DLL and licenses into
   `Source/ThirdParty/MuJoCo`, and generates the import library. These files are git-ignored.
3. Export the bundle, with the same motion set the tracker was trained on:
   ```
   set ATHLETE_MOTION_SET=100style
   set ATHLETE_MOTIONS=C:\Dev\ProjectAthlete\Saved\MuJoCo\motions_run9
   Intermediate\LocoMuJoCoPython\Scripts\python.exe Scripts\MuJoCo\export_unreal_bundle.py ^
       Saved\Cloud\athlete-athlete-tracking-1004-2157\results\agent\history\PPOJax_chunk055.pkl
   ```
   This writes `Saved/MuJoCo/unreal/`: the model, the policy, the motion database (~270 MB, 571k frames)
   and a golden session for the parity tests.
4. Build (`Scripts/Build.ps1`).

## Play

`Scripts/PlayLearnedAthlete.ps1` (the lab map with `AthleteTrackerGameMode`).

| Control | Keyboard | Gamepad |
|---|---|---|
| Forward (walk 1.5 m/s) | W | left stick up (analog) |
| Run (3 m/s) | hold Shift | hold right trigger |
| Stop | release W / S | release |
| Sidestep (0.5 m/s) | A / D | left stick left/right |
| Turn (1.2 rad/s) | Q / E | right stick |
| Reset | R | Y |

There's no walking backwards: the motion data has none. After a fall the athlete is reset after 2 s.
For demos and checks: `-AthleteDemo=1.5,0,0.4` drives it without a controller, and `-AthleteShotAt=6`
saves `Saved/Screenshots/LearnedAthlete.png` and quits.

## Tests (`Athlete.Tracker.*`)

| Test | Result (2026-10-05) |
|---|---|
| Parity.Policy: C++ network vs Python on 500 recorded observations | max action difference 1.6e-6 |
| Parity.Reset: start pose, queued reference, first observation | 1e-9 / 7e-8 |
| Parity.Session: the 5 s golden session | forced actions: path within 5e-6 for 0.5 s; C++ policy: ends **3 cm and 0.02 rad** from Python's athlete |
| Acceptance.Accelerate / Brake / Turn / TurnOnSpot / Run | **all 5/5 trials**, same numbers as Python (t90 1.5-2.0 s; stop 1.5-1.6 s; 3.1 rad; 4.0 rad; 2.6 m/s) |
| Acceptance.RandomCommands (200 episodes, at most 1 fall per 10) | **passes**: 11 falls (5.5 per 100), speed error 0.19 m/s (limit 0.4) |
| Study.FootballFallBar (400 episodes; football bar: at most 1 per 100) | 21 falls = 5.2 per 100: not met (reported, not asserted) |
| Study.PythonRandomSchedules (Python's exact 100 schedules) | 3 falls (Python, same schedules: 1) |
| Pawn.StandsAndWalksInAWorld | stands, walks 4.2 m in 4 s, real-time stepping, pelvis drawn 0.00 cm from MuJoCo's |

Speed: **0.55-0.6 ms per 10 ms control step** on one CPU core (MuJoCo + policy + matcher search over 571k
frames), about 17x real time.

### Why step-by-step parity holds only briefly

The closed loop is chaotic (contacts). In Python alone, 1e-6 of action noise moves the athlete 0.18 m
off its path within 0.5 s. So the session test checks exact agreement only at first, and for the whole
session checks the same behaviour: no fall, same end point. The acceptance tests compare behaviour.

### The random-command test

Fall rates per 20 s episode, chunk-55 tracker:

| | Episodes | Falls |
|---|---|---|
| Python, numpy schedules | 40 / 100 | 1 / 1 |
| C++, Python's exact 100 schedules | 100 | 3 |
| C++, its own schedules (same distribution) | 40 / 200 | 3 / 9 |

On identical schedules the two differ by 1 vs 3 in 100, within chance for a chaotic system (each episode
is effectively a random draw). Over 300 C++ episodes the rate is ~4%; Python's 1 in 100 is consistent
with ~1-4%. So the port behaves like Python, and the tracker falls in a few percent of 20 s random-command
episodes.

**The criterion (fixed 2026-10-07).** Milestone 4's goal is a rate, at most 1 fall per 10 episodes
(Milestone4_Locomotion.md 9.6). Both test suites checked "at most 1 fall" whatever the episode count:
right for Python's default 10 episodes, but four times too strict at 40. Both now check the rate. The C++
test runs 200 episodes (~4 min) because 10 are mostly luck: a tracker falling in 4% of episodes fails a
10-episode test 6% of the time. With the fix the test passes: 11 falls in 200.

**The football bar.** At most 1 fall per 100 episodes, about one fall per half hour of random play. It's
measured, not asserted, by Study.FootballFallBar (400 episodes). Chunk 55: 5.2 per 100, about 5x too many.

### LocoMuJoCo quirks reproduced on purpose

- **Reset:** MuJoCo's forward pass runs at the model's default pose; then the start state's qpos, qvel
  and the trajectory's body poses and velocities (xpos, xquat, cvel) are written in without another forward pass.
  So the first observation's site terms mix default-pose geometry with reference velocities.
- **After each step**, the observation uses MuJoCo's kinematics from the start of the last 2 ms substep
  (no forward pass after `mj_step`).

The policy was trained and tested with both, so the port keeps them.

## Limits and next steps

- The pawn is a lab tool: geometric shapes, a single athlete, the lab floor (MuJoCo's ground is an
  infinite plane at Z = 0).
- Several athletes in one MuJoCo world (contact between players) and a skinned mesh driven by the MuJoCo
  bodies are the next steps toward football.
- The bundle is ~270 MB (motion database as float32). It's fine for development, and can be compressed or
  indexed for a shipping build.
