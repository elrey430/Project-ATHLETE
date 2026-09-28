# Milestone 3: Standing and Balance

![Standing lab at 1.5 s](images/milestone3_standing_start.png)
![Standing lab at 3.8 s](images/milestone3_standing_pushes.png)
![Standing lab at 6.5 s](images/milestone3_standing_end.png)

*The standing lab at t = 1.5 s, 3.8 s and 6.5 s (`athlete.Debug.Balance 1`, `athlete.Debug.CenterOfMass 1`).
Left to right: A standing quietly; the reference male pushed at the pelvis with 10 N·s at t = 3 s; B pushed with
20 N·s; the reference male pushed with 40 N·s; the reference male with no neural control. The first three stay up.
The 40 N·s push puts his extrapolated center of mass (cyan) beyond his toes, and he falls. Without neural control
he collapses within about 2 s. Magenta: center of mass and its floor projection. Yellow: center of the feet.*

## 1. Problem

Make the athlete **stand by himself** and **recover from a disturbance**, with no scripted stability:

- he stays upright because his own joint torques hold him up;
- a push affects him, and he recovers by moving his body;
- a big enough push knocks him over;
- nothing locks him upright, and nothing outside his body helps.

## 2. Model

Two layers, mirroring biology.

### Muscles (inside every physics substep)

Each joint gets a torque-limited spring-damper about a target orientation, computed and applied by a Chaos
**sim callback** (`FAthleteMuscleSimCallback`) on every internal physics step (240 Hz), not once per frame. The
torque goes on equal and opposite to the two segments, so muscles are internal forces and can't move the whole
body on their own.

| Quantity | Value | Source |
|---|---|---|
| Stiffness (each axis) | `MuscleToneGain` × (supported mass × g × lever), shared by both legs for leg joints | Standing load. Tone 2 = twice the load |
| Hip frontal stiffness | the whole body's sideways-sway load (same as the ankle's) | Closed loop in double stance (see §5.5) |
| Damping | 2 × `DampingRatio` × √(stiffness × supported inertia) | Critical damping of the supported load |
| Torque limit | peak torque of the joint's anti-gravity action | Harbo et al. 2012 regressions (Milestone 1). Trunk and neck extension are ESTIMATES (3.0 and 0.4 N·m/kg) |

### Neural control (once per frame, with delays)

- **Balance** (reaction time 0.1 s): the extrapolated center of mass (XCoM, Hof 2005), ξ = CoM + k·v/ω₀ with
  ω₀ = √(g/h), is compared with the center of the feet.
  - **Ankle strategy:** lean the whole body against the error.
  - **Hip strategy:** once the error passes half the half-foot length, bend at the hips toward the fall.
- **Posture reflexes** (latency 0.05 s): each segment is held at a desired orientation **in space**, not at a
  joint angle.
  - Every joint's target is recomputed from the *perceived* orientation of the segment it pushes against.
  - Leg joints turn the segment above them, taking the ground up as the base.
  - Spine and neck turn the segment above them, taking the pelvis up as the base.
  - Arms hold their joint angles.
- **Foot awareness:** a foot can only push the ground between its heel and toes. The ankle's correction is
  limited to 80% of the torque that would tip the foot onto its toes (72 N·m per foot for the reference male),
  heel (20 N·m) or side edge (17 N·m).
- **Knowing he's down:** once his perceived center of mass drops below 70% of its standing height above the
  ground (`FallenComHeightFraction`), the motor state becomes `Fallen`. Balance and posture control stop, and the
  muscles hold their reference joint angles with normal tone. Getting up is Milestone 7.

Motor skill (`FAthleteMotorSkill`, per athlete) holds every control parameter. None of them adds force from
outside the body.

| Parameter | Default | Meaning |
|---|---|---|
| `MuscleToneGain` | 2.0 | stiffness as a multiple of load |
| `DampingRatio` | 1.0 | fraction of critical damping |
| `AnkleStrategyGain` | 0.25 | balance correction strength |
| `VelocityFeedbackGain` | 2.0 | weight of sway speed vs position |
| `HipStrategyGain` | 1.0 | hip strategy strength |
| `ReactionTimeS` | 0.1 s | balance delay |
| `PostureReflexDelayS` | 0.05 s | segment-orientation reflex delay |

With tone 2, the defaults give about 0.3 m·g·h·s of neural sway damping and add about 0.45 m·g·h of stiffness.
That's the same order as human posturography (Peterka 2002: neural damping about 0.3 m·g·h·s).

## 3. Simplifications

| Real world | ATHLETE | Why acceptable now |
|---|---|---|
| Hill muscles: force-length, force-velocity, activation dynamics | Linear spring-damper with a torque cap | Standing is near-static. **Force-velocity is the first addition for Milestone 4** |
| A saturated muscle still damps (force-velocity) | A capped muscle loses its damping too, so it swings (see `TorqueLimitIsStrength`) | Only matters at strength limits |
| Different strength per direction (flexion vs extension) | One vector torque cap per joint, from the anti-gravity action | Per-direction limits come with Hill muscles |
| Plantar pressure, toes, soft heel pad | Rigid box feet | Balance limits are set by the foot's lever arms, which the box keeps |
| Stepping to recover | Feet stay in place | Stepping is Milestone 7 |
| Continuous nervous system | Neural loops update once per frame (60 Hz) with delays quantized to frames | Human loops are 20–100+ ms. Muscles still run at 240 Hz |

## 4. Architecture

New module **`AthleteMotor`** (depends on AthleteCore, AthleteBody, AthletePhysics and Chaos):

```
UAthleteMotorComponent (TG_PrePhysics, per frame)
  sense body -> sensing history -> delayed samples
    balance:  AthleteBalanceController::Compute          (XCoM -> lean/hip command)
    posture:  MakeSegmentTargets + SolveJointTargets      (desired segment orientations -> joint targets)
    motor state: Standing -> Fallen (center of mass below 70% of standing height; neural loops stop)
    -> push FAthleteMuscleInput
FAthleteMuscleSimCallback (physics thread, every substep)
  per joint: error, relative spin -> spring + damper (numerically capped) -> strength cap -> equal/opposite torques
  outputs per-substep torque and error telemetry
AthleteMuscleModel::ComputeStandingImpedance              (stiffness, damping, limit per joint and axis)
```

This closes the Milestone 0 open decision: **controllers apply torques inside every physics step** through a
Chaos sim callback. Async physics is not needed.

## 5. What we learned (the path to a standing athlete)

Each item below was a real failure, found by measurement.

### 5.1 Chaos joint drives can't hold a standing body

The first version used Chaos's built-in joint drives (force mode, with the hidden 1.5× drive scale divided out;
the calibration tests pass). A drive's effective stiffness depends on the mass ratio of the two bodies it connects,
so a light foot against a heavy body made the ankle yield. Muscles are now **explicit torques** in a sim callback:
the torque is exactly k·error, observable in telemetry, and ready for a Hill model.

### 5.2 Chaos sleep faked stability

At 32 solver iterations the body "stood perfectly still". It was **asleep**: Chaos froze it for 72 of 360 frames.
Athlete bodies now never sleep (`UAthletePhysicalBodyComponent::bCanSleep = false`), and the standing tests assert
zero sleeping frames. With sleep off, iteration count stopped mattering.

### 5.3 A stack of joints held at fixed angles falls over

With every joint holding its reference angle at twice its load, the body still collapsed in a slow "C" curve.
When every joint gives a little in the same direction, the top of the stack moves by the sum, and the gravity
stiffness matrix beats a diagonal muscle stiffness matrix. (Three stacked joints with loads 4, 2, 1 and tone 2:
the all-same-direction mode has net stiffness 4 − 2 − 3·1 < 0.)

People don't stand that way. They keep their trunk and head oriented in space. With **world-referenced posture
targets**, each joint only carries its own share and tone 2 is enough. The collapse without neural control is
kept as a test (`NoNeuralControlFalls`).

### 5.4 Explicit muscles need numerical limits

Muscle stiffness and damping are sized for the main load (the ankle holds up the whole body), but the same values
act about axes where a segment barely resists turning: a foot about its long axis, a forearm about its own axis.
There, one 240 Hz substep is too long.

- **Explicit damper:** overshoots once d·dt/I > 2. The forearm twisted ±60° within 4 frames.
- **Explicit spring:** unstable past k·dt²/I = 4. The foot rolled over after a push.

The spring and the damper are each capped, along the direction they push, at what a substep can represent,
using the segments' inertia about their own centers (the inertia Chaos integrates a torque with). Using the inertia
about the joint let the damper overshoot, and the trunk folded within a second. Capping per joint axis instead of
along the push direction made the feet flip after a recovered push.

**Effect:** by estimate, the ankle's sagittal stiffness is trimmed a few percent (to about 615 of 653 N·m/rad). Twist and small segments are
capped hard. The caps scale with 1/dt², so a higher substep rate (a Milestone 6 decision) relaxes them.

### 5.5 Balance lives in the feet and the hips

- **Delays:** balance with a 0.1 s delay fell even when posture control alone stood. Posture reflexes and balance
  decisions have different latencies (about 50 ms vs 100+ ms). Modeling them separately fixed it.
- **Gains:** with neural gain 1.0, the loop was too stiff for the delay. The defaults were retuned toward a
  damping-heavy loop: quiet sway 12 → 1.2 mm RMS.
- **Foot tipping:** pushes tipped the feet. The controller asked the ankles for more torque than a foot can react,
  so the heels lifted. With foot-aware limits, 15 N·s forward became recoverable.
- **Sideways:** the legs rolled over sideways like a collapsing parallelogram. In double stance, sideways balance
  works by the hips shifting load between the feet (hip load/unload mechanism, Winter 1995). The hips now hold
  the thigh-to-pelvis angle in the frontal plane, with a frontal stiffness sized for the whole body's sideways
  sway. Sideways recovery went from 2.5 to 12.5 N·s.

### 5.6 Starting transient

The reference pose has the center of mass over the ankles, about 7 cm behind the balance target (the center of
the feet). For about 2.5 s after release the athlete glides forward and settles. A 10 N·s push *during* that
transient can knock him over. Pushes in tests and in the lab come at t = 3 s, after quiet standing has settled,
as in a real perturbation experiment.

### 5.7 Down means down (found in the lab)

After the 40 N·s push the athlete fell and then **twitched violently on the ground**: 5.4 J of peak kinetic
energy 5 s after landing. Balance and posture control were still trying to hold each body part upright in space,
so every muscle ran at its strength limit against the floor, with targets flipping as the segments rolled. Now
the motor system recognizes it's down (perceived center of mass below 70% of standing height) and stops
balancing. After the same fall he comes to rest: 0.00 J in the tests and 0.001 J in the lab over the last 2 s.
The height rule is deliberate: a trunk-angle rule would trip during a legitimate hip strategy, which bends the
trunk up to 60°.

### 5.8 Tried and reverted: clamping targets to the range of motion

A lab push once failed with the knee muscle saturated against the knee's hyperextension limit. Clamping every muscle
target inside the joint's range of motion seemed principled, but measurement said otherwise:

- It fixed nothing. That failure was the start transient (§5.6).
- It made balance worse. Sideways recovery dropped from 12.5 to 10 N·s. The reaction-time study stopped declining
  smoothly: 0.15 s went from 12.5 to 5 N·s.

The clamp scales the whole swing target down when one axis passes its limit, so a hip target past extension also
lost its sideways part. A muscle pressing a straight knee into its stop is also part of how a locked knee is held.
Targets are no longer clamped.

## 6. Tests

| Test | Checks |
|---|---|
| `Athlete.Motor.Muscle.StiffnessIsRealTorque` | Arm droop matches k·δ = T·cos δ (25.05° measured vs 25.79° predicted) |
| `Athlete.Motor.Muscle.TorqueLimitIsStrength` | Time-averaged gravity torque equals the strength cap (5.229 vs 5.271 N·m). Reported torque equals the cap |
| `Athlete.Motor.Muscle.ReferencePoseIsRelaxed` | Weightless reference pose: every muscle error < 0.5°, torque < 0.5 N·m, body still |
| `Athlete.Motor.Muscle.StandingImpedance` | Ankle load = m·g·h/2; tone scales stiffness; hip frontal carries the sway load |
| `Athlete.Motor.BalanceLaw.*` | Leans against the fall (position and velocity); hip strategy only beyond the feet; reference body gets reference targets; ankle asks only what the foot can take |
| `Athlete.Motor.Balance.QuietStanding` | 10 s upright, sway < 5 mm RMS, feet planted, **never asleep**, never judged fallen |
| `Athlete.Motor.Balance.NoNeuralControlFalls` | Muscle tone alone must collapse (proves nothing else holds him) |
| `Athlete.Motor.Balance.SmallPushRecovered.*` | 10 N·s forward, 7.5 backward, 10 sideways, and 10 forward facing back or sideways: upright, back within 10 cm, feet planted, settled |
| `Athlete.Motor.Balance.LargePushFalls` | A 40 N·s push must knock him over (no faked balance); he knows he's down and comes to rest (peak kinetic energy in the last 2 s < 0.5 J) |
| `Athlete.Motor.Balance.Deterministic` | The same push twice gives identical results to the last digit |
| `Athlete.Motor.Study.PushThresholds` | Study below |

All 48 project tests pass. Standing costs about 0.7 ms per athlete-frame in the test worlds.

## 7. Study: how hard a push can he take?

Largest pelvis push (0.1 s, 2.5 N·s steps) recovered with feet in place. Same skill and same general-population
strength regression for every athlete: only the body differs.

| Athlete | Forward | Backward | Sideways |
|---|---|---|---|
| Reference (1.74 m, 73 kg) | 15.0 N·s (0.21 m/s) | 10.0 N·s (0.14 m/s) | 12.5 N·s (0.17 m/s) |
| A (5'9", 190 lb) | 17.5 N·s (0.20 m/s) | 12.5 N·s (0.15 m/s) | 12.5 N·s (0.15 m/s) |
| B (6'4", 240 lb) | 22.5 N·s (0.21 m/s) | 20.0 N·s (0.18 m/s) | 20.0 N·s (0.18 m/s) |

Reference, forward, vs reaction time: 0.05 s → 17.5 N·s, 0.10 s → 15.0, 0.15 s → 12.5, 0.20 s → 10.0.

**Findings (emergent, not scripted):**

- **Bigger athletes take bigger pushes, but not faster ones.** B absorbs 50% more impulse, yet the recoverable
  center-of-mass *velocity* is about the same (0.21 m/s). Mass buys impulse capacity; the velocity limit comes from
  foot length vs height, which scales with size. That's what XCoM theory predicts.
- **Backward is the weakest direction,** because the heel lever (5.4 cm) is much shorter than the toe lever. That
  matches human data.
- **Reaction time costs balance:** each extra 50 ms costs about 2.5 N·s.
- **The absolute limits are low** compared with healthy adults' feet-in-place limits (about 0.3–0.4 m/s). The
  likely reasons are rigid box feet with no toes, frame-quantized 100 ms neural delays, a linear muscle model, and
  a hip strategy that is still crude. Stepping (Milestone 7) is how people really handle bigger pushes.

## 8. Lab

`AAthleteLabStanding` (5 instances, 8 m past the start line, generated by `build_athlete_lab.py`). Scenarios:
`QuietStanding`, `Push` (pelvis, `PushImpulseNs` in the athlete's frame, at `PushTimeS`), `NoNeuralControl`.
Telemetry is `Standing_*.csv`: CoM, XCoM error, balance command, ankle/knee/hip/lumbar muscle torques and
kinetic energy, plus a `fallen` flag, every frame. Debug: `athlete.Debug.Balance 1`.

## 9. Open risks and debt

| Item | Revisit |
|---|---|
| Numerical caps soften the stiffest muscles at 240 Hz | Substep-rate decision, Milestone 6 |
| Saturated muscles lose damping; no force-velocity relation | Hill-type muscle, Milestone 4 |
| Neural loops quantized to frame rate: balance timing changes with frame rate | Move neural loops into the sim callback with fixed-rate delays, before Milestone 12 scale tests |
| Recovery limits below human values | Feet with toes/soft heel, better hip strategy; stepping in Milestone 7 |
| Balance target is the center of the feet; the reference pose isn't balanced (2.5 s start transient) | Start from a balanced pose when athletes spawn in plays |
| Trunk/neck strength constants are ESTIMATES | Replace with measured data when available |
