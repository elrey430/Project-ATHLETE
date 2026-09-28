# Milestone 2: Physical Humanoid

![Passive bodies falling](images/milestone2_ragdolls_falling.png)
![Passive bodies settled](images/milestone2_ragdolls_settled.png)

*The three lab experiments at t ≈ 0.9 s and t = 4 s. Left: athlete A dropped from 1.5 m tilted 30°.
Middle: the reference male released standing. Right: athlete B pushed in the chest with 60 N·s. In the settled
shot, A lies about 2 m from where it landed. That's the impact energy-injection problem described below.*

## 1. Problem

Turn the computed body from Milestone 1 into a **simulated articulated body** and show that it behaves
correctly under gravity, contact and external forces. It's **passive**: no muscles and no balance yet
(that's Milestone 3), so a standing body is *supposed* to collapse.

## 2. Model

- **16 Chaos rigid bodies, one per segment.** Each body's mass, center of mass and principal inertia are
  written **directly** into the physics body from `FAthleteBodyModel` and verified to about 3×10⁻⁸ relative error.
- **15 ball joints** in a tree rooted at the pelvis (lower trunk), with hard anatomical limits from the athlete's
  `FAthleteMobilityProfile` (AAOS normal adult ranges by default).
- **Collision shapes sized from segment mass.** Capsules for the head and limbs, boxes for the trunk and feet.
  A heavier athlete of the same height really is wider.
- **Self-collision is on**, except for jointed pairs and pairs that overlap in the reference pose (arm against
  trunk, hand against thigh, thigh against thigh). That's 11 pairs, filtered with the same per-pair mechanism
  Unreal's skeletal-mesh ragdolls use.

### Sources

| Data | Source |
|---|---|
| Joint ranges of motion | AAOS (1965), *Joint Motion: Method of Measuring and Recording*, verified via goniometer.io's AAOS chart |
| Thoracolumbar totals (80/25/35/45°) | AAOS. **Not independently verified**; the lumbar/thoracic split is an ESTIMATE |
| Wrist-to-fingertip hand length (collision only) | de Leva (1996) Table 4, alternative hand row (187.9 mm vs 86.2 mm) |

## 3. Simplifications

| Real world | ATHLETE | Why acceptable now |
|---|---|---|
| Soft tissue, ligaments with gradual end-range stiffness | Hard joint limits | Passive milestone. Soft limits become part of the motor model |
| Knee has axial rotation when flexed | Knee is a hinge | Small effect for M3–5; revisit for cutting |
| Anatomical limits are combined, direction-dependent | Elliptical swing cone + separate twist, centered on each range's middle | Exact for single-plane motion |
| Segment shapes | Capsules and boxes | Contact only; mass properties don't depend on shapes |
| Arm touches trunk, thighs touch | Those pairs ignore each other | Standard physics-asset practice; a hand can pass through its own thigh |

## 4. Architecture

```
AthleteBody (pure data, grows)
  Anatomy/AthleteJoints       EAthleteJoint (15), topology, joint centers
  Anatomy/AthleteMobility     FAthleteJointRangeOfMotion, FAthleteMobilityProfile (AAOS defaults)
  UAthleteDefinition          + Mobility

AthletePhysics (new; depends on PhysicsCore + Chaos privately)
  Articulation/AthleteCollisionGeometry     mass-sized shapes (pure math)
  Articulation/AthleteJointSetup            anatomical joint frames and limit centers (pure math)
  Articulation/AthletePhysicalBodyComponent builds, owns and measures the physical body

ProjectAthlete
  Lab/AthleteLabRagdoll       Collapse / Push / Drop experiments with per-frame telemetry
```

`AthleteBody` stays free of physics-engine dependencies, so the athlete *definition* can be used by tools and
generators without Chaos.

## 5. Hidden physics modifiers found and removed

Unreal ships several stabilization "fudges" enabled by default. Each silently changes physical results.
ATHLETE turns them off (or fixes them) explicitly, per project principle 43:

| Engine default | What it does | ATHLETE |
|---|---|---|
| `bInertiaConditioning = true` (bodies) | Inflates solver inertia of long/thin jointed bodies; **reports the original inertia**, so it can't be seen | Off |
| `bEnableMassConditioning = true` (joints) | Non-physical mass/inertia scaling inside the joint solve | Off |
| `bEnableProjection = true` (joints) | Teleports bodies to hide joint error | Off |
| `bUseLinearJointSolver = true` | "Faster and less accurate" | Nonlinear solver |
| `MaxAngularVelocity = 3600°/s` | Clamps spin; measured losing angular momentum at impact; would cap a QB's throwing arm | 30,000°/s (numerical guard only) |
| Gyroscopic torque **off** | Missing rigid-body physics; angular momentum not conserved for spinning asymmetric bodies | On |
| `InertiaTensorScale` / `COMNudge` | Not what their names suggest: an "equivalent box" rescale, and a nudge that adds inertia along the offset axis | Not used; mass properties written directly |

## 6–7. Instrumentation and tests (10 new: 8 checks + 2 measurement studies)

- **Body measurements:** total mass, CoM, linear and angular momentum, kinetic energy, joint separation, exact
  lowest point, and joint angles decomposed like Chaos's own limits (swing cone + twist).
- **Lab telemetry:** `Ragdoll_<name>.csv` per experiment, every frame.

| Test | Proves |
|---|---|
| `Physics.Joints.TopologyIsATree` | 15 joints, one parent per segment, centers on shared endpoints |
| `Physics.Joints.FramesPointAnatomically` | Hip flexes forward, knee backward, abduction mirrored, ankle neutral is plantarflexed |
| `Physics.Geometry.ShapesMatchSegmentMass` | Shape volume × density = segment mass |
| `Physics.Body.MassPropertiesMatchModel` | Every body's mass, CoM and inertia match the model; no conditioning |
| `Physics.Body.FreeFallKeepsShapeAndFallsAtG` | Articulated free fall: CoM at g, no segment rotates, joints closed |
| `Physics.Body.ImpulseConservesMomentum` | Linear momentum = J exactly; angular momentum within 5 % |
| `Physics.Body.PassiveCollapseSettlesOnFloor` | Comes to rest, doesn't sink, joints hold, limits respected |
| `Physics.Body.CollapseIsDeterministic` | Two identical runs end bit-identical |
| `Physics.Study.*` (2) | Measurement studies (below); always pass, report numbers |

## 8. Measurements

- **Mass properties:** worst inertia error 3×10⁻⁸.
- **Determinism:** 0.00000000 cm difference between identical 2-second collapses.
- **Push:** B's CoM velocity right after a 60 N·s push was 0.55 m/s (J/M = 0.551).
- **Cost:** about 0.6 ms per athlete per 60 Hz frame at 240 Hz substeps.

**Angular-momentum accuracy** (chest impulse, zero gravity):

| Substep | Gyroscopic torque | Error after 1 frame | after 0.5 s | ms/frame |
|---|---|---|---|---|
| 1/240 | off (Chaos default) | 3.13 % | 3.64 % | 0.60 |
| 1/240 | **on** | **0.76 %** | 3.53 % | 0.61 |
| 1/480 | on | 0.29 % | 1.77 % | 0.95 |
| 1/960 | on | 0.13 % | 0.86 % | 1.78 |

With gyroscopic torque on, the error halves each time the step halves: **the simulation converges to correct
physics**. More solver iterations don't help this case.

**Violent impact: energy created by the solver** (1.5 m drop onto the floor, 86 kg body, tilts 15/30/45°; no
external work, so any rise in kinetic + potential energy is non-physical):

| Setup | Energy created (15° / 30° / 45°) | Deepest penetration | Worst joint gap | ms/frame |
|---|---|---|---|---|
| 240 Hz, default iterations (**current**) | 261 / 330 / 0 J | 5.3 cm | 13.3 mm | 0.6 |
| 240 Hz, 32/4 iterations | 224 / 33 / 0 J | 2.2 cm | 12.5 mm | 1.0 |
| 240 Hz, 64/8 iterations | 51 / 59 / 0 J | 1.9 cm | 9.8 mm | 1.3 |
| 960 Hz, default iterations | 0 / 9 / 0 J | 1.4 cm | 2.7 mm | 2.0 |

## 9. Findings and open decisions

1. **Violent contact is the top risk, now with numbers.** At the current 240 Hz, a hard impact can create up to
   ~330 J (about 16 % of the drop energy), visible as a body sliding ~2 m after landing. About 1 kHz physics
   fixes it, at roughly 2 ms per athlete-frame, which is ~44 ms for 22 athletes if cost scales linearly. **Decide
   in Milestone 6** (collision lab), with measurements: finer substeps, Chaos async fixed step, contact
   settings, or level of detail for athletes not in contact. Gentle scenarios (collapse, push) are clean at 240 Hz.
2. **The motor controller must run inside the physics step** (unchanged from Milestone 0; still open for Milestone 3).
3. **Mass properties are written directly to Chaos.** Anything that makes Unreal recompute a body's mass
   (scale, mass scale, welding) would overwrite them. The component never does; the MassProperties test would
   catch a regression.
4. The reference pose is now the **anatomical position (palms forward)**, which the AAOS ranges are measured
   from. Milestone 1's description said "palms inward"; hand orientation didn't affect Milestone 1's numbers.

## Next: Milestone 3 (Standing and Balance)

Keep this body upright with **active** control: joint torques computed each physics step, limited by the
athlete's strength, disturbable by a push, and attempting recovery.
