# Milestone 1: Physical Athlete Definition

![Athlete A, the reference male, and Athlete B in the lab](images/milestone1_body_previews.png)

*A (5'9" 190 lb), the de Leva reference male (1.74 m, 73 kg), and B (6'4" 240 lb). Capsule
thickness reflects segment mass; yellow = segment centers of mass; magenta = whole-body center of
mass with its projection to the floor. Blue = athlete's left, red = right.*

## 1. Problem

Give every athlete a physical body whose mass, center of mass, and rotational inertia are
**computed from morphology**, so that later milestones can show agility, momentum, and balance
emerging from physics instead of from ratings or archetype rules.

## 2. Model

A **16-segment rigid-body model** in a standing reference pose:

| Segment | Endpoints (de Leva 1996) |
|---|---|
| Head (incl. neck) | vertex → cervicale (C7) |
| Upper trunk | cervicale → xiphion |
| Middle trunk | xiphion → omphalion (navel) |
| Lower trunk (pelvis region) | omphalion → mid-hip joint centers |
| Upper arm, forearm, hand ×2 | shoulder JC → elbow JC → wrist JC → 3rd metacarpal |
| Thigh, shank ×2 | hip JC → knee JC → ankle JC |
| Foot ×2 | heel → toe tip |

For each segment: length, mass, center of mass (CoM), and principal moments of inertia
(I = m·(r·L)² with de Leva's radii of gyration r). The whole-body CoM is the mass-weighted mean;
whole-body inertia comes from the **parallel-axis theorem**.

**Inputs** (`FAthleteMorphology`): stature, mass, age; proportion scales (legs, arms, feet,
shoulder width, hip width); regional mass scales (head/neck, trunk, arms, legs).

**Physical capability** (`FAthleteStrengthProfile`): peak joint torque for 10 joint actions,
*with the test condition it was measured under* (isometric, or isokinetic at N °/s).

### Sources

| Data | Source | Verified how |
|---|---|---|
| Segment mass %, CoM %, radii of gyration, reference lengths | de Leva P. (1996) *J Biomech* 29:1223–1230, Table 4 (males) | Read from the paper's own table; cross-checked against Visual3D's documentation of the same parameters |
| Ankle height 0.039 H, hanging-wrist height 0.485 H | Winter (2009) *Biomechanics and Motor Control of Human Movement* Fig. 4.1, after Drillis & Contini (1966) | Standard textbook values |
| Strength baseline | Harbo, Brincks & Andersen (2012) *Eur J Appl Physiol* 112:267–275, male regressions (Tables 1–2) | Coefficients read from the paper; worked example in a test |

Two findings from verification worth remembering:

- de Leva gives **two shank definitions**: knee → lateral malleolus (CM 44.59 %) and knee → ankle
  joint center (CM 43.95 %). Many secondary sources mix them up. ATHLETE uses the ankle-joint-center
  row because the ragdoll will rotate about joint centers.
- de Leva's "sagittal" radius means rotation about the **front-to-back** axis. The paper doesn't say
  so explicitly; it's inferred from the data (wide segments have the larger sagittal radius).

## 3. Simplifications

| Real world | ATHLETE approximation | Why acceptable now |
|---|---|---|
| Continuous, deformable body | 16 rigid segments | Standard in biomechanics; matches what a ragdoll can simulate |
| Every person's proportions differ | de Leva mean proportions, adjustable with scales | Scales cover the variation that matters most for football (leg/arm length, widths, mass distribution) |
| Mean segment lengths don't stack to mean stature | Head-to-ankle chain shrunk uniformly by **2.1 %** so ankle height + chain = stature exactly | Keeps de Leva's relative proportions; knee and hip heights then match Winter within 0.2 % and 0.6 % of stature |
| Segments are 3D, off-axis | All joint centers in one frontal plane; the foot lies along the floor | Foot CoM error ≈ 3 cm; effect on whole-body CoM ≈ 1 mm |
| Longer segment → more mass (or not) | A segment longer than neutral keeps its cross-section: mass weight ∝ length, then renormalized | Uniform scaling leaves mass fractions unchanged (as de Leva); longer legs at equal height and weight carry more mass |
| Trunk radii depend on width, not length | Radii of gyration scale with each segment's own length | Width scales currently move joints only. Revisit if trunk inertia proves important in contact (M6) |
| Strength varies with angle and speed | One peak torque per action, with its test condition recorded | Nothing consumes strength yet; Milestones 3–4 build the torque-angle-velocity model |
| Football players ≠ general population | Harbo et al. baseline is **general population** (ages 15–83) | It's a sourced, size-aware starting point that authors raise per athlete; calibrated in M3–M5 |

### Estimates (not sourced; calibration candidates)

All in `AthleteAnthropometry.h`, never inline in model code:

| Constant | Value | Effect |
|---|---|---|
| Hip joint center half-separation | 0.0517 × stature (0.18 m apart at 1.741 m; from Bell et al. 1990 with an assumed 0.25 m inter-ASIS width) | Leg lateral spacing; roll and yaw inertia |
| Shoulder joint center half-separation | 0.100 × stature | Arm placement; **noticeably affects yaw inertia** |
| Heel to ankle joint center | 0.22 × foot length | Foot position; negligible for inertia |

## 4. Architecture

```
AthleteBody (new Runtime module; depends on AthleteCore)
  Anatomy/AthleteSegments        EAthleteSegment (16), kinds, regions, sides, mirror
  Anatomy/AthleteAnthropometry   de Leva table, landmark ratios, estimates, derived constants
  Anatomy/AthleteMorphology      FAthleteMorphology  (USTRUCT: the athlete's body inputs)
  Anatomy/AthleteInertiaTensor   symmetric 3x3 tensor + parallel-axis term
  Anatomy/AthleteBodyModel       FAthleteBodyModel::Build(morphology) → segments + totals
  Capability/AthleteStrength     FAthleteStrengthProfile + Harbo 2012 baseline
  Definition/AthleteDefinition   UAthleteDefinition (Data Asset: identity + morphology + strength)

ProjectAthlete
  Lab/AthleteLabBodyPreview      draws a body; records it to telemetry on BeginPlay
```

`FAthleteBodyModel` is plain C++ (no UObject): cheap, thread-safe to build, and trivial to test.
It's the data Milestone 2 turns into physics bodies and joints.

## 5–6. Implementation and instrumentation

- **Debug:** `athlete.Debug.Anatomy 0/1` (on by default) toggles body previews.
- **Telemetry:** each preview writes `Body_<Asset>_Segments.csv` (per-segment length, mass, CoM,
  inertia) and `Body_<Asset>_Summary.csv` (whole-body values plus strength, NaN = unspecified).
  `session.csv` holds the `segment_ids` legend.
- **Athlete assets:** `/Game/Athletes/DA_Athlete_Reference`, `DA_Athlete_A`, `DA_Athlete_B`, created
  reproducibly by `Scripts/Editor/create_sample_athletes.py`.

## 7. Tests (13 new, all passing)

| Test | What it proves |
|---|---|
| `Anthropometry.MassFractionsSumTo100` | Table transcription: 16 segments = 100.00 % of mass |
| `Anthropometry.ReferenceLandmarksMatchWinter` | Knee and hip heights agree with an **independent** source |
| `Model.ReferenceBodyTotals` | Masses sum exactly; CoM on midline; CoM height in the textbook range |
| `Model.StatureScalingLaw` | Lengths ∝ s, CoM ∝ s, inertia ∝ s² **exactly**. No hidden size rules |
| `Model.MassScalingLaw` | Inertia ∝ mass exactly; CoM unchanged |
| `Model.LeftRightSymmetry` | Mirror segments identical; no left-right coupling in the tensor |
| `Model.LongerLegsPreserveStatureAndRaiseCom` | Proportions change without changing stature or mass |
| `Model.RegionMassScaleRedistributes` | Regional mass scales renormalize exactly |
| `Model.InvalidMorphologyRejected` | Impossible bodies (zero height, legs longer than the body, NaN) refused with a reason |
| `Model.InertiaTensorMath` | Parallel-axis term against the textbook dumbbell |
| `Model.MorphologyExperimentPreview` | A vs B yaw inertia ratio = (mB/mA)(hB/hA)² exactly |
| `Strength.GeneralPopulationBaseline` | Harbo regression reproduces hand-computed values |
| `Lab.BodyPreview.DrawsWhenTicked` | The preview actually submits debug geometry when it ticks |

## 8. Measurements

| | Reference male | Athlete A | Athlete B |
|---|---|---|---|
| Stature / mass | 1.741 m / 73.0 kg | 1.753 m / 86.2 kg | 1.930 m / 108.9 kg |
| CoM height | 0.978 m (**56.19 %**) | 0.985 m | 1.085 m |
| Inertia about CoM: roll / pitch / yaw (kg·m²) | 11.96 / 11.36 / 0.905 | 14.31 / 13.59 / 1.083 | 21.93 / 20.82 / 1.660 |
| Knee extension baseline (isometric) | 261 N·m | 289 N·m (3.35 N·m/kg) | 357 N·m (3.28 N·m/kg) |

- Knee height 0.2866 H (Winter: 0.285 H). Hip joint height 0.5241 H (Winter trochanter: 0.530 H).
- **B vs A:** 26 % heavier, but **53 % more yaw inertia**. With the same proportions, rotating B at
  the same angular acceleration needs 53 % more torque. This follows from I ∝ m·L²; nothing in the
  code says "big athletes turn slower".
- Longer legs (×1.08) at the same height and weight: CoM rises from 56.19 % to 57.24 % of stature,
  leg mass 36.9 → 40.1 kg.

## 9. What's next

Milestone 2 (Physical Humanoid) builds Chaos rigid bodies and joint constraints from
`FAthleteBodyModel`: body masses and inertias from this model, collision geometry, joint ranges of
motion, and the substepping vs async-physics decision. The estimates above, especially shoulder
width, get calibrated in Milestone 5.

## Known issue (tooling only)

Editor sessions driven by `-ExecutePythonScript` with `set_keep_python_script_alive(True)` exit with
an access violation (0xC0000005) roughly half the time. This reproduced with the VS plugin disabled
and with none of ATHLETE's actors loaded. Normal editor open and quit, all commandlets
(tests, BuildLab), and RunLab exited cleanly in every run. Not proven unrelated to our modules
without a debugger dump, so it's recorded here. Screenshot capture tip: `HighResShot` re-renders and
misses one-frame debug drawing; use `Shot` during Play or Simulate.
