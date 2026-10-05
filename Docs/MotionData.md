# Motion data (route 2: better data for learned locomotion)

Prepared and imported 2026-10-04 (100STYLE downloaded with the user's approval; §6). Each further download needs the user's approval.

## 1. Why

The random-command test still falls 4-5 times in 20 episodes (target ≤ 2). The falls come at
walk-to-run transitions (Milestone4_Locomotion.md §9.6). The current mocap walks at 1-1.5 m/s and runs at
2.5-3 m/s, with **nothing between** and nothing above 3 m/s. Its running turns go up to only ~0.9 rad/s,
and it has no accelerations from a walk into a run. Asked for 2 m/s, the motion matcher jumps from a walk
straight into a full run, and the athlete falls trying to catch the reference. Playing existing clips
faster or slower filled the gap on paper, but made the tracker worse (it never trained on that motion).

What the data must add, in order:
1. Accelerations and decelerations between walking, jogging and running (walk → run, run → stop).
2. Jogging at 1.5-2.5 m/s.
3. Running turns and sidesteps at speed.
4. Later, for football: sprinting (6-9 m/s), cuts, backpedalling. No open dataset has these with a
   usable license (see §3). That needs our own capture.

## 2. Candidate sources

| Source | License | What it has | Format | Size |
|---|---|---|---|---|
| **100STYLE** (Mason, Starke, Komura 2022) | **CC BY 4.0**: commercial use OK with credit and a note of changes | 100 styles × forward/backward/sidestep walking and running, idling, and **transitions** (218 min: the walk↔run accelerations we lack); 115 min of forward running, 70 min of sidestep running | BVH, 60 fps, 28 bones; `Frame_Cuts.csv` trims T-poses, `Dataset_List.csv` lists styles | 1.5 GB (`100STYLE.zip`, raw BVH); the 14.8 GB processed set isn't needed |
| **CMU Graphics Lab** | Free to use, including **inside commercial products**; may not be resold as data, even converted | 2,500+ clips incl. walking, jogging, running, turns, starts and stops, some sports | ASF/AMC (BVH conversions exist), 120 fps | per subject, small |
| LAFAN1 (Ubisoft) | CC BY-NC-ND 4.0: **no commercial use** | good locomotion | BVH | — |
| AMASS, Bandai Namco, OpenBiomechanics, BOXRR-23 | non-commercial | — | — | — |
| Animation extracted from commercial games (e.g. the EA CFB27/Madden clips in the user's Codex outputs, 2026-10-04) | the publisher's: **not usable** - a policy trained on them is a derivative of their assets | backpedal, shuffles, QB scramble sprints (12 clips, ~22 s) | decoded FBX/JSON | — |
| LocoMuJoCo default mocap (current) | not stated: prototyping only | walk, run, turns | npz | — |

**Recommendation: 100STYLE first.** Its transitions are exactly the missing piece, and its license
allows shipping. Use only the plainer styles (e.g. "Neutral" and a handful of athletic-looking ones):
most of the 100 are deliberately stylised (old, zombie, ...), which isn't athletic motion. Pick them by
looking at the clips. **CMU second**, if running turns or higher speeds are still short after 100STYLE.
That needs an ASF/AMC reader (or the BVH conversion).

Risk: both were captured in studio volumes, so straight-line top speed is limited (likely < 4-5 m/s).
Football sprinting needs our own capture (Docs: video-as-source research, memory 2026-09-29).

## 3. Attribution (CC BY 4.0, required when shipping)

> 100STYLE dataset by Ian Mason, Sebastian Starke and Taku Komura (2022), "Real-Time Style Modelling of
> Human Locomotion via Feature-Wise Transformations and Local Motion Phases", licensed under CC BY 4.0
> (https://creativecommons.org/licenses/by/4.0/). Modified: retargeted to the ATHLETE body, trimmed,
> mirrored, resampled.

Source: https://www.ianxmason.com/100style/ (data: Zenodo record 8127870, DOI 10.5281/zenodo.8127870).

## 4. Pipeline (ready, tested without downloaded data)

- **`Scripts/MuJoCo/bvh.py`** reads BVH files (any channel order; Y up and centimetres converted to
  MuJoCo's Z up and metres) and runs forward kinematics. It can also write BVH files.
- **`retarget_motion.py --source bvh --files ... [--prefix 100style_] [--frames START END]`** fits BVH clips
  to the athlete with the same landmark method as the LocoMuJoCo clips. Joints are found by common names
  (`LeftFoot`/`LeftAnkle`, `LeftToeBase`/`LeftToe`, `LeftHand`/`LeftWrist`, `Head`). Shoulders, hips and
  knees are found as parents of the elbows, knees and ankles, so it works whether "LeftShoulder" means the
  upper arm or the collarbone. Units are guessed from leg length (or `--scale`). The leg scale comes from
  thigh + shank length, not hip height. Clips are written to `DEFAULT/mocap/AthleteReference/<prefix><name>.npz`,
  next to the current ones, so the matcher (`CLIPS`) and the training config (task list) can use them by name.
- **Self-test, `check_bvh_retarget.py`:** exports one of our fitted clips as a library-style BVH (Y up,
  cm, standard names), retargets it and compares. Run 6 s: speed 2.829 vs 2.829 m/s, landmarks within
  2 mm, median joint error 0.15° (worst 2.7°, ankle), root within 2.8 cm. Walkturn: root within 7 mm.
  PASS on both.
- The LocoMuJoCo path is unchanged (bit-identical output).

### Running had no flight phase (found while preparing this)

The retargeter put a foot on the floor **in every frame** ("walking always has a foot down"). The source
run has both feet > 4 cm up in 40% of frames, but our fitted run had none, and its pelvis bobbed 5 cm
instead of ~12 cm. The reference the tracker learned to run from was physically wrong.
- New `--floor clip` mode: one floor height per clip, from the stance frames. Same run: 58% of frames
  with both feet > 2 cm up, as in the source. **Default for BVH clips.**
- LocoMuJoCo clips stay on `--floor frame` until we decide, because switching changes the current
  training data (it should be switched with the next training run).
- Each clip's report now records `floor` and `share_of_frames_both_feet_above_2cm`.

## 5. Steps once approved

1. Download `100STYLE.zip` (1.5 GB, Zenodo) to `Saved/MotionData/100STYLE/` (git-ignored, 234 GB free).
2. Read `Dataset_List.csv` / `Frame_Cuts.csv`. Check the BVH joint names against `BVH_NAMES` and the
   frame rate.
3. Choose styles: Neutral plus a few plain ones, after looking at them. Movement types: forward run (FR),
   sidestep run (SR), forward walk (FW), and transitions (TR1-3).
4. Retarget (trimmed by Frame_Cuts, `--floor clip`), mirror (`mirror_motion.py`), and check each report:
   fit error, foot slide, joint limits, flight share. Then check them visually.
5. Coverage: the speed × turn-rate table (`Scripts/MuJoCo/motion_coverage.py`) should fill
   1.5-2.5 m/s and show walk→run accelerations.
6. Add the clips to the matcher `CLIPS` and the training task list. Keep the training set within
   ~760k frames (32 GB VM RAM).
7. Training run (~2 h on-demand, ~$2; needs the user's go). Fresh synthetic matcher clips come from the
   new database. Re-run the random-command test, push test and fall diagnosis.
8. Decide on `--floor clip` for the LocoMuJoCo clips in the same run.

## 6. Import results (2026-10-04)

`Saved/MotionData/100STYLE/` (1.47 GB zip, 3.2 GB unpacked; git-ignored). 60 fps BVH, joints
`Hips, Chest..Chest4, Neck, Head, L/R Collar, Shoulder, Elbow, Wrist, Hip, Knee, Ankle, Toe`: found by
`BVH_NAMES` without changes.

**Speeds are lower than hoped.** The capture volume was small. Forward runs in most styles: median
1.3-1.6 m/s, p90 1.6-2.1. The fastest plain styles are Rushed (p90 2.73, max 3.44), Angry (arms pumping;
p90 2.70) and BigSteps (p90 2.28). Across all 100 styles there are only ~217 s above 2 m/s.

**Chosen:** Neutral, Rushed, Angry, BigSteps, Followed, CrowdAvoidance, Proud, each with FR (forward run),
SR (sidestep run) and TR1 (transitions). That's 21 clips, 1,410 s, plus mirrors (`import_100style.py`,
~1.5 min on 16 cores).

**Fitting problems found and fixed (BVH sources only; the LocoMuJoCo path is unchanged):**
- The ankles sat at the dorsiflexion limit in ~50% of frames: "flat" was calibrated on push-off frames
  (toe down, heel up). Now the back of the foot must be down too, with relaxed thresholds for forefoot
  runners. Result: 3-5% of frames at the limit on walking transitions, as in the existing clips.
- One arm was locked in a wrong IK solution for whole clips (shoulder at its limit in up to 100% of
  frames, wrist 20-35 cm off). Now a frame with a landmark > 5 cm off is re-solved from the neutral pose
  and the better fit is kept. Every clip now has its worst landmark p95 ≤ 4.5 cm.
- Ankle abduction (side tilt) still reaches its ±10° limit often, mostly in swing: stance foot roll is
  -1.3 to -1.6° (median), p90 4-5° (existing clips: 2-3°).

**Matcher database coverage** (seconds; `motion_coverage.py`): 1.5-2.0 m/s 13 → 314; 2.0-2.5 m/s
0 → 116; running turns > 0.9 rad/s at 1.5-2.5 m/s now exist.

**Kinematic effect (matcher only, no physics):**
- Walk → run commands (0.8 → 2.2 / 2.5 m/s): the reference's 95th-percentile acceleration ~16 → 8-10 m/s²,
  i.e. the jolts that knocked the athlete over roughly halve.
- Random commands: acceleration p95/p99 9.6/16.9 → 8.1/14.4 m/s²; speed error 0.259 → 0.222 m/s;
  jumps 3.5 → 2.7/s.

**The current trackers can't use it without retraining:** runs 7 and 8 on the new database fall in 20/20
random episodes and fail acceleration too. As with the time-warp test, they only follow motion they
trained on. So the database switch is tied to the tracker: `ATHLETE_MOTION_SET=100style` (default
`base`).

**Training run 9 (done 2026-10-04; results in Milestone4_Locomotion.md §9.6):**
- `run.sh` imports the staged BVH (`Saved/MotionData/100STYLE/100style_bvh`, 73 MB upload) on the VM,
  mirrors it, and sets `ATHLETE_MOTION_SET=100style`, so the generated matcher clips include it.
- The config adds the 21 clips once each and drops walk_mirror. Matcher clips are 170 s each
  (memory: ~704k frames vs the ~760k limit).
- LocoMuJoCo's loader finds the clips by name (tested).
