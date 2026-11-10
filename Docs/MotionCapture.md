# Our own motion capture (recorded phone video)

Set up 2026-10-10. Recorded video of our own athletes → body landmarks (MediaPipe Pose) → later, 3D motion
fitted to the athlete for training. Why: the licensed data has no good examples of the moves the tracker
falls on (hard stops from a run, sharp cuts; Milestone4_Locomotion.md §9.6), and none of football's
(MotionData.md, MotionDataLicenses.md). Footage we film ourselves, with the people's consent, is ours to
train on.

## Pipeline

| Step | Status |
|---|---|
| 1. Record (phones), this guide | ready |
| 2. Landmarks per video: `Scripts/Capture/pose_video.py` (MediaPipe Pose, Apache 2.0) | ready |
| 3. Camera calibration + sync, triangulation to 3D (2+ phones) | next |
| 4. Fit to the athlete's skeleton (`retarget_motion.py`, a new landmark source) | later |
| 5. Into the motion set, train | later |

MediaPipe Pose gives 33 landmarks per frame, including **heels and toes** (MoveNet, the alternative, stops at
the ankles: no foot orientation, which the tracker's footwork needs). From one camera its 3D is a guess,
rough for the feet; two or more synchronized cameras give real 3D by triangulation (step 3).

Setup (once): `powershell -ExecutionPolicy Bypass -File Scripts\SetupCapturePython.ps1` (environment in
`Intermediate/CapturePython`, model in `Saved/Capture/models`). CPU only on this PC (AMD GPU): about
15-30 frames per second of processing.

**App:** the desktop shortcut **ATHLETE Capture** (made by the setup script; `Scripts/Capture/capture_app.py`).
Add videos, press Process, then "Play skeleton video" to check the tracking. "Camera check" shows a webcam
live with the skeleton drawn, to check framing and whether both feet are found.

From the command line instead:
```
Intermediate\CapturePython\Scripts\python.exe Scripts\Capture\pose_video.py Saved\Capture\videos\<session>\*.mp4 --overlay
```
Writes `Saved/Capture/poses/<video>.npz` (format in the script's header) and, with `--overlay`, a copy of
the video with the detected skeleton drawn (heel/toe points in red): watch it to check the tracking.

## Recording guide

**Cameras (phones)**
- **Two phones or more**, on tripods or fixed, about **60-90° apart**, both seeing the whole movement area.
  One phone works for trying things out; 3D needs two.
- **60 fps at least; 120 fps for sprints and cuts** (1080p is enough). Same settings on every phone.
- **Landscape.** No zooming or moving the phone during a session. **Turn off video stabilization** if the
  phone allows it (it warps the picture, which breaks calibration).
- Phone at about **hip height**, far enough that the **whole body, feet included, stays in frame** for the
  whole move, with margin above the head and below the feet.
- Even light, no strong backlight (don't film towards a window or the sun).

**Each session**
- **Calibration clip per phone** (for step 3; can be used later, but film it now, before moving the
  phones): a printed checkerboard (OpenCV 9x6 inner corners; measure one square in mm and write it down)
  waved slowly in front of each phone, tilted at different angles, covering the frame; then held where
  **all phones see it at once** for a few seconds.
- **Sync:** at the start of every take, a sharp **clap** or a jump landing, visible and audible to all
  phones.
- **Start and end each take standing still** for ~2 s (also gives the body proportions).
- Write down the athlete's **height**, and which phone filmed from where.

**The athlete**
- Fitted clothing that contrasts with the background, **shoes on**; one person in frame (others out of
  the picture).
- Consent: anyone filmed agrees that the footage is used to train the game's motion. The footage stays
  on this PC (`Saved/` is never committed).

**Moves (priority: what the tracker falls on, then football)** — each 5-10 times, **to both sides**:
1. **Hard stops from a run**: jog/run (2-3 m/s, then faster) to a full stop; also stopping **while
   turning** (the most common fall).
2. **Cuts at speed**: plant and change direction 45° and 90°, left and right.
3. Turn on the spot, both directions; run-turns (curving runs).
4. Accelerations from standing to a sprint; decelerations from a sprint.
5. Football: backpedal, shuffle (side steps without crossing feet), backpedal-to-sprint transitions
   (targets: Docs/MovementTargets.md).

Sprints need room: a straight line long enough to accelerate (20-30 m), with the phones placed so the
athlete stays in view at the fast part (side-on, farther back).

**Files:** `Saved/Capture/videos/<date>_<place>/<phone>_<take>_<move>.mp4`, for example
`2026-10-12_park/phoneA_03_stop-turn-left.mp4`.
