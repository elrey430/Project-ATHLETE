"""
Project ATHLETE - body landmarks from recorded video (MediaPipe Pose, Google, Apache 2.0).

For each video: 33 landmarks per frame (MediaPipe's BlazePose set: face, arms, hips, knees, ankles, HEELS and
TOES), both in the image and as rough 3D, saved to Saved/Capture/poses/<video name>.npz:
  time_s          (frames,)          the frame's timestamp in the video (phones record at variable frame rates)
  image           (frames, 33, 5)    x, y in pixels (of the upright image), z (MediaPipe's relative depth,
                                     same scale as x), visibility, presence; NaN where no person was found
  world           (frames, 33, 3)    metres, origin between the hips, MediaPipe's camera-aligned axes
                                     (x right, y down, z away from the camera); single-view estimate
  detected        (frames,)          a person was found
  names           (33,)              landmark names (NOSE ... RIGHT_FOOT_INDEX)
  width, height, fps_nominal, video
The world landmarks are one camera's guess of depth: rough, especially for the feet. True 3D comes from two
or more synchronized views (triangulation, next step); Docs/MotionCapture.md has the recording guide.

Usage (Intermediate/CapturePython, from Scripts/SetupCapturePython.ps1):
  python Scripts/Capture/pose_video.py VIDEO [VIDEO ...] [--overlay] [--out DIR]
  --overlay   also writes <name>_overlay.mp4 with the detected skeleton drawn, to check the tracking by eye
"""

import argparse
import sys
import time
from pathlib import Path

import cv2
import mediapipe as mp
import numpy as np
from mediapipe.tasks.python import BaseOptions, vision

PROJECT_ROOT = Path(__file__).resolve().parents[2]
MODEL = PROJECT_ROOT / "Saved" / "Capture" / "models" / "pose_landmarker_heavy.task"
OUT_DIR = PROJECT_ROOT / "Saved" / "Capture" / "poses"
NAMES = [landmark.name for landmark in vision.PoseLandmark]
CONNECTIONS = [(c.start, c.end) for c in vision.PoseLandmarksConnections.POSE_LANDMARKS]
FOOT = [NAMES.index(n) for n in ("LEFT_HEEL", "RIGHT_HEEL", "LEFT_FOOT_INDEX", "RIGHT_FOOT_INDEX")]


def process(video, out_dir, overlay, progress=None):
    """Landmarks for one video; returns a one-line summary. progress(done, total) is called as it goes."""
    video = Path(video)
    capture = cv2.VideoCapture(str(video))
    if not capture.isOpened():
        raise RuntimeError(f"cannot open {video}")
    fps = capture.get(cv2.CAP_PROP_FPS) or 30.0
    total = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    options = vision.PoseLandmarkerOptions(base_options=BaseOptions(model_asset_path=str(MODEL)),
                                           running_mode=vision.RunningMode.VIDEO, num_poses=1)
    landmarker = vision.PoseLandmarker.create_from_options(options)
    times, image, world, detected = [], [], [], []
    writer, last_ms, started = None, -1, time.time()
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        t = capture.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
        if t <= 0 and times:  # some containers report no timestamps: fall back to the nominal rate
            t = len(times) / fps
        # MediaPipe's video mode needs strictly increasing whole milliseconds.
        ms = max(int(round(t * 1000)), last_ms + 1)
        last_ms = ms
        height, width = frame.shape[:2]
        result = landmarker.detect_for_video(mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)), ms)
        found = len(result.pose_landmarks) > 0
        points = np.full((33, 5), np.nan)
        metres = np.full((33, 3), np.nan)
        if found:
            points[:] = [(p.x * width, p.y * height, p.z * width, p.visibility, p.presence) for p in result.pose_landmarks[0]]
            metres[:] = [(p.x, p.y, p.z) for p in result.pose_world_landmarks[0]]
        times.append(t); image.append(points); world.append(metres); detected.append(found)
        if overlay:
            if writer is None:
                target = Path(out_dir) / f"{Path(video).stem}_overlay.mp4"
                writer = cv2.VideoWriter(str(target), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height))
            if found:
                for a, b in CONNECTIONS:
                    if min(points[a, 3], points[b, 3]) > 0.5:
                        cv2.line(frame, tuple(int(v) for v in points[a, :2]), tuple(int(v) for v in points[b, :2]), (0, 255, 0), 2)
                for i in range(33):
                    if points[i, 3] > 0.5:
                        cv2.circle(frame, tuple(int(v) for v in points[i, :2]), 4, (0, 0, 255) if i in FOOT else (255, 255, 0), -1)
            writer.write(frame)
        if progress is not None and len(times) % 15 == 0:
            progress(len(times), total)
        elif progress is None and len(times) % 300 == 0:
            print(f"  {video.name}: {len(times)}/{total or '?'} frames", flush=True)
    capture.release()
    if writer is not None:
        writer.release()
    landmarker.close()
    if not times:
        raise RuntimeError(f"{video}: no frames read")
    image, world, detected, times = np.array(image), np.array(world), np.array(detected), np.array(times)
    target = Path(out_dir) / f"{Path(video).stem}.npz"
    np.savez_compressed(target, time_s=times, image=image, world=world, detected=detected, names=np.array(NAMES),
                        width=width, height=height, fps_nominal=fps, video=str(video))
    seconds = time.time() - started
    feet = np.nanmean(image[detected][:, FOOT, 3]) if detected.any() else float("nan")
    rate = (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 and times[-1] > times[0] else float("nan")
    summary = (f"{video.name}: {len(times)} frames, {times[-1] - times[0]:.1f} s at {rate:.1f} fps (nominal {fps:.1f}), "
               f"{width}x{height}; person found in {100 * detected.mean():.0f}% of frames; heel/toe visibility {feet:.2f}; "
               f"processed at {len(times) / seconds:.1f} frames/s -> {target}")
    print(summary, flush=True)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("videos", nargs="+", type=Path)
    parser.add_argument("--out", type=Path, default=OUT_DIR)
    parser.add_argument("--overlay", action="store_true", help="also write <name>_overlay.mp4 with the skeleton drawn")
    args = parser.parse_args()
    if not MODEL.exists():
        sys.exit(f"model missing: {MODEL} (run Scripts/SetupCapturePython.ps1)")
    args.out.mkdir(parents=True, exist_ok=True)
    for video in args.videos:
        process(video, args.out, args.overlay)


if __name__ == "__main__":
    main()
