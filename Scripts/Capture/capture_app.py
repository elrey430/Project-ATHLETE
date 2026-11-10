"""
Project ATHLETE Capture - a small desktop app around pose_video.py (MediaPipe Pose).

  Process videos   pick recorded videos (phones), get body landmarks per frame (Saved/Capture/poses) and a
                   copy of each video with the detected skeleton drawn, to check the tracking.
  Camera check     a live view from a webcam with the skeleton drawn: framing, light, are the feet found?

Start: the desktop shortcut "ATHLETE Capture", or
  Intermediate\\CapturePython\\Scripts\\pythonw.exe Scripts\\Capture\\capture_app.py
Recording guide: Docs/MotionCapture.md.
"""

import os
import queue
import subprocess
import sys
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

sys.path.insert(0, str(Path(__file__).resolve().parent))
import pose_video  # noqa: E402

VIDEOS = pose_video.PROJECT_ROOT / "Saved" / "Capture" / "videos"
POSES = pose_video.OUT_DIR
VIDEO_TYPES = [("Videos", "*.mp4 *.mov *.m4v *.avi *.mkv"), ("All files", "*.*")]


def live_preview(camera):
    """Webcam view with the skeleton drawn (own process: OpenCV windows don't mix with Tk's loop)."""
    import cv2
    import mediapipe as mp
    from mediapipe.tasks.python import BaseOptions, vision
    capture = cv2.VideoCapture(camera, cv2.CAP_DSHOW)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    if not capture.isOpened():
        print(f"camera {camera} not available", file=sys.stderr)
        return 1
    landmarker = vision.PoseLandmarker.create_from_options(vision.PoseLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=str(pose_video.MODEL)), running_mode=vision.RunningMode.VIDEO))
    title = f"ATHLETE camera check (camera {camera}) - Q or Esc to close"
    started, last_ms, shown = time.time(), -1, 0
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        ms = max(int((time.time() - started) * 1000), last_ms + 1)
        last_ms = ms
        h, w = frame.shape[:2]
        result = landmarker.detect_for_video(mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)), ms)
        message, colour = "no person found", (0, 0, 255)
        if result.pose_landmarks:
            points = [(int(p.x * w), int(p.y * h), p.visibility) for p in result.pose_landmarks[0]]
            for a, b in pose_video.CONNECTIONS:
                if min(points[a][2], points[b][2]) > 0.5:
                    cv2.line(frame, points[a][:2], points[b][:2], (0, 255, 0), 2)
            for i, (x, y, visible) in enumerate(points):
                if visible > 0.5:
                    cv2.circle(frame, (x, y), 5, (0, 0, 255) if i in pose_video.FOOT else (255, 255, 0), -1)
            feet = sum(points[i][2] > 0.5 for i in pose_video.FOOT)
            whole = all(0 <= points[i][1] < h for i in pose_video.FOOT)
            message = f"person found; heels/toes seen: {feet}/4" + ("" if whole else " - FEET OUT OF FRAME")
            colour = (0, 200, 0) if feet == 4 and whole else (0, 165, 255)
        shown += 1
        rate = shown / max(time.time() - started, 1e-3)
        cv2.putText(frame, f"{message}   ({rate:.0f} fps)", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, colour, 2)
        cv2.imshow(title, frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27) or cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
            break
    capture.release()
    cv2.destroyAllWindows()
    landmarker.close()
    return 0


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ATHLETE Capture")
        self.geometry("820x600")
        self.minsize(640, 480)
        self.files, self.events, self.worker = [], queue.Queue(), None
        VIDEOS.mkdir(parents=True, exist_ok=True)
        POSES.mkdir(parents=True, exist_ok=True)

        pad = {"padx": 10, "pady": 6}
        videos = ttk.LabelFrame(self, text="1. Process recorded videos")
        videos.pack(fill="both", expand=True, **pad)
        row = ttk.Frame(videos); row.pack(fill="x", padx=8, pady=6)
        ttk.Button(row, text="Add videos...", command=self.add).pack(side="left")
        ttk.Button(row, text="Remove selected", command=self.remove).pack(side="left", padx=6)
        self.overlay = tk.BooleanVar(value=True)
        ttk.Checkbutton(row, text="Also make a video with the skeleton drawn", variable=self.overlay).pack(side="left", padx=12)
        self.listbox = tk.Listbox(videos, height=6, selectmode="extended")
        self.listbox.pack(fill="x", padx=8)
        row = ttk.Frame(videos); row.pack(fill="x", padx=8, pady=6)
        self.process_button = ttk.Button(row, text="Process", command=self.process)
        self.process_button.pack(side="left")
        ttk.Button(row, text="Play skeleton video of selected", command=self.play).pack(side="left", padx=6)
        ttk.Button(row, text="Open results folder", command=lambda: os.startfile(POSES)).pack(side="left")
        self.progress = ttk.Progressbar(videos, mode="determinate")
        self.progress.pack(fill="x", padx=8)
        self.status = ttk.Label(videos, text="Add the videos you recorded (Saved/Capture/videos is a good place for them).")
        self.status.pack(fill="x", padx=8, pady=4)
        self.log = tk.Text(videos, height=8, wrap="word", state="disabled")
        self.log.pack(fill="both", expand=True, padx=8, pady=(0, 8))

        camera = ttk.LabelFrame(self, text="2. Camera check (live, with the skeleton drawn)")
        camera.pack(fill="x", **pad)
        row = ttk.Frame(camera); row.pack(fill="x", padx=8, pady=8)
        ttk.Label(row, text="Camera:").pack(side="left")
        self.camera = ttk.Combobox(row, values=["0", "1", "2"], width=4, state="readonly")
        self.camera.set("0")
        self.camera.pack(side="left", padx=6)
        ttk.Button(row, text="Start camera check", command=self.live).pack(side="left", padx=6)
        ttk.Label(row, text="Green text: whole body and both feet found. Q or Esc closes it.").pack(side="left", padx=6)

        ttk.Label(self, text="Recording guide: Docs/MotionCapture.md  |  Results: Saved/Capture/poses").pack(pady=(0, 8))
        self.after(100, self.poll)

    def add(self):
        for name in filedialog.askopenfilenames(title="Videos to process", initialdir=VIDEOS, filetypes=VIDEO_TYPES):
            if name not in self.files:
                self.files.append(name)
                self.listbox.insert("end", name)

    def remove(self):
        for index in reversed(self.listbox.curselection()):
            self.listbox.delete(index)
            del self.files[index]

    def write(self, text):
        self.log.configure(state="normal")
        self.log.insert("end", text + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def process(self):
        if self.worker and self.worker.is_alive():
            return
        if not self.files:
            messagebox.showinfo("ATHLETE Capture", "Add some videos first.")
            return
        files, overlay = list(self.files), self.overlay.get()
        self.process_button.configure(state="disabled")

        def run():
            for number, name in enumerate(files, 1):
                label = f"{number}/{len(files)} {Path(name).name}"
                self.events.put(("status", f"Processing {label} ..."))
                try:
                    summary = pose_video.process(Path(name), POSES, overlay,
                                                 progress=lambda done, total, label=label: self.events.put(("progress", done, total, label)))
                    self.events.put(("log", summary))
                except Exception as error:  # report and go on with the next video
                    self.events.put(("log", f"{Path(name).name}: FAILED - {error}"))
            self.events.put(("done", len(files)))

        self.worker = threading.Thread(target=run, daemon=True)
        self.worker.start()

    def poll(self):
        try:
            while True:
                event = self.events.get_nowait()
                if event[0] == "status":
                    self.status.configure(text=event[1])
                    self.progress.configure(value=0)
                elif event[0] == "progress":
                    _, done, total, label = event
                    self.progress.configure(maximum=max(total, 1), value=min(done, max(total, 1)))
                    self.status.configure(text=f"Processing {label}: frame {done}" + (f" of {total}" if total else ""))
                elif event[0] == "log":
                    self.write(event[1])
                elif event[0] == "done":
                    self.status.configure(text=f"Done: {event[1]} video(s). Results in Saved/Capture/poses.")
                    self.progress.configure(value=self.progress["maximum"])
                    self.process_button.configure(state="normal")
        except queue.Empty:
            pass
        self.after(100, self.poll)

    def play(self):
        chosen = [self.files[i] for i in self.listbox.curselection()] or self.files[:1]
        for name in chosen:
            overlay = POSES / f"{Path(name).stem}_overlay.mp4"
            if overlay.exists():
                os.startfile(overlay)
            else:
                messagebox.showinfo("ATHLETE Capture", f"No skeleton video yet for {Path(name).name}: process it first.")

    def live(self):
        python = Path(sys.executable).with_name("python.exe")
        subprocess.Popen([str(python), __file__, "--live", self.camera.get()],
                         creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))


if __name__ == "__main__":
    if len(sys.argv) == 3 and sys.argv[1] == "--live":
        sys.exit(live_preview(int(sys.argv[2])))
    App().mainloop()
