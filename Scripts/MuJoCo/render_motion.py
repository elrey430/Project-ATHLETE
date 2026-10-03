"""Project ATHLETE - renders a fitted motion clip (retarget_motion.py output) as a strip of frames and a video.

Usage (LocoMuJoCo Python):
    Intermediate/LocoMuJoCoPython/Scripts/python.exe Scripts/MuJoCo/render_motion.py <clip.npz> [--start 10] [--seconds 1.2]

Replays the clip kinematically on the athlete model (no physics: this checks the motion data itself) and
writes <clip>_strip.png (8 frames side by side, camera following from the side) and <clip>_preview.mp4.
"""

import argparse
import sys
from pathlib import Path

import cv2
import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import athlete_loco  # noqa: E402,F401
from loco_mujoco.environments import LocoEnv  # noqa: E402
from loco_mujoco.trajectory import Trajectory  # noqa: E402


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("clip")
    parser.add_argument("--start", type=float, default=10.0, help="seconds into the clip")
    parser.add_argument("--seconds", type=float, default=1.2, help="span of the strip (about one stride)")
    parser.add_argument("--video-seconds", type=float, default=6.0)
    args = parser.parse_args()

    traj = Trajectory.load(args.clip)
    qpos, frequency = np.asarray(traj.data.qpos), float(traj.info.frequency)
    model = LocoEnv.registered_envs["AthleteReference"]()._model
    model.vis.global_.offwidth, model.vis.global_.offheight = 640, 480
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, 480, 360)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.distance, camera.elevation = 3.2, -8.0
    options = mujoco.MjvOption()
    options.sitegroup[3] = 1  # show the marker sites

    def render(frame):
        data.qpos[:] = qpos[frame]
        mujoco.mj_forward(model, data)
        heading = np.degrees(np.arctan2(*reversed(np.asarray(data.qvel[:2]) + 1e-9)))
        camera.lookat[:] = [data.qpos[0], data.qpos[1], 0.9]
        camera.azimuth = heading + 90.0  # from the athlete's side
        renderer.update_scene(data, camera, options)
        return renderer.render()

    first = int(args.start * frequency)
    strip = [render(first + int(i * args.seconds * frequency / 7)) for i in range(8)]
    base = Path(args.clip).with_suffix("")
    cv2.imwrite(f"{base}_strip.png", cv2.cvtColor(np.hstack(strip), cv2.COLOR_RGB2BGR))

    writer = cv2.VideoWriter(f"{base}_preview.mp4", cv2.VideoWriter_fourcc(*"mp4v"), 30, (480, 360))
    for frame in range(first, first + int(args.video_seconds * frequency), max(1, int(frequency / 30))):
        writer.write(cv2.cvtColor(render(frame), cv2.COLOR_RGB2BGR))
    writer.release()
    print(f"{base}_strip.png\n{base}_preview.mp4")


if __name__ == "__main__":
    main()
