"""Project ATHLETE - drive the athlete with the keyboard (Milestone 4: controller input -> movement).

Usage (LocoMuJoCo Python, on this PC):
    Intermediate/LocoMuJoCoPython/Scripts/python.exe Scripts/MuJoCo/drive_athlete.py <PPOJax_saved.pkl>

Opens MuJoCo's viewer with the athlete on CPU MuJoCo, driven in real time by the trained policy.
Your keys set the command (the controller input). With a tracking policy, motion matching turns it into a
reference motion from the mocap and the policy follows it with joint torques (see locomotion_tests.Driver).
A lookahead policy reacts 0.3 s after a key (motion matching runs that far ahead):
    Up / Down      forward speed +/- 0.25 m/s   (0 .. 3.0; the mocap has no walking backwards)
    Left / Right   turning rate  +/- 0.2 rad/s  (-1.2 .. 1.2; left = counter-clockwise)
    A / D          sideways speed +/- 0.1 m/s    (-0.5 .. 0.5)
    Space          stop (all zero)
    R              reset to standing
The current command and the athlete's measured speed print in this console. Close the window to quit.
"""

import sys
import threading
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from locomotion_tests import Driver  # noqa: E402

KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT, KEY_SPACE, KEY_A, KEY_D, KEY_R = 265, 264, 263, 262, 32, 65, 68, 82


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    driver = Driver(sys.argv[1], horizon_s=3600.0)
    env = driver.env
    command = [0.0, 0.0, 0.0]
    reset = threading.Event()

    def on_key(key):
        if key == KEY_UP: command[0] = min(command[0] + 0.25, 3.0)
        elif key == KEY_DOWN: command[0] = max(command[0] - 0.25, 0.0)
        elif key == KEY_LEFT: command[2] = min(command[2] + 0.2, 1.2)
        elif key == KEY_RIGHT: command[2] = max(command[2] - 0.2, -1.2)
        elif key == KEY_A: command[1] = min(command[1] + 0.1, 0.5)
        elif key == KEY_D: command[1] = max(command[1] - 0.1, -0.5)
        elif key == KEY_SPACE: command[:] = [0.0, 0.0, 0.0]
        elif key == KEY_R: reset.set()

    obs = driver.reset(tuple(command))
    print(__doc__.split("Your keys")[1])
    with mujoco.viewer.launch_passive(env._model, env._data, key_callback=on_key) as viewer:
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        viewer.cam.trackbodyid = mujoco.mj_name2id(env._model, mujoco.mjtObj.mjOBJ_BODY, "LowerTrunk")
        viewer.cam.distance, viewer.cam.elevation = 4.0, -12.0
        last_print = 0.0
        while viewer.is_running():
            started = time.perf_counter()
            if reset.is_set():
                obs, _ = driver.reset(tuple(command)), reset.clear()
            obs, fell = driver.step(obs, tuple(command))
            if fell:
                print("
fell: resetting")
                obs = driver.reset(tuple(command))
            viewer.sync()
            now = time.perf_counter()
            if now - last_print > 0.5:
                q, v = env._data.qpos, env._data.qvel
                w, x, y, z = q[3:7]
                yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
                forward = v[0] * np.cos(yaw) + v[1] * np.sin(yaw)
                print(f"\rcommand fwd {command[0]:+.2f} m/s side {command[1]:+.2f} turn {command[2]:+.2f} rad/s | "
                      f"measured fwd {forward:+.2f} m/s turn {v[5]:+.2f} rad/s   ", end="", flush=True)
                last_print = now
            time.sleep(max(0.0, env.dt - (time.perf_counter() - started)))  # real time


if __name__ == "__main__":
    main()
