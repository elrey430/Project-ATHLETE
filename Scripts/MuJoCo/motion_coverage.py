"""Project ATHLETE - what the motion matcher's database covers: seconds of motion per (forward speed, turn rate).

Gaps in this table are commands the matcher can't answer with fitting motion (2026-10-04: nothing between
1.5 and 2.5 m/s, nothing above 3 m/s, running turns only up to ~0.9 rad/s; Docs/MotionData.md).

Usage:
    Intermediate/LocoMuJoCoPython/Scripts/python.exe Scripts/MuJoCo/motion_coverage.py
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import numpy as np  # noqa: E402
import motion_matching as mm  # noqa: E402

db = mm.MotionDatabase()
# 0.5 s smoothed (the matcher compares future trajectory, not instantaneous wobble)
k = 50; ker = np.ones(k) / k
fwd = np.convolve(db.pelvis_velocity[:, 0], ker, "same"); side = np.convolve(db.pelvis_velocity[:, 1], ker, "same")
yr = np.abs(np.convolve(db.yaw_rate, ker, "same"))
sb = [0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 9]; yb = [0, 0.2, 0.4, 0.6, 0.9, 1.2, 9]
H, _, _ = np.histogram2d(fwd, yr, [sb, yb]); H *= mm.DT
print("seconds of database; rows forward speed (m/s), columns |yaw rate| (rad/s)")
print("fwd\yaw   " + "".join(f"{yb[i]:.1f}-{yb[i+1]:.1f}".rjust(9) for i in range(len(yb) - 1)))
for i in range(len(sb) - 1):
    print(f"{sb[i]:.1f}-{sb[i+1]:.1f}  " + "".join(f"{H[i, j]:9.0f}" for j in range(len(yb) - 1)))
fast = fwd > 2.0
print(f"\nfwd > 2 m/s with |side| > 0.2 m/s: {np.sum(fast & (np.abs(side) > 0.2)) * mm.DT:.0f} s of {np.sum(fast) * mm.DT:.0f} s")
print("clips:", sorted(set(n.rsplit('_', 1)[0] if n[-1].isdigit() else n for n in db.range_names)))
