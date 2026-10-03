"""Project ATHLETE - a quiet-standing clip for training, from the mocap's own standing moments.

Usage (LocoMuJoCo Python, after retarget_motion.py and mirror_motion.py):
    Intermediate/LocoMuJoCoPython/Scripts/python.exe Scripts/MuJoCo/stand_motion.py [--seconds 60]

Writes stand.npz next to the other clips (Saved/MuJoCo/motions/DEFAULT/mocap/AthleteReference/).

Both step-in-place clips open with ~3.5 s of quiet standing before the stepping starts. Training starts
episodes anywhere in a clip, so those few seconds were rarely practised. The heading-invariant tracker
(2026-10-02) handled stepping in place, but drifted and fell after ~2.7 s whenever it had to stand still,
which every acceptance test starts with. This clip makes standing a behaviour of its own: the quiet openings
(until the first foot lift, minus a margin), each played forward then backward in turn (quiet sway
reverses naturally, so the joins stay continuous), then the same for the left/right mirror, up to --seconds.
"""

import argparse
import json
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from mirror_motion import mirror_map, mirror_qpos  # noqa: E402
from retarget_motion import ENV_NAME, PROJECT_ROOT, save_motion  # noqa: E402
from loco_mujoco.environments import LocoEnv  # noqa: E402

SOURCES = ("stepinplace1", "stepinplace2")
FOOT_LIFT_M = 0.02          # a foot this far above its lowest point: the stepping has started
MARGIN_S = 0.5              # stop this long before the first lift (weight shifts before it)


def quiet_opening(folder, clip):
    data = np.load(folder / f"{clip}.npz", allow_pickle=True)
    names = [str(s) for s in data["site_names"]]
    feet = np.asarray(data["site_xpos"])[:, [names.index("left_foot_mimic"), names.index("right_foot_mimic")], 2]
    first_lift = int(np.argmax(((feet - feet.min(axis=0)) > FOOT_LIFT_M).any(axis=1)))
    frequency = float(data["frequency"])
    end = max(int(first_lift - MARGIN_S * frequency), int(frequency))
    return np.asarray(data["qpos"], np.float64)[:end], frequency


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--motions", default=str(PROJECT_ROOT / "Saved" / "MuJoCo" / "motions"))
    parser.add_argument("--seconds", type=float, default=60.0)
    args = parser.parse_args()
    model = LocoEnv.registered_envs[ENV_NAME]()._model
    folder = Path(args.motions) / "DEFAULT" / "mocap" / ENV_NAME
    pieces, frequency = [], None
    for clip in SOURCES:
        qpos, frequency = quiet_opening(folder, clip)
        pieces.append(qpos)
    mapping = mirror_map(model)
    pieces += [mirror_qpos(piece, *mapping) for piece in pieces]

    # Forward then backward for each piece in turn; each piece's start is moved to where the last one ended
    # (same place and facing: quiet standing barely moves), so the clip is one continuous stand.
    frames = []
    while len(frames) < args.seconds * frequency:
        for piece in pieces:
            for segment in (piece, piece[::-1]):
                segment = segment.copy()
                if frames:
                    segment[:, :2] += frames[-1][:2] - segment[0, :2]
                frames.extend(segment)
    qpos = np.array(frames[:int(args.seconds * frequency)])
    qvel = np.zeros((len(qpos) - 2, model.nv))
    for i in range(1, len(qpos) - 1):
        mujoco.mj_differentiatePos(model, qvel[i - 1], 2.0 / frequency, qpos[i - 1], qpos[i + 1])
    target = folder / "stand.npz"
    save_motion(model, qpos[1:-1], qvel, frequency, target)
    print(json.dumps({"output": str(target), "seconds": round(len(qvel) / frequency, 1),
                      "pieces_s": [round(len(p) / frequency, 2) for p in pieces],
                      "pelvis_speed_p99_mps": round(float(np.percentile(np.linalg.norm(qvel[:, :2], axis=1), 99)), 3)}))


if __name__ == "__main__":
    main()
