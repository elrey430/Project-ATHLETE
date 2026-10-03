"""Project ATHLETE - left/right mirrored copies of the athlete's motion clips.

Usage (LocoMuJoCo Python):
    Intermediate/LocoMuJoCoPython/Scripts/python.exe Scripts/MuJoCo/mirror_motion.py --clips walk run walkturn random_walk

For each clip, writes <clip>_mirror.npz next to it (Saved/MuJoCo/motions/DEFAULT/mocap/AthleteReference/).
The mirror is the motion reflected in the athlete's sagittal plane, as if performed by the other side:
a left turn becomes a right turn and the lead foot swaps. The data has twice the turning, balanced
between directions; the tracker couldn't follow turns (2026-10-01).

The mapping comes from the model itself (mirror_map), so it follows the exported joint axes:
  - a Left/Right joint pair swaps values, with the sign that makes R_right(q') = M R_left(q) M, where
    M = diag(1, -1, 1). The export defines mirrored axes, so the sign is +1 for every pair;
  - a centre joint (spine, neck) keeps its angle about the left-right (y) axis and negates the others;
  - the pelvis: y position negated, orientation quaternion (w, x, y, z) -> (w, -x, y, -z).
Velocities are recomputed from the mirrored positions (central differences), as retarget_motion.py does.
"""

import argparse
import json
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from retarget_motion import ENV_NAME, PROJECT_ROOT, save_motion  # noqa: E402
from loco_mujoco.environments import LocoEnv  # noqa: E402

MIRROR = np.diag([1.0, -1.0, 1.0])
SUFFIX = "_mirror"


def mirror_map(model):
    """(source qpos index, sign) for every qpos index, plus a check that the body is symmetric."""
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j) for j in range(model.njnt)]
    source = np.arange(model.nq)
    sign = np.ones(model.nq)
    for j, name in enumerate(names):
        if model.jnt_type[j] != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        adr = model.jnt_qposadr[j]
        if "Left" in name or "Right" in name:
            other = names.index(name.replace("Left", "Right") if "Left" in name else name.replace("Right", "Left"))
            source[adr] = model.jnt_qposadr[other]
            # R(a, q) reflected = R(M a, -q): the other side's axis b gives q' = -(b . M a) q
            sign[adr] = -float(model.jnt_axis[other] @ (MIRROR @ model.jnt_axis[j]))
            assert abs(abs(sign[adr]) - 1) < 1e-6, f"{name}: axes aren't mirror images"
            assert np.allclose(model.body_pos[model.jnt_bodyid[other]], MIRROR @ model.body_pos[model.jnt_bodyid[j]], atol=1e-6), \
                f"{name}: body isn't symmetric"
        else:
            axis = model.jnt_axis[j]
            sign[adr] = -float(axis @ (MIRROR @ axis))
    return source, sign


def mirror_qpos(qpos, source, sign):
    mirrored = qpos[:, source] * sign
    mirrored[:, 0:3] = qpos[:, 0:3] * np.array([1.0, -1.0, 1.0])
    mirrored[:, 3:7] = qpos[:, 3:7] * np.array([1.0, -1.0, 1.0, -1.0])
    return mirrored


def mirror_clip(model, folder, clip):
    data = np.load(folder / f"{clip}.npz", allow_pickle=True)
    frequency = float(data["frequency"])
    qpos = mirror_qpos(np.asarray(data["qpos"], np.float64), *mirror_map(model))
    qvel = np.zeros((len(qpos) - 2, model.nv))
    for i in range(1, len(qpos) - 1):
        mujoco.mj_differentiatePos(model, qvel[i - 1], 2.0 / frequency, qpos[i - 1], qpos[i + 1])
    target = folder / f"{clip}{SUFFIX}.npz"
    save_motion(model, qpos[1:-1], qvel, frequency, target)
    yaw_rate = qvel[:, 5]
    return {"clip": clip, "output": str(target), "frames": len(qvel),
            "mean_turn_rate_rad_s": {"original": round(float(np.mean(np.asarray(data["qvel"])[:, 5])), 3),
                                     "mirror": round(float(np.mean(yaw_rate)), 3)}}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--clips", nargs="+", default=["walk", "run", "walkturn", "random_walk"])
    parser.add_argument("--motions", default=str(PROJECT_ROOT / "Saved" / "MuJoCo" / "motions"))
    args = parser.parse_args()
    model = LocoEnv.registered_envs[ENV_NAME]()._model
    folder = Path(args.motions) / "DEFAULT" / "mocap" / ENV_NAME
    for clip in args.clips:
        print(json.dumps(mirror_clip(model, folder, clip)), flush=True)


if __name__ == "__main__":
    main()
