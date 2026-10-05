"""Project ATHLETE - self-test of the BVH path: athlete clip -> BVH file -> retarget_motion -> athlete clip.

Exports a fitted athlete clip as a BVH file the way motion libraries ship them (Y up, centimetres, joints
named LeftUpLeg/LeftFoot/LeftToeBase/...), retargets that file with retarget_motion.py --source bvh, and
compares the result with the original. The athlete is its own source here, so the fit should come back
close to the original: this checks the BVH reader, axes, units, joint lookup and foot handling without
any downloaded data.

Usage:
    Intermediate/LocoMuJoCoPython/Scripts/python.exe Scripts/MuJoCo/check_bvh_retarget.py [--clip run] [--seconds 6]
"""

import argparse
import json
import subprocess
import sys
import tempfile
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bvh  # noqa: E402
from retarget_motion import ENV_NAME, PROJECT_ROOT  # noqa: E402

MOTIONS = PROJECT_ROOT / "Saved" / "MuJoCo" / "motions" / "DEFAULT" / "mocap" / ENV_NAME
# Athlete body -> library-style BVH name (only the ones the retargeter looks up; the rest keep their names).
RENAME = {"Head": "Neck", "FootLeft": "LeftFoot", "FootRight": "RightFoot", "HandLeft": "LeftHand", "HandRight": "RightHand"}
# Extra end joints at athlete sites: (BVH name, parent body, site).
EXTRA = [("LeftToeBase", "FootLeft", "left_ball_of_foot"), ("RightToeBase", "FootRight", "right_ball_of_foot"),
         ("Head", None, "head_mimic")]
# Pass marks (the fit is a least-squares IK, not an identity: small differences are expected).
MAX_ROOT_ERROR_M = 0.03
MAX_JOINT_RMS_DEG = 6.0
MAX_SPEED_ERROR = 0.03  # relative


def export(model, qpos, frequency, path):
    data = mujoco.MjData(model)
    bodies = list(range(1, model.nbody))
    names = [RENAME.get(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b), mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b))
             for b in bodies]
    parents = [None if model.body_parentid[b] == 0 else bodies.index(model.body_parentid[b]) for b in bodies]
    offsets = [model.body_pos[b].copy() for b in bodies]
    site_parent = []
    for name, body, site in EXTRA:
        s = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site)
        b = model.site_bodyid[s] if body is None else mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body)
        names.append(name)
        parents.append(bodies.index(b))
        offsets.append(model.site_pos[s].copy())
        site_parent.append(b)
    n, count = len(qpos), len(names)
    root, local = np.zeros((n, 3)), np.tile(np.eye(3), (n, count, 1, 1))
    for f in range(n):
        data.qpos[:] = qpos[f]
        mujoco.mj_kinematics(model, data)
        root[f] = data.xpos[bodies[0]]
        for i, b in enumerate(bodies):
            rotation = data.xmat[b].reshape(3, 3)
            parent = model.body_parentid[b]
            local[f, i] = rotation if parent == 0 else data.xmat[parent].reshape(3, 3).T @ rotation
    # Z up metres -> Y up centimetres, as libraries ship it.
    T = bvh.Y_UP_TO_Z_UP
    to_file = lambda v: (np.asarray(v) @ T) * 100.0  # row vectors: T.T @ v
    local = np.einsum("ji,fkjl,lm->fkim", T, local, T)  # T.T @ L @ T
    bvh.write(path, names, parents, [to_file(o) for o in offsets], to_file(root), local, 1.0 / frequency)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--clip", default="run")
    parser.add_argument("--seconds", type=float, default=6.0)
    args = parser.parse_args()

    source = np.load(MOTIONS / f"{args.clip}.npz", allow_pickle=True)
    frequency = float(source["frequency"])
    original = np.asarray(source["qpos"])[:int(args.seconds * frequency)]
    from loco_mujoco.environments import LocoEnv
    import athlete_loco  # noqa: F401  (registers the athlete)
    model = LocoEnv.registered_envs[ENV_NAME]()._model

    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        path = tmp / f"selftest_{args.clip}.bvh"
        export(model, original, frequency, path)
        motion = bvh.read(path)
        print(f"wrote {path.name}: {len(motion.joints)} joints, {motion.n_frames} frames at {motion.frequency:.0f} Hz")
        # --floor frame: as the original clip was placed (LocoMuJoCo source), so they compare like for like.
        run = subprocess.run([sys.executable, str(Path(__file__).with_name("retarget_motion.py")), "--source", "bvh",
                              "--files", str(path), "--out", str(tmp), "--floor", "frame"], capture_output=True, text=True)
        if run.returncode:
            sys.exit(f"retarget_motion.py failed:\n{run.stderr[-3000:]}")
        result = tmp / "DEFAULT" / "mocap" / ENV_NAME
        fitted = np.asarray(np.load(result / f"selftest_{args.clip}.npz")["qpos"])
        report = json.loads((result / f"selftest_{args.clip}_report.json").read_text())

    # The fit drops the first and last frame (central differences).
    original = original[1:1 + len(fitted)]
    fitted = fitted[:len(original)]
    root_error = np.linalg.norm(fitted[:, :3] - original[:, :3], axis=1)
    hinges = [j for j in range(model.njnt) if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE]
    joint_rms = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j):
                 float(np.degrees(np.sqrt(np.mean((fitted[:, model.jnt_qposadr[j]] - original[:, model.jnt_qposadr[j]]) ** 2))))
                 for j in hinges}
    speed = lambda q: np.linalg.norm(q[-1, :2] - q[0, :2]) / (len(q) / frequency)
    speed_error = abs(speed(fitted) - speed(original)) / max(speed(original), 1e-6)
    worst = sorted(joint_rms.items(), key=lambda kv: -kv[1])[:5]
    checks = {
        f"root position error p95 < {MAX_ROOT_ERROR_M} m": float(np.percentile(root_error, 95)) < MAX_ROOT_ERROR_M,
        f"worst joint RMS < {MAX_JOINT_RMS_DEG} deg": worst[0][1] < MAX_JOINT_RMS_DEG,
        f"average speed within {MAX_SPEED_ERROR:.0%}": speed_error < MAX_SPEED_ERROR,
    }
    print(f"scale guessed: {report.get('segment_ratios') is not None and 'ok'}; leg_scale {report['leg_scale']}")
    print(f"root position error: median {np.median(root_error):.4f} m, p95 {np.percentile(root_error, 95):.4f} m")
    print(f"joint RMS: median {np.median(list(joint_rms.values())):.2f} deg; worst " +
          ", ".join(f"{k} {v:.2f}" for k, v in worst))
    print(f"average speed: original {speed(original):.3f} m/s, fitted {speed(fitted):.3f} m/s")
    print(f"landmark fit error (m, mean): " + ", ".join(f"{k} {v['mean']}" for k, v in report["fit_error_m"].items()))
    for name, ok in checks.items():
        print(f"{'PASS' if ok else 'FAIL'} {name}")
    sys.exit(0 if all(checks.values()) else 1)


if __name__ == "__main__":
    main()
