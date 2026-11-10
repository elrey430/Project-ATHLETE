"""
Project ATHLETE - is a reference motion physically possible for the athlete's body?

Inverse dynamics (mj_inverse, constraints off) gives, per frame, the generalized forces the motion needs. The
root (pelvis free joint) has no motor, so its 6 components are the wrench the GROUND must supply. Per frame:
  - friction need = |Fxy| / Fz (feet on the floor, Fz > 0.3 weight): above MU = 0.7 no foot could supply it,
    the feet would slip. A hard physical limit, independent of foot geometry;
  - centre-of-pressure excursion: how far (m) the ground force's point of action lies outside the soles near
    the floor (0 inside);
  - flight force: |F| / weight with no sole near the floor (should be ~0: nothing to push on).
The last two are sensitive to modelling error (rigid segments, foot geometry vs mocap: a clip and its mirror
scored differently), so judge them against the mocap clips' own distribution, not absolutely. Joint-torque
limits are not checked (that needs the contact-force split between the feet).
History: Milestone4_Locomotion.md 9.6, "Reference feasibility check" (2026-10-09).

Usage: python reference_feasibility.py CLIP.npz [CLIP.npz ...]   (motion files from retarget_motion.py)
"""

import os
import sys
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial import ConvexHull

sys.path.insert(0, str(Path(__file__).resolve().parent))
from athlete_loco.env import FOOT_GEOMS, configure_contacts, sole_geom_names  # noqa: E402

MU = 0.7            # foot-floor friction in the model
CONTACT_TOL = 0.03  # m: a sole end this close to the floor can bear load
DIFF_HALF = 2       # frames each side for the acceleration (central difference over 40 ms at 100 Hz)

spec = mujoco.MjSpec.from_file(os.environ.get("ATHLETE_MJCF", str(Path(__file__).resolve().parents[2] / "Saved" / "MuJoCo" / "athlete_reference.xml")))
configure_contacts(spec)
model = spec.compile()
model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONSTRAINT
data = mujoco.MjData(model)
PELVIS = int(model.jnt_bodyid[0])
WEIGHT = float(sum(model.body_mass)) * 9.8
SOLES = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n) for f in FOOT_GEOMS for n in sole_geom_names(f)]
METRICS = ("friction", "cop_out", "flight")


def _outside(p, pts):
    """Distance from p to the convex hull of pts (0 inside)."""
    pts = np.unique(np.round(pts, 4), axis=0)
    if len(pts) == 1:
        return float(np.linalg.norm(p - pts[0]))
    edges = [(pts[0], pts[1])]
    if len(pts) >= 3:
        try:
            hull = ConvexHull(pts)
            if np.all(hull.equations[:, :2] @ p + hull.equations[:, 2] <= 1e-9):
                return 0.0
            edges = [(pts[a], pts[b]) for a, b in hull.simplices]
        except Exception:  # collinear
            edges = [(pts[i], pts[j]) for i in range(len(pts)) for j in range(i + 1, len(pts))]
    best = np.inf
    for a, b in edges:
        ab = b - a
        t = np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-12), 0, 1)
        best = min(best, float(np.linalg.norm(p - (a + t * ab))))
    return best


def pose_velocity(qpos, dt):
    """Velocities from the pose path itself (central differences), ignoring any stated qvel."""
    v = np.zeros((len(qpos), model.nv))
    for i in range(1, len(qpos) - 1):
        mujoco.mj_differentiatePos(model, v[i], 2 * dt, qpos[i - 1], qpos[i + 1])
    v[0], v[-1] = v[1], v[-2]
    return v


def analyse(qpos, qvel, dt, stride=1, from_pose=False):
    """Per-frame metrics (NaN where not applicable or not computed) for one continuous motion."""
    if from_pose:
        qvel = pose_velocity(qpos, dt)
    n, k = len(qpos), DIFF_HALF
    out = {key: np.full(n, np.nan) for key in METRICS + ("fz",)}
    for i in range(k, n - k, stride):
        data.qpos[:], data.qvel[:] = qpos[i], qvel[i]
        data.qacc[:] = (qvel[i + k] - qvel[i - k]) / (2 * k * dt)
        mujoco.mj_inverse(model, data)
        f = data.qfrc_inverse[:3].copy()
        torque = data.xmat[PELVIS].reshape(3, 3) @ data.qfrc_inverse[3:6]
        origin = data.xpos[PELVIS]
        points = []
        for g in SOLES:
            axis = data.geom_xmat[g].reshape(3, 3)[:, 2]
            for sign in (1.0, -1.0):
                end = data.geom_xpos[g] + sign * axis * model.geom_size[g][1]
                if end[2] - model.geom_size[g][0] < CONTACT_TOL:
                    points.append(end[:2])
        fz = f[2]
        out["fz"][i] = fz / WEIGHT
        if not points:
            out["flight"][i] = np.linalg.norm(f) / WEIGHT
        elif fz > 0.3 * WEIGHT:
            out["friction"][i] = np.hypot(f[0], f[1]) / fz
            # Centre of pressure on the floor (z = 0): torque about the pelvis = (p - o) x F (+ a free yaw torque).
            cop = np.array([origin[0] + (-origin[2] * f[0] - torque[1]) / fz, origin[1] + (torque[0] - origin[2] * f[1]) / fz])
            out["cop_out"][i] = _outside(cop, np.array(points))
    return out


def summary(out):
    need = out["friction"][np.isfinite(out["friction"])]
    flight = out["flight"][np.isfinite(out["flight"])]
    return {"frames": int(np.isfinite(out["fz"]).sum()), "impossible_grip_pct": 100 * float(np.mean(need > MU)) if len(need) else float("nan"),
            "friction_need_p99": float(np.percentile(need, 99)) if len(need) else float("nan"),
            "mean_vertical_support_weights": float(np.nanmean(out["fz"])),
            "flight_force_median_weights": float(np.median(flight)) if len(flight) else float("nan")}


def main():
    for path in sys.argv[1:]:
        clip = np.load(path, allow_pickle=True)
        qpos, qvel = np.asarray(clip["qpos"]), np.asarray(clip["qvel"])
        dt = 1.0 / float(clip["frequency"])
        s = summary(analyse(qpos, qvel, dt))
        print(f"{Path(path).stem:40s} {s['frames']:6d} frames | impossible grip {s['impossible_grip_pct']:5.2f}% "
              f"(friction need p99 {s['friction_need_p99']:.2f}) | mean vertical support {s['mean_vertical_support_weights']:.2f} weights | "
              f"force in flight median {s['flight_force_median_weights']:.2f} weights", flush=True)


if __name__ == "__main__":
    main()
