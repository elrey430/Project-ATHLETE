"""Project ATHLETE - checks the exported MuJoCo athletes (engine spike after Milestone 4).

Usage (after Scripts\\SetupMuJoCoPython.ps1 and the ProjectTools.Export.MuJoCo export):
    Intermediate\\MuJoCoPython\\Scripts\\python.exe Scripts\\MuJoCo\\check_athlete.py [model.xml ...]
With no arguments, checks every model in Saved\\MuJoCo\\.

For each model:
  1. It loads, and MuJoCo's own mass and center of mass match what Unreal computed (stored in the file).
  2. In the reference pose nothing overlaps except soles touching the floor (an overlap would push
     the body apart the moment physics starts).
  3. Passive collapse (no muscles): mechanical energy barely rises (nothing adds energy; a rise is
     energy made up by the solver or given back by MuJoCo's soft contacts and limits), the floor and
     the joint limits hold, and how long a simulated second takes.
For the Milestone 2 drop-test body (athlete_5ft9_190lb), also the drop study: 1.5 m, tilted
15/30/45 degrees forward, at 240, 500 and 1000 Hz, reported like Milestone 2 section 8 so the two
engines can be compared.
"""

import glob
import math
import os
import sys
import time

import mujoco
import numpy as np

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SAMPLE_HZ = 60.0  # energy sampled once per Unreal frame, as in Milestone 2


def custom(model, name):
    return np.array(model.numeric(name).data)


def floor_geom(model):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")


def check_structure(model, data, report):
    mujoco.mj_forward(model, data)
    mass = mujoco.mj_getTotalmass(model)
    expected_mass = custom(model, "athlete_total_mass_kg")[0]
    com = data.subtree_com[1]  # body 1 = the root (LowerTrunk): the whole athlete
    expected_com = custom(model, "athlete_center_of_mass_m")
    ok = True
    report.append(f"  bodies {model.nbody - 1}, hinges {model.njnt - 1}, motors {model.nu}, timestep {model.opt.timestep * 1000:.2f} ms")
    mass_ok = abs(mass - expected_mass) < 1e-3
    com_ok = np.linalg.norm(com - expected_com) < 1e-4
    ok &= mass_ok and com_ok
    report.append(f"  mass {mass:.4f} kg (Unreal {expected_mass:.4f}) {'OK' if mass_ok else 'MISMATCH'}")
    report.append(f"  center of mass {np.round(com, 5)} (Unreal {np.round(expected_com, 5)}) {'OK' if com_ok else 'MISMATCH'}")

    floor = floor_geom(model)
    overlaps = []
    deepest_floor = 0.0
    for i in range(data.ncon):
        c = data.contact[i]
        if floor in (c.geom1, c.geom2):
            deepest_floor = min(deepest_floor, c.dist)
        elif c.dist < 0.0:
            overlaps.append((model.geom(c.geom1).name, model.geom(c.geom2).name, c.dist))
    ok &= not overlaps
    report.append(f"  reference pose: {data.ncon} contacts, floor penetration {-deepest_floor * 1000:.2f} mm, "
                  f"self-overlaps: {', '.join(f'{a}-{b} {-d * 1000:.1f} mm' for a, b, d in overlaps) or 'none'}")
    return ok


def mechanical_energy(model, data):
    mujoco.mj_forward(model, data)
    return data.energy[0] + data.energy[1]  # potential (gravity) + kinetic


def limit_excess_rad(model, data):
    """How far the worst hinge is past its range (MuJoCo limits are soft, so they can be exceeded)."""
    hinges = slice(1, model.njnt)  # joint 0 is the free root
    q = data.qpos[model.jnt_qposadr[hinges]]
    low, high = model.jnt_range[hinges].T
    return max(0.0, float(np.max(np.maximum(low - q, q - high))))


def run(model, data, seconds):
    """Steps the model, sampling energy, floor penetration and joint-limit excess at 60 Hz. Returns
    (max energy created, deepest floor penetration m, worst limit excess rad, wall ms per simulated second)."""
    floor = floor_geom(model)
    steps_per_sample = max(1, round(1.0 / (SAMPLE_HZ * model.opt.timestep)))
    samples = round(seconds * SAMPLE_HZ)
    min_energy, max_gain, deepest, worst_limit = math.inf, 0.0, 0.0, 0.0
    stepping_s = 0.0
    for _ in range(samples):
        start = time.perf_counter()
        for _ in range(steps_per_sample):
            mujoco.mj_step(model, data)
        stepping_s += time.perf_counter() - start
        energy = mechanical_energy(model, data)
        min_energy = min(min_energy, energy)
        max_gain = max(max_gain, energy - min_energy)
        worst_limit = max(worst_limit, limit_excess_rad(model, data))
        for i in range(data.ncon):
            c = data.contact[i]
            if floor in (c.geom1, c.geom2):
                deepest = min(deepest, c.dist)
    simulated_s = samples * steps_per_sample * model.opt.timestep
    return max_gain, -deepest, worst_limit, 1000.0 * stepping_s / simulated_s


def set_rate(model, hz):
    """A new time step, with contacts and limits kept as stiff as MuJoCo allows (time constant 2 steps), as the export does."""
    model.opt.timestep = 1.0 / hz
    model.geom_solref[:, 0] = 2.0 / hz
    model.jnt_solref[:, 0] = 2.0 / hz


def place(model, data, origin_z, tilt_deg):
    """The whole body raised so its body-frame origin (floor between the ankles) is at origin_z, then
    tilted forward about that point, at rest (like FRotator(-tilt, 0, 0) in the Unreal study)."""
    mujoco.mj_resetData(model, data)
    half = math.radians(tilt_deg) / 2.0
    quat = np.array([math.cos(half), 0.0, math.sin(half), 0.0])  # about +Y: the top tips toward +X (forward)
    rotation = np.zeros(9)
    mujoco.mju_quat2Mat(rotation, quat)
    root_offset = model.body_pos[1]  # root position in the body frame (qpos0)
    data.qpos[0:3] = np.array([0.0, 0.0, origin_z]) + rotation.reshape(3, 3) @ root_offset
    data.qpos[3:7] = quat


def check_model(path):
    report = [os.path.relpath(path, PROJECT_ROOT)]
    model = mujoco.MjModel.from_xml_path(path)
    data = mujoco.MjData(model)
    ok = check_structure(model, data, report)

    # Passive collapse from standing. Soft constraints store a little energy and give it back, so the
    # bar is a fraction of the energy in play (weight x stature), not zero.
    place(model, data, 0.0, 0.0)
    gain, penetration, limit, ms = run(model, data, 3.0)
    energy_scale = mujoco.mj_getTotalmass(model) * -model.opt.gravity[2] * custom(model, "athlete_stature_m")[0]
    collapse_ok = gain < 0.005 * energy_scale and penetration < 0.01 and math.degrees(limit) < 5.0
    ok &= collapse_ok
    report.append(f"  passive collapse, 3 s: energy created {gain:.2f} J (bar {0.005 * energy_scale:.1f}), floor penetration "
                  f"{penetration * 100:.2f} cm (bar 1), joint limits exceeded by {math.degrees(limit):.1f} deg (bar 5) "
                  f"{'OK' if collapse_ok else 'FAIL'}; {ms / 60:.3f} ms per 60 Hz frame")

    if os.path.basename(path).startswith("athlete_5ft9_190lb"):
        report.append("  drop study (Milestone 2 section 8, Chaos: 240 Hz 261 / 330 / 0 J, 5.3 cm; 960 Hz 0 / 9 / 0 J, 1.4 cm):")
        for hz in (240, 500, 1000):
            set_rate(model, hz)
            results = []
            for tilt in (15, 30, 45):
                place(model, data, 1.5, tilt)
                results.append(run(model, data, 3.0))
            gains, depths, limits, costs = zip(*results)
            report.append(f"    {hz:4d} Hz: energy created {' / '.join(f'{g:.1f}' for g in gains)} J; deepest penetration "
                          f"{max(depths) * 100:.2f} cm; worst limit excess {math.degrees(max(limits)):.1f} deg; "
                          f"{np.mean(costs) / 60:.3f} ms per 60 Hz frame")
    report.append("  " + ("PASS" if ok else "FAIL"))
    print("\n".join(report))
    return ok


def main():
    paths = sys.argv[1:] or sorted(glob.glob(os.path.join(PROJECT_ROOT, "Saved", "MuJoCo", "*.xml")))
    if not paths:
        sys.exit("No models: run the ProjectTools.Export.MuJoCo export first.")
    print(f"MuJoCo {mujoco.__version__}")
    results = [check_model(path) for path in paths]
    sys.exit(0 if all(results) else 1)


if __name__ == "__main__":
    main()
