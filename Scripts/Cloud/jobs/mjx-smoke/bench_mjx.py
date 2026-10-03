"""Project ATHLETE - GPU smoke benchmark. Runs ON the cloud VM (see Scripts/Cloud/GpuSmokeTest.ps1).

Usage: python bench_mjx.py <model.xml> <results.json>

Questions, with the exported MuJoCo athlete (passive: no muscles, it collapses and lies on the floor):
  1. Does MJX (MuJoCo on the GPU, through JAX) accept the model as it is?
  2. Same physics as CPU MuJoCo? One world, first 0.2 s, GPU (float32) against CPU (float64).
  3. Throughput: thousands of athletes in parallel. Reported as simulated seconds per wall second
     and as training samples per second at 30 decisions per simulated second (the Milestone 4
     Learning Agents spike managed ~330 per second in Chaos; Unity's Walker ~1,200).
"""

import json
import sys
import time

import jax
import mujoco
import numpy as np
from mujoco import mjx

model_path, out_path = sys.argv[1], sys.argv[2]
results = {"model": model_path, "mujoco": mujoco.__version__, "jax": jax.__version__, "devices": [str(d) for d in jax.devices()]}
m = mujoco.MjModel.from_xml_path(model_path)
results["timestep_s"] = m.opt.timestep

# CPU reference on this VM: one world, single thread.
d = mujoco.MjData(m)
start = time.perf_counter()
for _ in range(2000):
    mujoco.mj_step(m, d)
results["cpu_one_world_sim_seconds_per_s"] = 2000 * m.opt.timestep / (time.perf_counter() - start)

# 1. Does MJX take the model? Features it doesn't support are switched off one at a time and recorded:
#    the energy flag (only for the CPU energy checks), then the implicit integrator (Euler instead).
#    Anything else is a real incompatibility and stops the run.
results["adjusted_for_mjx"] = []
while True:
    try:
        mx = mjx.put_model(m)
        break
    except NotImplementedError as error:
        message = str(error)
        results["adjusted_for_mjx"].append(message)
        energy_bit = int(mujoco.mjtEnableBit.mjENBL_ENERGY)
        if "ENERGY" in message and m.opt.enableflags & energy_bit:
            m.opt.enableflags &= ~energy_bit
        elif "INT" in message.upper() and m.opt.integrator != mujoco.mjtIntegrator.mjINT_EULER:
            m.opt.integrator = mujoco.mjtIntegrator.mjINT_EULER
        else:
            raise
results["integrator"] = mujoco.mjtIntegrator(m.opt.integrator).name
print("model accepted; integrator", results["integrator"], "; adjusted:", results["adjusted_for_mjx"] or "nothing", flush=True)

# 2. Same physics as the CPU: one world, 0.2 s of the collapse.
steps = int(round(0.2 / m.opt.timestep))
d_cpu = mujoco.MjData(m)
for _ in range(steps):
    mujoco.mj_step(m, d_cpu)
step = jax.jit(mjx.step)
dx = mjx.make_data(mx)
for _ in range(steps):
    dx = step(mx, dx)
gpu_qpos = np.array(dx.qpos)
results["gpu_vs_cpu_after_0_2_s"] = {
    "max_qpos_difference": float(np.max(np.abs(gpu_qpos - d_cpu.qpos))),
    "root_height_cpu_m": float(d_cpu.qpos[2]),
    "root_height_gpu_m": float(gpu_qpos[2]),
}
print("GPU vs CPU:", results["gpu_vs_cpu_after_0_2_s"], flush=True)

# 3. Throughput: many worlds, each starting from the reference pose with small random joint offsets.
one_second = int(round(1.0 / m.opt.timestep))


def body(_, batch):
    return jax.vmap(mjx.step, in_axes=(None, 0))(mx, batch)


run_one_second = jax.jit(lambda batch: jax.lax.fori_loop(0, one_second, body, batch))
base = mjx.make_data(mx)
any_ok = False
for worlds in (256, 1024, 4096, 8192):
    try:
        noise = jax.random.uniform(jax.random.PRNGKey(worlds), (worlds, m.nq - 7), minval=-0.05, maxval=0.05)
        batch = jax.vmap(lambda n: base.replace(qpos=base.qpos.at[7:].add(n)))(noise)
        start = time.perf_counter()
        batch = jax.block_until_ready(run_one_second(batch))  # compiles, then the first second (the fall)
        first_s = time.perf_counter() - start
        start = time.perf_counter()
        batch = jax.block_until_ready(run_one_second(batch))  # the second second: lying, contact-heavy
        wall_s = time.perf_counter() - start
        qpos = np.array(batch.qpos)
        entry = {
            "compile_and_first_second_s": first_s,
            "wall_s_per_simulated_second": wall_s,
            "sim_seconds_per_s": worlds / wall_s,
            "samples_per_s_at_30hz": 30 * worlds / wall_s,
            "samples_per_hour_at_30hz": 30 * worlds / wall_s * 3600,
            "worlds_with_nan": int(np.isnan(qpos).any(axis=1).sum()),
            "lowest_root_height_m": float(np.nanmin(qpos[:, 2])),
        }
        any_ok = True
    except Exception as error:  # e.g. out of GPU memory at the largest batch
        entry = {"error": f"{type(error).__name__}: {error}"[:800]}
    results[f"worlds_{worlds}"] = entry
    print(f"{worlds} worlds:", entry, flush=True)

with open(out_path, "w") as f:
    json.dump(results, f, indent=2)
sys.exit(0 if any_ok else 1)
