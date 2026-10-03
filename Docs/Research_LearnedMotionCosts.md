# Research: What Learned, Human-Like Motion Costs

*Compute and motion data for learning the athletes' movement. Prices checked 2026-09-28/29.
Input to the physics-engine decision that follows Milestone 4.*

## 1. Summary

- **Compute is cheap if the simulator is fast, and prohibitive if it isn't.** A published learned
  locomotion skill (AMP) needs about 39 million simulation samples: **6 minutes** on one GPU in a GPU
  simulator, **~30 hours** on 16 CPU cores, and **~32 hours** on this machine in Unreal/Chaos (our
  measured 1.2 million decisions per hour). A full human motion library (PHC) takes **about a week on one
  A100 GPU**, which in Chaos would take years.
- **Renting that GPU time is affordable**: an A100 is $1.19–1.99/h and an H100 $1.99–4.29/h on demand. A
  week-long library run is about **$200–700**; a prototype phase with many shorter runs is likely
  **$500–2,000**. The number of iterations, not the price per hour, sets the bill.
- **GPU training needs the athlete in a GPU simulator.** Chaos has none. This ties the cost question to
  the engine decision: MuJoCo (GPU variants for training) or NVIDIA PhysX (Isaac Lab).
- **Motion data: the big free datasets are mostly research-only.** CMU's is usable in a commercial game;
  AMASS and LaFAN1 are not (AMASS explicitly forbids training networks for commercial use). None covers
  football skills (stances, cuts at speed, tackling, blocking, catching).
- **Custom football capture: roughly $10,000–50,000** for a focused set (2–3 studio days, 30–60 minutes
  of usable motion). Clean-up is most of it. Markerless capture is far cheaper for prototyping.

## 2. Training compute

### Reference points

| Method / setup | Samples | Time | Source |
|---|---|---|---|
| AMP, GPU simulator (Isaac Gym, 4,096 parallel environments, 1 GPU) | ~39 M | ~6 min | Isaac Gym paper |
| AMP, original CPU implementation (PyBullet, 16 cores) | ~39 M | ~30 h | Isaac Gym paper |
| PHC: a whole motion library + composer, 1 A100, Isaac Gym | billions | ~1 week | PHC paper |
| **Project ATHLETE, Unreal/Chaos, 32 athletes, CPU training (measured)** | 1.18 M | 1 h | Milestone 4 spike |

At our measured rate, an AMP-scale skill is about 32 hours per training run, and every change to the
reward or the body means another run. Library-scale imitation isn't feasible in Chaos at all.

### Cloud GPU prices (on-demand, per GPU-hour)

| GPU | RunPod Community | RunPod Secure | Lambda |
|---|---|---|---|
| RTX 4090 | $0.34 | $0.74 | n/a |
| RTX 5090 | $0.69 | $0.99 | n/a |
| L40S | $0.79 | $1.09 | n/a |
| A100 (PCIe / SXM) | $1.19 / $1.39 | $1.59 | $1.99 (40 GB PCIe) / $2.79 (80 GB SXM, 8x) |
| H100 (PCIe / SXM) | $1.99 / $2.69 | $2.89 / $3.49 | $3.29 / $3.99–4.29 |
| B200 | $5.98 | $6.79 | $6.69–6.99 |

Marketplace hosts (Vast.ai) go lower still, with variable reliability.

### Budget scenarios

- **One skill (AMP scale), GPU simulator:** minutes to a few hours, **under $10** per run.
- **Motion library (PHC scale):** ~1 week on one A100, **~$200–270** (RunPod), or ~$450–700 on an H100
  (fewer hours).
- **Prototype phase:** tens of runs while the reward, body and observations settle, **~$500–2,000**.
- Renting beats buying unless the GPU runs most of the time: an RTX 4090 rents at $0.34–0.74/h.

### Google Cloud (the account set up for this project)

- **A free-trial account can't use GPUs** and can't request GPU quota. Upgrading to a paid account unlocks
  them; unused trial credit carries over until 90 days after sign-up.
- After upgrading, **request GPU quota**: all regions (`GPUS_ALL_REGIONS`) plus the GPU type in one region.
  It starts at 0.
- **Budgets don't stop Compute Engine spending.**
  - Budget alerts only email.
  - The newer spend-cap budgets (Preview) cover Gemini, Vertex AI and Cloud Run only, and don't stop
    running VMs or disks.
  - Protection has to come from the VMs themselves:
    - Spot VMs, and a maximum run duration (`--max-run-duration`) that deletes the VM when time is up.
    - Nothing left running.
    - An alert as a backstop.
- **Indicative GPU prices** (vary by region):

| GPU | On-demand | Spot |
|---|---|---|
| L4 | ~$0.70–1.32/h | ~$0.3–0.8/h |
| A100 40 GB | ~$3.67/h | ~$1.1–2.2/h |
| H100 | ~$10.6–11/h | ~$3.3–6.3/h |

  An L4 is enough for humanoid imitation training at our scale.

- **Measured for our setup** (Cloud Billing catalog, us-central1, 2026-09-29):

| Item | Price |
|---|---|
| Spot `g2-standard-8` (8 vCPU × $0.01499 + 32 GiB × $0.00176 + L4 $0.336) | **$0.51/h** |
| On-demand `g2-standard-8` | $0.85/h |
| 100 GiB balanced boot disk | $0.014/h while the VM exists |
| Cloud NAT external IP | $0.005/h **always**: ~$3.60/month even when no VM runs |
| Cloud NAT traffic | $0.045/GiB. A run's package install is ~3 GiB, ~$0.14. |

  **$300 buys roughly 550 Spot L4 hours.** The first day's 6 launches, 80 VM-minutes and 15.5 GiB of downloads
  cost ~$1.40, half of it downloads. A pre-built VM image would remove those.

Caveats: GPU simulators run on NVIDIA hardware (JAX GPU also on Linux or WSL2). This machine's AMD Radeon
can run the athletes at runtime but not the usual training stacks. Training would be remote.

## 3. Motion data

### Licenses

| Dataset | Commercial game use | Notes |
|---|---|---|
| CMU Graphics Lab Motion Capture Database | **Yes** | May be included in commercially sold products; may not be resold as data. General motion; varied quality; older optical capture. |
| AMASS (Max Planck) | **No** | Non-commercial research, education and art only. Explicitly forbids training networks for commercial use. A commercial license is available on request. |
| LaFAN1 (Ubisoft La Forge) | **No** | CC BY-NC-ND 4.0: non-commercial research only. |
| Commercial libraries (e.g. mocap vendors' sports packs) | Varies | Check each vendor's terms for **training** use, not just animation use. |

**The football gap.** General datasets don't cover the skills the game lives on: three-point stances and
starts, cuts at speed, tackling, blocking, catching and ball carrying. Those need custom capture.

### Custom capture

| Item | Typical 2026 cost |
|---|---|
| Optical studio (Vicon/OptiTrack), mid-tier | $1,500–3,000 per day (budget $500–900; premium $3,000–5,000+) |
| Performers | $200–600 per day (trained athletes likely more) |
| Clean-up | 2–4 h per minute of final motion, at $50–150/h |
| Markerless (e.g. Move AI) | about $2.50–10 per processed minute; quality for fast athletic motion unproven here |

**A focused football set** (2–3 studio days, 30–60 minutes of usable motion): studio $3,000–9,000,
performers $1,000–5,000, clean-up $4,500–36,000. That's **roughly $10,000–50,000**.

Physics-based imitation is more forgiving of imperfect data than animation playback is. The physics
won't reproduce impossible motion, and the policy learns what the body can actually do. So markerless or
cheaper capture may be good enough to prototype with.

### Motion from video

*Researched 2026-09-29.*

**The technique works.** *Learning Physically Simulated Tennis Skills from Broadcast Videos* (NVIDIA / Simon
Fraser, SIGGRAPH 2023, best paper honourable mention) trained physically simulated tennis players from
broadcast footage:
1. A pose estimator turns video into 3D motion on the ground. Current ones handle moving cameras:
   GVHMR, WHAM.
2. Physics-based imitation cleans the motion. Video poses are noisy (sliding feet, jitter, guessed depth),
   and physics discards what a real body can't do.
3. A high-level policy chains the skills.

It trained in a GPU simulator (Isaac Gym).

**No usable football skeletons exist.**

| Source | What it is | Usable? |
|---|---|---|
| NFL / Sony Hawk-Eye "SkeleTrack" | 29 points per player, 32 cameras per stadium | No: in background testing (2025), not released even to teams. Access would need a deal with the NFL. |
| Next Gen Stats / Big Data Bowl | Player positions, speed and facing at 10 Hz; no skeletons | No: licensed for the contest only, to be destroyed afterwards |
| SportsPose, AthletePose3D | Accurate 3D athletic motion | No: non-commercial research only (AthletePose3D), not football |

**Broadcast football is hard footage:**
- Pile-ups hide the bodies (linemen, tackles, blocks).
- Players are small, and the camera pans, zooms and cuts.
- Pads and helmets hide body shape and joints.
- Fast motion blurs at 30–60 fps.

Open-field movement would extract best: routes, cuts, backpedals.

**Legal (not legal advice).**
- The footage is the NFL's, and US courts have split on whether training on copyrighted material is fair use.
- Identifiable players' motion raises likeness questions (NFLPA group licensing).
- Most video-to-pose models and the SMPL body model they output are research-only. SMPL can be licensed
  commercially through Meshcapade.

**Plan:**
- Film our own athletes with 4–8 synchronized cameras: we own the footage and have consent. Multiple
  views are much more accurate than broadcast, and there are no pads.
- Use broadcast only for private research and validation (cut timing, stride at top speed).
- Keep CMU for general movement.

## 4. Open-source building blocks

*Checked 2026-09-29.*

A code license doesn't cover pretrained weights or motion data. Pretrained weights also belong to their
own body and physics, so these are mostly frameworks for training **our** athlete.

| Project | What it gives us | License | Catch |
|---|---|---|---|
| MimicKit (Xue Bin Peng) | DeepMimic, AMP, ASE, ADD, LCP, SMP, AWR | Apache-2.0 | Isaac Gym / Isaac Lab / Newton (NVIDIA GPU). Assets and data are downloaded separately; their license isn't stated on the repo page. |
| ProtoMotions3 (NVIDIA) | Full framework incl. MaskedMimic. Backends: Isaac Gym, Isaac Lab, Newton, **MuJoCo**, Genesis | Apache-2.0 | Its SMPL humans and AMASS data are non-commercial: use our body and CMU data |
| MoCapAct (Microsoft) | **Pretrained** policies for 2,500+ CMU clips plus a multi-clip policy (MuJoCo, dm_control) | Code MIT; data CDLA-Permissive-2.0 | The cleanest pretrained option (CMU data is commercial-OK), but DeepMind's CMU humanoid body, research-grade quality. A baseline to compare against. |
| MuJoCo Playground (DeepMind) | GPU (MJX) humanoid stand, walk and run training in minutes | Apache-2.0 | Test bed for the engine spike |
| LocoMuJoCo | Imitation benchmark incl. muscle-driven humans | MIT | Its bundled LAFAN1 and AMASS data are non-commercial |
| MyoSuite (Meta) | Musculoskeletal (muscle and tendon) human models in MuJoCo | Apache | The long-term "as human as possible" path |
| Meta Motivo | Pretrained whole-body "behavioral foundation model" | **CC BY-NC** | Research only |

**Engine finding.** Newton is an open-source GPU physics engine:
- Apache-2.0, run by the Linux Foundation; built by NVIDIA, Google DeepMind and Disney Research.
- It includes MuJoCo Warp.

So one MuJoCo model of the athlete can run on the CPU inside Unreal (runtime) and on GPUs (training):
the same physics on both sides.

## 5. The earlier Unity attempt

*Reviewed 2026-09-29.*

`Documents\Codex\2026-09-20\project-physics-first-football-simulator-you\outputs\` (Unity 6.6, ML-Agents)
has a capsule lab, A0/A1 ragdoll experiments, an ML-Agents toolchain test and `WalkerBaseline`.

**The walking policy there is Unity's pretrained ML-Agents Walker.** The project's own run was a
61,440-step pilot (0.2% of the 30M-step recipe; mean reward −0.64, not walking).

**It can't be converted.** The ONNX network would load in Unreal, but a policy only works in the body and
physics it was trained in:

| Unity Walker | ATHLETE |
|---|---|
| 1.5× gravity (−14.7 m/s²) | real gravity |
| joint projection on all 150 joints | projection off (fake stability) |
| engine joint motors with policy-set strength | Hill-limited muscles |
| 75 Hz physics | 480 Hz |
| its own proportions and masses | de Leva proportions |
| 243 inputs, 39 outputs | 248 inputs, 45 outputs |

**What transfers is the approach, which our Learning Agents spike already uses:**
- body parts observed in the athlete's own frame;
- a velocity-matching reward.

**It is also a data point.** Unity/PhysX trained at ~1,200 steps/s here (2–4× our Chaos rate, at a much
coarser 75 Hz). The Walker still needs 30M steps for a gait that isn't human-like, which supports
imitation learning in a GPU simulator.

## 6. Recommendations

1. **Choose a physics engine that has a GPU training path matching the runtime physics**: MuJoCo (CPU in
   Unreal, MuJoCo Warp / MJX / Newton on GPUs) first, NVIDIA PhysX (Isaac Lab) as the fallback. This is
   the engine spike.
2. **Train with an open framework** (MimicKit or ProtoMotions3) on **our** athlete's body, with CMU data
   plus our own multi-camera captures of a few football moves, on rented GPUs: about $500–2,000.
   MoCapAct's pretrained CMU policies are the quality baseline.
3. **Budget custom optical capture of football skills (~$10,000–50,000) for production**, once the
   pipeline is proven, and ask Max Planck about a commercial AMASS license if broad everyday motion is
   needed.

## Sources

- [Runpod pricing](https://www.runpod.io/pricing) (fetched 2026-09-28)
- [Lambda pricing](https://lambda.ai/pricing) (fetched 2026-09-28)
- [GPU cloud pricing comparisons, 2026](https://www.spheron.network/blog/gpu-cloud-pricing-comparison-2026/)
- [Isaac Gym: High Performance GPU-Based Physics Simulation for Robot Learning](https://arxiv.org/pdf/2108.10470) (AMP sample counts and timings)
- [Perpetual Humanoid Control for Real-time Simulated Avatars (PHC)](https://openreview.net/pdf?id=vYZwDiOATzn)
- [AMASS license](https://amass.is.tue.mpg.de/license.html)
- [CMU Graphics Lab Motion Capture Database (re3data)](https://www.re3data.org/repository/r3d100012183) and [mocap.cs.cmu.edu](https://mocap.cs.cmu.edu/)
- [LaFAN1 (resolved) repository](https://github.com/orangeduck/lafan1-resolved)
- [Motion capture cost guide (MoCap Online)](https://mocaponline.com/blogs/mocap-news/motion-capture-cost-guide)
- [Motion capture cost for video games, 2026](https://www.mimicgaming.com/post/motion-capture-cost-video-games)
- [Move AI pricing: plans and credits](https://docs.move.ai/knowledge/move-ai-pricing-plans-credits)
- [Learning Physically Simulated Tennis Skills from Broadcast Videos](https://xbpeng.github.io/projects/Vid2Player3D/index.html) ([ACM](https://dl.acm.org/doi/10.1145/3592408))
- [GVHMR](https://zju3dv.github.io/gvhmr/), [WHAM](https://wham.is.tue.mpg.de/)
- [NFL deep dive: 32 cameras, virtual measurement and skeletal tracking (Sports Video Group, 2025)](https://www.sportsvideo.org/2025/11/20/nfl-deep-dive-how-32-cameras-at-each-stadium-drive-virtual-measurement-boundary-replays-and-skeletal-tracking/), [Sportico](https://www.sportico.com/leagues/football/2025/nfl-virtual-measurement-down-tech-sony-cameras-1234868201/)
- [Big Data Bowl terms and conditions](https://operations.nfl.com/gameday/analytics/big-data-bowl/terms-and-conditions/)
- [SportsPose](https://arxiv.org/pdf/2304.01865), [AthletePose3D](https://github.com/calvinyeungck/AthletePose3D)
- [SMPL commercial licensing (Meshcapade)](https://meshcapade.com/smpl/)
- [MimicKit](https://github.com/xbpeng/MimicKit), [ProtoMotions](https://github.com/NVlabs/ProtoMotions), [MoCapAct](https://github.com/microsoft/MoCapAct), [MuJoCo Playground](https://github.com/google-deepmind/mujoco_playground), [LocoMuJoCo](https://github.com/robfiras/loco-mujoco), [MyoSuite models](https://github.com/MyoHub/myo_sim), [Meta Motivo](https://github.com/facebookresearch/metamotivo)
- [Newton](https://github.com/newton-physics/newton), [MuJoCo Warp](https://github.com/google-deepmind/mujoco_warp)
- [Google Cloud free program (GPU restriction)](https://docs.cloud.google.com/free/docs/free-cloud-features), [budgets and alerts](https://docs.cloud.google.com/billing/docs/how-to/budgets), [spend-cap budgets](https://docs.cloud.google.com/billing/docs/how-to/budgets-spend-caps), [GPU prices (CloudZero, 2026)](https://www.cloudzero.com/blog/cloud-gpu-pricing-comparison/)
