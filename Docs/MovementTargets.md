# Football movement targets

What ATHLETE's football movement should achieve: summary numbers for sprinting, backpedalling and
shuffling. They are future test targets (the learned locomotion runs at ≤ 3 m/s today, §9 of
Milestone4_Locomotion.md). Compiled 2026-10-04.

## Sources and what was taken from them

- **Game animation (reference only, facts only).** 12 locomotion clips the user decoded from commercial
  football games (EA CFB27/Madden assets, in the user's own files outside this project). Only summary
  statistics were measured: average and peak speed, step rate, step length, contact time, airborne
  share, step pattern, hip height and trunk lean, one number per clip. No trajectories, poses or timing
  sequences were copied, and the clips are not used as motion data (Docs/MotionData.md). Game animation
  is authored for feel, so its speeds can be exaggerated: it's one input, not ground truth.
- **Published measurements** of real people (links at the end).
- The measurement script lives outside the repo, because it reads the user's private files.

Method: forward kinematics of each clip (validated: travel distances match the user's own checks to
0.01 m); 30 fps source, so contact times are ±0.033 s. Step = a foot touchdown (foot within 3.5 cm of
the clip's floor). Step length is measured along the direction of travel. "Feet pass" = the feet change
order along the direction of travel (running: once per step; shuffling: never). Clips under 1 s give
only rough step numbers.

## 1. Sprint (straight line, top speed)

| | Game clips (4 sprint/scramble clips) | Real (published) |
|---|---|---|
| Top speed | 9.6-10.1 m/s (21.5-22.7 mph) | NFL ball carriers: ~9.4-9.8 m/s (21-22 mph) on breakaway plays; fastest recorded ~10.4 m/s (23.2 mph) |
| Step rate | 4.0-4.6 steps/s | elite sprinter max ~5.3 steps/s at 10.8 m/s |
| Step length | 2.0-2.5 m | ~2.0-2.3 m at max speed for team-sport athletes (ESTIMATE: speed ÷ step rate, not a cited measurement) |
| Foot contact | ~0.10-0.13 s | shorter contact goes with faster sprinting (team-sport and sprint studies) |
| Both feet off the ground | 45-62% of the time | |
| Trunk lean (forward) | 5-15° at top speed; more while accelerating | |
| Hip height | 0.93-0.99 m (≈ standing) | |

**ATHLETE target:** top speed 9-10 m/s for a fast skill player (position-dependent later), step rate 4-5
steps/s, contact ≤ 0.12 s, a clear flight phase. The game numbers match real NFL tracking here.

## 2. Running at moderate speed (agile run)

| | Game (1 clip, 3.6 s) | |
|---|---|---|
| Speed | 6.7-6.9 m/s | |
| Step rate / length | ~3.4 steps/s, ~2.0 m | |
| Contact | ~0.13 s; airborne ~60% | |
| Trunk lean | ~18° forward | |

## 3. Backpedal

| | Game (3 clips, ≤ 1 s each) | Real (published) |
|---|---|---|
| Speed | 3.4 (jog) / 5.2 (run) / 6.4 m/s (fast) | max backward running 5.1 m/s (30 active men); backward running ≈ 70% of forward max |
| Step rate | ~4.3-4.4 steps/s | ~4.1 steps/s at max backward speed |
| Step length | 1.2-1.5 m | ~1.25 m |
| Contact | ~0.18 s | stance ≈ 31% of stride time (longer than forward running's 26%) |
| Hip height | 0.88-0.89 m (~10 cm lower than running) | DBs "compress", lowering the centre of mass and shortening stride |
| Trunk lean | -15° to +4° (upright to leaning over the toes) | |

**ATHLETE target:** backpedal 3-5 m/s, ~4 steps/s, steps ~1.2 m, hips ~10 cm below running height. The
game's 6.4 m/s "fast" backpedal is beyond measured human backward running: don't target it.

## 4. Lateral and diagonal shuffle

| | Game (4 clips, ≤ 1.4 s each) | Real (published) |
|---|---|---|
| Speed | 4.7-4.8 (lateral), 6.0-6.2 m/s (diagonal) | lab lateral shuffles ~1.8-2.1 m/s (controlled tests; not max effort) |
| Step rate | 4.6-5.5 steps/s | |
| Feet pass each other | never (0 of all steps) | the defining feature of a shuffle: the lead foot stays ahead |
| Contact | ~0.20-0.27 s | |
| Hip height | 0.78-0.85 m (lateral 20 cm, diagonal 13 cm below running) | |
| Trunk | upright (within ±4°) | |

**ATHLETE target:** feet never cross; hips ~15-20 cm below running height; upright trunk; 4.5-5.5
steps/s. Speed: 2-3.5 m/s until better real data exists. The game's 4.7-6.2 m/s is about twice the
lab values (which aren't maximal), so treat the game speeds as an upper bound only.

## How to use this

- As acceptance tests once the athlete has these moves (a future milestone): the same measures can be
  computed from the simulation (foot contacts are exact there).
- As a check on any new motion data (licensed packs, our own capture): measure it the same way.
- Before relying on any number, prefer our own capture (the plan in memory/Docs) over both sources.

## References

- NFL Next Gen Stats top speeds: [NBC Sports Bay Area, fastest 2022 players](https://www.nbcsportsbayarea.com/nfl/san-francisco-49ers/breaking-down-fastest-2022-nfl-players-using-next-gen-stats/1435104);
  [SI, Tyreek Hill / Trey Palmer speeds](https://www.si.com/nfl/buccaneers/news/tampa-bay-buccaneers-bucs-news-trey-palmer-tyreek-hill-nfl-free-agency)
- Backward running: [Arata, Comparison of high speed backward & forward running](https://pages.uoregon.edu/btbates/backward/alan2.htm);
  [SimpliFaster, backpedal training](https://simplifaster.com/articles/backpedal-training-athletic-performance/)
- Lateral shuffle: [Comparison of lateral shuffle and side-step cutting (Gait & Posture)](https://www.sciencedirect.com/science/article/abs/pii/S096663621500987X);
  [Fatigue after 3-min lateral shuffle at different speeds (PMC11812171)](https://pmc.ncbi.nlm.nih.gov/articles/PMC11812171/)
- Sprint step frequency: [ISBS proceedings, elite sprinter step frequency](https://ojs.ub.uni-konstanz.de/cpa/article/view/6610/5969)
