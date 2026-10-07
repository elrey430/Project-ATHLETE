# Motion data licenses: what we can use

Researched 2026-10-05. Not legal advice: confirm the terms before shipping, and ask a lawyer about the
grey areas marked below. Prices and terms change, so re-check them when buying.

## What a license must allow

ATHLETE uses motion data in **two ways**, and a license must allow both:

- **(A) Runtime reference.** The clips ship inside the game: the motion matcher plays them as the
  reference the athlete follows. This is ordinary "use in a game".
- **(B) Training.** The tracking policy is trained on the clips (machine learning). Many licenses
  written since 2023 forbid this, or allow it only for a fee.

It must also allow **commercial** use, and **modification** (we retarget, trim, mirror and resample).

## Free

| Source | License | (A) runtime | (B) training | What it gives us | Verdict |
|---|---|---|---|---|---|
| **100STYLE** (in use) | CC BY 4.0 | yes | yes | walk/jog/run transitions, sidestep runs, 100 styles (slow: studio) | **Using it**; credit in Docs/MotionData.md §3 |
| **CMU Graphics Lab** | free, "may include in commercially-sold products; may not resell the data, even converted" | yes | yes (widely used for ML, e.g. MoCapAct) | 2,500+ clips: walking, jogging, running, turns, starts/stops, some sports | **Next free source.** Needs an ASF/AMC reader or the BVH conversion |
| **Epic Game Animation Sample** (UE 5.8) | free; "UE-Only Content": only in Unreal Engine products | yes (it's made for motion matching in UE) | **grey area**: Epic's terms forbid use "as training inputs to Generative AI Programs"; we found no definition, and whether a motion-tracking controller counts is unclear | 500+ AAA mocap animations: starts, stops, pivots, sprints, turns | **Best free quality.** Ask Epic (licensing) or a lawyer before training on it |
| Quaternius Universal Animation Library | CC0 (public domain) | yes | yes | 120+ hand-keyed animations incl. 8-way locomotion, jog, sprint | Usable, but hand-keyed, not mocap: physical plausibility unproven. Low priority |
| Mixamo (Adobe) | free for commercial games; **no ML training**, no bulk download for ML | yes | **no** | large library | **Not usable**: the tracker must train on what the matcher plays |
| LAFAN1, AMASS, Bandai Namco, SFU, MotionPersona, OpenBiomechanics, BOXRR-23, Human3.6M | non-commercial | no | no | | **Not usable** |
| ACCAD (Ohio State) | not found | | | | Not usable until its terms are confirmed |
| Animation extracted from commercial games | the publisher's | no | no | | **Not usable** (Docs/MotionData.md) |

## Paid

| Source | License | (B) training | Football content | Cost (as found) | Verdict |
|---|---|---|---|---|---|
| **MoCap Online** (Motus Digital) | Standard: perpetual, commercial games, no resale of raw files | **separate "AI Usage Permit"**: covers training, deployment, retargeting/synthesis *within your licensed projects*; forbids models that generate mocap data *for distribution* (ours doesn't) | none found (locomotion/"mobility" packs) | packs priced per product; **permit price not published (ask)** | **Clearest commercial route** for more locomotion: buy packs + permit |
| ActorCore (Reallusion) | Standard: games need the right tier; **no AI training** | only under an **Enterprise license** (supports ML training) | not checked | Enterprise: ask sales | Possible, via Enterprise only |
| Rokoko Motion Library | commercial use allowed; moves ~$3-6 each (150 free) | **not stated (ask)** | not checked | ~$3-6 per move | Only after written confirmation of ML use |
| Fab marketplace packs | Standard license, Personal (< $100k revenue) / Professional tiers | per listing: "NoAI" tag = no generative-AI data collection; Epic-provided materials excluded from generative-AI training | some sports packs exist | set by each seller | Check each listing's AI tag and ask the seller |

### Our own capture (football moves: cuts, jukes, backpedal, routes)

No dataset we found has football-specific movement with a license that allows both uses. Capturing our
own gives full ownership. Put **full rights including machine-learning use** in the contract with
the studio and the performers.

| Option | Cost (as found) | Notes |
|---|---|---|
| Optical mocap studio | $500-900/day (small/academic), **$1,500-3,000/day (typical mid-tier, Vicon/OptiTrack)**, $3,000-5,000+ (top tier) | Best quality and fast moves; volume size limits sprints; plus athletes' fees |
| Markerless multi-camera (Move.ai Pro) | $995 for a 30-day trial (~1,000 s of processing); or ~$0.043 per second per camera (×1.4 at 60 fps): 4 cameras ≈ $0.24 per captured second, **30 min ≈ $430** | Can be shot on a real field (full sprint speed). Check that the output is ours, including for ML |

## Recommendation

1. **Now, free:** add **CMU** running, jogging and turning clips, which have the clearest commercial and
   ML terms.
2. **Ask Epic** whether training a motion-tracking controller on the Game Animation Sample is allowed.
   If yes, it's the biggest free quality step: AAA locomotion with starts, stops and pivots, made for
   motion matching in UE.
3. **Paid locomotion:** MoCap Online packs + AI Usage Permit (ask the permit price first).
4. **Football moves:** our own capture. A day at a mid-tier optical studio (~$2-3k plus athletes), or
   markerless capture on a field (~$400-1,000), with a contract that grants us all rights, including
   ML training.

## Sources

- 100STYLE: [project page](https://www.ianxmason.com/100style/), [Zenodo](https://zenodo.org/record/8127870)
- CMU terms (quoted via [4TU FBX conversion](https://data.4tu.nl/datasets/0448aab2-3332-449f-a8e2-d208cb58c7df))
- Epic: [Game Animation Sample](https://www.unrealengine.com/en-US/blog/game-animation-sample),
  [Fab listing](https://www.fab.com/listings/880e319a-a59e-4ed2-b268-b32dac7fa016),
  [forum: UE-only use](https://forums.unrealengine.com/t/what-can-i-do-with-game-animation-sample-that-recently-was-at-100-off/1903980),
  [Fab licenses](https://dev.epicgames.com/documentation/en-us/fab/licenses-and-pricing-in-fab),
  [UE EULA](https://www.unrealengine.com/eula/unreal) (page blocked our fetch; generative-AI clause quoted from search results)
- Mixamo: [Adobe FAQ](https://helpx.adobe.com/creative-cloud/faq/mixamo-faq.html), [license guide](https://www.licenseorg.com/guide/3d-assets/mixamo)
- Quaternius: [Universal Animation Library](https://quaternius.com/packs/universalanimationlibrary.html)
- Non-commercial sets: [SFU](https://mocap.cs.sfu.ca/), [MotionPersona](https://huggingface.co/datasets/myshi/MotionPersona),
  [Bandai Namco](https://www.cgchannel.com/2022/05/download-3000-free-mocap-moves-from-bandai-namco-research/)
- MoCap Online: [legal](https://mocaponline.com/pages/legal), [AI Usage Permit](https://mocaponline.com/pages/ai-usage-permit)
- ActorCore: [Reallusion content EULA](https://www.reallusion.com/Content/EULA/EULA.htm), [Enterprise licensing](https://reallusion.com/plan-and-pricing/enterprise)
- Rokoko: [CG Channel on the Motion Library](https://www.cgchannel.com/2020/03/get-150-free-mocap-moves-from-rokokos-motion-library)
- Capture costs: [MoCap Online cost guide 2026](https://mocaponline.com/blogs/mocap-news/motion-capture-cost-guide),
  [Move.ai pricing](https://docs.move.ai/knowledge/move-ai-pricing-plans-credits)
