"""Project ATHLETE - imports 100STYLE clips: trims them (Frame_Cuts.csv) and fits them to the athlete.

100STYLE (Mason, Starke, Komura 2022), CC BY 4.0: credit and changes in Docs/MotionData.md. Raw BVH files
(60 fps) in Saved/MotionData/100STYLE/100STYLE/<Style>/<Style>_<type>.bvh. Types: FW/FR forward walk/run,
BW/BR backward, SW/SR sidestep, ID idle, TR1-3 transitions between them.

Each clip becomes DEFAULT/mocap/AthleteReference/100style_<style>_<type>.npz (+ _report.json), with feet
placed per clip (flight phases kept). Clips run in parallel processes.

Usage:
    Intermediate/LocoMuJoCoPython/Scripts/python.exe Scripts/MuJoCo/import_100style.py [--styles Neutral Rushed]
        [--types FR FW SR SW BR TR1 TR2 TR3] [--jobs 6] [--max-seconds N] [--data DIR] [--out DIR]
    ... import_100style.py --stage DIR   copies the default selection's BVH files + Frame_Cuts.csv (flat) to DIR,
                                         for upload to a training VM (run.sh imports them there)
"""

import argparse
import csv
import json
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from retarget_motion import PROJECT_ROOT  # noqa: E402

DATA = PROJECT_ROOT / "Saved" / "MotionData" / "100STYLE" / "100STYLE"
OUT = PROJECT_ROOT / "Saved" / "MuJoCo" / "motions"
# The plainer styles: normal legs, arms doing something an athlete might (Dataset_List.csv descriptions).
# Rushed, Angry (arms pumping, fists closed) and BigSteps run fastest: forward-run p90 2.3-2.7 m/s; most
# styles jog at 1.3-2.0 m/s (small capture volume). All 100 styles together have only ~217 s above 2 m/s.
DEFAULT_STYLES = ("Neutral", "Rushed", "Angry", "BigSteps", "Followed", "CrowdAvoidance", "Proud")
DEFAULT_TYPES = ("FR", "SR", "TR1", "TR2", "TR3") # forward walking left out: we have ~30 min of it
PREFIX = "100style_"


def frame_cuts(data):
    """{style: {type: (start, stop)}} from Frame_Cuts.csv (types without a capture are left out)."""
    cuts = {}
    with open(Path(data) / "Frame_Cuts.csv", newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            style = row["STYLE_NAME"]
            cuts[style] = {}
            for key in row:
                if key.endswith("_START") and row[key] not in ("N/A", ""):
                    kind = key[:-len("_START")]
                    cuts[style][kind] = (int(row[key]), int(row[f"{kind}_STOP"]))
    return cuts


def bvh_path(data, style, kind):
    """<data>/<Style>/<Style>_<type>.bvh (as unzipped) or <data>/<Style>_<type>.bvh (staged, flat)."""
    nested = Path(data) / style / f"{style}_{kind}.bvh"
    return nested if nested.exists() else Path(data) / f"{style}_{kind}.bvh"


def fit(job):
    style, kind, frames, max_seconds, data, out = job
    import retarget_motion as rm  # in the worker: each process loads MuJoCo/LocoMuJoCo itself
    source = rm.BvhSource(bvh_path(data, style, kind), prefix=PREFIX, frames=frames)
    source.description += " (100STYLE, CC BY 4.0)"
    try:
        report = rm.retarget(source, Path(out), max_seconds)
    except Exception as error:  # report it and carry on with the other clips
        return {"dataset": source.name, "error": repr(error)}
    return {k: report[k] for k in ("dataset", "frames", "duration_s", "root_speed_mps_median",
                                   "share_of_frames_both_feet_above_2cm", "stance_foot_slide_mps_median",
                                   "joints_at_limit_share_of_frames", "ik_restarts_kept", "seconds")} | {
        "fit_error_p95_max_m": max(v["p95"] for v in report["fit_error_m"].values()), "scale": source.scale}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--styles", nargs="+", default=list(DEFAULT_STYLES))
    parser.add_argument("--types", nargs="+", default=list(DEFAULT_TYPES))
    parser.add_argument("--jobs", type=int, default=6)
    parser.add_argument("--max-seconds", type=float, default=None)
    parser.add_argument("--data", default=str(DATA), help="100STYLE folder (unzipped, or staged flat)")
    parser.add_argument("--out", default=str(OUT), help="motions folder (as retarget_motion.py --out)")
    parser.add_argument("--stage", default=None, help="only copy the selected BVH files + Frame_Cuts.csv here")
    args = parser.parse_args()
    cuts = frame_cuts(args.data)
    jobs = [(style, kind, cuts[style][kind], args.max_seconds, args.data, args.out)
            for style in args.styles for kind in args.types if kind in cuts[style]]
    if args.stage:
        import shutil
        stage = Path(args.stage)
        stage.mkdir(parents=True, exist_ok=True)
        shutil.copy2(Path(args.data) / "Frame_Cuts.csv", stage)
        for style, kind, *_ in jobs:
            shutil.copy2(bvh_path(args.data, style, kind), stage)
        print(f"staged {len(jobs)} BVH files + Frame_Cuts.csv in {stage}")
        return
    print(f"{len(jobs)} clips: " + ", ".join(f"{job[0]}_{job[1]}" for job in jobs), flush=True)
    with ProcessPoolExecutor(max_workers=args.jobs) as pool:
        for result in pool.map(fit, jobs):
            print(json.dumps(result), flush=True)


if __name__ == "__main__":
    main()
