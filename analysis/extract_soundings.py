#!/usr/bin/env python3
"""extract_soundings.py -- join RFND / POS / ATT from an ArduPilot log into soundings.

The vehicle logs depth, position and attitude separately, at firmware rates, and never
joins them (ADR-0007). This does the join, on a laptop, where getting it right costs
nothing but time.

    python3 extract_soundings.py --log 2026-09-06-lake.BIN --out soundings_raw.csv \
        --draft-m 0.21 --sound-speed-scale 0.9767

Reads the same three message types you would pull with
    mavlogdump.py --types RFND,POS,ATT --format csv <log>
but in process, so the merge tolerances are visible in one place.

ORDER OF CORRECTIONS MATTERS:

    depth = (raw_distance * sound_speed_scale) + draft_m

The sounder measures a round-trip time and converts it with an assumed sound speed, so the
scale factor corrects the *measured water column* -- which starts at the transducer face.
Draft is a fixed geometric offset from the transducer face up to the waterline and is not
subject to that scaling. Applying draft first and then scaling would scale the draft too,
introducing an error proportional to the draft on every single sounding.

Nothing is defaulted. --draft-m and --sound-speed-scale are required, because a plausible
default is how an uncalibrated survey ends up looking calibrated. See
data/calibration/.
"""
import argparse
import sys

import pandas as pd

try:
    from pymavlink import mavutil
except ImportError:  # pragma: no cover
    sys.exit("pymavlink is not installed. pip install -r requirements.txt")

# AP_RangeFinder.h -- Status::Good
RNGFND_STATUS_GOOD = 4
# The downward sonar. A second instance would be a different sensor entirely.
RNGFND_INSTANCE = 0
# Half a second of boat travel at survey speed is metres of horizontal error, so the
# position/attitude match must be tight. 200 ms at 1 m/s is 0.2 m.
MERGE_TOLERANCE_MS = 200

WANTED = ("RFND", "POS", "ATT")


def read_log(path):
    """Pull RFND, POS and ATT rows out of a .BIN / .log into three DataFrames."""
    mlog = mavutil.mavlink_connection(path)
    rows = {name: [] for name in WANTED}

    while True:
        msg = mlog.recv_match(type=WANTED)
        if msg is None:
            break
        rows[msg.get_type()].append(msg.to_dict())

    frames = {}
    for name in WANTED:
        if not rows[name]:
            sys.exit(
                f"no {name} messages in {path}.\n"
                f"  RFND missing usually means the rangefinder was never configured or "
                f"never returned a good reading -- the bucket test in ops/bench-tests.md "
                f"exists to catch exactly this before a survey, not after."
            )
        frames[name] = pd.DataFrame(rows[name])
    return frames


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--log", required=True, help="ArduPilot .BIN / .log file")
    ap.add_argument("--out", required=True, help="output CSV of soundings")
    ap.add_argument("--draft-m", type=float, required=True,
                    help="transducer face to waterline, boat loaded as it runs "
                         "(data/calibration/draft.md)")
    ap.add_argument("--sound-speed-scale", type=float, required=True,
                    help="measured / assumed sound speed, e.g. 1465/1500 = 0.9767 "
                         "(data/calibration/sound-speed.md)")
    ap.add_argument("--tolerance-ms", type=int, default=MERGE_TOLERANCE_MS,
                    help=f"max time gap when matching POS/ATT to a sounding "
                         f"(default {MERGE_TOLERANCE_MS})")
    args = ap.parse_args()

    frames = read_log(args.log)
    rfnd, pos, att = frames["RFND"], frames["POS"], frames["ATT"]

    n_total = len(rfnd)

    # ---- filter to the downward sonar, good readings only ------------------------------
    if "Instance" in rfnd.columns:
        n_before = len(rfnd)
        rfnd = rfnd[rfnd["Instance"] == RNGFND_INSTANCE]
        dropped_instance = n_before - len(rfnd)
    else:
        dropped_instance = 0
        print("note: no Instance column in RFND -- single-rangefinder log, nothing to filter")

    n_before = len(rfnd)
    rfnd = rfnd[rfnd["Stat"] == RNGFND_STATUS_GOOD]
    dropped_status = n_before - len(rfnd)

    if rfnd.empty:
        sys.exit("every RFND row was filtered out. Nothing to extract.")

    # ---- join on time ------------------------------------------------------------------
    # merge_asof needs both sides sorted; direction="nearest" so a sounding can match a
    # position sample slightly after it as well as before.
    tol = args.tolerance_ms * 1000  # TimeUS is microseconds
    rfnd = rfnd.sort_values("TimeUS")
    pos = pos.sort_values("TimeUS")
    att = att.sort_values("TimeUS")

    df = pd.merge_asof(rfnd, pos[["TimeUS", "Lat", "Lng", "Alt"]],
                       on="TimeUS", direction="nearest", tolerance=tol,
                       suffixes=("", "_pos"))
    n_before = len(df)
    df = df.dropna(subset=["Lat", "Lng"])
    dropped_nopos = n_before - len(df)

    att_cols = [c for c in ("TimeUS", "Yaw", "Roll", "Pitch") if c in att.columns]
    df = pd.merge_asof(df, att[att_cols],
                       on="TimeUS", direction="nearest", tolerance=tol,
                       suffixes=("", "_att"))
    n_before = len(df)
    df = df.dropna(subset=["Yaw"])
    dropped_noatt = n_before - len(df)

    # ---- corrections, in this order ----------------------------------------------------
    df["raw_dist_m"] = df["Dist"]
    df["depth_m"] = df["raw_dist_m"] * args.sound_speed_scale + args.draft_m

    out = df[["TimeUS", "Lat", "Lng", "Yaw", "raw_dist_m", "depth_m"]].copy()
    out.to_csv(args.out, index=False)

    # ---- say what was dropped, and why -------------------------------------------------
    print(f"log:                    {args.log}")
    print(f"RFND rows read:         {n_total}")
    print(f"  dropped, wrong instance:      {dropped_instance}  (kept Instance == {RNGFND_INSTANCE})")
    print(f"  dropped, status not Good:     {dropped_status}  (kept Stat == {RNGFND_STATUS_GOOD})")
    print(f"  dropped, no position within {args.tolerance_ms} ms: {dropped_nopos}")
    print(f"  dropped, no attitude within {args.tolerance_ms} ms: {dropped_noatt}")
    print(f"corrections:            depth = raw * {args.sound_speed_scale} + {args.draft_m} m")
    print(f"soundings written:      {len(out)} -> {args.out}")
    if len(out):
        print(f"depth range:            {out['depth_m'].min():.2f} .. {out['depth_m'].max():.2f} m")
    if dropped_status > 0.2 * n_total:
        print("\nWARNING: more than 20% of readings were not Good. Check RNGFND1_MIN/MAX "
              "against the depths actually surveyed (F9), and check the TELEM2 ground.")


if __name__ == "__main__":
    main()
