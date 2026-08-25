#!/usr/bin/env python3
"""qc.py -- reject bad soundings before they become contours.

    python3 qc.py --in soundings_raw.csv --out soundings_qc.csv

Two filters, in this order:

1. ROLLING-MEDIAN SPIKE REJECTION (window 5, 0.5 m threshold)
   A single-beam sounder returns the strongest echo, which is not always the bottom.
   Weed, a fish, a thermocline, a bubble curtain: any of them produce one reading that
   disagrees with its neighbours by metres. A rolling median is used rather than a mean
   because the mean is dragged by the very outlier being tested for.

2. YAW-RATE GATE (10 deg/s)
   Prop-wash aeration under a skid-steer hull during pivots is the dominant garbage
   source in this vehicle's data. Both thrusters run hard in opposite directions while
   the boat is not moving forward, so the wash has nowhere to go but under the hull and
   past the transducer face. Air in the water column scatters the ping; the sounder
   returns the aerated layer or nothing at all. The reading is not noisy, it is wrong,
   and it is wrong in one direction (too shallow).

   Dropping turns costs almost no coverage. Pivots happen at the ends of legs, over
   ground the adjacent legs already cover, and they are a small fraction of the run
   time. Keeping them costs a ring of false shoals at every line end -- which looks
   exactly like real structure on a contour map.

Every rejection is counted and printed by reason. A QC step that quietly halves the
dataset is worse than no QC step.
"""
import argparse
import sys

import numpy as np
import pandas as pd

SPIKE_WINDOW = 5        # samples; odd, so the median is an actual sample
SPIKE_THRESHOLD_M = 0.5  # deviation from the local median that counts as a spike
YAW_RATE_LIMIT_DPS = 10.0


def wrap_deg(d):
    """Wrap a heading difference into (-180, 180]."""
    return (d + 180.0) % 360.0 - 180.0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--in", dest="infile", required=True,
                    help="CSV from extract_soundings.py")
    ap.add_argument("--out", required=True, help="output CSV of accepted soundings")
    ap.add_argument("--spike-window", type=int, default=SPIKE_WINDOW)
    ap.add_argument("--spike-threshold-m", type=float, default=SPIKE_THRESHOLD_M)
    ap.add_argument("--yaw-rate-limit-dps", type=float, default=YAW_RATE_LIMIT_DPS)
    ap.add_argument("--keep-turns", action="store_true",
                    help="disable the yaw-rate gate. For investigating a specific "
                         "artefact only -- do not produce a chart with this set.")
    args = ap.parse_args()

    df = pd.read_csv(args.infile)
    n_total = len(df)
    if n_total == 0:
        sys.exit(f"{args.infile} is empty.")

    df = df.sort_values("TimeUS").reset_index(drop=True)

    # ---- 1. rolling-median spike rejection ---------------------------------------------
    med = df["depth_m"].rolling(args.spike_window, center=True, min_periods=1).median()
    spike = (df["depth_m"] - med).abs() > args.spike_threshold_m
    n_spike = int(spike.sum())

    # ---- 2. yaw-rate gate --------------------------------------------------------------
    dt_s = df["TimeUS"].diff() / 1e6
    dyaw = wrap_deg(df["Yaw"].diff())
    yaw_rate = (dyaw / dt_s).abs()
    # First sample has no rate; keep it rather than invent one.
    yaw_rate = yaw_rate.fillna(0.0)
    df["yaw_rate_dps"] = yaw_rate

    if args.keep_turns:
        turning = pd.Series(False, index=df.index)
    else:
        turning = yaw_rate > args.yaw_rate_limit_dps
    n_turn = int(turning.sum())

    reject = spike | turning
    n_both = int((spike & turning).sum())
    out = df[~reject].copy()

    out.to_csv(args.out, index=False)

    print(f"in:                     {args.infile}  ({n_total} soundings)")
    print(f"  rejected as spikes:           {n_spike}  "
          f"(> {args.spike_threshold_m} m from rolling median, window {args.spike_window})")
    if args.keep_turns:
        print(f"  yaw-rate gate:                DISABLED (--keep-turns)")
    else:
        print(f"  rejected as taken mid-pivot:  {n_turn}  "
              f"(yaw rate > {args.yaw_rate_limit_dps} deg/s -- prop-wash aeration)")
    print(f"  (rejected by both:            {n_both})")
    print(f"accepted:               {len(out)} -> {args.out}  "
          f"({100.0 * len(out) / n_total:.1f}% kept)")

    if len(out) == 0:
        sys.exit("everything was rejected. Check the input, not the thresholds.")
    if n_turn > 0.4 * n_total:
        print("\nWARNING: more than 40% of soundings were taken while turning. That is not "
              "a normal survey track -- check for a nav stall loop, or a leg length so "
              "short the boat is pivoting almost continuously.")
    if n_spike > 0.15 * n_total:
        print("\nWARNING: high spike rate. Suspect weed, aeration, or motor current on the "
              "TELEM2 signal ground (hardware/telem2-depth-interface.md).")


if __name__ == "__main__":
    main()
