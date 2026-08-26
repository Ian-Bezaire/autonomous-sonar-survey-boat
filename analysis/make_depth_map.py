#!/usr/bin/env python3
"""make_depth_map.py -- run the full log -> depth map pipeline in one command.

A convenience wrapper. It does not replace the three stages -- each stays runnable
and inspectable on its own (see README.md for why: a pipeline that hides what it
dropped produces a clean-looking map of a subset you never chose). This just chains
them and keeps every stage's diagnostic output on screen.

    python3 make_depth_map.py --log 2026-09-06-lake.BIN --out lake-contours.png \
        --draft-m 0.21 --sound-speed-scale 0.9767

--draft-m and --sound-speed-scale are required, same as extract_soundings.py, for the
same reason: a plausible default is how an uncalibrated survey ends up looking
calibrated. See ../data/calibration/.

Intermediate soundings_raw.csv and soundings_qc.csv are written next to --out (or to
--work-dir if given) and kept, not deleted -- they are worth inspecting on their own.
"""
import argparse
import pathlib
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent


def run(cmd):
    print(f"\n$ {' '.join(cmd)}")
    result = subprocess.run(cmd)
    if result.returncode != 0:
        sys.exit(result.returncode)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--log", required=True, help="ArduPilot .BIN / .log file")
    ap.add_argument("--out", required=True, help="output PNG (final depth map)")
    ap.add_argument("--draft-m", type=float, required=True,
                    help="transducer face to waterline, boat loaded as it runs "
                         "(../data/calibration/draft.md)")
    ap.add_argument("--sound-speed-scale", type=float, required=True,
                    help="measured / assumed sound speed, e.g. 1465/1500 = 0.9767 "
                         "(../data/calibration/sound-speed.md)")
    ap.add_argument("--work-dir", default=None,
                    help="where to write soundings_raw.csv / soundings_qc.csv "
                         "(default: same directory as --out)")

    # Pass-through, forwarded verbatim to the stage that owns each one.
    ap.add_argument("--tolerance-ms", type=int, default=None,
                    help="extract_soundings.py: max time gap matching POS/ATT to a sounding")
    ap.add_argument("--spike-threshold-m", type=float, default=None,
                    help="qc.py: rolling-median spike-rejection threshold")
    ap.add_argument("--yaw-rate-limit-dps", type=float, default=None,
                    help="qc.py: reject soundings taken above this yaw rate")
    ap.add_argument("--keep-turns", action="store_true",
                    help="qc.py: disable the yaw-rate gate. For investigation only -- "
                         "do not produce a chart with this set.")
    ap.add_argument("--grid-res-m", type=float, default=None,
                    help="grid_and_plot.py: interpolation grid resolution")
    ap.add_argument("--contour-interval-m", type=float, default=None,
                    help="grid_and_plot.py: contour line spacing")
    ap.add_argument("--title", default=None, help="grid_and_plot.py: plot title")
    args = ap.parse_args()

    out_path = pathlib.Path(args.out)
    work_dir = pathlib.Path(args.work_dir) if args.work_dir else out_path.parent
    work_dir = work_dir if str(work_dir) else pathlib.Path(".")
    work_dir.mkdir(parents=True, exist_ok=True)
    raw_csv = work_dir / "soundings_raw.csv"
    qc_csv = work_dir / "soundings_qc.csv"

    py = sys.executable

    extract_cmd = [py, str(HERE / "extract_soundings.py"),
                   "--log", args.log, "--out", str(raw_csv),
                   "--draft-m", str(args.draft_m),
                   "--sound-speed-scale", str(args.sound_speed_scale)]
    if args.tolerance_ms is not None:
        extract_cmd += ["--tolerance-ms", str(args.tolerance_ms)]
    run(extract_cmd)

    qc_cmd = [py, str(HERE / "qc.py"), "--in", str(raw_csv), "--out", str(qc_csv)]
    if args.spike_threshold_m is not None:
        qc_cmd += ["--spike-threshold-m", str(args.spike_threshold_m)]
    if args.yaw_rate_limit_dps is not None:
        qc_cmd += ["--yaw-rate-limit-dps", str(args.yaw_rate_limit_dps)]
    if args.keep_turns:
        qc_cmd += ["--keep-turns"]
    run(qc_cmd)

    plot_cmd = [py, str(HERE / "grid_and_plot.py"), "--in", str(qc_csv), "--out", str(out_path)]
    if args.grid_res_m is not None:
        plot_cmd += ["--grid-res-m", str(args.grid_res_m)]
    if args.contour_interval_m is not None:
        plot_cmd += ["--contour-interval-m", str(args.contour_interval_m)]
    if args.title is not None:
        plot_cmd += ["--title", args.title]
    run(plot_cmd)

    print(f"\ndepth map written:      {out_path}")
    print(f"intermediate files in:  {work_dir}  (soundings_raw.csv, soundings_qc.csv)")


if __name__ == "__main__":
    main()
