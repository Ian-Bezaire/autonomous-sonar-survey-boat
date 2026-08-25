#!/usr/bin/env python3
"""grid_and_plot.py -- grid QC'd soundings and draw a contour map.

    python3 grid_and_plot.py --in soundings_qc.csv --out lake-contours.png

WHAT IT DOES
  1. Projects lat/lon into a LOCAL METRIC frame (equirectangular about the survey
     centroid). Over a small inland water body this is well under the GPS noise, and it
     keeps the whole pipeline free of a projection dependency.
  2. Interpolates onto a regular grid with scipy.interpolate.griddata,
     method="linear" -- a Delaunay triangulation over the sounding positions.
  3. Draws filled contours at 0.5 m and overplots the vehicle track.

WHY method="linear" AND NOT "cubic"
  A cubic interpolant overshoots between the survey lines. With single-beam data the
  lines are metres apart and everything between them is invention already; a cubic
  invents *structure* -- local minima and maxima that look exactly like shoals and
  holes, in the one place where a chart being wrong matters most. Linear cannot produce
  a value outside the range of the surrounding soundings. It looks less pretty. That is
  the correct trade for a depth chart.

WHY NO EXTRAPOLATION
  griddata's Delaunay method returns NaN outside the convex hull of the input points,
  and that is deliberate. Beyond the outermost line the boat measured nothing at all.
  A chart that fades confidently to shore is asserting a depth nobody sounded.

WHY THE TRACK IS OVERPLOTTED
  It is the coverage check, and it is not optional. A smooth contour map drawn from a
  track with a 30 m gap down the middle looks like a survey and is a guess. Look at the
  track before you look at the contours.
"""
import argparse
import math
import sys

import numpy as np
import pandas as pd
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from scipy.interpolate import griddata  # noqa: E402

EARTH_R = 6378137.0
CONTOUR_INTERVAL_M = 0.5
GRID_RES_M = 1.0


def to_local_metres(lat, lon):
    """Equirectangular projection about the centroid. Metres east/north."""
    lat0 = float(np.mean(lat))
    lon0 = float(np.mean(lon))
    x = np.radians(lon - lon0) * EARTH_R * math.cos(math.radians(lat0))
    y = np.radians(lat - lat0) * EARTH_R
    return x, y, lat0, lon0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--in", dest="infile", required=True, help="CSV from qc.py")
    ap.add_argument("--out", required=True, help="output PNG")
    ap.add_argument("--grid-res-m", type=float, default=GRID_RES_M)
    ap.add_argument("--contour-interval-m", type=float, default=CONTOUR_INTERVAL_M)
    ap.add_argument("--title", default=None)
    args = ap.parse_args()

    df = pd.read_csv(args.infile)
    if len(df) < 4:
        sys.exit(f"{args.infile} has {len(df)} soundings. A Delaunay triangulation needs "
                 f"at least 3 non-collinear points; this is not a survey.")

    # ArduPilot logs lat/lon as scaled integers (1e7). Detect and normalise.
    lat = df["Lat"].to_numpy(dtype=float)
    lon = df["Lng"].to_numpy(dtype=float)
    if np.nanmax(np.abs(lat)) > 180.0:
        lat = lat / 1e7
        lon = lon / 1e7
        print("note: Lat/Lng looked like 1e7-scaled integers; divided by 1e7")

    depth = df["depth_m"].to_numpy(dtype=float)
    x, y, lat0, lon0 = to_local_metres(lat, lon)

    res = args.grid_res_m
    gx = np.arange(x.min(), x.max() + res, res)
    gy = np.arange(y.min(), y.max() + res, res)
    GX, GY = np.meshgrid(gx, gy)

    # linear == Delaunay TIN. NaN outside the convex hull, by design.
    GZ = griddata((x, y), depth, (GX, GY), method="linear")

    n_cells = GZ.size
    n_filled = int(np.count_nonzero(~np.isnan(GZ)))

    lo = math.floor(np.nanmin(depth) / args.contour_interval_m) * args.contour_interval_m
    hi = math.ceil(np.nanmax(depth) / args.contour_interval_m) * args.contour_interval_m
    levels = np.arange(lo, hi + args.contour_interval_m, args.contour_interval_m)

    fig, ax = plt.subplots(figsize=(10, 8))
    cf = ax.contourf(GX, GY, GZ, levels=levels, cmap="Blues")
    ax.contour(GX, GY, GZ, levels=levels, colors="k", linewidths=0.3, alpha=0.5)

    # The coverage check. Look at this first.
    ax.plot(x, y, color="0.25", linewidth=0.6, alpha=0.8, zorder=3, label="vehicle track")
    ax.scatter(x, y, s=1.0, color="k", alpha=0.35, zorder=4)

    cb = fig.colorbar(cf, ax=ax)
    cb.set_label("depth below water surface (m)")
    cb.ax.invert_yaxis()

    ax.set_aspect("equal")
    ax.set_xlabel(f"east of {lat0:.6f}, {lon0:.6f} (m)")
    ax.set_ylabel("north (m)")
    ax.set_title(args.title or f"{args.infile} -- {len(df)} soundings, "
                               f"{args.contour_interval_m} m contours")
    ax.legend(loc="best", fontsize="small")
    fig.tight_layout()
    fig.savefig(args.out, dpi=150)

    print(f"in:                     {args.infile}  ({len(df)} soundings)")
    print(f"extent:                 {x.max() - x.min():.1f} x {y.max() - y.min():.1f} m")
    print(f"grid:                   {res} m, {GZ.shape[1]} x {GZ.shape[0]} cells")
    print(f"  interpolated:                 {n_filled} cells")
    print(f"  left NaN (outside hull):      {n_cells - n_filled} cells -- not extrapolated, "
          f"nothing was sounded there")
    print(f"depth range:            {np.nanmin(depth):.2f} .. {np.nanmax(depth):.2f} m")
    print(f"contours:               {levels[0]:.1f} .. {levels[-1]:.1f} m every "
          f"{args.contour_interval_m} m")
    print(f"written:                {args.out}")
    print("\nCheck the track before the contours. Contours between widely spaced lines are "
          "interpolation, not measurement.")


if __name__ == "__main__":
    main()
