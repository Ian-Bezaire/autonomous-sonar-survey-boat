#!/usr/bin/env python3
"""lake_feeder_complex.py -- feed simulated depth into ArduPilot SITL from a
synthetic lake with a real-world-shaped bottom, not a simple bowl.

Same job as lake_feeder.py -- tracks the vehicle's simulated GPS position and
computes depth from a synthetic bathymetry so depth_survey_mk2.lua can run a
full unattended sweep -- but the bathymetry itself is deliberately not
radially symmetric:

  - an IRREGULAR SHORELINE (multi-harmonic radius function -- lobes and coves,
    not a circle or a simple wobble)
  - TWO OFF-CENTRE BASINS of different depth, so the deepest water is not at
    the lake's centroid and there is a saddle between them
  - a SHOAL crossing part of the lake (a shallow ridge -- the kind of feature
    that produces a real mid-lake turn, not just a shore-triggered one)
  - low-amplitude TEXTURE (a few summed sinusoids) so the bottom is not flat
    between features

Everything is deterministic (no RNG) so a run is reproducible from the CLI
args alone -- same reasoning as lake_feeder.py's shoreline texture.

Usage (identical to lake_feeder.py; run in a second terminal after SITL is
armed and in GUIDED):
    python3 lake_feeder_complex.py
    python3 lake_feeder_complex.py --radius 50 --max-depth 14
    python3 lake_feeder_complex.py --center-lat 45.5 --center-lon -120.0
"""
import argparse
import math
import sys
import time

from pymavlink import mavutil

PORT = "tcp:127.0.0.1:5762"
ORIENT_DOWN = 25
SEND_HZ = 10
STATUS_PERIOD_S = 2.0

M_PER_DEG_LAT = 111320.0


def local_xy(center_lat, center_lon, lat, lon):
    dlat_m = (lat - center_lat) * M_PER_DEG_LAT
    dlon_m = (lon - center_lon) * M_PER_DEG_LAT * math.cos(math.radians(center_lat))
    return dlon_m, dlat_m  # (east, north) meters


def shoreline_radius_m(theta, base_radius_m):
    """Irregular shoreline: two broad lobes, a few coves, fine wobble."""
    factor = (1.0
              + 0.22 * math.sin(2 * theta + 0.6)
              + 0.13 * math.sin(5 * theta + 2.0)
              + 0.06 * math.sin(9 * theta + 0.9))
    return base_radius_m * max(0.45, factor)


def basin(x, y, cx, cy, depth_m, spread_m):
    """Gaussian depth bump centred at (cx, cy)."""
    d2 = (x - cx) ** 2 + (y - cy) ** 2
    return depth_m * math.exp(-d2 / (2.0 * spread_m * spread_m))


def shoal_ridge(x, y, x0, y0, x1, y1, height_m, width_m):
    """Shallowing (negative-depth) ridge along the segment (x0,y0)-(x1,y1)."""
    ex, ey = x1 - x0, y1 - y0
    seg_len2 = ex * ex + ey * ey
    if seg_len2 <= 0:
        t = 0.0
    else:
        t = max(0.0, min(1.0, ((x - x0) * ex + (y - y0) * ey) / seg_len2))
    px, py = x0 + t * ex, y0 + t * ey
    dist = math.hypot(x - px, y - py)
    return -height_m * math.exp(-(dist ** 2) / (2.0 * width_m * width_m))


def texture_m(x, y):
    """Low-amplitude summed sinusoids -- an uneven bottom, not a smooth one."""
    return (0.18 * math.sin(x * 0.11 + y * 0.07)
            + 0.12 * math.sin(x * 0.05 - y * 0.13 + 1.3)
            + 0.07 * math.sin(x * 0.23 + y * 0.19 + 2.7))


def lake_depth_m(x, y, radius_m, max_depth_m):
    theta = math.atan2(y, x)
    r = math.hypot(x, y)
    edge_r = shoreline_radius_m(theta, radius_m)
    frac = r / edge_r
    if frac >= 1.0:
        return 0.0

    # Two off-centre basins -- deepest water is not at the centroid, and
    # there is a shallower saddle between them, like a real two-lobed lake.
    d1 = basin(x, y, -0.30 * radius_m, 0.15 * radius_m, max_depth_m * 1.00, 0.35 * radius_m)
    d2 = basin(x, y, 0.35 * radius_m, -0.25 * radius_m, max_depth_m * 0.65, 0.22 * radius_m)
    depth = max(d1, d2)

    # A shoal crossing part of the lake -- a real mid-water turn trigger,
    # not only a shore-triggered one.
    depth += shoal_ridge(x, y,
                         -0.10 * radius_m, 0.50 * radius_m,
                          0.30 * radius_m, -0.20 * radius_m,
                          height_m=max_depth_m * 0.55, width_m=0.10 * radius_m)

    depth += texture_m(x, y)

    # Taper smoothly to 0 at the (irregular) shoreline.
    taper = 1.0 - frac * frac
    return max(0.0, depth) * taper


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=PORT,
                     help=f"MAVLink connection string (default {PORT}; try tcp:127.0.0.1:5763 if refused)")
    ap.add_argument("--radius", type=float, default=40.0, help="lake nominal radius in meters (default 40)")
    ap.add_argument("--max-depth", type=float, default=12.0, help="max lake depth in meters (default 12)")
    ap.add_argument("--center-lat", type=float, default=None, help="pin lake center latitude (default: vehicle's position at connect)")
    ap.add_argument("--center-lon", type=float, default=None, help="pin lake center longitude (default: vehicle's position at connect)")
    args = ap.parse_args()

    print(f"Connecting to {args.port} ...", flush=True)
    conn = mavutil.mavlink_connection(args.port, source_system=201)
    conn.wait_heartbeat()
    print(f"Connected (system {conn.target_system}).", flush=True)

    center_lat, center_lon = args.center_lat, args.center_lon
    if center_lat is None or center_lon is None:
        print("Waiting for a GPS fix to center the lake on the vehicle's start position...", flush=True)
        while True:
            msg = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=10)
            if msg is not None and msg.lat != 0:
                center_lat, center_lon = msg.lat / 1e7, msg.lon / 1e7
                break
    print(f"Lake center: {center_lat:.6f}, {center_lon:.6f}  radius={args.radius}m  max_depth={args.max_depth}m", flush=True)
    print("Bathymetry: irregular shoreline, two off-centre basins, one shoal ridge, texture.", flush=True)

    period = 1.0 / SEND_HZ
    last_status = 0.0
    last_pos = None  # (lat, lon)

    try:
        while True:
            msg = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=period)
            if msg is not None and msg.lat != 0:
                last_pos = (msg.lat / 1e7, msg.lon / 1e7)

            if last_pos is not None:
                x, y = local_xy(center_lat, center_lon, last_pos[0], last_pos[1])
                depth = lake_depth_m(x, y, args.radius, args.max_depth)
            else:
                x = y = 0.0
                depth = args.max_depth  # no fix yet: report deep so the survey doesn't false-trigger

            depth_cm = int(max(0.0, depth) * 100)
            conn.mav.distance_sensor_send(
                int(time.time() * 1000) & 0xFFFFFFFF,
                1, 4000, depth_cm, 0, 0, ORIENT_DOWN, 0,
            )

            now = time.time()
            if now - last_status >= STATUS_PERIOD_S:
                last_status = now
                r = math.hypot(x, y)
                print(f"[lake] pos=({x:+6.1f}, {y:+6.1f}) r={r:5.1f}m depth={depth:5.2f}m", flush=True)
    except KeyboardInterrupt:
        print("\nstopped.", flush=True)


if __name__ == "__main__":
    main()
