#!/usr/bin/env python3
"""lake_feeder.py -- feed simulated depth into ArduPilot SITL from a synthetic lake.

Why: depth_feeder.py requires a human to type "shallow"/"deep" by hand, which
only exercises one trigger at a time and can't run unattended. This instead
tracks the vehicle's real simulated GPS position and computes depth from a
bowl-shaped lake bathymetry centered near the start location, so the survey
script (depth_survey_mk1.lua) can run a full unattended multi-leg boustrophedon
sweep against a consistent, position-correct "lake" -- drive out from shore,
get shallow again on the far side, turn, repeat -- exactly like a real survey.

SETUP: same as depth_feeder.py (RNGFND1_TYPE=10, RNGFND1_ORIENT=25, min/max set).

Usage (in a second WSL tab, venv active, after SITL is armed and in GUIDED):
    python3 lake_feeder.py
    python3 lake_feeder.py --radius 60 --max-depth 15
    python3 lake_feeder.py --center-lat 45.5 --center-lon -120.0   # pin lake center

By default the lake is centered on the vehicle's position at connect time, so
it works regardless of the -l/--custom-location SITL was started with. Depth
is a simple radial bowl (deep in the middle, 0 m at the shore radius) with a
little texture so the shoreline isn't a perfect circle. No manual input --
runs until Ctrl+C.
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


def lake_depth_m(x, y, radius_m, max_depth_m):
    """Bowl-shaped lake: max_depth_m at center, 0 at radius_m, with mild
    shoreline texture so it isn't a perfect circle."""
    r = math.hypot(x, y)
    # slight radial texture (deterministic, not random) so the shore wobbles
    texture = 1.0 + 0.12 * math.sin(3 * math.atan2(y, x)) + 0.05 * math.sin(x * 0.15)
    effective_radius = radius_m * texture
    frac = r / effective_radius
    if frac >= 1.0:
        return 0.0
    # parabolic bowl: deep in the middle, tapering smoothly to 0 at the shore
    return max(0.0, max_depth_m * (1.0 - frac * frac))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=PORT,
                     help=f"MAVLink connection string (default {PORT}; try tcp:127.0.0.1:5763 if refused)")
    ap.add_argument("--radius", type=float, default=40.0, help="lake radius in meters (default 40)")
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
