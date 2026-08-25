#!/usr/bin/env python3
"""
depth_feeder.py — feed simulated depth (downward rangefinder) into ArduPilot SITL.

Why: SITL's world is flat ground, so the built-in RNGFND1_TYPE=100 sensor always
reads ~0 m. To test depth_survey_mk1.lua you need to CONTROL the depth. This
script sends MAVLink DISTANCE_SENSOR messages (orientation = down) at 10 Hz, so
the autopilot's MAVLink rangefinder (RNGFND1_TYPE=10) sees whatever depth you set.

SETUP (in the MAVProxy MAV> prompt):
    param set RNGFND1_TYPE 10     # 10 = MAVLink rangefinder
    param set RNGFND1_ORIENT 25   # down
    param set RNGFND1_MIN 0
    param set RNGFND1_MAX 40
    reboot

Then run this script (in a second Ubuntu tab, venv active):
    python3 depth_feeder.py

It connects to SITL's spare port tcp:127.0.0.1:5762 and starts sending 10 m
(deep water). Commands while it runs:
    <number>   set depth in meters   (e.g.  3   -> shallow, triggers a turn)
    deep       set 10 m
    shallow    set 3 m
    auto       cycle deep<->shallow every few seconds automatically
    stop       stop auto cycling (hold current depth)
    q          quit

TYPICAL DEMO:
    1. Start this script (depth = 10 m, "deep").
    2. In MAVProxy: mode GUIDED ; arm throttle  -> boat drives forward.
    3. Here, type  3  -> boat detects shallow, pivots + offsets + pivots.
    4. Type  10     -> back to deep, boat drives forward again.
    5. Watch the serpentine trace on the Mission Planner map.
"""

import sys
import time
import threading
import argparse
from pymavlink import mavutil

# ---- config ----
DEFAULT_PORT = "tcp:127.0.0.1:5762"   # SITL spare MAVLink port (5763 is a fallback)
SEND_HZ = 10
ORIENT_DOWN = 25          # MAV_SENSOR_ROTATION_PITCH_270 = straight down
MIN_CM = 1
MAX_CM = 4000             # 40 m, matches RNGFND1_MAX 40
AUTO_DEEP_M = 10.0
AUTO_SHALLOW_M = 3.0
AUTO_PERIOD_S = 6.0       # seconds at each depth in auto mode

# ---- shared state ----
state = {
    "depth_m": AUTO_DEEP_M,
    "auto": False,
    "run": True,
}
lock = threading.Lock()


def sender(conn):
    """Send DISTANCE_SENSOR at SEND_HZ using the current depth."""
    period = 1.0 / SEND_HZ
    t_auto = time.time()
    auto_deep = True
    while True:
        with lock:
            if not state["run"]:
                return
            if state["auto"]:
                if time.time() - t_auto >= AUTO_PERIOD_S:
                    auto_deep = not auto_deep
                    state["depth_m"] = AUTO_DEEP_M if auto_deep else AUTO_SHALLOW_M
                    t_auto = time.time()
                    print(f"\n[auto] depth -> {state['depth_m']:.1f} m")
                    print("depth> ", end="", flush=True)
            depth_cm = int(max(0.0, state["depth_m"]) * 100)
        conn.mav.distance_sensor_send(
            int(time.time() * 1000) & 0xFFFFFFFF,  # time_boot_ms
            MIN_CM,        # min_distance (cm)
            MAX_CM,        # max_distance (cm)
            depth_cm,      # current_distance (cm)
            0,             # type: 0 = laser (fine for sim)
            0,             # id
            ORIENT_DOWN,   # orientation: 25 = down
            0,             # covariance
        )
        time.sleep(period)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=DEFAULT_PORT,
                    help=f"MAVLink connection string (default {DEFAULT_PORT}; "
                         f"try tcp:127.0.0.1:5763 if this refuses)")
    ap.add_argument("--start-depth", type=float, default=AUTO_DEEP_M)
    args = ap.parse_args()

    with lock:
        state["depth_m"] = args.start_depth

    print(f"Connecting to {args.port} ...")
    conn = mavutil.mavlink_connection(args.port, source_system=200)
    conn.wait_heartbeat()
    print(f"Connected (system {conn.target_system}). Sending depth at {SEND_HZ} Hz.")
    print("Commands: <number> | deep | shallow | auto | stop | q\n")

    th = threading.Thread(target=sender, args=(conn,), daemon=True)
    th.start()

    try:
        while True:
            try:
                cmd = input("depth> ").strip().lower()
            except EOFError:
                break
            if cmd in ("q", "quit", "exit"):
                break
            elif cmd == "deep":
                with lock:
                    state["auto"] = False
                    state["depth_m"] = AUTO_DEEP_M
                print(f"depth = {AUTO_DEEP_M} m (deep)")
            elif cmd == "shallow":
                with lock:
                    state["auto"] = False
                    state["depth_m"] = AUTO_SHALLOW_M
                print(f"depth = {AUTO_SHALLOW_M} m (shallow)")
            elif cmd == "auto":
                with lock:
                    state["auto"] = True
                print("auto cycling deep<->shallow")
            elif cmd == "stop":
                with lock:
                    state["auto"] = False
                print("auto off, holding current depth")
            elif cmd == "":
                continue
            else:
                try:
                    d = float(cmd)
                    with lock:
                        state["auto"] = False
                        state["depth_m"] = d
                    print(f"depth = {d} m")
                except ValueError:
                    print(f"?  unknown: {cmd!r}")
    finally:
        with lock:
            state["run"] = False
        time.sleep(0.2)
        print("stopped.")


if __name__ == "__main__":
    main()
