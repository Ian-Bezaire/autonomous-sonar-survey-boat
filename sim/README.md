# SITL

Software-in-the-loop for Rover, with synthetic depth injected so the survey modules have
something to react to.

## Why a feeder is needed at all

SITL's world is flat ground, so the built-in rangefinder always reads about 0 m. Without an
injected depth signal the survey modules see permanent shallow water and turn immediately,
forever. The feeders send MAVLink `DISTANCE_SENSOR` messages with orientation = down, so
ArduPilot's MAVLink rangefinder backend sees whatever depth the feeder decides.

## Setup

Start SITL as a **skid-steer** rover — the modules assume a vehicle that can pivot in place:

```
sim_vehicle.py -v Rover -f rover-skid --console --map
```

At the `MAV>` prompt:

```
param set RNGFND1_TYPE 10     # MAVLink rangefinder -- SITL ONLY, not the boat
param set RNGFND1_ORIENT 25   # down
param set RNGFND1_MIN 0
param set RNGFND1_MAX 40
param set SCR_ENABLE 1
param set SCR_USER1 0
reboot
```

**`RNGFND1_TYPE 10` is a simulator setting.** The boat uses the NMEA serial backend against
the real ATU120A on TELEM2 — see `../hardware/telem2-depth-interface.md`. Never copy a SITL
parameter set onto the vehicle.

Copy `../vehicle/scripts/` into the SITL instance's `scripts/` directory, keeping the
`modules/` subdirectory. The same layout rule applies here as on the card: exactly one
`.lua` file directly in `scripts/`.

## The two feeders

| Script | Input | Use it for |
|---|---|---|
| [`depth_feeder.py`](depth_feeder.py) | **Manual** — you type depths at a prompt | Exercising one trigger at a time. First bring-up, and reproducing a specific transition. |
| [`lake_feeder.py`](lake_feeder.py) | **Automatic** — tracks the vehicle's simulated GPS position and computes depth from a synthetic bowl-shaped lake | Unattended full-sweep runs. This is the one that tests the survey. |
| [`lake_feeder_complex.py`](lake_feeder_complex.py) | **Automatic**, same interface as `lake_feeder.py` — irregular shoreline, two off-centre basins, a shoal ridge, and low-amplitude texture instead of a radial bowl | Checking the sweep against a bottom that isn't radially symmetric — an off-hub basin, a mid-lake shoal turn, a shoreline with coves. |

`depth_feeder.py` connects to SITL's spare port and starts at 10 m, accepting a number,
`deep`, `shallow` or `auto`. It cannot run unattended, which is exactly why `lake_feeder.py`
exists.

`lake_feeder.py` centres the lake on the vehicle's position at connect time (so it works
whatever `-l/--custom-location` SITL started with), and models a radial bowl — deep in the
middle, 0 m at the shore radius, with a little texture so the shoreline is not a perfect
circle. Run it in a second terminal once the vehicle is armed and in GUIDED:

```
python3 lake_feeder.py
python3 lake_feeder.py --radius 60 --max-depth 15
python3 lake_feeder.py --center-lat 45.5 --center-lon -120.0
```

Then set `SCR_USER1` to `1` or `2` and watch the Messages tab.

## The standing instruction: kill the feeder mid-leg

**A synthetic lake never exercises the boundaries.** `lake_feeder.py` produces a clean,
continuous, position-correct depth signal that never drops out, never goes stale, never
returns a bad status and never disagrees with the position estimate. Real water does all
four, and every failsafe in the modules exists for cases the feeder cannot produce.

**Every SITL run must include a deliberate mid-leg feeder kill.** Ctrl-C the feeder while
the boat is mid-forward-leg and watch what happens:

- The stale-depth watchdog should fire after `DEPTH_STALE_TIMEOUT_S` (3.0 s).
- The stop should be a **mode change to HOLD**, not a speed command — check the mode in
  the console, not just the GCS text. A message without a mode change is finding F1
  reappearing.
- The dispatcher should keep ticking. If dispatcher messages stop too, the module took the
  supervisor down with it — F2.

Additional boundaries the feeder cannot produce on its own, which need their own scenarios
or manual intervention:

- **Sensor dropout that recovers** — kill and restart the feeder mid-leg.
- **EKF / position loss** — force it in SITL and confirm the nil `ahrs:get_location()` path
  holds rather than throwing.
- **Controller pivot behaviour** — real skid-steer pivots are slower and sloppier than the
  simulator's, which is the whole risk behind F4 and `WP_PIVOT_RATE`.
- **A geofence tighter than `MAX_LEG_DISTANCE_M`** — set `FENCE_RADIUS` below 60 m and
  confirm the rejected `set_target_location()` is caught, not ignored (F7).

## Scenarios

[`scenarios/`](scenarios/) holds one file per bathymetry profile worth reproducing:

- **shallow-everywhere** — exercises F9, the band between `DEPTH_THRESHOLD_M` and
  `RNGFND1_MIN_CM` where the sensor may report out-of-range instead of shallow.
- **dropout** — a scripted mid-leg signal loss, so the F1/F2 check above is repeatable
  rather than a human with a Ctrl-C key.
- **small-radius lake** — a lake smaller than `MAX_LEG_DISTANCE_M`, so legs terminate on
  the edge condition rather than on the leg cap, and phases end by `EDGE_STREAK_COUNT`.

`TODO:` write these three. The directory is empty.

## Passing SITL is necessary and not sufficient

SITL proves the state machine is coherent and the arithmetic is right. It proves nothing
about a 4800-baud NMEA link with motor current on the ground reference, a transducer
aerated by prop wash during a pivot, a real EKF over real water, a pack with a real
discharge curve, or a skid-steer boat with real momentum.

A green SITL run permits the **bench** test. It does not permit water. See
[`../ops/prelaunch-checklist.md`](../ops/prelaunch-checklist.md).
