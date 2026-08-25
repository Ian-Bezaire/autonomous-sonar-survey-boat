# Goal

## Mission statement

Build a low-cost autonomous surface vehicle that maps the bottom of a small inland water body
without an operator in the loop.

## Success criterion

The project succeeds when the boat:

> executes a complete two-phase boustrophedon sweep of a defined water body and returns a
> georeferenced sounding set, with **no operator input between arming and mission end**.

Stated in falsifiable parts, all four must hold in a single run:

1. **Autonomy.** From the moment the vehicle is armed and placed in GUIDED, no stick input,
   no mode change and no GCS command is required until the run terminates.
2. **Coverage.** Both sweep phases complete — phase 1 offsetting to `base_bearing + 90°`,
   phase 2 to `base_bearing - 90°`, from the same hub — and the phase-end condition is
   reached by the edge-detection logic, not by a timeout, a watchdog or a failsafe.
3. **Return.** The vehicle returns to its start location under its own logic and finishes
   station-keeping there.
4. **Product.** The onboard log yields a sounding set that survives `analysis/qc.py` and
   grids into a contour map, with depths corrected for draft and sound speed.

A run that ends in `STATE_HALTED`, in HOLD, or on a `MISSION_TIMEOUT_S` expiry has not met
the criterion. It may still be a good run — the failsafes working is the second-best
outcome available — but it is recorded as such in `ops/incident-log.md`.

## Vertical accuracy is a separate claim

Coverage and autonomy can be demonstrated with an uncalibrated sounder. Depth *numbers*
cannot. A sounding is only meaningful once `data/calibration/draft.md` and
`data/calibration/sound-speed.md` carry measured values, and the water level is referenced
to the fixed shore mark recorded in `session.yaml`. Until then the product is a relative
contour shape, not a depth chart.

## Non-goals

These are excluded by design, not by lack of time. Each is a decision that shapes the
architecture, and reopening one is a new decision record, not a tweak.

- **Not obstacle avoidance.** The sonar looks down. It has no look-ahead whatsoever, so
  the vehicle physically cannot see anything in front of it. Operating water must be
  chosen to be clear, and the geofence — not the sensor — is what keeps the boat off the
  shore. See [ADR-0002](decisions/0002-no-slam-downward-sonar-only.md).
- **Not SLAM, not frontier exploration.** Same root cause; both families of algorithm
  assume forward-facing sensing. See [ADR-0002](decisions/0002-no-slam-downward-sonar-only.md).
- **Not multibeam.** One downward single-beam transducer, one sounding per ping, on the
  vehicle track only. Everything between the track lines is interpolation, and
  `grid_and_plot.py` refuses to extrapolate past the convex hull of the real soundings
  for exactly this reason.
- **Not survey-grade absolute accuracy.** Without RTK the horizontal position is
  consumer-GPS grade and the vertical datum is the water surface on the day, tied to a
  shore mark. This is adequate for "where is the channel, where is the shoal" and
  inadequate for anything a chart is drawn from.
- **Not a companion-computer vehicle.** Coverage logic runs in Lua on the flight
  controller. This is a hard constraint on how much the algorithm can do — see the F4
  heap discussion in `vehicle/params/verified.md` — and it is accepted in exchange for
  having exactly one computer to power, mount and fail.
- **Not a general-purpose ASV platform.** The scripts assume a skid-steer twin hull that
  can pivot in place. Nothing here is written to be portable to a rudder boat.
