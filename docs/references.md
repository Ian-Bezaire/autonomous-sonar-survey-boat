# References

## Closest published analogue — read this first

**Wilson, T. and Williams, S. B. — "Adaptive Path Planning for Depth Constrained Bathymetric
Mapping with an Autonomous Surface Vessel."** Australian Centre for Field Robotics, The
University of Sydney. arXiv:1603.06324 — <https://arxiv.org/abs/1603.06324>

Highest-priority reference for this project. The testbed is a small **twin-hull
differential-thrust vessel with single-beam sonar, GPS and IMU** — the same vehicle class,
the same sensing limitation. The method is a Gaussian process fitted online to sonar
readings, an algorithm that finds and follows the intersection of a target depth contour
with the survey boundary, and a Discrete Monotone Polygonal Partitioning scheme for
coverage, all running on embedded processors via incremental GP updates.

That is, in order, most of what [`roadmap.md`](roadmap.md) calls mk3. Read it before
designing that tier rather than after.

## ArduPilot source — the facts the scripts are built on

The scripts encode several behaviours that are only discoverable by reading the firmware.
Paths below are in <https://github.com/ArduPilot/ardupilot>. Check them against the
firmware version actually on the board — recorded in
[`../vehicle/params/verified.md`](../vehicle/params/verified.md) and in each
`session.yaml`.

| Claim relied on | Where it lives |
|---|---|
| `set_speed_max()` rejects anything below `AR_WPNAV_SPEED_MIN` (0.05 m/s) and returns false — so every zero-speed stop path is a no-op | `libraries/AR_WPNav/AR_WPNav.cpp`, `AR_WPNav.h` |
| `ModeGuided::_enter()` calls `wp_nav.init()` and preserves no target | `Rover/mode_guided.cpp` |
| Guided destination setters validate the destination and return false on rejection (notably against the geofence) | `Rover/mode_guided.cpp`, `Rover/mode.h` |
| Mode numbers: HOLD 4, LOITER 5, GUIDED 15 | `Rover/mode.h` |
| Rover reads the mode switch only on a *change* of switch position, so a script-commanded HOLD sticks | `Rover/RC_Channel_Rover.cpp` |
| `set_desired_turn_rate_and_speed()` takes turn rate in **degrees/sec**, despite the vehicle-agnostic Lua binding name | `Rover/Rover.cpp` |
| `ATC_STR_RAT_MAX` (default 120 °/s) is the achievable turn-rate ceiling | `libraries/APM_Control/AR_AttitudeControl.cpp` |
| `Status::Good == 4`; `read_timeout_ms()` holds a stale value up to 3000 ms | `libraries/AP_RangeFinder/AP_RangeFinder.h`, `AP_RangeFinder_Backend.cpp` |
| Scripting scans `APM/scripts/` non-recursively; `modules/` is reached only via `require()` | `libraries/AP_Scripting/AP_Scripting.cpp` |

`TODO:` replace the path column with permalinks pinned to the exact firmware commit on the
board, and carry across the specific source links cited in
[`reviews/2026-08-25-mk2-failsafe-review.md`](reviews/2026-08-25-mk2-failsafe-review.md).

## ArduPilot documentation

- Rover Lua scripting — <https://ardupilot.org/rover/docs/common-lua-scripts.html>
- Scripting library reference (bindings, `Location_ud`, `rangefinder`, `vehicle`) —
  <https://ardupilot.org/dev/docs/common-lua-scripts.html>
- Rover parameter list (`WP_*`, `RNGFND1_*`, `FENCE_*`, `SCR_*`, `ATC_STR_RAT_MAX`) —
  <https://ardupilot.org/rover/docs/parameters.html>
- Rangefinder setup — <https://ardupilot.org/rover/docs/common-rangefinder-landingpage.html>
- Geofencing — <https://ardupilot.org/rover/docs/common-geofencing-landing-page.html>
- SITL for Rover — <https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html>
- MAVFTP file transfer — <https://ardupilot.org/planner/docs/common-mavftp.html>

## Hardware

- **CruzPro ATU120A** thru-hull depth transducer / sounder.
  `TODO:` obtain and archive the datasheet, and record here: exact NMEA sentences emitted,
  update rate, output logic level, supply voltage and current, beam angle, and stated
  assumed sound speed. Every one of those is currently taken from observed behaviour or
  from the vendor page rather than from a specification document.
- **3DR SiK V1.3** telemetry radio — see
  [`../hardware/repairs/sik-radio-overvoltage.md`](../hardware/repairs/sik-radio-overvoltage.md).
- **Microchip MIC5219** 3.3 V LDO — datasheet at <https://www.microchip.com/en-us/product/MIC5219>.

## Method

- Boustrophedon cellular decomposition — the foundational coverage-path-planning result,
  and the reason the pattern here looks like every robot vacuum and every ag robot.
  See [ADR-0003](decisions/0003-boustrophedon-state-machine.md).
  `TODO:` add the Choset coverage-path-planning survey citation.
- `TODO:` add a crossline-comparison / vertical-uncertainty reference to back the
  `crossline_run` field in `session.yaml`. IHO S-44 is the standard usually cited; confirm
  which order, if any, is worth quoting for a non-survey-grade vehicle.
