# TELEM2 depth interface

How the transducer's depth reading reaches ArduPilot's rangefinder driver. This is the one
hardware document with substantive content available; everything unverified is marked.

## Sensor output

The **CruzPro ATU120A** thru-hull transducer emits **NMEA 0183** depth sentences:

| Sentence | Meaning |
|---|---|
| `$SDDBT` | **Depth Below Transducer** |
| `$SDDPT` | Depth, with transducer offset field |

- **Baud: 4800**, 8N1 — standard NMEA 0183.
- **Logic level: 5 V.**
- Update rate: approximately 1 Hz. *Established by observation, not from a datasheet.*

`$SDDBT` being depth *below the transducer* is the reason `data/calibration/draft.md`
exists. The transducer is not at the waterline, so:

```
depth_below_surface = reported_distance + DRAFT_M
```

Get the draft wrong and every contour on every chart is off by the same constant, with
nothing in the data to reveal it. See [`../data/calibration/draft.md`](../data/calibration/draft.md).

## Level shifting

The Pixhawk's TELEM2 RX pin is **3.3 V**; the transducer drives **5 V**. A **510 Ω**
resistive divider steps the signal down.

`TODO:` record the exact divider — both resistor values, tolerance, power rating, and the
measured voltage at the TELEM2 RX pin with the transducer transmitting. "510 Ω" is what is
established; which leg of the divider it is, and what the other leg is, is not recorded
anywhere yet and must not be guessed from this file.

`TODO:` confirm whether the transducer's RX line is connected at all. Nothing in this
system sends to the sounder, so a receive-only wiring is expected — record it explicitly
either way.

## Grounding

**Star-ground topology. Motor return current must not share a conductor with the UART
signal reference.**

The depth sentence is a voltage measured against the flight controller's ground. If the
thruster return path shares that conductor, thruster current modulates the reference, and
depth sentences corrupt precisely when the boat is drawing the most current — during every
pivot, which is also when prop-wash aeration is already degrading the reading. The two
failure modes stack, and they stack at the same moments.

This is why grounding appears as a constraint in
[`../docs/architecture.md`](../docs/architecture.md) rather than only as a wiring note, and
why `analysis/qc.py` gates soundings on yaw rate.

`TODO:` record the physical star point — see [`power-tree.md`](power-tree.md).

## ArduPilot side

The autopilot parses the NMEA stream through its serial rangefinder driver. Parameters
below are the ones that must agree with the wiring; the authoritative record of values and
how each was verified is
[`../vehicle/params/verified.md`](../vehicle/params/verified.md).

| Parameter | Purpose | Value |
|---|---|---|
| `SERIAL2_PROTOCOL` | Set TELEM2 to the rangefinder/NMEA protocol | `TODO:` record the value actually on the board |
| `SERIAL2_BAUD` | Must be 4800 to match the sensor | `TODO:` verify — the parameter is in units of kbaud-style codes, record the literal value set |
| `RNGFND1_TYPE` | NMEA rangefinder backend | `TODO:` record. Note: SITL uses `10` (MAVLink) for the feeders; the boat does **not** |
| `RNGFND1_ORIENT` | `25` = down | Established — the scripts test for `25` |
| `RNGFND1_MIN` / `RNGFND1_MIN_CM` | Minimum valid range; below it the driver reports out-of-range | `TODO:` measure. **Blocks F9** |
| `RNGFND1_MAX` / `RNGFND1_MAX_CM` | Maximum valid range | `TODO:` record |

`TODO:` confirm which parameter naming your firmware uses. The scripts read
`RNGFND1_MIN_CM`; `sim/depth_feeder.py` sets `RNGFND1_MIN` / `RNGFND1_MAX`. These are
different names across ArduPilot versions and **the mismatch itself is a live risk** — a
script reading a parameter name the firmware does not have gets `nil`, and
`preflight_check()` then compares against a default of 0. Record the firmware version and
the true names here.

## What the driver gives the scripts

- `rangefinder:distance_orient(25)` returns **metres**.
- Status `Good` is **4** (`AP_RangeFinder.h`). The scripts require it.
- Instance `0` is the downward sonar. `analysis/extract_soundings.py` filters
  `Instance == 0 & Stat == 4`.
- ArduPilot **holds a stale value for up to `read_timeout_ms()` = 3000 ms**. This is why
  the depth debounce in mk2 is time- and sample-based rather than tick-based (finding F6):
  three 100 ms ticks against a ~1 Hz sensor is usually the same sentence counted three
  times.

## Verification

The bucket test in [`../ops/bench-tests.md`](../ops/bench-tests.md) is the gate on this
whole chain. It is not passed by seeing a number in the GCS — it is passed by finding
`RFND` rows with `Stat == 4` in a downloaded onboard log, because that is the data the
survey actually produces.

## Diagram

`TODO:` labelled wiring diagram in [`diagrams/`](diagrams/): transducer pinout, divider
with both resistor values, TELEM2 connector pinout, supply feed, and the star point.
