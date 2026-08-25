# Power tree

**Status: stub. Nothing in this document has been measured.**

`TODO:` measure and record every value below. Until then, treat this file as a list of the
questions rather than as documentation.

## Pack

`TODO:` chemistry, nominal voltage, cell count, capacity, connector, resting and loaded
voltage. Feeds the mission-duration bound.

## Distribution

`TODO:` what the pack lands on — distribution board, BEC(s), harness gauge and lengths.
Note whether the flight controller and the thrusters share a conductor anywhere other than
the star point.

## Flight-controller rail

`TODO:` supply path to the Pixhawk, rail voltage measured at the board, current draw idle
and under load, and what happens to this rail on a thruster inrush transient.

## Sensor rail

`TODO:` transducer supply voltage and current, GPS supply, whether the transducer is fed
from the FC rail or separately. The transducer's 5 V logic output and its supply are
different questions — record both. Cross-reference
[`telem2-depth-interface.md`](telem2-depth-interface.md).

## Motor rail

`TODO:` ESC supply, stall and peak current per thruster, measured return path.

## Star ground

`TODO:` identify and photograph the single star point, and record which returns land on it.

This is not a wiring preference. Motor return current sharing a conductor with the UART
signal reference modulates the reference the depth sentences are measured against, and it
does so hardest during turns — exactly when the boat is generating the most interesting
data. See [`../docs/architecture.md`](../docs/architecture.md).

## Fusing

`TODO:` main fuse rating and type, any per-branch fusing, and the reasoning for each
rating. Record the E-stop / arming disconnect position in this chain.

## Known gap — battery monitoring is unconfigured

`BATT_MONITOR` is unset and `BATT_FS_LOW_ACT` has no configured action, so the vehicle has
**no awareness of pack state and no automatic response to a low pack**. Combined with an
autonomous mission, that means run duration is bounded only by `MISSION_TIMEOUT_S` in the
script — a wall-clock cap, not an energy cap.

Tracked as **F11 — battery and mission duration are unbounded**, labelled
`blocks-water-test`. Also recorded in
[`../vehicle/params/verified.md`](../vehicle/params/verified.md).

`TODO:` fit and calibrate a monitor, then set `BATT_MONITOR`, the voltage/current
calibration parameters, `BATT_LOW_VOLT`, `BATT_CRT_VOLT` and `BATT_FS_LOW_ACT` /
`BATT_FS_CRT_ACT`, and record each in `verified.md` with how it was verified.

## Diagram

`TODO:` labelled power diagram in [`diagrams/`](diagrams/), showing the star point
explicitly.
