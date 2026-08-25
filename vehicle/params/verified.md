# Verified parameters

One row per **non-default** parameter. This file is the *reasoning* record — why a value is
what it is, and how anyone knows it is right. [`survey.param`](survey.param) is the machine
record; the two must agree.

A row is only "verified" when the **verified how** column describes an observation, not an
intention. `Reasoned` and `Set, unverified` are honest values and are far more useful than
an optimistic `Verified`.

**Firmware version on the board:** `TODO: record exact ArduPilot Rover version and git hash.`
Several rows below depend on it — notably the `RNGFND1_MIN_CM` / `RNGFND1_MIN` naming.

## Scripting

| param | value | why | verified how | date |
|---|---|---|---|---|
| `SCR_ENABLE` | `1` | Enables the Lua scripting engine. Nothing on this vehicle works without it. Requires a reboot to take effect. | Observed — the dispatcher's boot banner appears in Messages. | 2026-08-25 |
| `SCR_USER1` | `0` | Module selector read by `survey_dispatcher.lua`: `0` idle, `1` mk1 dead-reckoning, `2` mk2 GPS-anchored. Deployed as `0` so the boat is never armed already selecting a survey module. Switching takes effect on the next 100 ms tick — no reboot, no re-upload. | Observed — `SURVEY: switched to …` on change. | 2026-08-25 |
| `SCR_HEAP_SIZE` | *stock* `44032` | See the note below. | Reasoned — see below. | 2026-08-25 |

### `SCR_HEAP_SIZE` — the F4 dead end (F10)

The Pixhawk 2.4.8 is an STM32F4 with **256 KB of SRAM in total**, shared by the entire
autopilot.

Both values that circulate as fixes for `PreArm: Scripting: out of memory` are impossible
on this board: **1000000** is roughly four times the total SRAM, and **65536** is a quarter
of it handed to scripting alone, on top of everything the EKF, the drivers and the logging
already need. The stock **44032** is approximately what the board can actually spare.

**The lever is script footprint, not the parameter.** If the scripts do not fit, the fix is
smaller scripts — fewer locals held across ticks, no accumulating tables, no string
building in the hot loop — or different hardware. Raising this number past what the board
has does not produce memory; it produces a different failure.

This is also the constraint that bounds mk3: a Gaussian process with online hyper-parameter
optimisation is not obviously something this heap can hold. Measure before committing to it
— see [`../../docs/roadmap.md`](../../docs/roadmap.md).

Tracked as **F10 — SCR_HEAP_SIZE dead end on F4 hardware** (`context`).

## Depth sensing

| param | value | why | verified how | date |
|---|---|---|---|---|
| `RNGFND1_ORIENT` | `25` | Down. Both modules test for it before trusting a reading. | Reasoned — required by the scripts. `TODO:` confirm set on the board. | — |
| `RNGFND1_TYPE` | `TODO` | Selects the NMEA serial backend for the ATU120A. **Not** the MAVLink type `10` used by the SITL feeders. | `TODO` | — |
| `RNGFND1_MIN_CM` | `TODO` | Minimum valid range. Below it the driver reports out-of-range rather than a short reading, which is what creates the F9 band. | `TODO: measure in the bucket test.` **Blocks F9.** | — |
| `RNGFND1_MAX_CM` | `TODO` | Maximum valid range. Must exceed the deepest water to be surveyed or the bottom simply disappears mid-lake. | `TODO` | — |
| `SERIAL2_PROTOCOL` | `TODO` | Puts TELEM2 on the rangefinder/NMEA protocol. | `TODO` | — |
| `SERIAL2_BAUD` | `TODO` | Must correspond to **4800** to match the ATU120A. | `TODO` | — |

`TODO:` confirm the parameter naming your firmware uses — the scripts read
`RNGFND1_MIN_CM`, while `sim/depth_feeder.py` sets `RNGFND1_MIN`. A script reading a name
the firmware does not have gets `nil`, and `preflight_check()` then compares against a
default of `0`, which silently disables the check it exists to perform.

### F9 — the unmeasurable band

`DEPTH_THRESHOLD_M` (5.0 m in both modules) is the depth at which the boat decides it is
shallow and turns. `RNGFND1_MIN_CM` is the depth below which the driver stops returning a
valid reading at all.

If the minimum sits well below the threshold, there is a band — potentially 1–5 m — where
the water is shallower than the trigger but the sensor still reports `Good`, or where it
has gone out-of-range and the script sees *no* reading rather than a *shallow* one. Those
two situations demand opposite responses and are not currently distinguishable.

Both numbers must be measured and then reconciled. Tracked as **F9** (`medium`, `params`).

## Navigation and turning

| param | value | why | verified how | date |
|---|---|---|---|---|
| `WP_RADIUS` | `TODO` (default `2.0`) | ArduPilot marks a destination reached within this radius, so the last `WP_RADIUS` metres of every commanded step are never travelled. mk2 reads it at `init()` and scales both arrival tests off it (F12). `OFFSET_DISTANCE_M` must be comfortably larger — at 2.0 m the offset target was already "reached" the moment it was set, which is the unexplained SITL hang in this project's notes. | `TODO: record the value on the board.` | — |
| `WP_PIVOT_ANGLE` | `TODO` | Heading error above which Rover pivots in place rather than turning while moving. Required for clean skid-steer pivots. Note the real names are `WP_PIVOT_*`, not `PIVOT_TURN_*`. | `TODO` | — |
| `WP_PIVOT_RATE` | `TODO` | Commanded pivot rate. If it is too low for the boat, a pivot takes longer than the nav-stall watchdog expects. | `TODO: unverified on the board.` **Blocks F4.** | — |
| `ATC_STR_RAT_MAX` | `TODO` (default `120` °/s) | The achievable turn-rate ceiling. mk1's `TURN_RATE_DEGPS` cannot exceed it. | `TODO` | — |

## Geofence

| param | value | why | verified how | date |
|---|---|---|---|---|
| `FENCE_ENABLE` | `TODO` | `preflight_check()` reads it and warns if the fence and `MAX_LEG_DISTANCE_M` contradict each other. | `TODO` | — |
| `FENCE_RADIUS` | `TODO` | **Must exceed `MAX_LEG_DISTANCE_M` (60.0 m).** Rover's guided destination setter rejects a target outside the fence and returns false; mk2 now checks that return, so the symptom is a refused leg rather than a runaway — but a fence tighter than the leg length means the survey simply cannot run. | `TODO` — **F7** | — |
| `FENCE_ACTION` | `TODO` | What the vehicle does on a breach. Must be compatible with a script that stops by commanding HOLD. | `TODO` | — |
| `FENCE_TYPE` | `TODO` | Circle, polygon, or both. | `TODO` | — |

## Battery — currently unconfigured (F11)

| param | value | why | verified how | date |
|---|---|---|---|---|
| `BATT_MONITOR` | `TODO` — **unset** | With no monitor the vehicle has no awareness of pack state. Run duration is bounded only by `MISSION_TIMEOUT_S` (1800 s) in the script — a wall-clock cap, not an energy cap. | `TODO` — **F11**, `blocks-water-test` | — |
| `BATT_FS_LOW_ACT` | `TODO` — **no action** | No automatic response to a low pack during an unattended autonomous mission. | `TODO` — **F11** | — |
| `BATT_LOW_VOLT` | `TODO` | | `TODO` | — |
| `BATT_CRT_VOLT` | `TODO` | | `TODO` | — |
| `BATT_FS_CRT_ACT` | `TODO` | | `TODO` | — |

## Script-side tunables are not parameters

`DEPTH_THRESHOLD_M`, `OFFSET_DISTANCE_M`, `MAX_LEG_DISTANCE_M`, `MISSION_TIMEOUT_S` and the
rest live as `local`s at the top of each module, not as flight-controller parameters. They
change by editing and re-uploading the script, which means **they change by commit** — and
that is deliberate, because a tunable that can be changed from the GCS mid-session is a
tunable that will not be recorded in `session.yaml`.

`preflight_check()` in mk2 exists to catch the resulting risk: script constants and
flight-controller parameters describing the same physical quantity, drifting apart. It
warns over the GCS. Read those warnings.
