# ADR-0007 — Bathymetry logging stays in firmware, not Lua

## Context

The deliverable of a survey is a set of georeferenced soundings, so something must record
depth alongside position and attitude. Lua can write log messages, and writing a single
purpose-built `SNDG` message per ping — depth, lat, lon, yaw, all pre-joined — is tempting
because it removes the post-processing join entirely.

Against that: the module's `update()` runs on the same 100 ms tick that drives the boat,
the sensor updates at roughly 1 Hz, and the vehicle is an F4 with 256 KB of SRAM total
where script heap is already the binding constraint.

## Decision

Bathymetry is logged by the **firmware**, as `RFND` (rangefinder), `POS` (position) and
`ATT` (attitude), at the firmware's own rates. Lua writes only **state transitions** — the
survey state machine's own events, which exist nowhere else and are what makes a log
readable six months later.

The join between `RFND`, `POS` and `ATT` happens afterwards, on a laptop, in
[`../../analysis/extract_soundings.py`](../../analysis/extract_soundings.py).

## Consequences

- **Better fidelity.** `RFND`, `POS` and `ATT` are logged at full rate with firmware
  timestamps, not at whatever rate a 100 ms Lua tick happens to sample them at. The
  post-hoc `merge_asof` on `TimeUS` with a tolerance is a more honest reconstruction than a
  Lua-side snapshot that silently pairs a fresh position with a sensor value ArduPilot has
  been holding for up to 3 seconds.
- **Zero risk to the driving loop.** No log write happens inside the code path that steers
  the boat. A logging problem cannot become a navigation problem.
- **The F4 heap is not there to spend.** Script memory is the scarcest resource on this
  vehicle; buffering soundings in Lua would consume it for something the firmware already
  does for free.
- The cost is a post-processing step that must get the join right, including the
  200 ms tolerance and the `Instance == 0 & Stat == 4` filter. That logic lives in one
  reviewed script rather than being reimplemented on the vehicle.
- Verifying that `RFND` rows actually appear in a real onboard log is therefore a
  pre-launch gate, not an assumption — it is in
  [`../../ops/prelaunch-checklist.md`](../../ops/prelaunch-checklist.md). A survey that
  drives perfectly and logs no `RFND` produces nothing at all.

## Status

Accepted.
