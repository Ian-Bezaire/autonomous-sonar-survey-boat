# ADR-0005 — Dispatcher-as-supervisor, modules inert until required

## Context

Several complete survey algorithms need to live on the flight controller at once — mk1 for
comparison, mk2 as the current tier, more later. ArduPilot's scripting engine auto-runs
**every** `.lua` file directly in `APM/scripts/`, and scans that directory
non-recursively. Two survey scripts dropped in side by side would both run, both gate
themselves on armed+GUIDED, and both command the vehicle. Selecting between algorithms by
deleting files from the SD card is a deployment step performed on a dock, in the wind,
under time pressure — which is to say, a way to end up running the wrong one.

Files under `scripts/modules/` are not auto-run. They execute only when something
`require()`s them.

## Decision

Exactly one file lives in `scripts/`: `survey_dispatcher.lua`. Every algorithm lives in
`scripts/modules/` as a `require()`-able module exposing a fixed contract —
`NAME`, `init(now_ms)`, `update(now_ms)`, `stop(now_ms)`, and optionally `resume(now_ms)`.

The dispatcher owns everything the modules therefore do not: reading `SCR_USER1` to select
a module, gating on armed **and** GUIDED, scheduling at 100 ms, and isolating module
failures. Switching is `stop()` → lookup → `init()` within a single tick, with no scheduler
cycle in between, so no stale command from the outgoing module survives into the incoming
one.

## Consequences

- Algorithms are switched at runtime by setting `SCR_USER1` — no re-upload, no reboot. `0`
  is idle and is the default, so the boat is safe out of the box.
- Mode gating and scheduling exist in exactly one place. A module cannot forget to check
  whether it is armed, because it is never called unless it is.
- A `require()`d module runs inside the dispatcher's own chunk, so an uncaught module error
  would take the supervisor with it — no more ticks, no mode gate, and the boat still
  driving to the last Guided target. Every module entry point is therefore called through
  `pcall`; an error commands HOLD and deactivates the module until `SCR_USER1` is toggled.
  This is the fix for finding F2.
- The gate is edge-detected, and the module gets `resume()` on the rising edge, because
  `ModeGuided::_enter()` calls `wp_nav.init()` and preserves no target. Without it, the
  Hold→Guided flip you *will* perform on the water leaves the module believing a target is
  loaded that ArduPilot has already discarded. This is the fix for finding F8.
- Re-entering GUIDED deliberately **resumes** rather than re-inits. A genuine restart is
  `SCR_USER1` → 0 → back. This is a real footgun and is documented in both the dispatcher
  header and `vehicle/DEPLOY.md`.
- A module that does not implement `resume()` (mk1) is simply not called; nothing breaks.
- **`vehicle/scripts/` in this repo must mirror `APM/scripts/` byte for byte**, because the
  layout *is* the mechanism. A debug variant such as `survey_dispatcher_mk2only.lua` must
  not be placed in `scripts/` — it would be auto-run alongside the real dispatcher and both
  would drive the boat. See `vehicle/DEPLOY.md`.

## Status

Accepted.
