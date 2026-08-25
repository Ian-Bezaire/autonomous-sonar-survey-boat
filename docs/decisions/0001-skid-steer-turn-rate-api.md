# ADR-0001 — Use `set_desired_turn_rate_and_speed()`, not `set_steering_and_throttle()`

## Context

A skid-steer catamaran turns by differential thrust, and the survey pattern needs pivots
that complete accurately and repeatably. Two Lua APIs appear to offer control over that:

- `vehicle:set_steering_and_throttle()` — direct, low-level, and the obvious first choice.
- `vehicle:set_desired_turn_rate_and_speed()` — goes through the rate controller.

`set_steering_and_throttle()` has a documented bug affecting skid-steer vehicles. Taking
the low-level path also means reimplementing rate control in Lua, on a 100 ms tick, with no
access to the tuning the firmware already has.

## Decision

All script-commanded motion uses `vehicle:set_desired_turn_rate_and_speed()`.
`set_steering_and_throttle()` is not used anywhere in `vehicle/scripts/`.

## Consequences

- Turn rate is bounded by `ATC_STR_RAT_MAX` (default 120 °/s). The script cannot command
  more than the tuning allows, which is correct but means a weak rate tuning shows up as a
  turn timeout rather than as a slow turn.
- **The turn-rate argument is in degrees per second**, despite the vehicle-agnostic
  binding name. Passing a radian value — e.g. `3.0` intended as ~172 °/s — commands ~3 °/s,
  turning the vehicle more than fifty times slower than intended and starving the turn
  timeout. This has bitten this project once; it is recorded in the mk1 header for the
  next person.
- Motion quality now depends on skid-steer tuning (`ATC_STR_RAT_*`, `WP_PIVOT_*`) being
  correct on the board. That is the right place for it, but it moves a dependency from the
  script into the parameter set — which is part of why params are versioned alongside
  scripts.

## Status

Accepted.
