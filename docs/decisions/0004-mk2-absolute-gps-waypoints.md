# ADR-0004 — mk2 navigates on absolute GPS waypoints

## Context

mk1 is dead-reckoned. Each leg's heading is achieved by pivoting a nominal 90° twice from
whatever heading the previous leg ended on, with turn progress integrated from the compass.
Every source of turn error — overshoot from momentum, rate-controller undershoot, compass
error, wind and current during the pivot — is therefore inherited by the next leg and
carried forward. Over a survey of any length the lines fan out, the spacing stops being the
spacing, and there is no mechanism anywhere in the loop that pulls the pattern back.

The vehicle already has an absolute position estimate and an EKF that is being trusted for
everything else.

## Decision

mk2 captures `base_bearing_deg` from the compass and `start_loc` from the EKF **once per
activation**, at `init()`. Every subsequent leg and offset target is computed as an absolute
`Location` from those two captured values. Legs are commanded with
`set_target_location()` and driven by `AR_WPNav`, which holds the line by cross-track
correction.

## Consequences

- **Turn error cannot compound.** Leg *n+1*'s heading derives from `base_bearing_deg`, not
  from leg *n*'s achieved heading. A sloppy pivot costs the entry to one leg and nothing
  after it.
- Cross-track correction now keeps the boat on the commanded line, so a crosswind produces
  a crab angle instead of a displaced track.
- Compass error becomes a single fixed rotation of the whole survey rather than a
  compounding drift. Wrong, but wrong consistently, and correctable after the fact.
- A dependency on EKF position is created *inside the navigation loop*, not just in
  logging. `ahrs:get_location()` is bound as `Location'Null` and returns `nil` with no
  position estimate — so every use must be nil-checked, and a nil must be a hold rather
  than a Lua error. This is the direct cause of finding F2.
- Two sweep phases from a common hub (`base + 90°`, then `base - 90°`) become possible and
  cheap, which is what lets a mid-lake start still cover the whole water body. Under
  dead reckoning the second phase would start from wherever the first one drifted to.
- Arrival tests must account for `WP_RADIUS`: ArduPilot marks a destination reached within
  that radius and the boat holds station, so the last `WP_RADIUS` metres of any commanded
  step are never travelled.
- Offset arrival must be measured as the **projection onto the offset bearing**, not as a
  scalar radius from the offset start. A boat has no brakes; coasting along the old
  forward heading otherwise satisfies the offset with no lateral movement at all, and the
  boat retraces the line it just ran.

## Status

Accepted.
