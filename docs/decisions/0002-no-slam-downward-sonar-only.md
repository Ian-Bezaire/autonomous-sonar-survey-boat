# ADR-0002 — No SLAM, no frontier exploration

## Context

"Autonomous boat that maps something" invites the mobile-robot exploration literature:
SLAM, frontier-based exploration, occupancy grids. Both families assume the robot can sense
*where it has not yet been* — a forward-facing lidar, camera or scanning sonar that returns
the boundary between known and unknown space ahead of the vehicle.

This vehicle has GPS for position and a single downward-looking transducer for depth. The
transducer has no look-ahead whatsoever: it reports the depth under the hull, now. There is
no frontier to detect, and no exteroceptive measurement to close a loop against.

## Decision

No SLAM and no frontier exploration. Position comes from GPS/EKF and is trusted as-is.
Coverage is planned geometrically ([ADR-0003](0003-boustrophedon-state-machine.md)) rather
than driven by an exploration objective.

## Consequences

- Rules out obstacle avoidance as a consequence, not as a separate choice — the vehicle
  cannot see obstacles. Operating water must be chosen clear, and the geofence, not the
  sensor, is what keeps the boat off the shore.
- Position error is whatever the GPS gives. There is no mechanism to correct it, so the
  survey inherits consumer-GPS horizontal accuracy. Recorded as a non-goal in
  [`../goal.md`](../goal.md).
- The onboard computation problem stays small enough for Lua on an F4. A GP or an
  occupancy grid would not fit — see [ADR-0007](0007-logging-in-firmware-not-lua.md) and
  the `SCR_HEAP_SIZE` row in `vehicle/params/verified.md`.
- Positively: this places the vehicle in **established USV survey methodology** — plan
  lines, run lines, grid the soundings — rather than in mobile-robot exploration. That
  literature is the right one to borrow from, and it is what Wilson & Williams is doing.
- mk3's contour following is an *adaptive line plan*, not exploration. It still uses no
  forward sensing; it fits a model to soundings already taken. This ADR does not block it.

## Status

Accepted.
