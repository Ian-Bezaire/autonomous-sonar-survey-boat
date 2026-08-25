# ADR-0003 — Boustrophedon cellular decomposition as the coverage pattern

## Context

With no forward sensing ([ADR-0002](0002-no-slam-downward-sonar-only.md)), coverage must be
planned geometrically. The requirement is to pass a downward single-beam sensor over as much
of an area as possible, at roughly even spacing, with a track that a boat with a turning
circle can actually follow.

## Decision

Boustrophedon ("ox-turning") cellular decomposition: parallel lines with alternating
direction, offset by a fixed spacing at each end. Implemented as an explicit state machine
in the module — mk1 `FORWARD → TURN_1 → OFFSET → TURN_2`, mk2 `FORWARD → OFFSET` on absolute
bearings with two sweep phases from a common hub.

## Consequences

- It is the foundational coverage algorithm across robot vacuums, agricultural robots and
  USV bathymetry. Choosing it means the failure modes are known and the literature applies.
- Line spacing is the single knob that trades coverage density against run time
  (`OFFSET_DISTANCE_M`). Spacing must comfortably exceed `WP_RADIUS` or the offset target is
  "reached" the instant it is set — this is not theoretical; it hung mk2 in SITL.
- Even spacing over a non-uniform bottom is effort spent where there is nothing to see.
  That is precisely the inefficiency mk3's depth-adaptive spacing is meant to remove, and
  it does not invalidate this decision — mk3 is still boustrophedon, with a varying pitch.
- A boustrophedon sweep gives no independent check on its own vertical accuracy. That is
  what the manual perpendicular crossline in `session.yaml` is for.
- An explicit state machine means every state is a named thing that can be logged, and a
  stall in any one of them is detectable. This is what makes the nav-stall watchdog
  possible at all.

## Status

Accepted.
