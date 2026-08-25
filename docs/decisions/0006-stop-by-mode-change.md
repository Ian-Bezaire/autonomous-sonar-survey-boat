# ADR-0006 — Stop by mode change, not `set_desired_speed(0)`

## Context

Every failsafe path in the survey modules needs to stop the boat: depth signal lost, nav
stall, mission timeout, an invalid target, a nil position. The obvious implementation is
`vehicle:set_desired_speed(0)`, and it is what the early scripts used.

It does nothing. `AR_WPNav::set_speed_max()` rejects anything below `AR_WPNAV_SPEED_MIN`
(0.05 m/s) and **returns false**. Because the return value was not checked, every halt path
in the script was a `gcs:send_text()` call and nothing else — while the boat continued
driving toward a Guided target that could be up to `MAX_LEG_DISTANCE_M` away.

This is the worst class of bug available on this vehicle: the failsafe reports that it
fired, the log says the script noticed, and the boat keeps going.

## Decision

Stopping is done by commanding a **mode change**, not by commanding a speed.
`enter_hold()` commands `MODE_HOLD`. Where the boat should hold position rather than drift
at the end of a completed mission, `MODE_LOITER` is used instead.

Every `set_target_location()` return value is checked, for the same reason: a setter that
can reject silently must not be trusted to have succeeded.

## Consequences

- A commanded stop actually stops the vehicle, through the firmware's own mode logic rather
  than through a speed the navigation controller is entitled to ignore.
- It also **drops the vehicle out of GUIDED**, which closes the dispatcher's armed+GUIDED
  gate — so the module stops being called as a direct consequence of stopping the boat.
  One action, two correct effects.
- Rover reads the mode switch only on a *change* of switch position, so a script-commanded
  HOLD sticks until the operator physically moves SWC. That is the desired behaviour for a
  failsafe and a surprise for anyone who expects the transmitter to still be in charge; it
  belongs on the pre-launch briefing, not only in this file.
- Recovery is an operator action by design. Re-entering GUIDED resumes the mission via
  `resume()`; it does not restart it.
- **This ADR is `Proposed`, not `Accepted`, because the fix is unverified on hardware.**
  The mode-change stop path is implemented in the current mk2 module, but the mode-gating
  bench test with props off has not been run and no log has been inspected to confirm the
  transition actually occurs. Implementation and verification are tracked in the **F1**
  issue; this record moves to `Accepted` when that closes.

## Status

Proposed — implementation tracked in the F1 issue, and gated on the mode-gating bench test
in [`../../ops/bench-tests.md`](../../ops/bench-tests.md).
