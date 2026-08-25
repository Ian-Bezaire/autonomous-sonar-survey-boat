# Decision records

One file per architectural decision, numbered, in the order they were taken.

## Why these exist

Six months on, the useful question is never "what does this code do" — the code answers
that. It is "why is it like this, and what would break if I changed it back?" An ADR is the
only place that answer survives. Several decisions here exist specifically because the
obvious alternative *looks* correct and silently does nothing on this vehicle; without a
record, they get re-broken.

## Format

Four headings, in this order:

- **Context** — the forces in play. What was true when the decision was needed.
- **Decision** — what was chosen, stated in the active voice.
- **Consequences** — what follows, including what got worse. An ADR with only upsides in
  this section is not finished.
- **Status** — `Proposed` | `Accepted` | `Superseded by ADR-NNNN`.

## Rules

- **Numbered, never renumbered.** `NNNN-kebab-case-title.md`.
- **Never edited after acceptance.** If a decision changes, write a new ADR that supersedes
  it and set the old one's status to `Superseded by ADR-NNNN`. The wrong turn is part of the
  record; editing it away destroys the only evidence of why it was a wrong turn.
- **One decision per file.** If the title needs an "and", it is two ADRs.

## Index

| # | Decision | Status |
|---|---|---|
| [0001](0001-skid-steer-turn-rate-api.md) | Use `set_desired_turn_rate_and_speed()`, not `set_steering_and_throttle()` | Accepted |
| [0002](0002-no-slam-downward-sonar-only.md) | No SLAM, no frontier exploration | Accepted |
| [0003](0003-boustrophedon-state-machine.md) | Boustrophedon cellular decomposition as the coverage pattern | Accepted |
| [0004](0004-mk2-absolute-gps-waypoints.md) | mk2 navigates on absolute GPS waypoints | Accepted |
| [0005](0005-dispatcher-supervisor-pattern.md) | Dispatcher-as-supervisor, modules inert until required | Accepted |
| [0006](0006-stop-by-mode-change.md) | Stop by mode change, not `set_desired_speed(0)` | Proposed |
| [0007](0007-logging-in-firmware-not-lua.md) | Bathymetry logging stays in firmware, not Lua | Accepted |
