# Roadmap

Three maturity tiers. They are **files, not branches** — mk1 and mk2 coexist as sibling
modules because `survey_dispatcher.lua` genuinely selects between them at runtime via
`SCR_USER1`. That is a feature of the vehicle, not a version-control artefact. Git history
records how each module got the way it is; the module files record what the boat can
currently be told to do.

| Tier | `SCR_USER1` | Status |
|---|---|---|
| mk1 — relative-heading state machine | `1` | Superseded, retained |
| mk2 — GPS-anchored two-phase sweep | `2` | **Current.** Not yet water-tested |
| mk3 — boundary polygon + adaptive contour following | — | Not started |

---

## mk1 — relative-heading state machine

**Scope.** Dead-reckoned boustrophedon. `FORWARD → TURN_1 → OFFSET → TURN_2`, driven by
`set_desired_turn_rate_and_speed()` with turn progress measured by integrating per-step
heading change. Depth below `DEPTH_THRESHOLD_M` triggers the turn; `FORWARD_COOLDOWN_S`
prevents an immediate re-trigger over the same shallow patch.

**Entry criteria.** Met historically.

**Exit criteria.** Met — the tier demonstrated depth-triggered turning in SITL and
established the two API facts that mk2 depends on: turn rate is degrees/sec despite the
binding name, and `ATC_STR_RAT_MAX` is the real ceiling.

**Status: superseded, retained for comparison.** Its structural limit is that each leg's
heading derives from the previous leg's *achieved* heading, so every degree of turn error
compounds down the survey. Retained because it is the only thing to A/B a mk2 track
against, and because it is cheap to keep — one file, inert until `SCR_USER1 = 1`.

## mk2 — absolute GPS waypoints, two-phase sweep, dispatcher supervision

**Scope.** `base_bearing_deg` and `start_loc` are captured once per activation. Every leg
aims at an absolute target computed from those two, so leg *n+1* does not inherit leg *n*'s
error. Phase 1 sweeps `base + 90°`, phase 2 sweeps `base - 90°`, both from the same hub, so
a mid-lake start still covers the whole water body. A phase ends on `EDGE_STREAK_COUNT`
consecutive short legs, or on loss of the depth signal mid-leg. `MISSION_TIMEOUT_S` bounds
the whole run; the vehicle returns to `start_loc` and station-keeps.

**Entry criteria.** Met — dispatcher supervision in place, absolute-waypoint navigation
implemented, failsafe hardening pass applied against the pre-launch review.

**Exit criteria — none of these are met yet.**

- [ ] Every issue labelled `blocks-water-test` closed.
- [ ] Full unattended sweep in SITL against `sim/lake_feeder.py`, **including a deliberate
      mid-leg feeder kill** — a synthetic lake that never breaks proves nothing about the
      boundaries.
- [ ] Bench mode-gating test passed with props off (`ops/bench-tests.md`).
- [ ] `RFND` rows confirmed present in a real onboard log during the bucket test.
- [ ] Draft and sound speed measured (`data/calibration/`).
- [ ] Battery monitoring and failsafe action configured (F11).
- [ ] First water run completed manual-first, tethered, with a thumb on the E-stop.

**Status: current, pre-first-Guided-run.** Passing SITL is necessary and not sufficient.

## mk3 — pre-loaded boundary, adaptive contour following

**Scope.** Three changes, each of which removes an assumption mk2 has to make:

- **Pre-loaded boundary polygon.** The survey area is given rather than discovered, so
  phase ends are geometric instead of inferred from short legs, and the sweep can be
  planned to cover a real shoreline rather than a rectangle around a hub.
- **Gaussian-process depth model with contour following.** Build a GP over the soundings
  online and steer along a target depth contour, which is where the interesting structure
  is and where a fixed lawnmower spends the least time.
- **Depth-adaptive line spacing.** Widen spacing over flat deep water, tighten it where
  the gradient is high. Coverage effort follows information, not geometry.

**Entry criteria.** mk2 exit criteria met, and at least one real survey log processed
end-to-end through `analysis/` — a GP fitted to soundings whose QC has never been
exercised on real data is a model of the noise.

**Exit criteria.** TODO: define once mk2 has flown. Likely "matches an mk2 survey of the
same water body to within the crossline-derived vertical uncertainty, in materially less
run time".

**Status: not started.** The closest published analogue is Wilson & Williams — an ACFR
twin-hull differential-thrust ASV doing exactly this with single-beam sonar. Read it before
designing this tier: [`references.md`](references.md).

**Open question, unresolved.** Whether a GP and online hyper-parameter optimisation can
live on an F4 at all, or whether mk3 forces a companion computer — which would reverse a
non-goal in [`goal.md`](goal.md). Do not assume the answer; measure it. See the
`SCR_HEAP_SIZE` row in [`../vehicle/params/verified.md`](../vehicle/params/verified.md).
