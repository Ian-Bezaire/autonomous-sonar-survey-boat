# Bench tests

Reusable procedures with explicit pass criteria. [`prelaunch-checklist.md`](prelaunch-checklist.md)
references these rather than duplicating them, so a procedure improves in one place.

**Every procedure here is run with props off**, unless it says otherwise. None of them
require the boat to move.

A test passes only if the stated pass criterion is observed. "It printed the message" is
not a pass criterion for anything on this vehicle — finding F1 was precisely a set of stop
paths that printed the message and did nothing.

---

## Sonar bucket test

**Question it answers:** does the whole depth chain — transducer, divider, TELEM2, driver,
logging — actually produce usable data?

1. Suspend the transducer in a bucket or tank of water, face down, fully submerged, clear of
   the sides. Note the water depth under the face with a rule.
2. Power up. Do not arm.
3. Watch the rangefinder reading in the GCS. Confirm it updates at roughly 1 Hz and is
   approximately the measured depth.
4. Raise and lower the transducer by a known amount and confirm the reading tracks.
5. Lift it clear of the water and confirm the status goes bad rather than reporting a
   plausible number.
6. **Download the onboard log** and inspect it.

**Pass criteria — all of:**

- `RFND` rows present in the downloaded log.
- `Instance == 0` and `Stat == 4` on the good readings.
- Values in metres, tracking the physical depth to within the tank's own uncertainty.
- Out-of-water readings are *not* status Good.

**This test is not passed by seeing a number in the GCS.** The survey product comes from
the log; if `RFND` is not in the log there is no survey, and finding that out after a lake
run costs a day.

`TODO:` while doing this, measure `RNGFND1_MIN_CM` empirically — the depth below which the
driver stops returning Good — and record it in `vehicle/params/verified.md`. That number
is what closes F9.

---

## Mode-gating test

**Question it answers:** does the dispatcher's armed+GUIDED gate actually control whether
the module drives the boat, and does a script-commanded stop actually change mode?

**Props off. Watch the servo outputs, not the boat.**

1. Power up, `SCR_USER1 = 0`. Confirm no survey messages.
2. Arm, but stay in a mode that is **not** GUIDED. Set `SCR_USER1 = 2`.
   Confirm the switch message appears and **no motion is commanded** — outputs stay
   neutral.
3. Enter GUIDED. Confirm the module begins commanding.
4. Leave GUIDED (switch to HOLD). Confirm commanding stops immediately.
5. Re-enter GUIDED. Confirm the module **resumes** — it re-sends its target rather than
   sitting idle, and it does **not** re-init (F8).
6. Disarm while in GUIDED. Confirm commanding stops.
7. Force a stop condition and confirm the vehicle **changes mode to HOLD** (F1).

**Pass criteria — all of:**

- No output is commanded unless armed **and** in GUIDED.
- The Hold → Guided flip produces a resumed target, not a stalled module.
- A commanded stop is visible as a **mode change in the log**, not only as a GCS message.
- A script-commanded HOLD sticks until SWC is physically moved.

---

## Depth-loss test

**Question it answers:** what does the boat do when the depth signal goes away mid-leg —
which it will, over weed, over a thermocline, and during every hard pivot?

Run in SITL with `sim/lake_feeder.py`, and again on the bench with the transducer lifted out
of the bucket.

1. Get the module into `STATE_FORWARD` with valid depth.
2. Remove the depth signal — Ctrl-C the feeder, or lift the transducer.
3. Wait past `DEPTH_STALE_TIMEOUT_S` (3.0 s).
4. Restore the signal and observe recovery.
5. Separately: activate the module with **no** depth signal at all, and wait past
   `DEPTH_FIRST_FIX_TIMEOUT_S` (10.0 s).

**Pass criteria — all of:**

- Loss during a forward leg produces a **mode change**, not just a warning (F1).
- The dispatcher keeps ticking throughout — a module fault must not take the supervisor
  down (F2).
- Activation with no depth ever seen refuses to survey rather than declaring phase 1
  finished (F3).
- Loss *outside* `STATE_FORWARD` warns once and carries on. Stopping mid-lake is strictly
  worse than completing the trip back to the hub, and that is deliberate.

---

## Fence-rejection test

**Question it answers:** what happens when the geofence rejects a commanded target?

Rover's guided destination setter validates the destination and returns **false** when it
rejects one — notably against the fence. Unchecked, that is a leg that silently never
happens.

1. In SITL, set `FENCE_ENABLE = 1` and `FENCE_RADIUS` **below** `MAX_LEG_DISTANCE_M`
   (60.0 m) — e.g. 30 m.
2. Run the module and let it command a full-length leg.
3. Observe.
4. Repeat with `FENCE_RADIUS` comfortably above 60 m and confirm normal operation.

**Pass criteria — all of:**

- The rejected target is **caught**, reported, and results in a stop — not ignored.
- `preflight_check()` warns at activation when the fence and `MAX_LEG_DISTANCE_M`
  contradict each other (F9's mechanism, applied to F7).
- With an adequate fence, legs run normally.

**Note:** this test also establishes the operational rule — `FENCE_RADIUS` must exceed
`MAX_LEG_DISTANCE_M`, and both must fit inside the actual water body. A fence that fits the
lake but not the leg length means the survey cannot run at all.

---

## `TODO:` procedures not yet written

- **Pivot-rate characterisation.** Measure the boat's real achievable pivot rate with
  `WP_PIVOT_RATE` as set, in water, and compare against `NAV_STALL_TIMEOUT_S` (15.0 s) and
  `TURNING_YAW_DEG_PER_TICK` (0.5). This is what actually closes F4; the code change only
  makes the watchdog pivot-aware.
- **Battery discharge characterisation.** Needed before `BATT_LOW_VOLT` and
  `BATT_CRT_VOLT` mean anything (F11).
- **Compass check against a known bearing**, with the thrusters running — skid-steer
  current next to a compass is a classic interference source.
