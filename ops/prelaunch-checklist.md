# Pre-launch checklist

**The operational gate. Nothing in `vehicle/` goes on the water without this.**

Work top to bottom. A section is not started until the one above it is complete. Procedures
live in [`bench-tests.md`](bench-tests.md) and are referenced, not duplicated — if a
procedure changes, it changes in one place.

Copy this file into the session directory and tick it there, so the ticked copy is the
record of that session rather than of the last one.

---

## 0. Before the first Guided run — one time only

These gate the **first** autonomous run and are open until every one is done. Each links its
finding issue; the milestone **"First Guided water run"** reaching zero is the real go/no-go
criterion, and it is a better gate than a checklist someone can skim.

- [ ] **F1** — every stop path in the script is a no-op. Verified fixed: a commanded stop
      produces an actual **mode change to HOLD**, confirmed in a log, not just a GCS message.
- [ ] **F2** — GPS/EKF dropout kills the script and leaves the boat under power. Verified
      fixed: a nil `ahrs:get_location()` holds, and a module error leaves the dispatcher
      still ticking.
- [ ] **F3** — activating without a valid depth reading silently skips phase 1. Verified
      fixed.
- [ ] **F4** — a normal pivot looks exactly like a nav stall. Verified fixed, **and**
      `WP_PIVOT_RATE` verified on the board.
- [ ] **F5** — the offset can be satisfied by coasting forward. Verified fixed.
- [ ] **F6** — the depth debounce doesn't debounce. Verified fixed.
- [ ] **F7** — geofence and `MAX_LEG_DISTANCE_M` will fight each other. `FENCE_RADIUS`
      confirmed to exceed 60 m, and a rejected target confirmed to be caught.
- [ ] **F9** — `DEPTH_THRESHOLD_M` vs `RNGFND1_MIN_CM` leaves a band. Both measured and
      reconciled.
- [ ] **F11** — battery and mission duration are unbounded. `BATT_MONITOR` and
      `BATT_FS_LOW_ACT` configured.
- [ ] **Draft measured** and recorded in [`../data/calibration/draft.md`](../data/calibration/draft.md).
- [ ] **Sound speed calibrated** and recorded in [`../data/calibration/sound-speed.md`](../data/calibration/sound-speed.md).
- [ ] SiK radio either repaired and isolation-confirmed, or the run planned without
      telemetry ([`../hardware/repairs/sik-radio-overvoltage.md`](../hardware/repairs/sik-radio-overvoltage.md)).

`TODO:` reconcile this list against the review's own "before the first Guided run" section
once [`../docs/reviews/2026-08-25-mk2-failsafe-review.md`](../docs/reviews/2026-08-25-mk2-failsafe-review.md)
is in the repository. The items above are derived from the finding titles, not from the
review text.

---

## 1. Bench — props off

**Props off. Every item in this section is done with the thrusters unable to move the boat.**

- [ ] Props physically removed, or thrusters unbolted and clear of the water.
- [ ] `git status` clean; working tree matches the tag about to be deployed.
- [ ] Full parameter dump pulled **before** any change, saved to
      `vehicle/params/full/YYYY-MM-DD-preflight.param`.
- [ ] Scripts uploaded per [`../vehicle/DEPLOY.md`](../vehicle/DEPLOY.md) — MAVFTP, no card
      removal.
- [ ] `APM/scripts/` listed and contains **exactly one `.lua` file plus `modules/`**. No
      debug variant, no backup, no duplicate.
- [ ] Rebooted, and the dispatcher boot banner seen in Messages.
- [ ] No `PreArm: Scripting: out of memory`, no Lua load error naming a file.
- [ ] `SCR_USER1` toggled 0 → 2 → 0 and the switch messages seen.
- [ ] **Sonar bucket test** passed — [`bench-tests.md#sonar-bucket-test`](bench-tests.md#sonar-bucket-test).
- [ ] **`RFND` rows confirmed present in a downloaded onboard log** from that bucket test,
      with `Stat == 4`. A number in the GCS is not this test; the log is the deliverable.
- [ ] **Mode-gating test** passed — [`bench-tests.md#mode-gating-test`](bench-tests.md#mode-gating-test).
- [ ] **Depth-loss test** passed — [`bench-tests.md#depth-loss-test`](bench-tests.md#depth-loss-test).
- [ ] **Fence-rejection test** passed — [`bench-tests.md#fence-rejection-test`](bench-tests.md#fence-rejection-test).
- [ ] Battery pack charged, resting voltage recorded.
- [ ] Hull leak check done; no water in either pontoon.

## 2. SITL

- [ ] Full unattended sweep completed against `sim/lake_feeder.py` — both phases, edge
      detection, return to hub, station-keep.
- [ ] **Deliberate mid-leg feeder kill performed**, and the boat entered HOLD via a mode
      change while the dispatcher kept ticking. A SITL run without this does not count.
- [ ] Fence tighter than `MAX_LEG_DISTANCE_M` tried, and the rejected target caught.
- [ ] The SITL parameter set is **not** the one going on the boat. `RNGFND1_TYPE 10` is a
      simulator setting.

## 3. Dockside — in the water, tethered, not yet autonomous

- [ ] Boat in the water, **tether attached**, operator holding it.
- [ ] Transducer wet; depth readings sane and updating at roughly 1 Hz.
- [ ] Depth cross-checked against a weighted tape beside the transducer.
- [ ] Water level read against the fixed shore mark, recorded in `session.yaml`.
- [ ] GPS 3D fix, HDOP acceptable, EKF happy.
- [ ] Compass sane — bow pointed at a known bearing and checked.
- [ ] **Manual drive first.** Forward, reverse, and a full pivot in both directions on the
      sticks. Confirm skid steer behaves and the boat pivots in place.
- [ ] E-stop / disarm tested from the transmitter, in the water, with the boat moving.
- [ ] `SCR_USER1 = 0` confirmed before arming.
- [ ] Tag created **before** the session, from the deployed tree, and written into
      `session.yaml`.
- [ ] `session.yaml` filled in: draft, sound-speed scale, line spacing, tag, module,
      operator, weather, water-level datum.

## 4. Water — the autonomous run

- [ ] Operating area confirmed clear. **The boat cannot see obstacles** — the sonar looks
      down. The fence, not the sensor, keeps it off the shore.
- [ ] Geofence enabled and confirmed larger than `MAX_LEG_DISTANCE_M`.
- [ ] Transmitter on, in range, **thumb on the E-stop**, tether within reach for the first
      run.
- [ ] Second person watching the boat while the operator watches the GCS.
- [ ] Arm, enter GUIDED, then set `SCR_USER1 = 2`. Watch for the switch message.
- [ ] First leg observed to completion before the boat is allowed further out.
- [ ] Know the recovery: a script-commanded HOLD **sticks** until SWC is physically moved,
      because Rover reads the mode switch only on a change of position.
- [ ] Know the restart: re-entering GUIDED **resumes**; a genuine restart is
      `SCR_USER1` → 0 → back.

## 5. After the run

- [ ] Log downloaded, SHA-256 taken, `MANIFEST.md` filled in.
- [ ] `session.yaml` completed — outcome, incidents.
- [ ] **Crossline run** — one manual perpendicular pass. This is the only real vertical
      uncertainty estimate the survey will ever have.
- [ ] Anything unexpected written up in [`incident-log.md`](incident-log.md), on water or
      bench, whether or not it broke anything.
- [ ] `CHANGELOG.md` entry added for the tag.
