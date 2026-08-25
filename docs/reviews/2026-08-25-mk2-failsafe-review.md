# mk2 pre-launch failsafe review — 25 August 2026

> **PLACEHOLDER — THIS FILE IS NOT THE REVIEW.**
>
> `TODO:` replace this entire file with the owner's existing review document, verbatim.
> The source file is referred to as `mk2_failsafe_review.md` in the headers of
> `vehicle/scripts/survey_dispatcher.lua` and `vehicle/scripts/modules/depth_survey_mk2.lua`,
> and it was not available when this repository was built. Nothing in it has been
> reconstructed here, because a reconstructed failsafe review is worse than a missing one.
>
> Three things depend on this file and are incomplete until it lands:
>
> 1. Issues **F1–F12** — each body should be the finding text verbatim, plus a
>    *Definition of done*, plus a link to the `#fn` anchor in this document.
> 2. The **"Before the first Guided run"** section of `ops/prelaunch-checklist.md`.
> 3. The specific ArduPilot source links cited in the review, which belong in the table in
>    [`../references.md`](../references.md).
>
> When pasting it in, keep the heading anchors `#f1` … `#f12` intact — the issues link to
> them.

## What is known about this review without the document

The findings are already referenced by number in the two script headers, which were
hardened against them. That gives the *subject* of each finding but not the review's own
text, its reasoning, or its severity assignment.

| # | Subject, as referenced from the scripts | Where the reference lives |
|---|---|---|
| F1 | Every stop path is a no-op — `set_desired_speed(0)` is rejected below `AR_WPNAV_SPEED_MIN` | mk2 header |
| F2 | GPS/EKF dropout: `ahrs:get_location()` returns nil, and an uncaught module error takes the dispatcher down with it | mk2 header, dispatcher header |
| F3 | `last_valid_depth_ms` starting at 0 meant the stale-depth watchdog was pre-expired at first tick | mk2 header |
| F4 | The nav-stall watchdog could not tell a pivot from a stall | mk2 header |
| F5 | Offset arrival measured as a scalar radius could be satisfied by coasting forward | mk2 header |
| F6 | Tick-based depth debounce counted the same NMEA sentence repeatedly | mk2 header |
| F7 | `set_target_location()` return values unchecked; `MAX_LEG_DISTANCE_M` vs `FENCE_RADIUS` | mk2 header |
| F8 | `ModeGuided::_enter()` discards the target; re-entering GUIDED left a stale belief | mk2 header, dispatcher header |
| F9 | Script tunables and flight-controller parameters could contradict each other | mk2 header (`preflight_check()`) |
| F10 | `SCR_HEAP_SIZE` dead end on F4 hardware | not referenced in the scripts — see `vehicle/params/verified.md` |
| F11 | Mission duration unbounded — `MISSION_TIMEOUT_S` added | mk2 header |
| F12 | Arrival tests ignored `WP_RADIUS`; the 2.0 m offset target was already "reached" when set | mk2 header |

**Note that the scripts as committed already contain hardening for F1–F9, F11 and F12.**
The issues remain open regardless: the code changes are **unverified** — no SITL run, no
bench run and no water run is recorded against them — and F4, F9, F10 and F11 additionally
require parameter or hardware work that no script change can supply.

`TODO:` F12 does not appear in the repository design spec's issue list. Confirm whether it
is a twelfth finding in the review or was raised separately, and open it either way.
