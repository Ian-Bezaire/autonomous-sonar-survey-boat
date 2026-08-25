# Changelog

The units here are **field states**, not software releases. One entry per tag.

Tags are `<module>-<venue>-<date>` — e.g. `mk2-water-2026-09-06`. A *state* tag
(`<module>-review-<date>`) marks a tree that was reviewed but has not run anywhere;
it is labelled as such below so it is never mistaken for a session.

Each entry records: what changed in `vehicle/`, what was tested and where
(SITL / bench / water / untested), and what is known-broken at that state.

Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

---

## [mk2-review-2026-08-25] — 2026-08-25 — *state tag, not a session*

First state committed to git. Nothing at this state has run on water.

### Changed in `vehicle/`

- Imported `survey_dispatcher.lua` and `modules/depth_survey_mk2.lua` as they exist on the
  SD card, verbatim. No defects patched in the import — see Known broken.
- `modules/depth_survey_mk1.lua` retained as a sibling module; the dispatcher selects between
  tiers at runtime via `SCR_USER1`.
- Seeded `params/verified.md` and `params/survey.param` with the non-default parameters
  established so far.

### Tested

**Untested.** No SITL run, no bench run, no water run is recorded against this tag.

### Known broken

Findings F1–F12 from the pre-launch failsafe review, tracked as issues and recorded in
`docs/reviews/2026-08-25-mk2-failsafe-review.md`. Everything labelled `blocks-water-test`
is open.

The imported scripts already contain hardening for F1–F9, F11 and F12 — but **none of it
is verified**. No SITL run, no bench run and no water run has been performed against these
changes, and F4, F9, F10 and F11 additionally need parameter or hardware work that no
script change can supply. An unverified fix to a stop path is not a working stop path, so
this state must not be armed on water.

The review document itself is a placeholder in this tree; the real file has not been
imported yet, so the issue bodies and the checklist's first-Guided-run section are both
derived from finding titles rather than from the review text.

Outside the review: the SiK V1.3 telemetry radio has an unisolated downstream short and
must not be reconnected to the Pixhawk (`hardware/repairs/sik-radio-overvoltage.md`);
draft and sound-speed are unmeasured, so no sounding produced at this state is
vertically referenced.
