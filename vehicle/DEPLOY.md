# Deploying to the boat

## The rule

**`vehicle/scripts/` in this repo is a byte-for-byte mirror of `APM/scripts/` on the SD
card. Deployment is a copy, never a transform.**

Any workflow that assembles, concatenates or rewrites files before upload will eventually
put a stale script on the boat, and you will not find out until the water. If a file needs
to change, change it here, commit it, then copy it.

Target layout on the card:

```
APM/
└── scripts/
    ├── survey_dispatcher.lua          ← the ONLY file directly in scripts/
    └── modules/
        ├── depth_survey_mk1.lua
        └── depth_survey_mk2.lua
```

ArduPilot scans `APM/scripts/` **non-recursively** and auto-runs everything it finds there.
`modules/` is reached only via `require()`. That is the whole mechanism —
[ADR-0005](../docs/decisions/0005-dispatcher-supervisor-pattern.md).

## Never put a debug variant in `scripts/`

`survey_dispatcher_mk2only.lua` is a **debug variant of the dispatcher, not a third
module.** If it is placed in `scripts/`, ArduPilot auto-runs it *alongside* the real
dispatcher and **both will drive the boat**, each with its own idea of the current target.

Keep it out of `scripts/`. If it needs to be versioned, it goes in `vehicle/variants/` —
which is a repo-only directory that is never copied to the card.

The same applies to any `.lua.bak`, `depth_survey_mk2 (1).lua`, or editor backup that ends
up in `scripts/`. Before every upload, list the directory and confirm it contains exactly
one file plus `modules/`.

## Scripts and params deploy together

A Lua script plus the wrong `RNGFND1_MIN_CM` is a different vehicle. Never upload one
without confirming the other.

1. Pull a full parameter dump **before** you change anything, into
   `params/full/YYYY-MM-DD-context.param`.
2. Apply `params/survey.param` if anything in it has drifted.
3. Upload the scripts.
4. Tag the tree (see below) and record the tag in the session's `session.yaml`.

## Upload procedure — MAVFTP, no card removal

Removing the SD card means opening the enclosure, which means the enclosure is open on a
dock. Use MAVFTP.

1. Connect the GCS over USB or telemetry. (Note: the SiK radio is currently faulty — see
   `../hardware/repairs/sik-radio-overvoltage.md`. Use USB.)
2. Open the MAVFTP file browser and navigate to `APM/scripts/`.
3. Upload `survey_dispatcher.lua` into `APM/scripts/`.
4. Upload both modules into `APM/scripts/modules/`, creating the directory if it does not
   exist.
5. **List `APM/scripts/` and confirm there is exactly one `.lua` file plus `modules/`.**
6. Confirm `SCR_ENABLE = 1` and set **`SCR_USER1 = 0`** before rebooting. `0` is idle; the
   boat should never come up already selecting a survey module.
7. Reboot the flight controller. Scripts are only loaded at boot.

## Post-upload verification — watch the Messages tab

On reboot you must see:

```
SURVEY: dispatcher loaded -- SCR_USER1: 0=idle, 1=mk1 dead-reckoning, 2=mk2 GPS-anchored
```

If that line does not appear, the dispatcher did not load. Then:

- Set `SCR_USER1 = 2` and confirm `SURVEY: switched to '<name>' (SCR_USER1=2)`. Switching
  takes effect on the next tick — no reboot, no re-upload.
- Set it back to `0` before doing anything else.

**Watch specifically for:**

| Message | Meaning |
|---|---|
| `PreArm: Scripting: out of memory` | The scripts do not fit. See the `SCR_HEAP_SIZE` row in [`params/verified.md`](params/verified.md) — the lever is script footprint, not the parameter. |
| A Lua syntax or load error naming a file | That file did not load. Do not proceed. |
| `SURVEY: SCR_USER1=n has no matching algorithm` | The selector value has no module. |
| `SURVEY: init failed, staying idle: …` | The module loaded but `init()` threw. |
| No `SURVEY:` line at all | The dispatcher is not running — check `SCR_ENABLE` and that you rebooted. |

A silent boot is a failed deployment, not a clean one.

## Tag before the session

Every state that runs anywhere real gets a tag, made **before** the session, from the tree
actually deployed:

```
git tag -a mk2-water-2026-09-06 -m "deployed for <water body>"
```

Format is `<module>-<venue>-<date>` — venue is `sitl`, `bench` or `water`. Record the tag in
that session's `session.yaml`. A session without a tag is a session whose data cannot be
trusted later.

## Rollback

**Last tag known to have been on the boat: `mk2-review-2026-08-25`** — a *state* tag. That
tree has never been deployed to hardware and has never run. There is no verified-good state
to roll back to yet.

`TODO:` update this line after every deployment. It is the only place that records what is
actually on the card right now.

To roll back:

```
git checkout <tag> -- vehicle/scripts vehicle/params
```

then repeat the upload procedure above, including the reboot and the Messages check.
Rolling back scripts without rolling back params is not a rollback.
