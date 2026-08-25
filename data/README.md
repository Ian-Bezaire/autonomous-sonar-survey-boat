# Data

## The contract

**Provenance in git. Payload out of it.**

Raw `.BIN` flight logs are **not committed**. A single survey log dwarfs this entire source
tree, git cannot diff it, and once a binary is in history it is in history forever. What
git holds instead is everything needed to identify a log, find it, and prove the file you
found is the file that was recorded:

```
data/sessions/2026-09-06-mill-pond/
├── session.yaml     ← conditions, constants, and the tag that was on the boat
├── MANIFEST.md      ← filename, SHA-256, byte count, where the bytes live
└── prelaunch.md     ← the ticked checklist for this session (copy it in)
```

Every session directory is named for its date and water body, and names its tag inside
`session.yaml`.

## Why the tag matters more than anything else here

A depth number is meaningless without the code that produced it, the parameters that shaped
it, and the constants it was corrected with. Six months on, a weird contour is a mystery
unless you can reconstruct all three — and a git tag reconstructs all three exactly, for the
cost of one command before the session.

```
git tag -a mk2-water-2026-09-06 -m "deployed for Mill Pond"
```

Tag **before** the session, from the tree that was actually deployed, and write the tag into
`session.yaml`. **A session without a tag is a session whose data cannot be trusted later.**
It is not recoverable afterwards: "it was roughly the August code" is not a parameter set.

## Calibration

Two constants turn a raw distance into a depth, and both are measured, not assumed:

- [`calibration/draft.md`](calibration/draft.md) — transducer face to waterline.
- [`calibration/sound-speed.md`](calibration/sound-speed.md) — the scale factor between the
  sounder's assumed sound speed and the water's actual one.

Both are recorded per session in `session.yaml`, because both change: draft with load,
sound speed with temperature. The values in `calibration/` are the current best and the
method for re-measuring; the values in `session.yaml` are what was actually applied that
day.

## Starting a session

1. Copy [`sessions/TEMPLATE/`](sessions/TEMPLATE/) to `sessions/YYYY-MM-DD-water-body/`.
2. Fill in `session.yaml` **before** launching — everything except `outcome` and
   `incidents`.
3. Copy `ops/prelaunch-checklist.md` in and tick it there.
4. After the run: download the log, `sha256sum` it, fill in `MANIFEST.md`, complete
   `session.yaml`, and add a `CHANGELOG.md` entry for the tag.

## Where the bytes actually live

`MANIFEST.md` must record a real, findable location — an external drive by its label, a
cloud path, a NAS share. "My laptop" is not a location. If a log is deleted or lost, say so
in the manifest rather than leaving a row that implies it exists.

`TODO:` decide and record the canonical archive location for raw logs, and put it here.

## If large-file storage is ever wanted

The migration path is `git-lfs` tracking `data/sessions/**/*.BIN`. It is deliberately **not**
the default: LFS adds a dependency for every clone, needs a server willing to hold gigabytes,
and solves a problem the manifest already solves for a one-person project. Revisit only if
these files genuinely need to travel with the repository.

## Derived products are not committed either

`data/derived/`, `*.png`, `*.tif` and `*.geotiff` are gitignored. Every one of them is
reproducible from a log plus the `analysis/` scripts plus the constants in `session.yaml` —
and a committed chart is a chart that will silently disagree with the pipeline the first
time a QC threshold changes.
