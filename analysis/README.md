# Analysis

Log → soundings → QC → grid → contour. Three scripts, run in order.

```
extract_soundings.py  →  qc.py  →  grid_and_plot.py
```

Each runs standalone, takes `--log` / `--in` and `--out`, and **prints what it dropped and
why**. That last part is the point: a bathymetry pipeline that quietly discards data
produces a clean-looking map of a subset you never chose.

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt

python3 extract_soundings.py --log 2026-09-06-lake.BIN --out soundings_raw.csv \
    --draft-m 0.21 --sound-speed-scale 0.9767
python3 qc.py --in soundings_raw.csv --out soundings_qc.csv
python3 grid_and_plot.py --in soundings_qc.csv --out lake-contours.png
```

## Where the numbers come from

`--draft-m` and `--sound-speed-scale` are **not** defaulted to anything plausible, because a
plausible default is exactly how an uncalibrated survey ends up looking calibrated. Measure
them and record them:

- [`../data/calibration/draft.md`](../data/calibration/draft.md)
- [`../data/calibration/sound-speed.md`](../data/calibration/sound-speed.md)

Both also belong in that session's `session.yaml`, so a re-run six months from now uses the
values that were true on the day.

## Why the join happens here and not on the boat

The vehicle logs `RFND`, `POS` and `ATT` separately, at firmware rates, and never joins
them. That is [ADR-0007](../docs/decisions/0007-logging-in-firmware-not-lua.md): better
fidelity, no logging code in the driving loop, and no F4 heap spent on something the
firmware already does. The cost is this join, and getting it right is what these scripts
are for.

## Raw logs are not in this repository

`.BIN` files are gitignored. Each session directory carries a `MANIFEST.md` recording the
filename, SHA-256 and where the bytes actually live. Provenance in git, payload out of it —
[`../data/README.md`](../data/README.md).
