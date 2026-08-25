# Manifest — `<session directory name>`

Raw logs are not committed. This file is how a log recorded here is found again and proven
to be the file that was recorded. See [`../../README.md`](../../README.md).

| file | sha256 | bytes | location |
|---|---|---|---|
| | | | |

## Filling this in

```bash
sha256sum 00000042.BIN
stat -c '%s' 00000042.BIN
```

- **file** — the filename as downloaded from the flight controller. Do not rename it; the
  autopilot's own numbering is the only link back to the board's log index.
- **sha256** — the full digest. This is what makes the row a proof rather than a note.
- **bytes** — exact size. A truncated download is a common and quiet failure; size plus
  digest catches it.
- **location** — a real, findable place. An external drive **by its label**, a cloud path,
  a NAS share. "My laptop" is not a location. If a log is lost or deliberately deleted, say
  so in this column rather than leaving a row that implies it still exists.

One row per file. Include telemetry logs (`.tlog`) as well as the onboard `.BIN` if both
were captured — they are not interchangeable, and the `.tlog` is the only record of what the
GCS saw if the boat did not come back.
