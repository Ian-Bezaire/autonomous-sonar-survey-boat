# Sound-speed calibration

**Calibrate. Do not assume.**

## The problem

An echo sounder measures a round-trip time and converts it to a distance using an assumed
speed of sound. Sounders are typically calibrated for **seawater, around 1500 m/s**.

Fresh water at about 15 °C carries sound at roughly **1465 m/s**.

A sounder assuming ~1500 m/s in water that actually carries sound at ~1465 m/s reports
distances about **2.4 % too deep**. On a 10 m bottom that is roughly 24 cm — a systematic
error, in one direction, on every reading.

Note the difference from draft: a draft error is a constant **offset**; a sound-speed error
is a **gain** error. Deep water is wrong by more than shallow water, so slopes read steeper
or shallower than they are, and the shape of the bottom is distorted rather than merely
displaced.

## The correction

A single scale factor, applied to the raw distance **before** draft is added:

```
depth = raw_distance * sound_speed_scale + draft_m
sound_speed_scale = true_sound_speed / sounder_assumed_sound_speed
```

For 1465 / 1500 that is 0.9767 — but **do not use that number**. It is an illustration of
the arithmetic, not a value for your water. Measuring the ratio directly cancels the assumed
speed out entirely, which is what makes the bar-check method below robust: you never need to
know what the sounder assumed.

## Current value

**`TODO: measure.`** No calibration has been performed.
`analysis/extract_soundings.py` requires `--sound-speed-scale` explicitly and has no
default, deliberately.

| date | scale factor | water temp (°C) | depth used (m) | tape reading | sounder reading | measured by |
|---|---|---|---|---|---|---|
| | | | | | | |

## Procedure — bar check with a weighted tape

1. At the dock, in about **3 m** of still water, transducer wet, boat floating as it runs.
2. Lower a weighted tape (or a marked line with a plate on the end) into the water
   **immediately beside the transducer**, straight down, and read the depth. Beside the
   transducer matters — a metre away is a different bottom.
3. Read the sounder at the same moment.
4. Compute:

   ```
   sound_speed_scale = tape_depth_below_transducer / sounder_reading
   ```

   Both quantities must be measured from the **same datum** — the transducer face. If the
   tape was read at the waterline, subtract the draft before taking the ratio.
5. Repeat at two or three depths if the water allows. The ratio should be consistent; if it
   is not, something other than sound speed is wrong and the calibration is not the thing
   to fix.
6. Record the water temperature. Sound speed in fresh water varies by roughly 3 m/s per °C
   near 15 °C, so a survey in spring and one in August are not the same calibration.

## Re-measure when

- Water temperature has changed materially since the last calibration.
- Surveying a different water body.
- At the start of each season.

Record the value in use in each `session.yaml`. Two sessions with different scale factors
are not directly comparable until both are corrected — which is exactly why the value lives
per-session rather than only here.

## The independent check

None of the above tells you whether the *whole chain* is right; it only calibrates the
sounder. The **crossline run** does: one manual perpendicular pass across the survey lines,
compared against the interpolated surface at the crossing points. The scatter at those
crossings is the only real vertical uncertainty estimate this survey will ever have.

`crossline_run` is a field in `session.yaml` for that reason. It defaults to `false` and
should not stay that way.
