# Draft calibration

**Draft is the vertical distance from the transducer face to the waterline, with the boat
loaded exactly as it runs.**

## Why it matters

The ATU120A emits `$SDDBT` — **Depth Below Transducer**. The transducer is not at the
surface, so the reported distance is short of the true depth by the draft, every time:

```
depth_below_surface = reported_distance + DRAFT_M
```

`analysis/extract_soundings.py` applies it **after** the sound-speed scale factor, because
draft is a fixed geometric offset and is not part of the measured water column. Scaling it
would introduce an error proportional to the draft on every sounding.

## Why getting it wrong is the worst kind of wrong

A draft error is a **constant offset applied to every sounding in the survey**. It does not
add noise, it does not make anything look broken, and it does not vary — so there is
nothing in the data that reveals it. Every contour is displaced by the same amount, the map
looks entirely self-consistent, and it is uniformly wrong.

It is also the easiest constant to get wrong, because it changes with load and nobody
notices the boat got heavier.

## Current value

**`TODO: measure.`** No draft has been measured. Until one is, no sounding this vehicle
produces is vertically referenced — and `analysis/extract_soundings.py` requires an explicit
`--draft-m` rather than defaulting to something plausible.

| date | draft (m) | load condition | method | measured by |
|---|---|---|---|---|
| | | | | |

## Procedure

`TODO:` write this up properly once it has been done once. Outline:

1. Load the boat **exactly as it will run** — battery in, all electronics fitted, hatches
   closed. This is the whole point; a draft measured on a bare hull is a different number.
2. Float it in still water.
3. Measure vertically from the transducer face to the waterline. If the face is not
   accessible in the water, measure from a known hull reference to the face on the bench,
   and from that reference to the waterline afloat, then add.
4. Record the load condition alongside the number.
5. Check both pontoons' waterlines and record any list — a listing catamaran means the beam
   is not vertical, and a tilted beam over a sloping bottom reads long.

## Re-measure when

- The battery changes size or position.
- Anything is added, removed or moved.
- The hull has taken on water. A foam-cored pontoon that has wicked water gains weight
  invisibly and changes draft with it — see
  [`../../hardware/transducer-install.md`](../../hardware/transducer-install.md).
- At the start of each season, on principle.

Record the value in use in each `session.yaml`. The number here is the current best; the
number in the session file is what was actually applied that day.
