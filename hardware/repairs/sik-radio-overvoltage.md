# Repair log — 3DR SiK V1.3 telemetry radio, overvoltage event

Running log. **Newest entry last.** Each entry is dated and records what was done, what was
observed, and what is believed — kept separate, in that order.

> ## STATUS: DO NOT RECONNECT TO THE PIXHAWK
>
> A downstream short is suspected and has not been isolated. Connecting this radio to the
> flight controller risks pulling down the FC's 3.3 V or 5 V rail — which is a flight
> controller failure, not a radio failure. It stays disconnected until the resistance check
> below passes.

## The unit

| | |
|---|---|
| Item | 3DR SiK telemetry radio, V1.3 |
| Suspected failed device | `MIC5219-3.3YM5` 3.3 V LDO regulator |
| Replacement part | `MIC5219-3.3YM5-TR`, DigiKey, approx. $1.19 (BOM row) |

## Entries

### Entry 1 — overvoltage event

The radio was subjected to an overvoltage event.

`TODO:` record what actually happened — applied voltage, source, duration, and whether the
radio was connected to the Pixhawk at the time. If it was connected, the flight controller
must be checked too, and that check recorded here.

### Entry 2 — open component found in the VCC path, bridged

An open component was found in the VCC path — a **ferrite bead or a PTC fuse**; which of
the two is not established. It was **bridged** to restore continuity.

`TODO:` identify the component definitively (silkscreen reference, package, measured
resistance before bridging) and record how the bridge was made.

Note for the record: the component that went open-circuit was doing its job. A ferrite bead
or PTC in a supply path is a sacrificial protection element, and an open one is evidence of
the fault, not the fault itself.

### Entry 3 — bridging exposed a downstream short

With the bridge in place, the supply rail is shorted. The open protection component had
been masking a fault further downstream.

Most likely candidates, in order:

1. The **`MIC5219-3.3YM5` LDO** — a regulator that has seen overvoltage commonly fails
   shorted input-to-ground.
2. A **decoupling capacitor** on the input or output rail — ceramics fail short.

`TODO:` record the measured resistance across VCC/GND, on which rail, and with the bridge
in or out.

## Next steps

1. **Resistance check across VCC and GND, with the bridge in place.** Record the value and
   the rail. A few ohms or less confirms a hard short; hundreds of ohms suggests a partial
   failure and a different search.
2. **Isolate the regulator** — desolder it, or lift its input leg — and re-measure.
   - Short **clears** → the LDO is the failed part. Fit the replacement from the BOM.
   - Short **remains** → the LDO is innocent; work along the rail's decoupling caps,
     removing one at a time and re-measuring.
3. **Only after the rail measures sane with everything back on**, power the radio from a
   current-limited bench supply, standalone, and verify it enumerates and links.
4. **Only after that**, reconnect to the Pixhawk.

`TODO:` record the results of each step above as new entries.

## Lesson

**Bridging a sacrificial protection component can unmask a hidden downstream short.**

The bead or PTC going open was the protection working. Bridging it removed the protection
and reconnected a faulted circuit to whatever is upstream. That is an acceptable
*diagnostic* step on a bench supply with current limiting, and an unacceptable one on a
flight controller's rail.

Generalised: **confirm isolation before reconnecting a repaired device to any host.** The
device you are debugging is the cheap part. The host it is plugged into is not.
