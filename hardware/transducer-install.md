# Transducer installation

## Design: thru-hull, in a foam-cored fibreglass pontoon

The ATU120A is a thru-hull transducer mounted through one of the catamaran's foam-cored
fibreglass pontoons, looking straight down.

**The sealed fitting is a non-negotiable mechanical requirement.** A foam core is not a
waterproof structure — it is a sponge behind a skin. A penetration that leaks does not
produce a visible drip into a bilge; it wicks water into the core, where it stays. The
pontoon gains weight, the draft changes silently (which corrupts every sounding), the
foam-to-skin bond degrades, and none of it is visible from outside until the hull is soft.

There is no in-water repair for this and no way to detect it from the data. It must be
right at installation.

## What is settled

- Thru-hull, not transom-mount and not shoot-through. A single downward beam.
- Mounted in a pontoon, looking down, clear of the thruster wash where possible.
- The fitting is sealed against the foam core, not merely against the outer skin.

## What is not — do not guess these

`TODO:` **Hull penetration location.** Record it as a measured offset from a fixed
reference on the hull (e.g. bow tip and centreline), with a photograph. This matters twice:
once for the mechanical work, and once because the transducer's horizontal offset from the
GPS antenna is a fixed lever arm that biases every sounding by that offset in the direction
of travel. Record the GPS-to-transducer offset in the same note.

`TODO:` **Core sealing method.** The standard approach is to overbore the hole, dig back
the foam, fill the annulus with thickened epoxy, then re-drill through solid epoxy so the
fastener and the fitting never touch the core. Confirm whether that is what was done, and
record it. If it was not, say so — this is exactly the detail that gets remembered wrongly.

`TODO:` **Sealant and backing plate specification.** Product, cure time, backing plate
material and dimensions, torque or tightening approach. Note compatibility with the
laminate and with the fitting material.

`TODO:` **Measured draft** — transducer face to waterline, with the boat loaded as it runs.
This is a survey constant, not a build note; record it in
[`../data/calibration/draft.md`](../data/calibration/draft.md) and reference it from
`session.yaml`. Re-measure whenever the load changes.

`TODO:` **Orientation check.** Confirm the transducer face is level with the boat at rest
and record any measured tilt. A tilted beam over a sloping bottom reads long, and does so
consistently.

`TODO:` **Leak check procedure and result.** Record the date, the method, and what was
observed. Repeat before the first water session of each season and after any hull work; log
each check in [`../ops/incident-log.md`](../ops/incident-log.md) if anything is found.

## Photographs

`TODO:` before-sealing, after-sealing and in-water waterline photographs into
[`diagrams/`](diagrams/).
