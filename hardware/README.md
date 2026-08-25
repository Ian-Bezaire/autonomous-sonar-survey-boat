# Hardware

Physical build, electrical interfaces, and repair history.

| File | What it holds |
|---|---|
| [`bom.csv`](bom.csv) | Parts, sources, costs. Machine-readable, one row per part. |
| [`power-tree.md`](power-tree.md) | Pack → distribution → rails → star ground → fusing. **Stub.** |
| [`telem2-depth-interface.md`](telem2-depth-interface.md) | Transducer → divider → TELEM2, and the ArduPilot side. |
| [`transducer-install.md`](transducer-install.md) | Thru-hull mounting in a foam-cored fibreglass pontoon. |
| [`diagrams/`](diagrams/) | Wiring and layout diagrams. Empty — see the TODOs in each doc. |
| [`repairs/`](repairs/) | One running log per repaired item, newest entry last. |

## Ground rules

- **No invented values.** Anything not measured or read off a part is a `TODO`, not a
  plausible number. A wrong resistor value in a wiring document is worse than a blank one,
  because a blank one gets checked.
- **Every electrical claim says how it was established** — measured, read from a datasheet,
  or observed working. These are not the same level of confidence.
- **Star ground is a signal-integrity requirement**, not a wiring preference. It appears in
  [`../docs/architecture.md`](../docs/architecture.md) for that reason.

## Known-bad hardware, right now

- **3DR SiK V1.3 telemetry radio — do not reconnect to the Pixhawk.** An unisolated
  downstream short is suspected. See
  [`repairs/sik-radio-overvoltage.md`](repairs/sik-radio-overvoltage.md).
- **Battery monitoring and battery failsafe action are unconfigured.** Mission duration and
  pack state are both unbounded. Tracked as F11; see
  [`power-tree.md`](power-tree.md) and `../vehicle/params/verified.md`.
