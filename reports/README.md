# CAD validation

KiCad CLI 10.0.5 checked the unchanged Rev-A PCB and schematic on
2026-09-18. The project symbol library resolves through `${KIPRJMOD}`.

| Check | Errors | Warnings | Unconnected items |
|---|---:|---:|---:|
| DRC | 0 | 4 | 0 |
| ERC | 0 | 1 | Not applicable |

The four DRC warnings concern the local H1-H4 mounting-hole footprints, which
are absent from the configured `MountingHole` library. The ERC warning is a
U1 library-symbol mismatch: the embedded symbol hides pin numbers and uses
`~` for the datasheet field, while the library copy does not hide the numbers
and leaves that field empty. All 44 pin definitions match, including numbers,
names, electrical types, positions, and lengths.

The reports include all severities and no excluded violations. The existing
project configuration ignores these checks:

- DRC: missing courtyard, track endpoint not centered on via, tuning profile
  geometry, footprint filter mismatch, and footprint type mismatch.
- ERC: single-use global label, four-way junction, SPICE model issue, and
  footprint filter mismatch.

Run these commands from the project root with KiCad 10.0.5:

```sh
kicad-cli pcb drc --format json --severity-all -o reports/drc.json sh03-controller.kicad_pcb
kicad-cli sch erc --format json --severity-all -o reports/erc.json sh03-controller.kicad_sch
```

The Gerber outline matches the board at 100 × 56 mm. All nine BOM/CPL
references match the board, including placement coordinates and rotations
with the exported Y-axis sign convention. C1 pad 1 connects to +5V and pad 2
to GND.

These reports cover CAD checks. The project author has also personally validated
correct operation of the assembled Rev-A board in the dryer.
