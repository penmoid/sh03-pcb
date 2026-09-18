# Rev-A layout

This document describes the current board geometry and the constraints used by
the placement and routing scripts. Dimensions are millimetres.

## Enclosure and board

| Feature | Dimension or location |
|---|---|
| Enclosure cavity | 135.2 × 82.2 |
| Cable window | X 21.5 to 33.5, Y 15.5 to 66.7 |
| Board outline | 100 × 56, KiCad X 36 to 136 and Y 33 to 89 |
| Mounting holes | Four 3.2 mm finished holes, approximately 4 mm from board corners |

The board clears the enclosure bosses within the cavity Y band 10.5 to 71.6.
J1 and J2 sit in the cable window on the back side. J2 is the 15-position
connector in the north slot. J1 is the 14-position connector in the south slot.
Both connectors have pin 1 at the north end of their span.

## Placement

| Ref | Side | KiCad centre | Rotation | Function |
|---|---|---:|---:|---|
| J2 | B.Cu | (47.4, 49.625) | 270° | 15-position vents connector |
| J1 | B.Cu | (47.4, 73.625) | 270° | 14-position thermal and air connector |
| R1 | B.Cu | (75.95, 43.0) | 270° | TH2 33k pull-up |
| R2 | B.Cu | (78.49, 43.0) | 270° | TH1 33k pull-up |
| R3 | B.Cu | (108.97, 78.0) | 270° | MC_HOT2 10k pull-down |
| R4 | B.Cu | (111.51, 78.0) | 270° | MC_HOT1 10k pull-down |
| C1 | B.Cu | (117.0, 42.0) | 0° | 470 µF +5V bulk capacitor |
| C3 | B.Cu | (110.0, 42.0) | 0° | 10 µF +5V bypass |
| C2 | B.Cu | (115.0, 78.0) | 0° | 100 nF near J3 |
| U1 | F.Cu | (95.0, 61.1) | 90° | ESP32-S3-DevKitC-1 |
| J3 | F.Cu | (62.0, 85.2) | 180° | 4-position XH connector |

U1 is the height driver at about 7.2 mm above the PCB. J1 and J2 are about
4.25 mm high. C1 is about 4.3 mm high. Trim header tails flush after fitting
U1.

The four Hall signals use U1 GPIO4, GPIO6, GPIO7, and GPIO10. These are ADC1
inputs and remain usable while WiFi is active. See
[docs/gpio-remap.md](gpio-remap.md).

J2 pin 13 is no-connect by design. It isolates the driver-board 3V3 signal.
R3 and R4 provide the heater pull-downs and must be present.

## Reproduction

The source of truth for placement is the `PLACEMENT` dictionary in
`scripts/place_footprints.py`. The regeneration order is:

```text
mkdir -p build
kicad-cli sch export netlist --format kicadxml -o build/sh03.net \
  sh03-controller.kicad_sch
python3 scripts/place_footprints.py
python3 scripts/route_board.py
python3 scripts/add_ground_pour.py
python3 scripts/add_fiducials.py
```

Run these checks after regeneration:

```text
python3 scripts/check_placement.py
python3 scripts/report_pads.py
python3 scripts/check_determinism.py
```

`check_placement.py` checks courtyard overlap, board-edge clearance, requested
connector-hole clearances, and antenna keepout. `report_pads.py` reports the
connector and U1 pad mapping. `check_determinism.py` runs the pipeline twice
and compares the resulting board files.

The scripts use KiCad 10.0.5's standalone `pcbnew` Python module. They resolve
the project footprint libraries through the local library tables. Run the
pipeline on a disposable copy because the generation scripts overwrite the
board and generated pad reports.

For a Linux host with KiCad 10.0.5 installed, run the commands above from the
project root. To use the KiCad container, check the version and bind-mount the
project directory:

```text
docker run --rm -v "$PWD:/work" -w /work kicad/kicad:10.0 \
  kicad-cli --version
```

The current project was checked with KiCad 10.0.5. The container image may
resolve to a different 10.0 release, so confirm the version before generating
files. The base image does not provide every 3D model package. Install or
mount the KiCad 3D model data separately before rendering.

The fab guide contains the complete `kicad-cli` export commands. The committed
STEP export has no resolved J2 3D model. Treat its J2 visualization as
incomplete and check the physical part dimensions against the footprint.
