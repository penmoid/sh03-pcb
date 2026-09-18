# Fabrication package

This directory contains the Rev-A order package generated from
`sh03-controller.kicad_pcb` with KiCad 10.0.5:

| File | Contents |
|---|---|
| `sh03-controller-gerbers.zip` | Gerbers, Excellon drill, and job file |
| `sh03-controller-cpl.csv` | Nine back-side SMD placements |
| `sh03-controller-bom.csv` | BOM covering nine SMD references |
| `sh03-controller.step` | Board and available 3D models |

The committed archive is the first-order package and remains unchanged for
reference. It uses a merged drill file. The fabrication house finished the
four mounting holes as plated 3.2 mm holes on that order. A future export should use
`--excellon-separate-th` and must be reviewed as a new package because that
flag changes the drill outputs.

## Assembly

All nine SMD references are on B.Cu: J1, J2, R1-R4, and C1-C3. J3 and U1's
2×22, 2.54 mm header arrangement are through-hole work. Direct soldering U1
is also supported.

Before ordering, confirm the JLCPCB Assembly Viewer orientation:

- J2, the 15-position vents connector, is in the north slot.
- J1, the 14-position thermal and air connector, is in the south slot.
- Both connectors are back-side placements with CPL rotation 270°.
- C1's positive stripe is on the +5V side.

The first order required a +180° correction in the Assembly Viewer for J1 and
J2. The correction was made in the viewer and was not silently applied to the
CPL. Review the native 270° CPL values and make any assembly-house correction
explicit during order review.

The committed STEP export does not contain a resolved J2 3D model. Use the
footprint and datasheet dimensions for J2 mechanical checks.

## Gerbers and drill

Run these commands from the project root. Create output directories first.

```text
mkdir -p build/gerbers

kicad-cli pcb export gerbers -o build/gerbers \
  -l F.Cu,B.Cu,F.SilkS,B.SilkS,F.Mask,B.Mask,F.Paste,B.Paste,Edge.Cuts \
  --no-x2 --no-netlist sh03-controller.kicad_pcb

kicad-cli pcb export drill -o build/gerbers/ --format excellon \
  --drill-origin absolute -u mm --excellon-zeros-format decimal \
  --generate-map --map-format gerberx2 sh03-controller.kicad_pcb
```

The merged drill export may cause the fabrication CAM process to treat NPTH
mounting holes as plated. Choose either the merged drill set or a separate
PTH/NPTH drill set for an upload. Never include both drill sets in one upload.
Use this command for a future separate export, then inspect the replacement
package:

```text
mkdir -p build/drill-separated

kicad-cli pcb export drill -o build/drill-separated/ --format excellon \
  --drill-origin absolute -u mm --excellon-zeros-format decimal \
  --excellon-separate-th --generate-map --map-format gerberx2 \
  sh03-controller.kicad_pcb
```

Generate a drill report when reviewing a package:

```text
kicad-cli pcb export drill -o build/gerbers/ --format excellon \
  --drill-origin absolute -u mm --excellon-zeros-format decimal \
  --generate-report --report-path build/gerbers/drill-report.txt \
  sh03-controller.kicad_pcb
```

The Gerber and CPL coordinate origin is the board file origin. Gerber and CPL
Y coordinates use the opposite sign from KiCad page coordinates.

## CPL

Export the placement list with:

```text
kicad-cli pcb export pos -o build/gerbers/sh03-controller-back-pos.csv \
  --side back --format csv --units mm sh03-controller.kicad_pcb
```

Filter the output to `J1`, `J2`, `R1`, `R2`, `R3`, `R4`, `C1`, `C2`, and `C3`.
Use the JLCPCB headers `Designator,Mid X,Mid Y,Layer,Rotation`, normalize
negative rotations into the 0 to 360 range, and keep all nine rows on the
bottom layer. Do not apply an undocumented connector rotation offset.

## SVG and 3D exports

These commands use the project root as the working directory. Run them with
KiCad 10.0.5 and the KiCad 3D model data available to the renderer.

```text
mkdir -p build/svg renders

kicad-cli pcb export svg -o build/svg/top-2d.svg \
  --layers F.Cu,F.SilkS,F.Mask,F.Paste,Edge.Cuts --page-size-mode 2 \
  --exclude-drawing-sheet --mode-single sh03-controller.kicad_pcb

kicad-cli pcb export svg -o build/svg/bottom-2d.svg \
  --layers B.Cu,B.SilkS,B.Mask,B.Paste,Edge.Cuts --page-size-mode 2 \
  --exclude-drawing-sheet --mode-single --mirror sh03-controller.kicad_pcb

kicad-cli pcb render -o renders/sh03-controller-top.png --side top \
  --width 1568 --height 984 --quality high --floor sh03-controller.kicad_pcb

kicad-cli pcb render -o renders/sh03-controller-bottom.png --side bottom \
  --width 1568 --height 984 --quality high --floor sh03-controller.kicad_pcb

kicad-cli pcb render -o renders/sh03-controller-angled.png --side bottom \
  --width 1568 --height 984 --quality high --perspective \
  --rotate 20,0,-25 --zoom 1.0 --background opaque sh03-controller.kicad_pcb

kicad-cli pcb render -o renders/sh03-controller-top-2d.png --side top \
  --width 1568 --height 984 --quality basic sh03-controller.kicad_pcb

kicad-cli pcb render -o renders/sh03-controller-bottom-2d.png --side bottom \
  --width 1568 --height 984 --quality basic sh03-controller.kicad_pcb
```

Export the schematic PDF separately:

```text
kicad-cli sch export pdf -o renders/sh03-controller-schematic.pdf \
  sh03-controller.kicad_sch
```

## Validation

The tracked reports record KiCad DRC and ERC runs. See
[`reports/README.md`](../reports/README.md). DRC reports 0 errors and 0
unconnected items with four mounting-hole library warnings. ERC reports one
U1 library-symbol warning. The 44 U1 pins are equivalent; the warning concerns
symbol display and metadata.

The project author has personally validated correct operation of the assembled
Rev-A board in the dryer.

Before releasing an order, inspect the Gerber viewer, drill map, BOM/CPL
reference alignment, C1 polarity, J1/J2 placement, and the J2 no-connect pin.
