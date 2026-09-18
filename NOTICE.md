# Licensing and attribution

Copyright 2026 penmoid.

Source location: <https://github.com/penmoid/sh03-pcb>.

The original hardware design, documentation, and generated design outputs use
[CERN-OHL-P-2.0](LICENSE), subject to the third-party notices below. The helper
scripts in `scripts/` use the [MIT license](LICENSES/MIT.txt).

The design is provided without warranty under the applicable license.

## KiCad library material

KiCad library contributors provide the standard symbols, footprints, and 3D
models used in the design. Their library material uses
[CC-BY-SA-4.0](LICENSES/CC-BY-SA-4.0.txt) with the
[KiCad library exception](LICENSES/KiCad-Libraries.md).

The two `sh03.pretty/JST_GH_BM*_UNIVERSAL_*.kicad_mod` library files adapt
KiCad's JST GH footprints. They retain the KiCad library license. The changes
combine the JST and JUSHUO pad envelopes and rename the footprints. These
adaptations were made in August 2026.

Upstream libraries:

- [KiCad footprints](https://gitlab.com/kicad/libraries/kicad-footprints)
- [KiCad symbols](https://gitlab.com/kicad/libraries/kicad-symbols)
- [KiCad 3D packages](https://gitlab.com/kicad/libraries/kicad-packages3D)

The project-specific ESP32-S3-DevKitC-1 socket symbol and footprint are included
with the original design. Manufacturer names and part numbers identify compatible
components. This project is an independent controller design for the Sovol SH03.
