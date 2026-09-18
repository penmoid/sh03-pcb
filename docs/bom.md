# Bill of materials

This file records the parts used by the Rev-A board and the alternatives that
must be checked before an order. Values and part numbers are design data.
Distributor stock, pricing, and assembly-library status change over time.

## SMD parts

| Ref | Value | Part | LCSC | Library |
|---|---|---|---|---|
| C1 | 470 µF, 10 V, MnO2 tantalum, EIA 7343-43 | Kyocera AVX TPSE477K010R0100 | `C444831` | Extended |
| R1, R2 | 33k, 0805, 1% | Uniroyal Elec 0805W8F3302T5E | `C17633` | Basic |
| R3, R4 | 10k, 0805, 1% | Uniroyal Elec 0805W8F1002T5E | `C17414` | Basic |
| C2 | 100nF, 50V, X7R, 0805 | YAGEO CC0805KRX7R9BB104 | `C49678` | Basic |
| C3 | 10uF, 25V, X5R, 0805 | Samsung Electro-Mechanics CL21A106KAYNNNE | `C15850` | Basic |

C1 is the 5 V rail bulk capacitor. The design uses a 10 V rating and a 50%
voltage derating target. The estimated 1 ms, 300 mA transient produces about
640 mV of capacitive droop at 470 µF and about 30 mV of ESR step at 100 mΩ.
The earlier candidate was KEMET T530X477M010A*E020. It is not used in Rev-A.

All nine SMD references are on B.Cu for single-side assembly: J1, J2, R1-R4,
and C1-C3. C1 is polarized. Confirm its positive marking in the assembly
viewer and on the assembled board.

## Connectors and through-hole parts

| Ref | Part | Notes |
|---|---|---|
| J1 | BM14B-GHS-TBT, or GH125-S14CCA-00 (`C2886792`) | 14-position GH connector |
| J2 | BM15B-GHS-TBT, or GH125-S15CCA-00 (`C2886793`) | 15-position GH connector |
| J3 | JST XH 4-position right-angle | Through-hole, fitted by the builder |
| U1 | ESP32-S3-DevKitC-1 | Suitable 2×22, 2.54 mm headers/socket arrangement, or direct solder |

J1's genuine JST part is BM14B-GHS-TBT (`C265384`). J2's genuine JST part is
BM15B-GHS-TBT. The JUSHUO GH125 parts are JLCPCB Extended-library options.
The custom footprints cover the pad envelopes for the genuine and clone parts.
The clone-to-genuine-housing mate was unverified when the order package was
prepared. A genuine JST connector has a reported fit with the genuine JST
housings. That report does not prove clone compatibility. Test each clone and
housing combination before an assembly run.

The cable-side housings are GHR-14V-S (`C566400`) and GHR-15V-S (`C594572`).
The GH crimp contact is SSHL-002T-P0.2. The cut-tape part is
`MINI-SSHL-002T-P0.2`. `SXH-001T-P0.6` is an XH-series contact and is not a
GH substitute.

A 14- or 15-position cable requires a matching cable assembly, loose contacts
and a housing, or a re-terminated OEM cable. The pre-crimped GH-to-GH
AGHGH28K152 assembly is an option to evaluate. Confirm its pin-to-pin
continuity before use.

## Electrical constraints

R3 and R4 are mandatory 10k heater pull-downs. Do not power the dryer through
this board with either resistor missing.

J2 pin 13 is intentionally no-connect. The driver-board 3V3 signal is isolated
on this revision. Do not populate or wire it as a 3V3 output.

The connector choice, cable construction, housing mate, and order-time
assembly-library availability remain order checks. This document does not
claim that any distributor inventory is current.
