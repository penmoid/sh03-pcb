# Bring-up status and checks

The project author has personally validated correct operation of the assembled
Rev-A board in the dryer. The repository contains CAD validation, fabrication
outputs, and continuity-check guidance. Complete these checks before
connecting each new board to a dryer.

## Safety rules

- Never connect USB and the dryer harness at the same time. The DevKitC-1 can
  feed USB power back into the dryer harness.
- Fit J1 to the PCB before powering the dryer. R3 and R4 are mandatory 10k
  pull-downs for the heater lines.
- Keep J2 pin 13 unconnected. It isolates the driver-board 3V3 signal.
- Unplug the dryer, allow the low-voltage rail to discharge, and verify 0 V at
  the +5V rail before changing cables, headers, or the DevKitC-1.

## Before power

1. Inspect every J1/J2 solder joint for bridges, missing parts, and lifted
   leads. Confirm that C1's positive stripe is on the +5V net.
2. Confirm the board is 100 × 56 mm and that the mounting holes accept M3
   hardware. Rev-A mounting holes are plated with a 3.2 mm finished diameter.
3. Confirm J2 is the north connector and J1 is the south connector. Pin 1 is
   at the north end of each connector.
4. Check cable continuity end to end. Require pin 1 to pin 1 through pin N to
   pin N. Reject reversed or cross-wired cables.
5. Check that every cable contact is retained in its housing. Test the
   connector mate before applying force to a clone or genuine JST part.

## Unpowered board checks

Use a DMM with the DevKitC-1 removed.

- Check for no sustained short between +5V and GND. A brief capacitor charge
  response is expected.
- Check +5V continuity between J1.13, J3.1, U1.21, C1 positive, C2.1, and
  C3.1.
- Check GND continuity between J1.14, J2.14, J2.15, J3.2, U1.22, U1.23,
  U1.43, and U1.44.
- Measure about 10k from J1.3 and J1.4 to GND through R3 and R4. An open
  reading means the heater pull-down is missing.
- Check J2.13 for isolation from adjacent pins, +5V, and GND.
- Check every connected connector-to-U1 path against
  [`pad-report.txt`](pad-report.txt) and the GPIO table in
  [`gpio-remap.md`](gpio-remap.md).

## Assembly checks

U1 may use suitable 2×22, 2.54 mm headers or sockets. Direct solder is also a
supported assembly method and is the current build arrangement. Fit the
DevKitC-1 only after checking header alignment and rechecking the +5V/GND
neighbours around U1.21 and U1.22.

## Power-up boundary

Do the first powered test from USB with the dryer harness disconnected. Check
for stable 5 V and 3.3 V rails, 0 V on both heater outputs at boot, and the
expected GPIO map from `docs/gpio-remap.md`. Use a separate bench firmware
configuration if firmware testing is needed. This repository supplies no
firmware.

After USB testing, unplug USB and discharge the board before connecting the
dryer. Keep the dryer disconnected until cable order, J2.13 isolation, C1
polarity, and both 10k heater pull-downs have passed.

The production cutover requires a firmware build using the current PCB map.
Do not apply that map to the earlier perfboard wiring.
