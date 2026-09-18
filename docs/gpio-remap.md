# GPIO map

This table is the current PCB map. It includes the Hall ADC1 correction. Use
it for firmware written for the PCB.

| Signal | PCB GPIO | ADC or function |
|---|---:|---|
| TH2 | 1 | ADC1 |
| TH1 | 2 | ADC1 |
| MC_HOT2 | 11 | Digital |
| MC_HOT1 | 12 | Digital |
| MC_K_FAN3_FG | 38 | Tacho input |
| MC_K_FAN3 | 21 | Digital |
| MC_K_FAN2_FG | 48 | Tacho input |
| MC_K_FAN2 | 14 | Digital |
| MC_K_FAN1_FG | 47 | Tacho input |
| MC_K_FAN1 | 13 | Digital |
| MC_HUM_SDA | 8 | I2C |
| MC_HUM_SCL | 9 | I2C |
| MC_HALLA | 4 | ADC1_CH3 |
| MC_HALLB | 10 | ADC1_CH9 |
| MC_HALLC | 7 | ADC1_CH6 |
| MC_HALLD | 6 | ADC1_CH5 |
| MC_XQA1 | 41 | Digital |
| MC_XQA2 | 40 | Digital |
| MC_XQB1 | 3 | Digital, strapping pin |
| MC_XQB2 | 18 | Digital |
| MC_XQC1 | 15 | Digital |
| MC_XQC2 | 17 | Digital |
| MC_XQD1 | 5 | Digital |
| MC_XQD2 | 42 | Digital |
| MCU_TX | 16 | UART |
| MCU_RX | 39 | UART |

TH2 uses ADC1 GPIO1. TH1 uses ADC1 GPIO2. The UART is GPIO16/GPIO39. USB and
UART0 GPIO43/GPIO44 are unused by the board. The Hall sensors are ratiometric
analog inputs, so all four must stay on ADC1 GPIO1 through GPIO10 when WiFi is
active. GPIO11 through GPIO20 use ADC2 and are unsuitable for these inputs in
that condition. GPIO42 has no ADC channel.

## Perfboard to PCB changes

The current PCB changes these ten signal assignments from the earlier
perfboard wiring:

| Signal | Perfboard GPIO | PCB GPIO |
|---|---:|---:|
| MC_HALLB | 5 | 10 |
| MC_HALLC | 6 | 7 |
| MC_HALLD | 7 | 6 |
| MC_XQA1 | 17 | 41 |
| MC_XQA2 | 15 | 40 |
| MC_XQB1 | 10 | 3 |
| MC_XQC1 | 3 | 15 |
| MC_XQC2 | 40 | 17 |
| MC_XQD1 | 42 | 5 |
| MC_XQD2 | 41 | 42 |

MC_HALLA remains on GPIO4. MC_XQB2 remains on GPIO18. The TH, HOT, fan,
tacho, I2C, and UART assignments are listed in the authoritative table above.

## Firmware cutover

The repository contains no firmware. Keep the existing device on its old map
until the PCB has passed the bring-up checks and is ready for cutover. Apply
the current table only to firmware running with this PCB. A firmware update on
the earlier wiring changes the vent and Hall signals.
