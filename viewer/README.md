# Python BMS Viewer

UART desktop viewer for the STM32 + LTC6811 senior design BMS.

## Features

- 6-cell live voltage cards with warning/fault coloring
- Temperature display using the repo's 2-thermistor to 6-cell mapping
- Pack summary for voltage, current, SOC, SOH, and imbalance
- Fault panel for OV, UV, OT, UT, imbalance, and communication health
- Embedded live plots for voltage, temperature, and current
- CSV logging to `viewer/logs/`
- Demo mode when the STM32 is not connected

## Packet Support

- Current firmware packet from `bms_2025/Core/Src/main.c`

## Run

```bash
python3 -m pip install -r viewer/requirements.txt
python3 viewer/bms_viewer.py
```

Demo mode:

```bash
python3 viewer/bms_viewer.py --demo
```

For the April 26 balancing demo, set `ENABLE_POST_BALANCE_SIMULATION` in
`bms_2025/Core/Src/main.c`: `0` streams pre-balancing data and `1` streams
post-balancing data.

UART simulation:

```bash
python3 viewer/uart_sim.py --port /dev/tty.usbmodemXXXX
```
