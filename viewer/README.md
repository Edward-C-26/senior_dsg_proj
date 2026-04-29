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

For STM UART testing, set `ENABLE_SENSOR_SIMULATION` in
`bms_2025/Core/Src/main.c`: `1` streams simulated voltage/temperature data,
and `0` uses the LTC6811 read path. With the LTC disconnected, the real read
path reports 0 V cells and the viewer shows undervoltage faults.

UART simulation:

```bash
python3 viewer/uart_sim.py --port /dev/tty.usbmodemXXXX
```
