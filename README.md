# BMS Viewer (Python GUI)

A real-time Battery Management System (BMS) visualization tool designed for a **6S Li-ion battery pack** using the **LTC6811** and an STM32-based firmware backend.

This viewer provides **live monitoring, fault visualization, and analytics** for battery operation during discharge.

---

## Overview

The BMS Viewer connects to the STM32 firmware via **UART (115200 baud)** and displays:

* Individual cell voltages (6 cells)
* Cell temperatures (mapped from 2 thermistors)
* Pack voltage
* Fault conditions (OV, UV, OT, UT)
* System status flags
* Optional SOC/SOH estimation (viewer-side)

The firmware transmits structured telemetry packets at ~5 Hz–10 Hz using a custom binary protocol.

---

## System Architecture

```
[LTC6811] → [STM32 Firmware] → UART → [Python Viewer]
```

### Embedded Side Responsibilities

* Voltage + temperature acquisition via SPI
* Fault detection & protection
* Balancing decisions
* UART telemetry transmission

### Viewer Responsibilities

* Packet decoding + validation (CRC)
* Visualization (UI)
* Logging (optional)
* SOC / SOH estimation (optional)

---

## UART Telemetry Format

The firmware sends a packed struct:

```c
typedef struct
{
  uint8_t sof1;            // 0xAA
  uint8_t sof2;            // 0x55
  uint8_t payload_len;

  uint32_t timestamp_ms;

  uint16_t cell_voltage_mV[6];
  int16_t  cell_temp_cC[6];

  uint16_t pack_voltage_mV;
  uint8_t  status_bytes[6];

  uint16_t crc;
} BmsUartPacket_t;
```

### Key Notes

* **Voltage conversion**: raw LTC6811 → mV (`/10`)
* **Temperature mapping**:

  * Thermistor 1 → Cells 1–3
  * Thermistor 2 → Cells 4–6
* **CRC**: CRC16-CCITT for packet integrity

---

## 🎨 Viewer UI Design

### Layout Overview

```
+--------------------------------------------------+
|                 PACK SUMMARY                     |
| Voltage | Current | SOC | SOH | Status          |
+--------------------------------------------------+

+---------------- CELL MONITOR --------------------+
| Cell 1 | Cell 2 | Cell 3 | Cell 4 | Cell 5 | Cell 6 |
|  bar   |  bar   |  bar   |  bar   |  bar   |  bar   |
|  mV    |  mV    |  mV    |  mV    |  mV    |  mV    |
|  °C    |  °C    |  °C    |  °C    |  °C    |  °C    |
+--------------------------------------------------+

+---------------- FAULT PANEL ---------------------+
| OV | UV | OT | UT | Imbalance | Communication   |
+--------------------------------------------------+

+---------------- LOGGING / PLOTS -----------------+
| Voltage trends | Temperature trends | Current    |
+--------------------------------------------------+
```

---

## Data Visualization Strategy

### Cell Voltages

* Vertical bar graph per cell
* Real-time updates (~5 Hz)
* Shows:

  * Voltage value (mV)
  * Relative balance across cells

### Temperature Display

* Same temperature repeated across grouped cells:

  * Cells 1–3 → Thermistor 1
  * Cells 4–6 → Thermistor 2
* Displayed in °C

---

## Color Coding (Critical for Demo)

| Condition               | Color     |
| ----------------------- | --------- |
| Normal                  | 🟢 Green  |
| Warning (within ~5–10%) | 🟡 Yellow |
| Fault                   | 🔴 Red    |

### Voltage Thresholds

* Overvoltage: **> 4.20 V**
* Undervoltage: **< 3.00 V**

### Temperature Thresholds

* Overtemp: **> 60°C**
* Undertemp: **< 0°C**

---

## Fault Visualization

Derived from `status_bytes[6]`

### Bit Mapping (example interpretation)

| Fault             | Bit              |
| ----------------- | ---------------- |
| Overvoltage (OV)  | status[0] & 0x01 |
| Undervoltage (UV) | status[0] & 0x02 |
| Overtemp (OT)     | status[0] & 0x04 |
| Undertemp (UT)    | status[0] & 0x08 |

### UI Behavior

* Fault lights turn **red**
* Affected cell highlighted
* Optional alert popup/log entry

---

## Advanced Features 

### 1. SOC Estimation (Viewer Side)

* **Coulomb Counting**

  * Integrate current over time
* Optional fallback:

  * OCV lookup table

### 2. SOH Estimation

* Based on:

  * Voltage sag under load
  * Capacity fade (logged discharge cycles)

### 3. Data Logging

* Save to CSV:

  ```
  timestamp, cell1, cell2, ..., temp1, ..., pack_voltage
  ```
* Enables:

  * SOC refinement
  * SOH tracking

### 4. Trend Graphs

* Live matplotlib plots:

  * Voltage vs time
  * Temperature vs time
  * Current vs time

---

## Update Rates

From firmware:

* Voltage polling: **200 ms**
* Temperature polling: **500 ms**
* UART transmission: **200 ms**

Viewer should:

* Read continuously
* Refresh UI at ~5 Hz

---

## Safety Logic Integration

The viewer reflects embedded logic:

* Balancing only when:

  * No faults
  * Imbalance > threshold (~150 counts)

* Invalid data flagged via PEC/CRC mismatch

---

## Tech Stack (Recommended)

* Python 3.10+
* UI:

  * `PyQt5` or `Tkinter`
* Serial:

  * `pyserial`
* Visualization:

  * `matplotlib` / `pyqtgraph`

---

## Key Design Decisions

* UART chosen over CAN for simplicity
* Viewer handles analytics (SOC/SOH)
* Firmware focuses on safety-critical logic
* Temperature mapped across grouped cells (hardware constraint)
* CRC + PEC ensures robust communication

---

## Future Improvements

* Implement battery health dashboard
* Add fault history timeline
* Add pack power calculation (V × I)
* Add remote monitoring (WiFi/Bluetooth)

---

## Summary

This viewer is not just a display — it is a **diagnostic and analytics interface** for your BMS:

* Real-time safety monitoring
* Clear visual feedback (color-coded)
* Expandable for SOC/SOH
* Built directly on your firmware architecture
