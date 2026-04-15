# ADC Shunt Current Measurement Setup

## Overview
This document describes the ADC1 shunt current measurement configuration at **200Hz** for the BMS system on the STM32F446.

## Hardware Configuration

### Pin Configuration
- **ADC Input**: ADC1_IN1 (PA1)
- **Shunt Resistor**: 10 mOhm (0.01 Ω)
- **ADC Resolution**: 12-bit
- **ADC Reference Voltage**: 3.3V

### Shunt Measurement Equation
```
Shunt Voltage (V) = ADC_RAW * 3.3V / 4095
Current (A) = Shunt_Voltage / Shunt_Resistor
Current (mA) = Current (A) * 1000
```

### Example
- ADC reads: 2048 (mid-scale)
- Shunt Voltage: 2048 * 3.3 / 4095 = 1.65V
- Current: 1.65V / 0.01Ω = 165A

## Software Configuration

### Files Modified
1. **Core/Src/adc.c** - Updated GPIO and channel configuration
   - Changed from PA2 (ADC_CHANNEL_2) to PA1 (ADC_CHANNEL_1)
   - Updated sampling time to 15 cycles for better stability

2. **Core/Inc/adc.h** - Added ADC configuration constants
   - Shunt resistor value: 0.01Ω
   - Channel and GPIO pin definitions

3. **Core/Src/main.c** - Implemented ADC reading and integration
   - `read_adc_shunt_current()` function reads ADC and converts to current (Amps)
   - Called at 200Hz in main loop (5ms period)
   - Result stored in `BMSCriticalInfo.packCurrent`
   - Transmitted via UART in telemetry packet

## Reading Frequency

The ADC is read at **200Hz** (every 5ms):
- Timing controlled by `VOLTAGE_POLL_PERIOD_MS` (200U)
  - ADC read occurs when UART telemetry transmits
  - Located in main while loop

## Data Output

### UART Telemetry Packet
The shunt current is included in the BMS telemetry packet:
```c
typedef struct {
    uint8_t sof1;                 // 0xAA
    uint8_t sof2;                 // 0x55
    uint8_t length;               // 37 bytes
    uint32_t timestamp_ms;
    uint16_t cell_voltage_mV[6];
    int16_t cell_temp_cC[6];
    uint16_t pack_voltage_mV;
    int16_t pack_current_deciA;   // Current in deci-Amps (0.1A units)
    uint8_t status[6];
    uint16_t crc;
} BmsUartPacket_t;
```

**pack_current_deciA** = packCurrent * 10
- If `packCurrent = 5.0A`, then `pack_current_deciA = 50`

## Calibration Notes

For accurate current measurement with a different shunt resistor:
1. Update `SHUNT_RESISTOR` constant in `Core/Inc/adc.h`
   ```c
   #define SHUNT_RESISTOR_OHMS  0.005f   // For 5 mOhm shunt
   ```

2. Update the same value in `Core/Src/main.c`:
   ```c
   static const float SHUNT_RESISTOR = 0.005f;  // In ohms
   ```

3. Expected ADC voltage range:
   - For ±200A with 10mΩ shunt: ±2V
   - Fits within 0-3.3V ADC range with margin

## Testing

### Debug Output
Global variables for monitoring:
- `adc_shunt_reading_mv` - Last shunt voltage in mV
- `adc_shunt_current_ma` - Last current reading in mA
- `BMSCriticalInfo.packCurrent` - Current in Amps

### Quick Test
1. Set a breakpoint in `read_adc_shunt_current()`
2. Monitor `adc_raw` value while applying different loads
3. Verify linear relationship with external ammeter

## Timing Diagram
```
Main Loop (continuous)
├─ Every 200ms (0.2s):
│  ├─ Read ADC1 shunt @ PA1
│  ├─ Convert to current
│  ├─ Store in BMSCriticalInfo.packCurrent
│  └─ Send UART telemetry
└─ Repeat
```

## Constants Reference

| Constant | Value | Unit | Location |
|----------|-------|------|----------|
| SHUNT_RESISTOR | 0.01 | Ω | main.c, adc.h |
| ADC_REF_VOLTAGE | 3.3 | V | main.c |
| ADC_MAX_VALUE | 4095 | counts | main.c |
| Sampling Time | 15 | ADC cycles | adc.c |
| Read Frequency | 200 | Hz | main.h |
| ADC Channel | 1 (PA1) | - | adc.c, adc.h |

## System Block Diagram

```
Shunt Resistor (0.01Ω)
        │
        ├─→ Voltage across shunt (0-2V typical)
        │
        └─→ PA1 (ADC1_IN1)
                │
                └─→ ADC1 Peripheral
                    │
                    └─→ HAL_ADC_GetValue()
                        │
                        └─→ read_adc_shunt_current()
                            │
                            ├─→ Convert: I = V/R
                            └─→ Store in packCurrent
                                │
                                └─→ UART Telemetry
```
