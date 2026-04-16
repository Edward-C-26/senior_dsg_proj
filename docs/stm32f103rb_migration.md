# STM32F103RB Migration Guide

This project is currently generated for `STM32F446RE`. The clean way to move it to the `NUCLEO-F103RB` is:

1. Create a fresh `STM32F103RBTx` CubeMX/CubeIDE project.
2. Recreate only the peripherals that the current firmware actually uses.
3. Copy the application-layer source files into the new project.
4. Re-test pin wiring, clocks, UART telemetry, LTC6811 SPI traffic, and ADC current sensing.

Do not try to mechanically convert the existing F4 Cube-generated files. Regenerate the MCU-specific startup, HAL, linker, IRQ, GPIO, clock, and peripheral init files for F1.

## Recommended Target Topology

Recommended bring-up for a flashable NUCLEO-F103RB build:

- `GPIO`
- `SPI1` for the LTC6811 chain
- `USART2` for telemetry over the Nucleo on-board ST-LINK Virtual COM Port
- `ADC1` for current sense
- `SysTick` for software scheduling via `HAL_GetTick()`

Peripherals that can be dropped for the first F103 build:

- `SPI3`
- `TIM13`
- `TIM7`
- `TIM1`
- `TIM2`
- `TIM3`

These are either unused in the current firmware or only partially configured.

## Exact Peripheral Replacements

### Required now

| Current F446 peripheral | Current role | F103RB replacement | Recommendation |
| --- | --- | --- | --- |
| `SPI1` | Active LTC6811 bus | `SPI1` | Keep |
| `USART1` | UART telemetry | `USART2` | Move to Nucleo VCP |
| `ADC1` | Current-sense ADC | `ADC1` | Keep |
| `SysTick` | Main-loop timing | `SysTick` | Keep |

### Remove for first port

| Current F446 peripheral | Current role | F103RB action |
| --- | --- | --- |
| `SPI3` | Intended external ADC SPI bus, currently unused | Remove from first port |
| `TIM13` | Configured but not started or used | Remove |
| `TIM7` | Configured but not started or used | Remove |
| `TIM1` | Configured but not started or used | Remove |
| `TIM2` | Configured for input capture but not used | Remove |
| `TIM3` | Intended external ADC clock output, not started | Remove |

### Future-only replacements if you later re-enable the external ADC path

| Current F446 signal/peripheral | Current pin(s) | F103RB recommendation | Notes |
| --- | --- | --- | --- |
| `SPI3` | `PC10/PC11/PC12` | `SPI2` on `PB13/PB14/PB15` | F103RB has no `SPI3` |
| `SPI_ADC_CS` | `PA15` | `PB12` | Avoids JTAG conflict on `PA15` |
| `ADC_CLKIN` via `TIM3_CH1` | `PC6` | `PC6` | Can stay if you later need timer clock output |
| `ADC_DRDY` | `PC7` | `PC7` | Can stay as GPIO/EXTI |
| `ADC_RST` | `PC8` | `PC8` | Can stay as GPIO |

## Exact Pin Replacement Plan

### Recommended pin plan for the first F103RB build

This plan optimizes for:

- fastest bring-up on Nucleo
- ST-LINK USB serial without external adapters
- minimal peripheral count

| Function | Current F446 pin | F103RB pin | Reason |
| --- | --- | --- | --- |
| `BMS_FLT_EN` | `PC13` | `PC13` | Safe direct carry-over |
| `SHUTDOWN_ACTIVE` | `PC3` | `PC3` | Safe direct carry-over |
| `LV_PWR_MONITOR` | `PC4` | `PC4` | Safe direct carry-over |
| `SPI_UCOMM_CS` | `PA4` | `PA4` | Keep with `SPI1` |
| `SPI_UCOMM_SCK` | `PA5` | `PA5` | `SPI1_SCK`, keep |
| `SPI_UCOMM_MISO` | `PA6` | `PA6` | `SPI1_MISO`, keep |
| `SPI_UCOMM_MOSI` | `PA7` | `PA7` | `SPI1_MOSI`, keep |
| `UART_TX` | `PA9` via `USART1_TX` | `PA2` via `USART2_TX` | Uses Nucleo ST-LINK VCP |
| `UART_RX` | `PA10` via `USART1_RX` | `PA3` via `USART2_RX` | Uses Nucleo ST-LINK VCP |
| Current-sense ADC input | `PA2` / `ADC1_IN2` | `PB0` / `ADC1_IN8` | Frees `PA2` for VCP UART |
| `CHARGE_EN` input | `PA8` | `PA8` | Safe direct carry-over |
| `SWDIO` | `PA13` | `PA13` | Keep reserved for debug |
| `SWCLK` | `PA14` | `PA14` | Keep reserved for debug |

### Optional alternative if you do not want to move the ADC wire

If the analog current-sense wire must stay on `PA2`, then do this instead:

- keep `ADC1_IN2` on `PA2`
- do not use `USART2`
- use either:
  - `USART1` on `PA9/PA10` with an external USB-UART adapter, or
  - `USART3` on `PC10/PC11` with flying wires to the Nucleo ST-LINK header as described in `UM1724`

That option avoids rewiring the ADC input, but it is not the cleanest Nucleo setup.

## Fresh CubeMX Project Recipe

Create a new project targeting `STM32F103RBTx` or `NUCLEO-F103RB`.

Configure these peripherals:

- `RCC`
  - Use `HSE = Bypass Clock Source` if you target the Nucleo board defaults
  - Set SYSCLK to `72 MHz`
- `SYS`
  - Debug = `Serial Wire`
- `SPI1`
  - Mode: `Full-Duplex Master`
  - NSS: `Software`
  - Data size: `8-bit`
  - Polarity/phase: match current LTC6811 settings
  - SCK/MISO/MOSI: `PA5/PA6/PA7`
- `USART2`
  - Asynchronous
  - `115200 8N1`
  - Pins `PA2/PA3`
- `ADC1`
  - Single conversion
  - One channel only
  - Recommended channel: `IN8` on `PB0`
- `GPIO`
  - Add the output and input pins from the table above

Do not enable these in the first F103 build:

- `SPI2`
- `TIM1`
- `TIM2`
- `TIM3`
- `TIM4`
- `TIM6`
- `TIM7`

## Files To Copy Into The New F103 Project

Copy these application files from the current project into the fresh F103 project and then fix includes as needed:

- `Core/Src/LTC6811.c`
- `Core/Src/BMSconfig.c`
- `Core/Src/PackCalculations.c`
- `Core/Src/Fault.c`
- `Core/Src/main.c`
- `Core/Inc/LTC6811.h`
- `Core/Inc/BMSconfig.h`
- `Core/Inc/PackCalculations.h`
- `Core/Inc/Fault.h`

You will also need the relevant shared types and declarations from:

- `Core/Inc/main.h`

## Files You Should Regenerate, Not Copy

Do not copy these F4-generated files into the F103 project:

- `Core/Src/gpio.c`
- `Core/Src/usart.c`
- `Core/Src/adc.c`
- `Core/Src/SPI.c`
- `Core/Src/tim.c`
- `Core/Src/stm32f4xx_it.c`
- `Core/Src/stm32f4xx_hal_msp.c`
- `Core/Src/system_stm32f4xx.c`
- `Core/Startup/startup_stm32f446retx.s`
- `Core/Inc/stm32f4xx_hal_conf.h`
- `Core/Inc/stm32f4xx_it.h`
- all `STM32F4xx_HAL_Driver` files
- all `CMSIS/Device/ST/STM32F4xx` files
- F446 linker scripts

Regenerate the F103 equivalents instead.

## Code Changes Required In `main.c`

The following logic changes are required after you bring in your application code:

1. Replace `USART1` usage with `USART2` if you take the recommended VCP route.
2. Replace the ADC current-sense channel from `PA2` to the new ADC pin.
3. Remove `MX_SPI3_Init()`.
4. Remove `MX_TIM1_Init()`, `MX_TIM2_Init()`, `MX_TIM3_Init()`, `MX_TIM7_Init()`, and `MX_TIM13_Init()` from the first F103 build.
5. Remove `adc_spi = &hspi3;`.
6. Keep `ltc_spi = &hspi1;`.
7. Replace all `stm32f4xx_*` includes with `stm32f1xx_*` includes where still needed.

## Build-to-Flash Checklist

Use this sequence:

1. Generate a fresh F103 Cube project.
2. Confirm it builds before copying any application code.
3. Add only `SPI1`, `USART2`, `ADC1`, and the required GPIO pins.
4. Copy the application files listed above.
5. Fix include names and peripheral handle names.
6. Verify the project still builds.
7. Flash the board.
8. Confirm UART telemetry arrives over the Nucleo ST-LINK VCP.
9. Confirm LTC6811 SPI traffic works.
10. Confirm ADC current-sense reads sane values.

## Board-Specific Notes

- On the NUCLEO-F103RB, `USART2` on `PA2/PA3` is the default path to the on-board ST-LINK Virtual COM Port.
- If you later want those pins for external wiring instead, you need to change the solder bridge configuration on the board.
- Avoid using `PA13` and `PA14` for anything except SWD.
- Avoid using `PA15`, `PB3`, and `PB4` in the first port unless you explicitly reconfigure SWJ/JTAG, because they are tied up by default debug/JTAG functions on F1.

## Recommended First Flash Goal

The first F103 flash should prove only these three things:

- SPI communication with the LTC6811 chain works on `SPI1`
- telemetry packets go out over UART
- current-sense ADC reads correctly

Do not try to preserve every configured F446 peripheral in the first flashable port. Bring up the minimum working system first, then add anything else back only if it is proven necessary.

## Sources

- ST `STM32F103RB` datasheet: https://www.st.com/resource/en/datasheet/stm32f103rb.pdf
- ST `UM1724` Nucleo-64 user manual: https://www.st.com/resource/en/user_manual/DM00105823.pdf
