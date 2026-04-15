#!/bin/bash

# QEMU emulation script for STM32F446 BMS firmware
# This uses the Olimex STM32-H405 board (similar Cortex-M4 architecture)

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ELF_FILE="$SCRIPT_DIR/Debug/bms_2025.elf"

echo "======================================"
echo "STM32F446 BMS Emulation with QEMU"
echo "======================================"
echo ""
echo "Firmware: $ELF_FILE"
echo "Board: Olimex STM32-H405 (Cortex-M4)"
echo ""
echo "Running QEMU..."
echo "Serial output will appear below:"
echo "======================================"
echo ""

# Run QEMU with:
# -M olimex-stm32-h405: Use STM32-compatible board
# -cpu cortex-m4: Use Cortex-M4 CPU
# -kernel: Load the ELF firmware
# -serial stdio: Connect serial port to stdout for telemetry viewing
# -nographic: Run without GUI

qemu-system-arm \
  -M olimex-stm32-h405 \
  -cpu cortex-m4 \
  -kernel "$ELF_FILE" \
  -serial stdio \
  -nographic \
  -monitor none

echo ""
echo "======================================"
echo "Emulation ended"
echo "======================================"
