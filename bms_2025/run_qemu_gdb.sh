#!/bin/bash

# QEMU emulation script with GDB server support for STM32F446 BMS firmware
# This allows debugging the firmware in real-time using GDB

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ELF_FILE="$SCRIPT_DIR/Debug/bms_2025.elf"

echo "======================================"
echo "STM32F446 BMS Emulation with QEMU+GDB"
echo "======================================"
echo ""
echo "Firmware: $ELF_FILE"
echo "Board: Olimex STM32-H405 (Cortex-M4)"
echo "GDB Server: tcp::3333"
echo ""
echo "QEMU started in debug mode..."
echo "Connect with: arm-none-eabi-gdb $ELF_FILE"
echo "Then in GDB: target remote :3333"
echo "======================================"
echo ""

# Run QEMU with GDB server support:
# -gdb tcp::3333: Start GDB server on port 3333
# -S: Start paused (waits for GDB to connect)
# -serial stdio: Connect serial port to stdout for telemetry viewing
# -nographic: Run without GUI

qemu-system-arm \
  -M olimex-stm32-h405 \
  -cpu cortex-m4 \
  -kernel "$ELF_FILE" \
  -serial stdio \
  -serial pty \
  -nographic \
  -gdb tcp::3333 \
  -S \
  -monitor none

echo ""
echo "======================================"
echo "QEMU emulation ended"
echo "======================================"
