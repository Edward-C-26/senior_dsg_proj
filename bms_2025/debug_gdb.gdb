# GDB script to connect and debug STM32F446 BMS firmware in QEMU

# Connect to QEMU's GDB server on localhost:3333
target remote :3333

# Show connection status
monitor version

# Load the program into QEMU's simulated memory
load

# Set a breakpoint at simulate_bms_data function
break simulate_bms_data

# Print debugging info
printf "\n===== Connected to QEMU STM32 Emulator =====\n"
printf "Firmware loaded and ready to debug\n"
printf "Breakpoint set at simulate_bms_data\n"
printf "Continuing execution...\n"
printf "========================================\n\n"

# Start execution
continue
