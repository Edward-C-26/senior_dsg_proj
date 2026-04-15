# STM32F446 BMS Firmware Debugging with QEMU + GDB

This guide walks you through debugging the BMS firmware using QEMU emulation and GDB.

## Prerequisites

Make sure you have the ARM toolchain installed:

```bash
# Check if arm-none-eabi-gdb is installed
which arm-none-eabi-gdb
```

If not installed via Homebrew:
```bash
brew install arm-none-eabi-binutils
```

## Quick Start

### Step 1: Terminal 1 - Start QEMU with GDB Server

```bash
cd bms_2025
chmod +x run_qemu_gdb.sh
./run_qemu_gdb.sh
```

You should see:
```
======================================
STM32F446 BMS Emulation with QEMU+GDB
======================================
...
QEMU started in debug mode...
Connect with: arm-none-eabi-gdb Debug/bms_2025.elf
Then in GDB: target remote :3333
======================================
```

This starts QEMU listening for GDB connections on port 3333.
**Leave this terminal running.**

### Step 2: Terminal 2 - Connect with GDB

Open a new terminal and run:

```bash
cd bms_2025
arm-none-eabi-gdb Debug/bms_2025.elf
```

You should see GDB prompt:
```
(gdb) 
```

### Step 3: Connect to QEMU

At the GDB prompt, enter:

```gdb
target remote :3333
load
continue
```

Or automatically with the script:

```bash
arm-none-eabi-gdb -x debug_gdb.gdb Debug/bms_2025.elf
```

## Useful GDB Commands

Once connected, you can use these debugging commands:

```gdb
# Execution control
continue              # Run the firmware
step                  # Step one line of code
next                  # Step over function calls
finish                # Run until function returns
break main            # Set breakpoint at main
break line_number     # Set breakpoint at line
clear BREAKPOINT_NUM  # Remove breakpoint

# Inspection
print variable_name   # Print variable value
print &variable_name  # Print variable address
info breakpoints      # List all breakpoints
backtrace             # Show call stack
info registers        # Show CPU registers
info threads          # Show active threads

# Memory/watchpoints
watch variable_name   # Break when variable changes
display variable_name # Show variable every step
x 0x08000000          # Examine memory at address

# QEMU-specific
monitor help          # Show QEMU monitor commands
monitor info registers # Show QEMU registers
quit                  # Exit GDB (stops QEMU)
```

## Example Debugging Session

```bash
# Terminal 1: Start QEMU
./run_qemu_gdb.sh

# Terminal 2: Start GDB session
arm-none-eabi-gdb Debug/bms_2025.elf

(gdb) target remote :3333
(gdb) load
(gdb) break main
(gdb) continue
# Firmware runs until main()

(gdb) break uart_send_bms_telemetry
(gdb) continue
# Firmware pauses at telemetry function

(gdb) print simulation_tick
$1 = 0
(gdb) print BMSCriticalInfo
$2 = {packCurrent = 0, isoAdcPackVoltage = 0, ...}

(gdb) step
# Step through telemetry function line by line

(gdb) continue
# Resume execution
```

## Testing the Telemetry

While GDB is running the firmware:

1. **Monitor UART output** (in Terminal 1):
   - You should see binary telemetry packets (if UART redirection works)
   - The packets are 43 bytes with format: `0xAA 0x55 ...`

2. **Set breakpoints** to inspect telemetry before transmission:
   ```gdb
   break 362  # Line where uart_send_bms_telemetry is called
   ```

3. **Inspect packet data**:
   ```gdb
   print packet_data[0]@43  # Print first 43 bytes of packet
   ```

## Limitations

QEMU's STM32H405 board emulation:
- ✅ Core CPU execution works
- ✅ Memory and register access
- ✅ Breakpoints and watchpoints
- ⚠️ Limited UART simulation (may not output correctly)
- ⚠️ SPI/I2C peripheral support is minimal
- ⚠️ ADC conversions are partially stubbed

For real hardware testing, flash the firmware to your actual STM32F446 board.

## Troubleshooting

### GDB won't connect
```bash
# Check if QEMU is still running
ps aux | grep qemu
# If not, restart QEMU in Terminal 1
```

### Port already in use
```bash
# Check what's using port 3333
lsof -i :3333
# Kill the process if needed
kill -9 <PID>
```

### Firmware crashes in emulation
- QEMU firmware emulation is not cycle-accurate
- Some I/O or timing-dependent code may not work
- Use real hardware for final validation

## Exit

To stop debugging:
```gdb
(gdb) quit
```

This will terminate GDB and QEMU.
