#!/usr/bin/env python3
from __future__ import annotations

import argparse
import time

from telemetry import BatteryTelemetrySimulator


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Transmit simulated BMS UART packets for the viewer")
    parser.add_argument("--port", required=True, help="Serial port to transmit on, for example /dev/tty.usbmodem1234")
    parser.add_argument("--baud", type=int, default=115200, help="UART baud rate")
    parser.add_argument("--period", type=float, default=0.2, help="Packet period in seconds")
    parser.add_argument("--duration", type=float, default=0.0, help="Stop after N seconds; 0 runs until Ctrl+C")
    return parser.parse_args()


def main() -> None:
    try:
        import serial
    except ImportError as exc:  # pragma: no cover - runtime dependency
        raise SystemExit("pyserial is required. Run: python3 -m pip install -r viewer/requirements.txt") from exc

    args = parse_args()
    start = time.time()
    simulator = BatteryTelemetrySimulator()

    with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
        print(f"Streaming firmware-format packets to {args.port} @ {args.baud} baud")
        try:
            while True:
                if args.duration > 0 and (time.time() - start) >= args.duration:
                    break

                packet = simulator.build_packet(timestamp_ms=int((time.time() - start) * 1000))
                ser.write(packet)
                ser.flush()
                time.sleep(args.period)
        except KeyboardInterrupt:
            pass

    print("Simulation stopped")


if __name__ == "__main__":
    main()
