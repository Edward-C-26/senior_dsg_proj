#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import random
import struct
import time


SOF = b"\xAA\x55"
CELL_COUNT = 6
LEGACY_PAYLOAD_LEN = 38
EXTENDED_PAYLOAD_LEN = 40
LEGACY_PACKET_FORMAT = "<2sBI6H6hH6BH"
EXTENDED_PACKET_FORMAT = "<2sBI6H6hHh6BH"


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_packet(timestamp_ms: int, tick: int, include_current: bool) -> bytes:
    t = tick / 5.0

    base_mv = [4090, 4065, 4030, 3995, 3960, 3925]
    voltages_mv = [
        max(3000, min(4200, base + int(18 * math.sin(t * 0.8 + index * 0.4)) - tick // 18))
        for index, base in enumerate(base_mv)
    ]

    temp_group_1_c = 27.0 + 2.0 * math.sin(t / 1.8)
    temp_group_2_c = 29.5 + 2.8 * math.cos(t / 2.4)
    temps_cc = [
        int(temp_group_1_c * 100),
        int(temp_group_1_c * 100),
        int(temp_group_1_c * 100),
        int(temp_group_2_c * 100),
        int(temp_group_2_c * 100),
        int(temp_group_2_c * 100),
    ]

    pack_voltage_mv = min(0xFFFF, sum(voltages_mv))
    status0 = 0
    if max(voltages_mv) > 4200:
        status0 |= 0x01
    if min(voltages_mv) < 3000:
        status0 |= 0x02
    if max(temps_cc) > 6000:
        status0 |= 0x04
    if min(temps_cc) < 0:
        status0 |= 0x08
    status_bytes = [status0, 0, 0, 0, 0, 0]

    if include_current:
        pack_current_a = 8.5 + 1.8 * math.sin(t / 1.6) + random.uniform(-0.15, 0.15)
        pack_current_ca = int(round(pack_current_a * 100))
        body = struct.pack(
            EXTENDED_PACKET_FORMAT,
            SOF,
            EXTENDED_PAYLOAD_LEN,
            timestamp_ms,
            *voltages_mv,
            *temps_cc,
            pack_voltage_mv,
            pack_current_ca,
            *status_bytes,
            0,
        )
    else:
        body = struct.pack(
            LEGACY_PACKET_FORMAT,
            SOF,
            LEGACY_PAYLOAD_LEN,
            timestamp_ms,
            *voltages_mv,
            *temps_cc,
            pack_voltage_mv,
            *status_bytes,
            0,
        )

    crc = crc16_ccitt(body[:-2])
    return body[:-2] + struct.pack("<H", crc)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Transmit simulated BMS UART packets for the viewer")
    parser.add_argument("--port", required=True, help="Serial port to transmit on, for example /dev/tty.usbmodem1234")
    parser.add_argument("--baud", type=int, default=115200, help="UART baud rate")
    parser.add_argument("--period", type=float, default=0.2, help="Packet period in seconds")
    parser.add_argument("--duration", type=float, default=0.0, help="Stop after N seconds; 0 runs until Ctrl+C")
    parser.add_argument(
        "--mode",
        choices=("extended", "legacy"),
        default="extended",
        help="Use extended packets with current or legacy packets without current",
    )
    return parser.parse_args()


def main() -> None:
    try:
        import serial
    except ImportError as exc:  # pragma: no cover - runtime dependency
        raise SystemExit("pyserial is required. Run: python3 -m pip install -r viewer/requirements.txt") from exc

    args = parse_args()
    include_current = args.mode == "extended"
    start = time.time()
    tick = 0

    with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
        print(f"Streaming {args.mode} packets to {args.port} @ {args.baud} baud")
        try:
            while True:
                if args.duration > 0 and (time.time() - start) >= args.duration:
                    break

                packet = build_packet(int((time.time() - start) * 1000), tick, include_current)
                ser.write(packet)
                ser.flush()
                tick += 1
                time.sleep(args.period)
        except KeyboardInterrupt:
            pass

    print("Simulation stopped")


if __name__ == "__main__":
    main()
