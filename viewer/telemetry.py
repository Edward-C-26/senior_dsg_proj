from __future__ import annotations

import math
import queue
import random
import struct
import threading
import time
from dataclasses import dataclass
from typing import Optional


SOF = b"\xAA\x55"
CELL_COUNT = 6


# Current firmware payload length: timestamp through CRC.
FIRMWARE_LENGTH_BYTE = 40
FIRMWARE_PACKET_SIZE = 43

# Firmware packet layout: SOF, len, timestamp, cells, temps, pack V, current, status, CRC.
FIRMWARE_PACKET_FORMAT = "<2sBI6H6hHh6BH"

MAX_BUFFER_BYTES = 4096
DEFAULT_PACKET_PERIOD_S = 0.2


@dataclass(slots=True)
class TelemetryFrame:
    # Host or device timestamp in milliseconds.
    timestamp_ms: int
    # Per-cell voltages in millivolts.
    cell_voltage_mv: list[int]
    # Per-cell temperatures in Celsius.
    cell_temp_c: list[float]
    # Pack voltage in millivolts.
    pack_voltage_mv: int
    # Raw status bytes from firmware/device.
    status_bytes: list[int]
    # Pack current in amps.
    pack_current_a: float
    # CRC validity flag (currently set true on decode success).
    crc_ok: bool = True

    @property
    def avg_cell_v(self) -> float:
        # Average cell voltage converted to volts.
        return sum(self.cell_voltage_mv) / (len(self.cell_voltage_mv) * 1000.0)

    @property
    def min_cell_v(self) -> float:
        # Lowest cell voltage converted to volts.
        return min(self.cell_voltage_mv) / 1000.0

    @property
    def max_cell_v(self) -> float:
        # Highest cell voltage converted to volts.
        return max(self.cell_voltage_mv) / 1000.0

    @property
    def imbalance_mv(self) -> int:
        # Cell imbalance in millivolts (max - min).
        return max(self.cell_voltage_mv) - min(self.cell_voltage_mv)


def crc16_ccitt(data: bytes) -> int:
    # CRC-16/CCITT-FALSE: init 0xFFFF, polynomial 0x1021.
    crc = 0xFFFF
    for value in data:
        # Mix next byte into high CRC byte.
        crc ^= value << 8
        for _ in range(8):
            # Shift and conditionally apply polynomial.
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


class PacketParser:
    def __init__(self) -> None:
        # Rolling byte buffer for arbitrary chunked serial input.
        self._buffer = bytearray()

    def feed(self, data: bytes) -> list[TelemetryFrame]:
        # Append newly received bytes.
        self._buffer.extend(data)

        # Trim oldest bytes if buffer exceeds configured cap.
        if len(self._buffer) > MAX_BUFFER_BYTES:
            del self._buffer[:-MAX_BUFFER_BYTES]

        # Output list for all fully decoded frames in this call.
        frames: list[TelemetryFrame] = []

        while True:
            # Find next possible packet boundary.
            sof_index = self._buffer.find(SOF)
            if sof_index < 0:
                # Keep only trailing byte in case it is first SOF byte.
                if len(self._buffer) > 1:
                    del self._buffer[:-1]
                break

            # Drop noise bytes before SOF.
            if sof_index > 0:
                del self._buffer[:sof_index]

            # Need at least SOF + length byte.
            if len(self._buffer) < 3:
                break

            # Read payload length from packet header.
            payload_len = self._buffer[2]
            # Reject unknown packet formats.
            if payload_len != FIRMWARE_LENGTH_BYTE:
                del self._buffer[0]
                continue

            # Total packet bytes = SOF(2) + len(1) + payload.
            packet_size = FIRMWARE_PACKET_SIZE
            # Wait for more bytes if packet is incomplete.
            if len(self._buffer) < packet_size:
                break

            # Extract candidate packet for CRC and decode.
            packet_bytes = bytes(self._buffer[:packet_size])
            # CRC is stored in final two bytes, little-endian.
            expected_crc = struct.unpack_from("<H", packet_bytes, packet_size - 2)[0]
            # CRC is computed over all bytes except CRC field itself.
            computed_crc = crc16_ccitt(packet_bytes[:-2])
            if expected_crc != computed_crc:
                # On CRC failure, resync by dropping one byte.
                del self._buffer[0]
                continue

            # Decode and emit valid packet.
            frames.append(decode_packet(packet_bytes))
            # Remove consumed packet from buffer.
            del self._buffer[:packet_size]

        return frames


def decode_packet(packet: bytes) -> TelemetryFrame:
    _, _, timestamp_ms, *rest = struct.unpack(FIRMWARE_PACKET_FORMAT, packet)
    cell_voltage_mv = rest[0:6]
    cell_temp_cc = rest[6:12]
    pack_voltage_mv = rest[12]
    pack_current_decia = rest[13]
    status_bytes = rest[14:20]
    return TelemetryFrame(
        timestamp_ms=timestamp_ms,
        cell_voltage_mv=list(cell_voltage_mv),
        # Temperatures are stored as centi-degrees C.
        cell_temp_c=[value / 100.0 for value in cell_temp_cc],
        pack_voltage_mv=pack_voltage_mv,
        status_bytes=list(status_bytes),
        # Firmware stores current as signed deci-amps.
        pack_current_a=pack_current_decia / 10.0,
    )


def build_firmware_packet(
    timestamp_ms: int,
    cell_voltage_mv: list[int],
    cell_temp_c: list[float],
    pack_voltage_mv: int,
    pack_current_a: float,
    status_bytes: list[int],
) -> bytes:
    # Convert temperature floats to signed centi-degrees for transport.
    temps_cc = [int(round(value * 100.0)) for value in cell_temp_c]
    # Match firmware: convert amps to signed deci-amps.
    current_decia = int(round(pack_current_a * 10.0))
    # Pack structure with placeholder CRC at the end.
    packet = struct.pack(
        FIRMWARE_PACKET_FORMAT,
        SOF,
        FIRMWARE_LENGTH_BYTE,
        timestamp_ms & 0xFFFFFFFF,
        *cell_voltage_mv,
        *temps_cc,
        pack_voltage_mv,
        current_decia,
        *status_bytes,
        0,
    )
    # Compute and insert CRC over packet excluding CRC field.
    crc = crc16_ccitt(packet[:-2])
    return packet[:-2] + struct.pack("<H", crc)


class BatteryTelemetrySimulator:
    def __init__(self) -> None:
        # Simulation tick counter.
        self._tick = 0
        # Initial state-of-charge fraction.
        self._soc = 0.88
        # Fixed per-cell offsets to emulate manufacturing mismatch.
        self._cell_offsets_mv = [-8.0, 5.0, 12.0, 3.0, -4.0, -10.0]
        # Last emitted voltages, used to keep demo SOC moving downward.
        self._last_cell_voltage_mv: Optional[list[int]] = None
        # Lumped thermal states for two 3-cell groups.
        self._temp_group_a = 27.0
        self._temp_group_b = 28.5

    def build_packet(self, timestamp_ms: Optional[int] = None) -> bytes:
        # Use current wall-clock time unless caller provides one.
        packet_time_ms = int(time.time() * 1000) if timestamp_ms is None else timestamp_ms
        # Generate next synthetic telemetry sample.
        sample = self._next_sample()
        return build_firmware_packet(
            timestamp_ms=packet_time_ms,
            cell_voltage_mv=sample.cell_voltage_mv,
            cell_temp_c=sample.cell_temp_c,
            pack_voltage_mv=sample.pack_voltage_mv,
            pack_current_a=sample.pack_current_a or 0.0,
            status_bytes=sample.status_bytes,
        )

    def _next_sample(self) -> TelemetryFrame:
        # Fixed simulation timestep.
        dt = DEFAULT_PACKET_PERIOD_S
        t = self._tick * dt

        # Simulate discharge current with slow sinusoid + small noise.
        pack_current_a = 6.5 + 1.0 * math.sin(t / 7.0) + random.uniform(-0.15, 0.15)
        pack_current_a = max(0.0, pack_current_a)

        # Integrate SoC drop from current draw and pack capacity.
        pack_capacity_ah = 4.0
        self._soc -= (pack_current_a * dt) / (3600.0 * pack_capacity_ah)
        self._soc = max(0.05, self._soc)

        # Estimate cell OCV from SoC and apply load-induced voltage droop.
        base_ocv_v = self._ocv_from_soc(self._soc)
        load_droop_v = pack_current_a * 0.018

        cell_voltage_mv: list[int] = []
        for index in range(CELL_COUNT):
            # Slowly drift one cell upward to emulate growing imbalance.
            imbalance_drift_mv = 0.04 * self._tick if index == 2 else 0.0
            # Add small per-sample voltage noise.
            noise_mv = random.uniform(-2.0, 2.0)
            # Combine OCV, load droop, static offset, drift, and noise.
            voltage_v = base_ocv_v - load_droop_v + (self._cell_offsets_mv[index] + imbalance_drift_mv + noise_mv) / 1000.0
            # Clamp to realistic bounds and quantize to mV integer.
            voltage_mv = max(2800, min(4300, int(round(voltage_v * 1000.0))))
            if self._last_cell_voltage_mv is not None:
                voltage_mv = min(voltage_mv, self._last_cell_voltage_mv[index])
            cell_voltage_mv.append(voltage_mv)

        # Drive two thermal groups toward current-dependent targets.
        target_a = 27.0 + 0.9 * pack_current_a
        target_b = 28.0 + 1.1 * pack_current_a
        self._temp_group_a += 0.08 * (target_a - self._temp_group_a) + random.uniform(-0.03, 0.03)
        self._temp_group_b += 0.08 * (target_b - self._temp_group_b) + random.uniform(-0.03, 0.03)
        # Map group temperatures to 6 cells.
        cell_temp_c = [
            self._temp_group_a,
            self._temp_group_a,
            self._temp_group_a,
            self._temp_group_b,
            self._temp_group_b,
            self._temp_group_b,
        ]

        # Pack voltage is the sum of series cell voltages.
        pack_voltage_mv = sum(cell_voltage_mv)
        self._last_cell_voltage_mv = cell_voltage_mv.copy()
        # Build status bitfield byte 0 from simple threshold checks.
        status0 = 0
        if max(cell_voltage_mv) > 4200:
            status0 |= 0x01
        if min(cell_voltage_mv) < 3000:
            status0 |= 0x02
        if max(cell_temp_c) > 60.0:
            status0 |= 0x04
        if min(cell_temp_c) < 0.0:
            status0 |= 0x08

        # Advance simulation clock by one sample.
        self._tick += 1
        return TelemetryFrame(
            timestamp_ms=0,
            cell_voltage_mv=cell_voltage_mv,
            cell_temp_c=cell_temp_c,
            pack_voltage_mv=pack_voltage_mv,
            status_bytes=[status0, 0, 0, 0, 0, 0],
            pack_current_a=pack_current_a,
        )

    @staticmethod
    def _ocv_from_soc(soc: float) -> float:
        # Clamp SoC to [0, 1] range before piecewise mapping.
        soc = max(0.0, min(1.0, soc))
        # Top plateau region.
        if soc > 0.90:
            return 4.00 + ((soc - 0.90) / 0.10) * 0.18
        # Mid linear region.
        if soc > 0.20:
            return 3.65 + ((soc - 0.20) / 0.70) * 0.35
        # Lower knee region.
        return 3.00 + (soc / 0.20) * 0.65


class DemoSerialReaderThread(threading.Thread):
    def __init__(
        self,
        out_queue: queue.Queue[tuple[str, object]],
        stop_event: threading.Event,
        *,
        packet_period_s: float = DEFAULT_PACKET_PERIOD_S,
        chunk_delay_range_s: tuple[float, float] = (0.002, 0.02),
        chunk_size_range: tuple[int, int] = (1, 8),
    ) -> None:
        super().__init__(daemon=True)
        # Queue used to publish status updates and parsed frames.
        self._queue = out_queue
        # Cooperative stop signal.
        self._stop_event = stop_event
        # Target cadence for full packets.
        self._packet_period_s = packet_period_s
        # Random delay between emitted serial chunks.
        self._chunk_delay_range_s = chunk_delay_range_s
        # Random chunk sizes to emulate UART fragmentation.
        self._chunk_size_range = chunk_size_range
        # Parser reconstructs frames from chunked stream.
        self._parser = PacketParser()
        # Synthetic packet source.
        self._simulator = BatteryTelemetrySimulator()

    def run(self) -> None:
        # Announce simulator mode on startup.
        self._queue.put(("status", "Demo UART mode running"))

        while not self._stop_event.is_set():
            # Generate one full packet from simulator.
            packet = self._simulator.build_packet()
            # Track elapsed time so outer loop meets requested period.
            burst_start = time.monotonic()
            index = 0

            while index < len(packet) and not self._stop_event.is_set():
                # Emit a random-sized packet fragment.
                chunk_len = random.randint(*self._chunk_size_range)
                chunk = packet[index:index + chunk_len]
                index += chunk_len

                # Parse any complete frames created by this fragment.
                for frame in self._parser.feed(chunk):
                    self._queue.put(("frame", frame))

                # Pause briefly before next fragment.
                time.sleep(random.uniform(*self._chunk_delay_range_s))

            # Sleep remainder of cycle to maintain packet period.
            remaining_sleep = self._packet_period_s - (time.monotonic() - burst_start)
            if remaining_sleep > 0:
                time.sleep(remaining_sleep)
