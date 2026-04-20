#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import glob
import os
import queue
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Optional

# FOR GUI
import tkinter as tk
from tkinter import messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:  
    serial = None
    list_ports = None


VIEWER_DIR = Path(__file__).resolve().parent
CACHE_DIR = VIEWER_DIR / ".cache"
CACHE_DIR.mkdir(parents=True, exist_ok=True)

os.environ.setdefault("MPLCONFIGDIR", str(CACHE_DIR / "matplotlib"))
os.environ.setdefault("XDG_CACHE_HOME", str(CACHE_DIR))

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from telemetry import CELL_COUNT, DemoSerialReaderThread, PacketParser, TelemetryFrame


UART_BAUD = 115200

VOLTAGE_OV = 4.20
VOLTAGE_UV = 3.00
VOLTAGE_WARN_HIGH = 4.10
VOLTAGE_WARN_LOW = 3.15

TEMP_OT = 60.0
TEMP_UT = 0.0
TEMP_WARN_HIGH = 55.0
TEMP_WARN_LOW = 5.0

# App background color.
BG = "#F4F1EA"
# Card/panel background color.
CARD = "#FFFDF9"
# Primary text color.
INK = "#1F2933"
# Secondary/muted text color.
MUTED = "#5D6B78"
# Grid and border accent color.
GRID = "#D7D2C8"
# Good/normal state color.
GOOD = "#2E7D32"
# Warning state color.
WARN = "#D4A017"
# Fault/error state color.
BAD = "#C0392B"
# Informational/plot accent color.
INFO = "#2563EB"
# Button/interactive accent color.
ACCENT = "#C56A2D"

class SerialReaderThread(threading.Thread):
    def __init__(
        self,
        port: str,
        baudrate: int,
        out_queue: queue.Queue[tuple[str, object]],
        stop_event: threading.Event,
    ) -> None:
        super().__init__(daemon=True)
        self._port = port
        self._baudrate = baudrate
        self._queue = out_queue
        self._stop_event = stop_event
        self._parser = PacketParser()

    def run(self) -> None:
        if serial is None:
            self._queue.put(("error", "pyserial is not installed. Run: pip install -r viewer/requirements.txt"))
            return

        try:
            with serial.Serial(self._port, self._baudrate, timeout=0.2) as ser:
                self._queue.put(("status", f"Connected to {self._port} @ {self._baudrate}"))
                while not self._stop_event.is_set():
                    raw = ser.read(256)
                    if not raw:
                        continue
                    # The reader thread never touches Tk widgets directly. It only
                    # pushes parsed events into a queue so the main UI thread stays safe.
                    for frame in self._parser.feed(raw):
                        self._queue.put(("frame", frame))
        except Exception as exc: 
            self._queue.put(("error", f"Serial error: {exc}"))


class CellCard(ttk.Frame):
    def __init__(self, master: tk.Misc, cell_index: int) -> None:
        super().__init__(master, style="Card.TFrame", padding=(14, 12))
        self.cell_index = cell_index
        self.columnconfigure(0, weight=1)

        self.title_label = ttk.Label(self, text=f"Cell {cell_index + 1}", style="CellTitle.TLabel")
        self.title_label.grid(row=0, column=0, sticky="w")

        self.canvas = tk.Canvas(self, width=112, height=238, bg=CARD, highlightthickness=0)
        self.canvas.grid(row=1, column=0, sticky="nsew", pady=(8, 12))

        self.voltage_label = ttk.Label(self, text="0.000 V", style="CellValue.TLabel")
        self.voltage_label.grid(row=2, column=0, sticky="w")

        self.temp_label = ttk.Label(self, text="0.0 °C", style="CellMeta.TLabel")
        self.temp_label.grid(row=3, column=0, sticky="w", pady=(2, 0))

        self.state_label = ttk.Label(self, text="Waiting", style="CellMeta.TLabel")
        self.state_label.grid(row=4, column=0, sticky="w", pady=(2, 0))

        self._draw_bar(0.0, GRID)

    def update_card(self, voltage_v: float, temp_c: float, color: str, state_text: str) -> None:
        # Map voltage into a 0..1 fill range for the vertical bar.
        normalized = max(0.0, min(1.0, (voltage_v - 2.8) / 1.5))
        # Redraw the battery bar using the current state color.
        self._draw_bar(normalized, color)
        # Show latest voltage and temperature text.
        self.voltage_label.configure(text=f"{voltage_v:.3f} V", foreground=color)
        self.temp_label.configure(text=f"{temp_c:.1f} °C")
        # Show state label (Normal/Warning/Fault) in matching color.
        self.state_label.configure(text=state_text, foreground=color)

    def reset_card(self) -> None:
        # Return this card to its startup/idle visuals.
        self._draw_bar(0.0, GRID)
        self.voltage_label.configure(text="0.000 V", foreground=INK)
        self.temp_label.configure(text="0.0 °C")
        self.state_label.configure(text="Waiting", foreground=MUTED)

    def _draw_bar(self, fill_ratio: float, color: str) -> None:
        # Clear previous drawing before rendering the new bar state.
        self.canvas.delete("all")
        # Fixed geometry for the battery outline area.
        left, right, top, bottom = 32, 80, 20, 214
        self.canvas.create_rectangle(left, top, right, bottom, outline=GRID, width=2)
        # Convert fill ratio into the Y coordinate where fill starts.
        fill_top = bottom - ((bottom - top) * fill_ratio)
        self.canvas.create_rectangle(left + 3, fill_top, right - 3, bottom - 3, fill=color, width=0)
        # Draw simple low/high voltage reference labels.
        self.canvas.create_text((left + right) / 2, 226, text="2.8V", fill=MUTED, font=("Avenir Next", 9))
        self.canvas.create_text((left + right) / 2, 10, text="4.3V", fill=MUTED, font=("Avenir Next", 9))


class FaultBadge(ttk.Frame):
    def __init__(self, master: tk.Misc, label: str) -> None:
        super().__init__(master, style="Card.TFrame", padding=(10, 8))
        self.columnconfigure(0, weight=1)
        self.label = ttk.Label(self, text=label, style="FaultTitle.TLabel")
        self.label.grid(row=0, column=0, sticky="w")
        self.value = ttk.Label(self, text="Idle", style="FaultValue.TLabel")
        self.value.grid(row=1, column=0, sticky="w", pady=(4, 0))

    def set_state(self, active: bool, text: str) -> None:
        color = BAD if active else GOOD
        self.value.configure(text=text, foreground=color)


class BmsViewerApp:
    def __init__(self, root: tk.Tk, start_demo: bool = False) -> None:
        self.root = root
        self.root.title("ECE 445 BMS Viewer")
        self.root.geometry("1380x900")
        self.root.minsize(1180, 780)
        self.root.configure(bg=BG)

        self.data_queue: queue.Queue[tuple[str, object]] = queue.Queue()
        self.reader_stop_event: Optional[threading.Event] = None
        self.reader_thread: Optional[threading.Thread] = None

        self.logging_enabled = False
        self.log_file = None
        self.log_writer = None
        self.last_frame_time: Optional[float] = None
        self.latest_frame: Optional[TelemetryFrame] = None

        self.time_history: deque[float] = deque(maxlen=120)
        self.voltage_history: list[deque[float]] = [deque(maxlen=120) for _ in range(CELL_COUNT)]
        self.temp_history: list[deque[float]] = [deque(maxlen=120) for _ in range(CELL_COUNT)]
        self.current_history: deque[float] = deque(maxlen=120)
        # These histories back both the summary cards and the optional trends window.
        # Keeping them bounded makes the UI predictable even if the app stays open all day.

        self.soc_estimate = 50.0
        self.soh_estimate = 100.0

        self.port_var = tk.StringVar()
        self.status_var = tk.StringVar(value="Disconnected")

        self.summary_labels: dict[str, ttk.Label] = {}
        self.cell_cards: list[CellCard] = []
        self.fault_badges: dict[str, FaultBadge] = {}
        self.plot_window: Optional[tk.Toplevel] = None
        self.plot_canvas: Optional[FigureCanvasTkAgg] = None
        self.voltage_ax = None
        self.temp_ax = None
        self.current_ax = None
        self.current_line = None
        self.voltage_lines: list[object] = []
        self.temp_lines: list[object] = []
        self.voltage_line_labels: list[object] = []

        self._configure_styles()
        self._build_layout()
        self.refresh_ports()
        self.root.after(100, self._poll_queue)
        self.root.after(1000, self._watchdog_link)

        if start_demo:
            self.start_demo()

    def _configure_styles(self) -> None:
        style = ttk.Style()
        style.theme_use("clam")

        style.configure("App.TFrame", background=BG)
        style.configure("Card.TFrame", background=CARD, relief="flat")
        style.configure("Header.TLabel", background=BG, foreground=INK, font=("Avenir Next Demi Bold", 24))
        style.configure("Subtle.TLabel", background=BG, foreground=MUTED, font=("Avenir Next", 10))
        style.configure("Control.TLabel", background=BG, foreground=INK, font=("Avenir Next Demi Bold", 10))
        style.configure("Title.TLabel", background=CARD, foreground=MUTED, font=("Avenir Next Demi Bold", 10))
        style.configure("Metric.TLabel", background=CARD, foreground=INK, font=("Avenir Next Demi Bold", 24))
        style.configure("MetricFoot.TLabel", background=CARD, foreground=MUTED, font=("Avenir Next", 10))
        style.configure("CellTitle.TLabel", background=CARD, foreground=MUTED, font=("Avenir Next Demi Bold", 11))
        style.configure("CellValue.TLabel", background=CARD, foreground=INK, font=("Avenir Next Demi Bold", 20))
        style.configure("CellMeta.TLabel", background=CARD, foreground=MUTED, font=("Avenir Next", 11))
        style.configure("FaultTitle.TLabel", background=CARD, foreground=MUTED, font=("Avenir Next Demi Bold", 11))
        style.configure("FaultValue.TLabel", background=CARD, foreground=GOOD, font=("Avenir Next Demi Bold", 18))
        style.configure(
            "Accent.TButton",
            font=("Avenir Next Demi Bold", 9),
            padding=(10, 6),
            background=ACCENT,
            foreground=CARD,
            borderwidth=0,
        )
        style.map(
            "Accent.TButton",
            background=[("active", "#B35E27"), ("pressed", "#9D5222")],
            foreground=[("disabled", "#F2E9E2")],
        )

    def _build_layout(self) -> None:
        outer = ttk.Frame(self.root, style="App.TFrame", padding=18)
        outer.pack(fill="both", expand=True)
        outer.columnconfigure(0, weight=1)
        outer.rowconfigure(3, weight=1)

        header = ttk.Frame(outer, style="App.TFrame")
        header.grid(row=0, column=0, sticky="ew")
        header.columnconfigure(0, weight=1)

        ttk.Label(header, text="Battery Management System Viewer", style="Header.TLabel").grid(row=0, column=0, sticky="w")

        controls = ttk.Frame(outer, style="Card.TFrame", padding=14)
        controls.grid(row=1, column=0, sticky="ew", pady=(16, 14))
        for index in range(8):
            controls.columnconfigure(index, weight=0)
        controls.columnconfigure(8, weight=1)

        ttk.Label(controls, text="UART Port", style="Control.TLabel").grid(row=0, column=0, sticky="w")
        self.port_box = ttk.Combobox(controls, textvariable=self.port_var, width=18, state="readonly")
        self.port_box.grid(row=1, column=0, sticky="ew", padx=(0, 12), pady=(6, 0))

        ttk.Button(controls, text="Refresh Ports", command=self.refresh_ports, style="Accent.TButton").grid(row=1, column=1, padx=(0, 8), pady=(6, 0))
        ttk.Button(controls, text="Connect", command=self.connect_serial, style="Accent.TButton").grid(row=1, column=2, padx=(0, 8), pady=(6, 0))
        ttk.Button(controls, text="Demo", command=self.start_demo, style="Accent.TButton").grid(row=1, column=3, padx=(0, 8), pady=(6, 0))
        ttk.Button(controls, text="Disconnect", command=self.disconnect_reader, style="Accent.TButton").grid(row=1, column=4, padx=(0, 8), pady=(6, 0))
        ttk.Button(controls, text="CSV Log", command=self.toggle_logging, style="Accent.TButton").grid(row=1, column=5, padx=(0, 8), pady=(6, 0))
        self.trends_button = ttk.Button(controls, text="Open Trends", command=self.toggle_trends_window, style="Accent.TButton")
        self.trends_button.grid(row=1, column=6, padx=(0, 8), pady=(6, 0))
        ttk.Label(controls, text="UART @ 115200 baud, packet CRC16-CCITT", style="Subtle.TLabel").grid(row=1, column=8, sticky="e", pady=(6, 0))

        summary = ttk.Frame(outer, style="App.TFrame")
        summary.grid(row=2, column=0, sticky="ew", pady=(0, 4))
        for index in range(6):
            summary.columnconfigure(index, weight=1)

        summary_specs = [
            ("Pack Voltage", "pack_voltage", "0.00 V"),
            ("Pack Current", "pack_current", "Awaiting firmware"),
            ("Avg Cell", "avg_cell", "0.000 V"),
            ("Delta V", "imbalance", "0 mV"),
            ("SOC Estimate", "soc", "0.0 %"),
            ("SOH Estimate", "soh", "100.0 %"),
        ]
        for index, (title, key, default) in enumerate(summary_specs):
            card = ttk.Frame(summary, style="Card.TFrame", padding=(16, 14))
            card.grid(row=0, column=index, sticky="nsew", padx=(0, 10 if index < 5 else 0))
            ttk.Label(card, text=title, style="Title.TLabel").pack(anchor="w")
            metric = ttk.Label(card, text=default, style="Metric.TLabel")
            metric.pack(anchor="w", pady=(8, 2))
            foot = ttk.Label(card, text="Waiting for data", style="MetricFoot.TLabel")
            foot.pack(anchor="w")
            self.summary_labels[key] = metric
            self.summary_labels[f"{key}_foot"] = foot

        body = ttk.Frame(outer, style="App.TFrame")
        body.grid(row=3, column=0, sticky="nsew", pady=(14, 0))
        body.columnconfigure(0, weight=7)
        body.columnconfigure(1, weight=4)
        body.rowconfigure(0, weight=1)

        left = ttk.Frame(body, style="App.TFrame")
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 14))
        left.columnconfigure(0, weight=1)
        left.rowconfigure(0, weight=1)

        cell_section = ttk.Frame(left, style="Card.TFrame", padding=(18, 16))
        cell_section.grid(row=0, column=0, sticky="nsew")
        cell_section.columnconfigure(0, weight=1)
        cell_section.rowconfigure(2, weight=1)
        ttk.Label(cell_section, text="Cell Monitor", style="Header.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(
            cell_section,
            text="Per-cell voltage and temperature mapped from the STM32 UART packet",
            style="MetricFoot.TLabel",
        ).grid(row=1, column=0, sticky="w", pady=(4, 12))

        cards = ttk.Frame(cell_section, style="Card.TFrame")
        cards.grid(row=2, column=0, sticky="nsew")
        cards.rowconfigure(0, weight=1)
        for index in range(CELL_COUNT):
            cards.columnconfigure(index, weight=1)
            cell_card = CellCard(cards, index)
            cell_card.grid(row=0, column=index, sticky="nsew", padx=(0, 12 if index < CELL_COUNT - 1 else 0))
            self.cell_cards.append(cell_card)

        right = ttk.Frame(body, style="App.TFrame")
        right.grid(row=0, column=1, sticky="nsew")
        right.columnconfigure(0, weight=1)
        right.rowconfigure(0, weight=1)
        right.rowconfigure(1, weight=2)

        status_card = ttk.Frame(right, style="Card.TFrame", padding=(18, 16))
        status_card.grid(row=0, column=0, sticky="nsew")
        status_card.columnconfigure(0, weight=1)
        ttk.Label(status_card, text="Link Status", style="Header.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(status_card, textvariable=self.status_var, style="Metric.TLabel").grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Label(status_card, text="Green indicates healthy communication and valid CRC frames.", style="MetricFoot.TLabel").grid(row=2, column=0, sticky="w", pady=(4, 0))

        fault_card = ttk.Frame(right, style="Card.TFrame", padding=(18, 16))
        fault_card.grid(row=1, column=0, sticky="nsew", pady=(14, 0))
        fault_card.columnconfigure(0, weight=1)
        fault_card.rowconfigure(2, weight=1)
        ttk.Label(fault_card, text="Fault Panel", style="Header.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(fault_card, text="Mirrors `status_bytes[0]` and derived pack checks from the repo README.", style="MetricFoot.TLabel").grid(row=1, column=0, sticky="w", pady=(4, 12))
        badge_grid = ttk.Frame(fault_card, style="Card.TFrame")
        badge_grid.grid(row=2, column=0, sticky="nsew")
        for row in range(2):
            badge_grid.rowconfigure(row, weight=1)
        for index in range(6):
            badge_grid.columnconfigure(index % 3, weight=1)
        fault_names = ["OV", "UV", "OT", "UT", "Imbalance", "Comm"]
        for index, name in enumerate(fault_names):
            badge = FaultBadge(badge_grid, name)
            badge.grid(row=index // 3, column=index % 3, sticky="nsew", padx=(0, 10), pady=(0, 10))
            self.fault_badges[name] = badge

    def _build_plots(self, master: ttk.Frame) -> None:
        figure = Figure(figsize=(7.8, 2.55), dpi=100)
        figure.patch.set_facecolor(CARD)
        self.voltage_ax = figure.add_subplot(311)
        self.temp_ax = figure.add_subplot(312)
        self.current_ax = figure.add_subplot(313)
        figure.subplots_adjust(left=0.11, right=0.95, top=0.9, bottom=0.16, hspace=0.8)

        for ax, title, ylabel in (
            (self.voltage_ax, "Cell Voltage Trend", "Volts"),
            (self.temp_ax, "Temperature Trend", "°C"),
            (self.current_ax, "Pack Current Trend", "Amps"),
        ):
            ax.set_facecolor(CARD)
            ax.set_title(title, loc="left", fontsize=10, color=INK, pad=6)
            ax.set_ylabel(ylabel, color=MUTED, fontsize=8)
            ax.tick_params(colors=MUTED, labelsize=7)
            ax.grid(True, color=GRID, linewidth=0.7)
            for spine in ax.spines.values():
                spine.set_color(GRID)
        self.current_ax.set_xlabel("Time (s)", color=MUTED, fontsize=8)

        self.voltage_lines = []
        self.temp_lines = []
        self.voltage_line_labels = []
        palette = ["#0F766E", "#2563EB", "#A16207", "#BE185D", "#4D7C0F", "#7C3AED"]
        for index in range(CELL_COUNT):
            line_v, = self.voltage_ax.plot([], [], color=palette[index], linewidth=2, label=f"C{index + 1}")
            line_t, = self.temp_ax.plot([], [], color=palette[index], linewidth=2, label=f"C{index + 1}")
            self.voltage_lines.append(line_v)
            self.temp_lines.append(line_t)
            label = self.voltage_ax.text(
                0,
                0,
                f"C{index + 1}",
                color=palette[index],
                fontsize=7,
                va="center",
                ha="left",
                clip_on=True,
            )
            self.voltage_line_labels.append(label)
        self.current_line, = self.current_ax.plot([], [], color=INFO, linewidth=2)

        canvas = FigureCanvasTkAgg(figure, master=master)
        canvas.draw()
        canvas.get_tk_widget().grid(row=1, column=0, sticky="nsew", pady=(8, 0))
        self.plot_canvas = canvas

    def toggle_trends_window(self) -> None:
        if self.plot_window is not None and self.plot_window.winfo_exists():
            self._close_trends_window()
            return
        self._open_trends_window()

    def _open_trends_window(self) -> None:
        self.plot_window = tk.Toplevel(self.root)
        self.plot_window.title("BMS Trends")
        self.plot_window.geometry("900x520")
        self.plot_window.minsize(760, 440)
        self.plot_window.configure(bg=BG)
        self.plot_window.protocol("WM_DELETE_WINDOW", self._close_trends_window)

        plot_frame = ttk.Frame(self.plot_window, style="Card.TFrame", padding=14)
        plot_frame.pack(fill="both", expand=True, padx=18, pady=18)
        plot_frame.columnconfigure(0, weight=1)
        plot_frame.rowconfigure(1, weight=1)

        ttk.Label(plot_frame, text="Trends", style="Header.TLabel").grid(row=0, column=0, sticky="w")
        self._build_plots(plot_frame)
        self.trends_button.configure(text="Close Trends")
        self._update_plots()

    def _close_trends_window(self) -> None:
        if self.plot_window is not None and self.plot_window.winfo_exists():
            self.plot_window.destroy()
        self.plot_window = None
        self.plot_canvas = None
        self.voltage_ax = None
        self.temp_ax = None
        self.current_ax = None
        self.current_line = None
        self.voltage_lines = []
        self.temp_lines = []
        self.voltage_line_labels = []
        self.trends_button.configure(text="Open Trends")

    def refresh_ports(self) -> None:
        ports: list[str] = []
        if list_ports is not None:
            ports = [port.device for port in list_ports.comports()]

        # macOS can expose USB CDC devices under /dev/cu.* and /dev/tty.*
        # even when pyserial's enumerator does not return them. Prefer the
        # callout device for outbound connections from the viewer.
        if os.name == "posix":
            ports.extend(sorted(glob.glob("/dev/cu.*")))
            ports.extend(sorted(glob.glob("/dev/tty.*")))

        ports = list(dict.fromkeys(ports))
        self.port_box["values"] = ports
        if ports:
            preferred_port = next((port for port in ports if "/dev/cu." in port), ports[0])
            self.port_var.set(preferred_port)
        else:
            self.port_var.set("")

    def connect_serial(self) -> None:
        # Connect to the selected real UART port.
        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("UART Port", "Select a serial port first.")
            return
        stop_event = threading.Event()
        self._start_reader(SerialReaderThread(port, UART_BAUD, self.data_queue, stop_event), stop_event)
        self.status_var.set(f"Opening {port}")

    def start_demo(self) -> None:
        # Start synthetic telemetry when hardware is not connected.
        stop_event = threading.Event()
        self._start_reader(
            DemoSerialReaderThread(
                self.data_queue,
                stop_event,
            ),
            stop_event,
        )
        self.status_var.set("Demo UART Mode")

    def disconnect_reader(self) -> None:
        # Signal reader thread to stop, then reset UI state.
        if self.reader_stop_event is not None:
            self.reader_stop_event.set()
        self.reader_thread = None
        self.reader_stop_event = None
        self._reset_telemetry_view()
        self.status_var.set("Disconnected")

    def _start_reader(self, reader_thread: threading.Thread, stop_event: threading.Event) -> None:
        # Ensure only one reader thread is active at a time.
        self.disconnect_reader()
        self.reader_thread = reader_thread
        self.reader_stop_event = stop_event
        self.reader_thread.start()

    def toggle_logging(self) -> None:
        # Toggle CSV logging on/off from the same button.
        if self.logging_enabled:
            self._close_log()
            self.status_var.set("Logging stopped")
            return

        log_dir = Path(__file__).resolve().parent / "logs"
        log_dir.mkdir(parents=True, exist_ok=True)
        log_path = log_dir / f"bms_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.log_file = log_path.open("w", newline="", encoding="utf-8")
        self.log_writer = csv.writer(self.log_file)
        header = [
            "host_time_iso",
            "mcu_timestamp_ms",
            "pack_voltage_mv",
            "pack_current_a",
            "status0",
            "status1",
            "status2",
            "status3",
            "status4",
            "status5",
        ]
        header.extend([f"cell_{index + 1}_mv" for index in range(CELL_COUNT)])
        header.extend([f"cell_{index + 1}_temp_c" for index in range(CELL_COUNT)])
        self.log_writer.writerow(header)
        self.logging_enabled = True
        self.status_var.set(f"Logging to {log_path.name}")

    def _close_log(self) -> None:
        # Close file handles cleanly to avoid corrupted CSV output.
        self.logging_enabled = False
        if self.log_file is not None:
            self.log_file.close()
        self.log_file = None
        self.log_writer = None

    def _poll_queue(self) -> None:
        # Drain all pending events from the worker thread queue.
        try:
            while True:
                kind, payload = self.data_queue.get_nowait()
                if kind == "frame":
                    self._handle_frame(payload)
                elif kind == "status":
                    self.status_var.set(str(payload))
                elif kind == "error":
                    self.status_var.set("Link fault")
                    messagebox.showerror("BMS Viewer", str(payload))
        except queue.Empty:
            pass
        finally:
            # Tkinter's `after` loop is our bridge from background threads back into the
            # GUI event loop. This keeps rendering responsive without blocking on serial I/O.
            self.root.after(100, self._poll_queue)

    def _handle_frame(self, frame: TelemetryFrame) -> None:
        # One incoming frame updates all visual sections and logs.
        now = time.time()
        self.last_frame_time = now
        self.latest_frame = frame

        # We chart against viewer-relative time instead of MCU timestamps because we care
        # more about a stable live trend than absolute synchronization across sessions.
        relative_t = 0.0 if not self.time_history else self.time_history[-1] + 0.2
        self.time_history.append(relative_t)
        for index in range(CELL_COUNT):
            self.voltage_history[index].append(frame.cell_voltage_mv[index] / 1000.0)
            self.temp_history[index].append(frame.cell_temp_c[index])
        self.current_history.append(frame.pack_current_a if frame.pack_current_a is not None else float("nan"))

        self._update_estimates(frame)
        self._update_summary(frame)
        self._update_cells(frame)
        self._update_faults(frame)
        self._update_plots()
        self._write_log(frame)

    def _reset_telemetry_view(self) -> None:
        # Return dashboard to a neutral "waiting" state.
        self.last_frame_time = None
        self.latest_frame = None
        self.time_history.clear()
        self.current_history.clear()
        for history in self.voltage_history:
            history.clear()
        for history in self.temp_history:
            history.clear()

        self.soc_estimate = 50.0
        self.soh_estimate = 100.0

        self.summary_labels["pack_voltage"].configure(text="0.00 V")
        self.summary_labels["pack_voltage_foot"].configure(text="Waiting for data")
        self.summary_labels["pack_current"].configure(text="Awaiting firmware")
        self.summary_labels["pack_current_foot"].configure(text="Waiting for data")
        self.summary_labels["avg_cell"].configure(text="0.000 V")
        self.summary_labels["avg_cell_foot"].configure(text="Waiting for data")
        self.summary_labels["imbalance"].configure(text="0 mV")
        self.summary_labels["imbalance_foot"].configure(text="Waiting for data")
        self.summary_labels["soc"].configure(text="0.0 %")
        self.summary_labels["soc_foot"].configure(text="Viewer-side voltage estimate")
        self.summary_labels["soh"].configure(text="100.0 %")
        self.summary_labels["soh_foot"].configure(text="Health estimate updates slowly over time")

        for card in self.cell_cards:
            card.reset_card()

        self.fault_badges["OV"].set_state(False, "Idle")
        self.fault_badges["UV"].set_state(False, "Idle")
        self.fault_badges["OT"].set_state(False, "Idle")
        self.fault_badges["UT"].set_state(False, "Idle")
        self.fault_badges["Imbalance"].set_state(False, "Waiting")
        self.fault_badges["Comm"].set_state(False, "Waiting")

        if self.plot_canvas is not None:
            self._clear_plots()

    def _update_estimates(self, frame: TelemetryFrame) -> None:
        # SOC/SOH here are lightweight viewer-side heuristics for quick demos, not a full
        # battery model. The firmware still owns the ground truth if that is added later.
        self.soc_estimate = max(0.0, min(100.0, ((frame.avg_cell_v - VOLTAGE_UV) / (VOLTAGE_OV - VOLTAGE_UV)) * 100.0))
        spread_penalty = max(0.0, (frame.imbalance_mv - 25) * 0.08)
        temp_penalty = 0.0
        hottest = max(frame.cell_temp_c)
        if hottest > 45.0:
            temp_penalty = (hottest - 45.0) * 0.6
        target_soh = max(70.0, min(100.0, 100.0 - spread_penalty - temp_penalty))
        # Ease toward the new SOH target so the card does not jump around on every frame.
        self.soh_estimate += (target_soh - self.soh_estimate) * 0.02

    def _update_summary(self, frame: TelemetryFrame) -> None:
        # Update top-row metric cards.
        pack_voltage_v = frame.pack_voltage_mv / 1000.0
        self.summary_labels["pack_voltage"].configure(text=f"{pack_voltage_v:.2f} V")
        self.summary_labels["pack_voltage_foot"].configure(text=f"STM32 timestamp {frame.timestamp_ms} ms")

        self.summary_labels["pack_current"].configure(text=f"{frame.pack_current_a:.2f} A")
        self.summary_labels["pack_current_foot"].configure(text=f"Approx. {pack_voltage_v * frame.pack_current_a:.1f} W")

        self.summary_labels["avg_cell"].configure(text=f"{frame.avg_cell_v:.3f} V")
        self.summary_labels["avg_cell_foot"].configure(text=f"Min {frame.min_cell_v:.3f} V / Max {frame.max_cell_v:.3f} V")

        self.summary_labels["imbalance"].configure(text=f"{frame.imbalance_mv} mV")
        self.summary_labels["imbalance_foot"].configure(text="Derived from highest minus lowest cell")

        self.summary_labels["soc"].configure(text=f"{self.soc_estimate:.1f} %")
        self.summary_labels["soc_foot"].configure(text="Viewer-side voltage estimate")

        self.summary_labels["soh"].configure(text=f"{self.soh_estimate:.1f} %")
        self.summary_labels["soh_foot"].configure(text="Slow health estimate based on spread and temperature")

    def _update_cells(self, frame: TelemetryFrame) -> None:
        # Push latest per-cell values into each cell card widget.
        for index, card in enumerate(self.cell_cards):
            voltage_v = frame.cell_voltage_mv[index] / 1000.0
            temp_c = frame.cell_temp_c[index]
            state_text, color = self._cell_state(voltage_v, temp_c)
            card.update_card(voltage_v, temp_c, color, state_text)

    def _update_faults(self, frame: TelemetryFrame) -> None:
        # The first status byte mirrors firmware fault bits; imbalance is derived locally
        # so we can still surface pack spread even if firmware does not flag it yet.
        status0 = frame.status_bytes[0]
        ov = bool(status0 & 0x01)
        uv = bool(status0 & 0x02)
        ot = bool(status0 & 0x04)
        ut = bool(status0 & 0x08)
        imbalance = frame.imbalance_mv >= 150

        self.fault_badges["OV"].set_state(ov, "Fault" if ov else "Normal")
        self.fault_badges["UV"].set_state(uv, "Fault" if uv else "Normal")
        self.fault_badges["OT"].set_state(ot, "Fault" if ot else "Normal")
        self.fault_badges["UT"].set_state(ut, "Fault" if ut else "Normal")
        self.fault_badges["Imbalance"].set_state(imbalance, f"{frame.imbalance_mv} mV" if imbalance else "Balanced")
        self.fault_badges["Comm"].set_state(False, "Live")

        if ov or uv or ot or ut:
            self.status_var.set("Pack Fault Active")
        else:
            self.status_var.set("Live UART Telemetry")

    def _update_plots(self) -> None:
        # Refresh line data only when plot window is open and has data.
        times = list(self.time_history)
        if not times or self.plot_canvas is None or self.voltage_ax is None or self.temp_ax is None or self.current_ax is None or self.current_line is None:
            return

        for index in range(CELL_COUNT):
            self.voltage_lines[index].set_data(times, list(self.voltage_history[index]))
            self.temp_lines[index].set_data(times, list(self.temp_history[index]))
            latest_voltage = self.voltage_history[index][-1]
            # Move the inline label with the newest point so the user can identify lines
            # without a separate legend eating up space in a small window.
            label_x = times[-1] + 0.15
            self.voltage_line_labels[index].set_position((label_x, latest_voltage))
        self.current_line.set_data(times, list(self.current_history))

        for axis in (self.voltage_ax, self.temp_ax, self.current_ax):
            axis.set_xlim(times[0], (times[-1] if times[-1] > times[0] else times[0] + 1) + 0.8)
            axis.relim()
            axis.autoscale_view(scalex=False, scaley=True)

        self.plot_canvas.draw_idle()

    def _clear_plots(self) -> None:
        # Clear existing traces after disconnect/reset.
        if self.plot_canvas is None or self.voltage_ax is None or self.temp_ax is None or self.current_ax is None or self.current_line is None:
            return

        for line in self.voltage_lines:
            line.set_data([], [])
        for line in self.temp_lines:
            line.set_data([], [])
        for label in self.voltage_line_labels:
            label.set_position((0, 0))
        self.current_line.set_data([], [])

        for axis in (self.voltage_ax, self.temp_ax, self.current_ax):
            axis.set_xlim(0, 1)
            axis.relim()
            axis.autoscale_view(scalex=False, scaley=True)

        self.plot_canvas.draw_idle()

    def _watchdog_link(self) -> None:
        # Periodically mark comm health based on time since last valid frame.
        if self.last_frame_time is None:
            self.fault_badges["Comm"].set_state(False, "Waiting")
        else:
            age = time.time() - self.last_frame_time
            # Treat a 1 second gap as a comm issue. Normal telemetry is expected roughly
            # every 200 ms, so this leaves room for jitter without hiding a dropped link.
            if age > 1.0:
                self.fault_badges["Comm"].set_state(True, "Timeout")
                if self.status_var.get() == "Live UART Telemetry":
                    self.status_var.set("Telemetry Timeout")
            else:
                self.fault_badges["Comm"].set_state(False, "Healthy")
        self.root.after(1000, self._watchdog_link)

    def _write_log(self, frame: TelemetryFrame) -> None:
        # Append one CSV row per telemetry frame.
        if not self.logging_enabled or self.log_writer is None or self.log_file is None:
            return

        row = [
            datetime.now().isoformat(timespec="milliseconds"),
            frame.timestamp_ms,
            frame.pack_voltage_mv,
            f"{frame.pack_current_a:.3f}",
        ]
        row.extend(frame.status_bytes)
        row.extend(frame.cell_voltage_mv)
        row.extend([f"{value:.2f}" for value in frame.cell_temp_c])
        self.log_writer.writerow(row)
        self.log_file.flush()

    def _cell_state(self, voltage_v: float, temp_c: float) -> tuple[str, str]:
        if voltage_v > VOLTAGE_OV or voltage_v < VOLTAGE_UV or temp_c > TEMP_OT or temp_c < TEMP_UT:
            return "Fault", BAD
        if voltage_v >= VOLTAGE_WARN_HIGH or voltage_v <= VOLTAGE_WARN_LOW or temp_c >= TEMP_WARN_HIGH or temp_c <= TEMP_WARN_LOW:
            return "Warning", WARN
        return "Normal", GOOD


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Senior design BMS UART viewer")
    parser.add_argument("--demo", action="store_true", help="Start in demo mode without a UART device")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    root = tk.Tk()
    app = BmsViewerApp(root, start_demo=args.demo)

    def on_close() -> None:
        app.disconnect_reader()
        app._close_log()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
