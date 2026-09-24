#!/usr/bin/env python3
"""PTA monitor UI for 24-channel RS485 pressure transducer arrays.

This app implements:
- Start/Stop PTA polling
- Select All / Clear All channel controls
- 24 live gauge bars with active LEDs
- Unit selection (bar, mbar, psi, psf, in H2O)
- Min/Max display range controls
- Logging modes (Snapshot, Sample, Stream)
- Manual Take Data + Save Data support

Protocol assumptions are based on observed traffic:
request:  [addr 49 01 cmd qchk]
response: [addr 49 d0 d1 d2 d3 status c0 c1]           (legacy hypothesis)
response: [addr rtype d0 d1 chk] immediately after req (observed capture)
value:    derived from observed payload bytes and baseline-tared in UI
"""

from __future__ import annotations

import csv
import math
import queue
import struct
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

import serial
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from serial.tools import list_ports


# Channel -> request bytes [addr, 0x49, 0x01, cmd, checksum]
CHANNEL_REQUESTS: Dict[int, bytes] = {
    1: bytes([0x01, 0x49, 0x01, 0x50, 0xD6]),
    2: bytes([0x02, 0x49, 0x01, 0x50, 0x26]),
    3: bytes([0x03, 0x49, 0x01, 0x90, 0x77]),
    4: bytes([0x04, 0x49, 0x01, 0x51, 0xC6]),
    5: bytes([0x05, 0x49, 0x01, 0x91, 0x97]),
    6: bytes([0x06, 0x49, 0x01, 0x91, 0x67]),
    7: bytes([0x07, 0x49, 0x01, 0x51, 0x36]),
    8: bytes([0x08, 0x49, 0x01, 0x52, 0x06]),
    9: bytes([0x09, 0x49, 0x01, 0x92, 0x57]),
    10: bytes([0x0A, 0x49, 0x01, 0x92, 0xA7]),
    11: bytes([0x0B, 0x49, 0x01, 0x52, 0xF6]),
    12: bytes([0x0C, 0x49, 0x01, 0x93, 0x47]),
    13: bytes([0x0D, 0x49, 0x01, 0x53, 0x16]),
    14: bytes([0x0E, 0x49, 0x01, 0x53, 0xE6]),
    15: bytes([0x0F, 0x49, 0x01, 0x93, 0xB7]),
    16: bytes([0x10, 0x49, 0x01, 0x55, 0x86]),
    17: bytes([0x11, 0x49, 0x01, 0x95, 0xD7]),
    18: bytes([0x12, 0x49, 0x01, 0x95, 0x27]),
    19: bytes([0x13, 0x49, 0x01, 0x55, 0x76]),
    20: bytes([0x14, 0x49, 0x01, 0x94, 0xC7]),
    21: bytes([0x15, 0x49, 0x01, 0x54, 0x96]),
    22: bytes([0x16, 0x49, 0x01, 0x54, 0x66]),
    23: bytes([0x17, 0x49, 0x01, 0x94, 0x37]),
    24: bytes([0x18, 0x49, 0x01, 0x97, 0x07]),
}

CHANNEL_CMD_BY_ADDR: Dict[int, int] = {ch: req[3] for ch, req in CHANNEL_REQUESTS.items()}

UNIT_FACTORS = {
    "bar": 1.0,
    "mbar": 1000.0,
    "psi": 14.5037738,
    "psf": 2088.54342,
    "in H2O": 401.463078,
}

# Captured from OG startup traffic. This is the full 0x30 init progression
# observed during a successful cold start.
STARTUP_INIT_FRAMES: List[bytes] = [
    bytes([0x01, 0x30, 0x34, 0x00, 0x01, 0x30, 0x05, 0x14, 0x05, 0x32, 0x0A, 0x00, 0x31, 0x26]),
    bytes([0x02, 0x30, 0xC4, 0x00, 0x02, 0x30, 0x05, 0x14, 0x05, 0x32, 0x0A, 0x00, 0x24, 0x66]),
    bytes([0x03, 0x30, 0x54, 0x01, 0x03, 0x30, 0x05, 0x14, 0x05, 0x32, 0x0A, 0x00, 0xE8, 0xA7]),
    bytes([0x04, 0x30, 0x64, 0x03, 0x04, 0x30, 0x05, 0x14, 0x05, 0x32, 0x0A, 0x00, 0x0E, 0xE6]),
    bytes([0x05, 0x30, 0xF4, 0x02, 0x05, 0x30, 0x05, 0x14, 0x05, 0x32, 0x0A, 0x00, 0xC2, 0x27]),
    bytes([0x06, 0x30, 0x04, 0x02, 0x06, 0x30, 0x05, 0x14, 0x05, 0x32, 0x0A, 0x00, 0xD7, 0x67]),
    bytes([0x07, 0x30, 0x94, 0x03, 0x07, 0x30, 0x05, 0x14, 0x05, 0x32, 0x0A, 0x00, 0x1B, 0xA6]),
    bytes([0x08, 0x30, 0x64, 0x06, 0x08, 0x30, 0x05, 0x14, 0x05, 0x32, 0x0A, 0x00, 0x5B, 0xE6]),
    bytes([0x09, 0x30, 0xF4, 0x07, 0x09, 0x30, 0x05, 0x14, 0x05, 0x32, 0x0A, 0x00, 0x97, 0x27]),
    bytes([0x0A, 0x30, 0x04, 0x07, 0x0A, 0x30, 0x05, 0x14, 0x05, 0x32, 0x0A, 0x00, 0x82, 0x67]),
    bytes([0x0B, 0x30, 0x94, 0x06, 0x0B, 0x30, 0x05, 0x14, 0x05, 0x32, 0x0A, 0x00, 0x4E, 0xA6]),
    bytes([0x0C, 0x30, 0xA4, 0x04, 0x0C, 0x30, 0x05, 0x14, 0x05, 0x32, 0x0A, 0x00, 0xA8, 0xE7]),
    bytes([0x0D, 0x30, 0x34, 0x05, 0x0D, 0x30, 0x05, 0x14, 0x05, 0x32, 0x0A, 0x00, 0x64, 0x26]),
    bytes([0x0E, 0x30, 0xC4, 0x05, 0x0E, 0x30, 0x05, 0x14, 0x05, 0x32, 0x0A, 0x00, 0x71, 0x66]),
    bytes([0x0F, 0x30, 0x54, 0x04, 0x0F, 0x30, 0x05, 0x14, 0x05, 0x32, 0x0A, 0x00, 0xBD, 0xA7]),
    bytes([0x10, 0x30, 0x64, 0x0C, 0x10, 0x30, 0x05, 0x14, 0x05, 0x32, 0x0A, 0x00, 0xF1, 0xE6]),
    bytes([0x11, 0x30, 0xF4, 0x0D, 0x11, 0x30, 0x05, 0x14, 0x05, 0x32, 0x0A, 0x00, 0x3D, 0x27]),
    bytes([0x12, 0x30, 0x04, 0x0D, 0x12, 0x30, 0x05, 0x14, 0x05, 0x32, 0x0A, 0x00, 0x28, 0x67]),
    bytes([0x13, 0x30, 0x94, 0x0C, 0x13, 0x30, 0x05, 0x14, 0x05, 0x32, 0x0A, 0x00, 0xE4, 0xA6]),
    bytes([0x14, 0x30, 0xA4, 0x0E, 0x14, 0x30, 0x05, 0x14, 0x05, 0x32, 0x0A, 0x00, 0x02, 0xE7]),
    bytes([0x15, 0x30, 0x34, 0x0F, 0x15, 0x30, 0x05, 0x14, 0x05, 0x32, 0x0A, 0x00, 0xCE, 0x26]),
    bytes([0x16, 0x30, 0xC4, 0x0F, 0x16, 0x30, 0x05, 0x14, 0x05, 0x32, 0x0A, 0x00, 0xDB, 0x66]),
]

# During init bootstrap, touch every channel so all transducers leave cold-start state.
STARTUP_BOOTSTRAP_CHANNELS: List[int] = list(range(1, 25))


@dataclass
class Sample:
    timestamp: float
    channel: int
    addr: int
    cmd: int
    raw_bar: float
    status: int
    raw_bytes_hex: str


class ChannelGauge:
    def __init__(self, parent: tk.Widget, channel: int, on_toggle_channel=None) -> None:
        self.channel = channel
        self.on_toggle_channel = on_toggle_channel
        self.frame = ttk.Frame(parent, padding=0)
        self.frame.configure(width=34, height=404)
        self.frame.pack_propagate(False)
        self.frame.grid_propagate(False)

        self.base_frame_w = 34
        self.base_bar_h = 280
        self.base_bar_w = 16
        self.base_value_w = 24
        self.base_value_h = 96
        self.base_led = 14
        self.base_title_h = 18
        self.base_gap = 4
        self.base_pad = 4

        self._current_ratio = 0.0
        self.bar_draw_top = 0
        self.bar_draw_bottom = 0

        self.title = ttk.Label(self.frame, text=f"{channel:02d}")
        self.title.place(x=0, y=0)

        self.led_canvas = tk.Canvas(self.frame, width=14, height=14, bg="#0f172a", highlightthickness=0)
        self.led = self.led_canvas.create_oval(2, 2, 12, 12, fill="#334155", outline="#1e293b")

        self.led_canvas.bind("<Button-1>", self._handle_toggle_click)
        self.title.bind("<Button-1>", self._handle_toggle_click)

        self.canvas_h = self.base_bar_h
        self.canvas_w = self.base_bar_w
        self.canvas = tk.Canvas(self.frame, width=self.canvas_w, height=self.canvas_h, bg="#0b1220", highlightthickness=1, highlightbackground="#1f2937")
        self.bar = self.canvas.create_rectangle(4, self.canvas_h - 2, self.canvas_w - 4, self.canvas_h - 2, fill="#1f9d55", outline="")

        self.value_canvas = tk.Canvas(self.frame, width=self.base_value_w, height=self.base_value_h, bg="#111827", highlightthickness=0)
        self.value_text = self.value_canvas.create_text(
            self.base_value_w // 2,
            self.base_value_h // 2,
            text="--",
            fill="#dbeafe",
            angle=90,
            anchor="center",
            font=("Consolas", 8),
        )

        self.configure_layout(1.0)

    def _format_value_text(self, value: float) -> str:
        abs_v = abs(value)
        if abs_v >= 10000:
            return f"{value:.0f}"
        if abs_v >= 1000:
            return f"{value:.1f}"
        return f"{value:.2f}"

    def _handle_toggle_click(self, _event) -> None:
        if self.on_toggle_channel:
            self.on_toggle_channel(self.channel)

    def set_led(self, enabled: bool, active: bool) -> None:
        if not enabled:
            color = "#334155"
        elif active:
            color = "#22c55e"
        else:
            color = "#f59e0b"
        self.led_canvas.itemconfig(self.led, fill=color)

    def set_value(self, value: float, vmin: float, vmax: float, unit: str) -> None:
        if vmax <= vmin:
            vmax = vmin + 1.0

        ratio = (value - vmin) / (vmax - vmin)
        ratio = max(0.0, min(1.0, ratio))
        self._current_ratio = ratio
        self._render_bar()

        if ratio < 0.6:
            color = "#2ecc71"
        elif ratio < 0.85:
            color = "#f1c40f"
        else:
            color = "#e74c3c"
        self.canvas.itemconfig(self.bar, fill=color)
        self.value_canvas.itemconfig(self.value_text, text=self._format_value_text(value))

    def set_text(self, text: str) -> None:
        self.value_canvas.itemconfig(self.value_text, text=text)

    def configure_layout(self, scale: float) -> None:
        title_h = max(14, int(self.base_title_h * scale))
        led = max(10, int(self.base_led * scale))
        gap = max(2, int(self.base_gap * scale))
        pad = max(2, int(self.base_pad * scale))
        self.canvas_h = max(150, int(self.base_bar_h * scale))
        self.canvas_w = max(10, int(self.base_bar_w * scale))
        value_h = max(52, int(self.base_value_h * scale))
        value_w = max(16, int(self.base_value_w * scale))

        frame_w = max(self.canvas_w + 12, value_w + 6, int(self.base_frame_w * scale))
        frame_h = pad + title_h + gap + led + gap + self.canvas_h + gap + value_h + pad
        self.frame.configure(width=frame_w, height=frame_h)

        self.title.configure(font=("Segoe UI", max(7, int(9 * scale))))
        self.title.place(x=0, y=pad, width=frame_w, height=title_h)

        led_x = (frame_w - led) // 2
        led_y = pad + title_h + gap
        self.led_canvas.configure(width=led, height=led)
        self.led_canvas.place(x=led_x, y=led_y, width=led, height=led)
        self.led_canvas.coords(self.led, 1, 1, max(2, led - 2), max(2, led - 2))

        bar_x = (frame_w - self.canvas_w) // 2
        bar_y = led_y + led + gap
        self.canvas.configure(width=self.canvas_w, height=self.canvas_h)
        self.canvas.place(x=bar_x, y=bar_y, width=self.canvas_w, height=self.canvas_h)

        self.bar_draw_top = bar_y + 2
        self.bar_draw_bottom = bar_y + self.canvas_h - 2

        value_x = (frame_w - value_w) // 2
        value_y = bar_y + self.canvas_h + gap
        self.value_canvas.configure(width=value_w, height=value_h)
        self.value_canvas.place(x=value_x, y=value_y, width=value_w, height=value_h)
        self.value_canvas.itemconfig(self.value_text, font=("Consolas", max(6, int(8 * scale))))
        self.value_canvas.coords(self.value_text, value_w // 2, value_h // 2)

        self._render_bar()

    def _render_bar(self) -> None:
        filled_h = int(self._current_ratio * (self.canvas_h - 4))
        y0 = self.canvas_h - 2 - filled_h
        self.canvas.coords(self.bar, 2, y0, self.canvas_w - 2, self.canvas_h - 2)


class PTAMonitorApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("PTA Monitor")
        self.root.geometry("1760x920")
        self._init_style()

        self.running = False
        self.poll_thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()
        self.serial_port: Optional[serial.Serial] = None
        self.rx_buffer = bytearray()

        self.ui_queue: "queue.Queue[Sample]" = queue.Queue()
        self.channel_latest: Dict[int, Sample] = {}
        self.channel_last_seen: Dict[int, float] = {}
        self.pending_samples: Dict[int, Sample] = {}
        self.last_accepted_raw_bar: Dict[int, float] = {}
        self.pending_step_raw_bar: Dict[int, float] = {}
        self.max_single_step_bar = 5.0
        self.step_confirm_tolerance_bar = 0.5

        self.channel_enabled: Dict[int, tk.BooleanVar] = {
            ch: tk.BooleanVar(value=False) for ch in range(1, 25)
        }
        self.tare_samples = 12

        self.baseline_bar: Dict[int, float] = {}
        self.baseline_acc: Dict[int, List[float]] = {ch: [] for ch in range(1, 25)}

        self.capture_lock = threading.Lock()
        self.captured_cycles: List[List[Sample]] = []
        self.capture_mode_active = False
        self.capture_mode_running = False
        self.sample_target_cycles = 0
        self.sample_captured_cycles = 0
        self.take_data_count = 0
        self.last_cycle_ms = 0.0
        self.last_cycle_count = 0
        self.total_samples_received = 0
        self.rx_full9_count = 0
        self.rx_short5_count = 0
        self.init_runs = 0
        self.last_init_ms = 0.0
        self.last_init_sent = 0
        self.last_init_note = "not-run"
        self.poll_start_index = 0
        self.live_layout_job: Optional[str] = None
        self.live_scale = 1.0
        self.live_base_axis_w = 56
        self.live_axis_w = self.live_base_axis_w
        self.live_bar_top = 8
        self.live_bar_bottom = 288

        self._build_ui()
        self._refresh_ports()
        self.root.bind("<Configure>", self._on_root_configure)
        self.root.after(100, self._apply_live_layout)
        self.root.after(50, self._drain_ui_queue)

    def _init_style(self) -> None:
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        bg = "#0f172a"
        panel = "#111827"
        fg = "#dbeafe"
        muted = "#93c5fd"
        btn = "#1d4ed8"

        self.root.configure(bg=bg)
        style.configure("TFrame", background=bg)
        style.configure("Panel.TFrame", background=panel)
        style.configure("TLabel", background=bg, foreground=fg, font=("Segoe UI", 9))
        style.configure("Header.TLabel", background=bg, foreground=muted, font=("Segoe UI Semibold", 9))
        style.configure("TCheckbutton", background=bg, foreground=fg)
        style.configure("TButton", background=btn, foreground="white", padding=6)
        style.map("TButton", background=[("active", "#2563eb")])
        style.configure("TEntry", fieldbackground="#0b1220", foreground=fg)
        style.configure("TCombobox", fieldbackground="#0b1220", foreground=fg, arrowcolor=fg)
        style.map(
            "TCombobox",
            fieldbackground=[("readonly", "#0b1220")],
            background=[("readonly", "#0b1220")],
            foreground=[("readonly", "#e2e8f0")],
            selectbackground=[("readonly", "#1d4ed8")],
            selectforeground=[("readonly", "#f8fafc")],
        )
        self.root.option_add("*TCombobox*Listbox.background", "#0b1220")
        self.root.option_add("*TCombobox*Listbox.foreground", "#e2e8f0")
        self.root.option_add("*TCombobox*Listbox.selectBackground", "#2563eb")
        self.root.option_add("*TCombobox*Listbox.selectForeground", "#f8fafc")

    def _build_ui(self) -> None:
        container = ttk.Frame(self.root, padding=10)
        container.pack(fill=tk.BOTH, expand=True)

        sidebar = ttk.Frame(container, style="Panel.TFrame", padding=(10, 10, 10, 10), width=205)
        sidebar.pack_propagate(False)
        sidebar.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))

        ttk.Label(sidebar, text="PTA Monitor", style="Header.TLabel").pack(anchor="w", pady=(0, 10))

        ttk.Label(sidebar, text="Port").pack(anchor="w")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(sidebar, width=14, textvariable=self.port_var, state="readonly")
        self.port_combo.pack(anchor="w", fill=tk.X, pady=(4, 8))

        ttk.Button(sidebar, text="Refresh Ports", command=self._refresh_ports).pack(anchor="w", fill=tk.X, pady=(0, 8))
        ttk.Button(sidebar, text="Start PTA", command=self.start_pta).pack(anchor="w", fill=tk.X, pady=(0, 6))
        ttk.Button(sidebar, text="Stop PTA", command=self.stop_pta).pack(anchor="w", fill=tk.X, pady=(0, 10))

        ttk.Label(sidebar, text="Measured").pack(anchor="w", pady=(6, 0))
        self.measured_var = tk.StringVar(value="--")
        self.measured_label = ttk.Label(
            sidebar,
            textvariable=self.measured_var,
            width=18,
            anchor="w",
            font=("Consolas", 9),
        )
        self.measured_label.pack(anchor="w", pady=(2, 0), fill=tk.X)

        main = ttk.Frame(container)
        main.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        controls_top = ttk.Frame(main, padding=(0, 0, 0, 8))
        controls_top.pack(fill=tk.X)

        ttk.Button(controls_top, text="Select All", command=self.select_all_channels).pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(controls_top, text="Clear All", command=self.clear_all_channels).pack(side=tk.LEFT, padx=(0, 10))

        ttk.Label(controls_top, text="Unit:").pack(side=tk.LEFT)
        self.unit_var = tk.StringVar(value="mbar")
        self.unit_combo = ttk.Combobox(controls_top, width=8, textvariable=self.unit_var, state="readonly")
        self.unit_combo["values"] = list(UNIT_FACTORS.keys())
        self.unit_combo.pack(side=tk.LEFT, padx=(4, 10))

        ttk.Label(controls_top, text="Min:").pack(side=tk.LEFT)
        self.min_var = tk.StringVar(value="-50")
        tk.Entry(
            controls_top,
            width=8,
            textvariable=self.min_var,
            bg="#0b1220",
            fg="#e2e8f0",
            insertbackground="#f8fafc",
            insertwidth=2,
            relief="flat",
        ).pack(side=tk.LEFT, padx=(4, 8))

        ttk.Label(controls_top, text="Max:").pack(side=tk.LEFT)
        self.max_var = tk.StringVar(value="200")
        tk.Entry(
            controls_top,
            width=8,
            textvariable=self.max_var,
            bg="#0b1220",
            fg="#e2e8f0",
            insertbackground="#f8fafc",
            insertwidth=2,
            relief="flat",
        ).pack(side=tk.LEFT, padx=(4, 8))

        ttk.Label(controls_top, text="Cycle ms (0=continuous):").pack(side=tk.LEFT)
        self.cycle_ms_var = tk.StringVar(value="200")
        tk.Entry(
            controls_top,
            width=7,
            textvariable=self.cycle_ms_var,
            bg="#0b1220",
            fg="#e2e8f0",
            insertbackground="#f8fafc",
            insertwidth=2,
            relief="flat",
        ).pack(side=tk.LEFT, padx=(4, 8))

        gauges_panel = ttk.Frame(main, style="Panel.TFrame")
        gauges_panel.pack(fill=tk.BOTH, expand=True)
        gauges_panel.grid_rowconfigure(1, weight=1)
        gauges_panel.grid_columnconfigure(0, weight=1)

        ttk.Label(
            gauges_panel,
            text="Click each LED (or channel number) to toggle channel on/off",
            style="Header.TLabel",
        ).grid(row=0, column=0, columnspan=24, pady=(6, 0))

        bars_holder = ttk.Frame(gauges_panel, style="Panel.TFrame")
        bars_holder.grid(row=1, column=0, columnspan=24, sticky="nsew", pady=(2, 0))
        bars_holder.grid_rowconfigure(0, weight=1)
        bars_holder.grid_columnconfigure(1, weight=1)
        self.bars_holder = bars_holder

        self.axis_canvas = tk.Canvas(
            bars_holder,
            width=56,
            height=404,
            bg="#111827",
            highlightthickness=0,
        )
        self.axis_canvas.grid(row=0, column=0, sticky="ns", padx=(6, 4))

        self.axis_tick_lines = []
        self.axis_tick_text = []
        for _ in range(5):
            line = self.axis_canvas.create_line(44, 0, 54, 0, fill="#6b7280")
            text = self.axis_canvas.create_text(40, 0, text="", fill="#cbd5e1", anchor="e", font=("Segoe UI", 8))
            self.axis_tick_lines.append(line)
            self.axis_tick_text.append(text)

        bars_frame = ttk.Frame(bars_holder, style="Panel.TFrame")
        bars_frame.grid(row=0, column=1, sticky="nsew")
        self.bars_frame = bars_frame

        self.gauges: Dict[int, ChannelGauge] = {}
        for idx, ch in enumerate(range(1, 25)):
            g = ChannelGauge(bars_frame, ch, on_toggle_channel=self._toggle_channel_from_led)
            g.frame.grid(row=0, column=idx, padx=1, pady=0, sticky="n")
            self.gauges[ch] = g

        logging_panel = ttk.Frame(main, style="Panel.TFrame", padding=(8, 8, 8, 8))
        logging_panel.pack(fill=tk.X, pady=(8, 0))

        ttk.Label(logging_panel, text="Log Mode:").pack(side=tk.LEFT)
        self.log_mode_var = tk.StringVar(value="Stream")
        self.log_mode_combo = ttk.Combobox(logging_panel, width=9, textvariable=self.log_mode_var, state="readonly")
        self.log_mode_combo["values"] = ["Snapshot", "Sample", "Stream"]
        self.log_mode_combo.pack(side=tk.LEFT, padx=(4, 8))
        self.log_mode_combo.bind("<<ComboboxSelected>>", self._on_log_mode_changed)

        ttk.Label(logging_panel, text="Sample Count:").pack(side=tk.LEFT)
        self.sample_count_var = tk.StringVar(value="100")
        self.sample_count_entry = tk.Entry(
            logging_panel,
            width=6,
            textvariable=self.sample_count_var,
            bg="#0b1220",
            fg="#e2e8f0",
            insertbackground="#f8fafc",
            insertwidth=2,
            relief="flat",
        )
        self.sample_count_entry.pack(side=tk.LEFT, padx=(4, 10))

        self.log_dir_var = tk.StringVar(value=str(Path.cwd()))
        ttk.Entry(logging_panel, width=34, textvariable=self.log_dir_var).pack(side=tk.LEFT, padx=(0, 4))
        ttk.Button(logging_panel, text="Folder", command=self.browse_log_folder).pack(side=tk.LEFT)
        ttk.Button(logging_panel, text="Take Data", command=self.take_data).pack(side=tk.LEFT, padx=(12, 4))
        ttk.Button(logging_panel, text="Save Data", command=self.save_data).pack(side=tk.LEFT, padx=4)
        ttk.Button(logging_panel, text="Clear Data", command=self.clear_data).pack(side=tk.LEFT, padx=4)
        self.take_data_label = ttk.Label(logging_panel, text="Captured: 0", font=("Segoe UI Semibold", 12))
        self.take_data_label.pack(side=tk.LEFT, padx=(8, 0))
        self._sync_sample_count_state()

        bottom = ttk.Frame(self.root, padding=(8, 4, 8, 8))
        bottom.pack(fill=tk.X)
        self.status_var = tk.StringVar(value="Idle")
        ttk.Label(bottom, textvariable=self.status_var).pack(side=tk.LEFT)

    def _refresh_ports(self) -> None:
        ports = [p.device for p in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def browse_log_folder(self) -> None:
        path = filedialog.askdirectory(title="Select log output folder")
        if path:
            self.log_dir_var.set(path)

    def _on_log_mode_changed(self, _event=None) -> None:
        self._sync_sample_count_state()

    def _sync_sample_count_state(self) -> None:
        is_sample_mode = self.log_mode_var.get() == "Sample"
        if is_sample_mode:
            self.sample_count_entry.config(
                state="normal",
                bg="#0b1220",
                fg="#e2e8f0",
                disabledbackground="#0b1220",
                disabledforeground="#e2e8f0",
                insertbackground="#f8fafc",
            )
        else:
            self.sample_count_entry.config(
                state="disabled",
                disabledbackground="#1f2937",
                disabledforeground="#94a3b8",
                insertbackground="#94a3b8",
            )

    def select_all_channels(self) -> None:
        for ch, var in self.channel_enabled.items():
            was_enabled = var.get()
            var.set(True)
            if not was_enabled:
                self._reset_channel_tare(ch)

    def clear_all_channels(self) -> None:
        for var in self.channel_enabled.values():
            var.set(False)

    def _reset_channel_tare(self, channel: int) -> None:
        self.baseline_acc[channel] = []
        if channel in self.baseline_bar:
            del self.baseline_bar[channel]

    def _toggle_channel_from_led(self, channel: int) -> None:
        var = self.channel_enabled[channel]
        new_state = not var.get()
        var.set(new_state)
        if new_state:
            self._reset_channel_tare(channel)

    def _update_axis_labels(self, vmin: float, vmax: float, unit: str) -> None:
        if vmax <= vmin:
            vmax = vmin + 1.0

        top_y = self.live_bar_top
        bot_y = self.live_bar_bottom
        span = bot_y - top_y
        for i in range(5):
            frac = i / 4.0
            y = bot_y - frac * span
            val = vmin + frac * (vmax - vmin)
            line_x2 = self.live_axis_w - 2
            line_x1 = self.live_axis_w - 12
            text_x = self.live_axis_w - 16
            self.axis_canvas.coords(self.axis_tick_lines[i], line_x1, y, line_x2, y)
            self.axis_canvas.coords(self.axis_tick_text[i], text_x, y)
            self.axis_canvas.itemconfig(self.axis_tick_text[i], text=f"{val:.0f}")

        self.axis_canvas.delete("axis_unit")
        self.axis_canvas.create_text(
            12,
            (top_y + bot_y) / 2,
            text=unit,
            fill="#93c5fd",
            angle=90,
            tags="axis_unit",
            font=("Segoe UI Semibold", max(7, int(8 * self.live_scale))),
        )

    def _on_root_configure(self, event) -> None:
        if event.widget is not self.root:
            return
        if self.live_layout_job is not None:
            try:
                self.root.after_cancel(self.live_layout_job)
            except Exception:
                pass
        self.live_layout_job = self.root.after(60, self._apply_live_layout)

    def _apply_live_layout(self) -> None:
        self.live_layout_job = None
        if not hasattr(self, "bars_holder"):
            return

        holder_w = max(1, self.bars_holder.winfo_width())
        holder_h = max(1, self.bars_holder.winfo_height())
        if holder_w <= 1 or holder_h <= 1:
            return

        base_gauge_w = 34
        base_gauge_h = 404
        base_axis_w = self.live_base_axis_w
        gauge_gap = 2

        base_total_w = (24 * base_gauge_w) + (24 * gauge_gap) + base_axis_w + 10
        base_total_h = base_gauge_h

        scale_w = holder_w / max(1, base_total_w)
        scale_h = holder_h / max(1, base_total_h)
        scale = max(0.58, min(1.8, min(scale_w, scale_h)))
        self.live_scale = scale

        for gauge in self.gauges.values():
            gauge.configure_layout(scale)

        gauge_h = max(g.frame.winfo_height() for g in self.gauges.values())
        self.live_axis_w = max(44, int(base_axis_w * scale))
        self.axis_canvas.configure(width=self.live_axis_w, height=gauge_h)

        # Axis labels must align with the actual bar drawable region.
        ref_gauge = self.gauges[1]
        self.live_bar_top = ref_gauge.bar_draw_top
        self.live_bar_bottom = ref_gauge.bar_draw_bottom

    def start_pta(self) -> None:
        if self.running:
            return

        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("No Port", "Select a serial port first.")
            return

        try:
            self.serial_port = serial.Serial(
                port=port,
                baudrate=115200,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0,
                write_timeout=0.05,
            )
        except serial.SerialException as exc:
            messagebox.showerror("Serial Error", f"Could not open {port}: {exc}")
            return

        self.status_var.set("Initializing PTA...")
        self.root.update_idletasks()

        # One-shot OG-style init before polling; no retry loop.
        self._run_startup_init()

        # Start PTA selects all channels by default, matching legacy workflow.
        self.select_all_channels()

        self.running = True
        self.stop_event.clear()
        self.rx_buffer.clear()
        self.pending_samples.clear()
        self.last_accepted_raw_bar.clear()
        self.pending_step_raw_bar.clear()
        self.baseline_bar.clear()
        self.baseline_acc = {ch: [] for ch in range(1, 25)}
        self.rx_full9_count = 0
        self.rx_short5_count = 0

        self.poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self.poll_thread.start()
        self.status_var.set(
            f"Running | init {self.last_init_ms:.0f} ms, sent={self.last_init_sent}, {self.last_init_note}"
        )

    def _run_startup_init(self) -> None:
        if self.serial_port is None:
            return

        ser = self.serial_port
        t0 = time.time()
        sent = 0
        note = "ok"
        self.init_runs += 1
        try:
            try:
                ser.reset_input_buffer()
                ser.reset_output_buffer()
            except Exception:
                pass

            rr_idx = 0
            for frame in STARTUP_INIT_FRAMES:
                # Captures show this as two adjacent 0x30 transactions.
                ser.write(frame[:4])
                sent += 1
                time.sleep(0.002)
                ser.write(frame[4:])
                sent += 1

                # While init is active, keep live 0x49 traffic flowing.
                ch = STARTUP_BOOTSTRAP_CHANNELS[rr_idx % len(STARTUP_BOOTSTRAP_CHANNELS)]
                rr_idx += 1
                ser.write(CHANNEL_REQUESTS[ch])
                sent += 1
                time.sleep(0.003)
                ser.read(64)

            # Bootstrap all channels after init frames so cold-start state clears
            # consistently before the regular poll loop starts.
            for _ in range(2):
                for ch in STARTUP_BOOTSTRAP_CHANNELS:
                    req = CHANNEL_REQUESTS[ch]
                    ser.write(req)
                    sent += 1
                    time.sleep(0.002)

            # Drain startup chatter so polling parser begins from a clean boundary.
            t_end = time.time() + 0.12
            while time.time() < t_end:
                ser.read(256)
        except serial.SerialTimeoutException:
            note = "write-timeout"
        except serial.SerialException:
            note = "serial-error"
        finally:
            self.last_init_ms = (time.time() - t0) * 1000.0
            self.last_init_sent = sent
            self.last_init_note = note
            print(
                f"[init] run={self.init_runs} ms={self.last_init_ms:.1f} sent={self.last_init_sent} note={self.last_init_note}",
                flush=True,
            )

    def stop_pta(self) -> None:
        self.running = False
        self.stop_event.set()
        if self.poll_thread and self.poll_thread.is_alive():
            self.poll_thread.join(timeout=1.0)
        self.poll_thread = None

        if self.serial_port:
            try:
                self.serial_port.close()
            except Exception:
                pass
            self.serial_port = None

        self.status_var.set("Stopped")

    def _default_filename(self) -> str:
        now = datetime.now()
        month = now.strftime("%b").lower()
        return f"PTA_{month}_{now.day}_{now.hour:02d}_{now.minute:02d}_{now.second:02d}.csv"

    def take_data(self) -> None:
        mode = self.log_mode_var.get()

        if mode == "Snapshot":
            snapshot: List[Sample] = []
            for ch in range(1, 25):
                if self.channel_enabled[ch].get() and ch in self.channel_latest:
                    snapshot.append(self.channel_latest[ch])
            if not snapshot:
                return
            with self.capture_lock:
                self.captured_cycles.append(snapshot)
                self.take_data_count += 1
            self.take_data_label.config(text=f"Captured: {self.take_data_count}")
            self.capture_mode_active = False
            self.capture_mode_running = False
            self.status_var.set("Snapshot captured")
            return

        if mode == "Sample":
            try:
                target = max(1, int(self.sample_count_var.get()))
            except ValueError:
                target = 100
                self.sample_count_var.set("100")
            self.sample_target_cycles = target
            self.sample_captured_cycles = 0
            self.capture_mode_active = True
            self.capture_mode_running = True
            self.status_var.set(f"Sample capture running ({target} cycles target)")
            return

        # Stream mode: Take Data toggles start/stop.
        if self.capture_mode_running:
            self.capture_mode_running = False
            self.capture_mode_active = False
            self.status_var.set("Stream capture stopped")
        else:
            self.capture_mode_active = True
            self.capture_mode_running = True
            self.status_var.set("Stream capture running")

    def save_data(self) -> None:
        with self.capture_lock:
            rows = list(self.captured_cycles)

        if not rows:
            messagebox.showinfo("No Data", "No captured data available yet.")
            return

        out_dir = Path(self.log_dir_var.get().strip())
        if not str(out_dir):
            out_dir = Path.cwd()
        out_dir.mkdir(parents=True, exist_ok=True)
        out = out_dir / self._default_filename()
        out.parent.mkdir(parents=True, exist_ok=True)

        unit = self.unit_var.get()
        factor = UNIT_FACTORS.get(unit, 1.0)
        if unit == "bar":
            value_fmt = "{:.6f}"
        elif unit == "psi":
            value_fmt = "{:.4f}"
        else:
            value_fmt = "{:.3f}"

        with out.open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            header = ["timestamp"] + [f"P{ch:02d} ({unit})" for ch in range(1, 25)]
            w.writerow(header)

            for snap in rows:
                by_ch = {s.channel: s for s in snap}
                ts = ""
                if snap:
                    ts = datetime.fromtimestamp(snap[0].timestamp).isoformat(timespec="milliseconds")

                row = [ts]
                for ch in range(1, 25):
                    s = by_ch.get(ch)
                    if s is None:
                        row.append("")
                        continue
                    tared_bar = s.raw_bar - self.baseline_bar.get(ch, 0.0)
                    row.append(value_fmt.format(tared_bar * factor))
                w.writerow(row)

        with self.capture_lock:
            self.captured_cycles.clear()
            self.take_data_count = 0
        self.take_data_label.config(text="Captured: 0")
        self.capture_mode_active = False
        self.capture_mode_running = False

        messagebox.showinfo("Saved", f"Saved {len(rows)} captures to:\n{out}")

    def clear_data(self) -> None:
        with self.capture_lock:
            self.captured_cycles.clear()
            self.take_data_count = 0
        self.take_data_label.config(text="Captured: 0")
        self.capture_mode_active = False
        self.capture_mode_running = False
        self.status_var.set("Captured data cleared")

    def _poll_loop(self) -> None:
        assert self.serial_port is not None
        ser = self.serial_port

        while not self.stop_event.is_set():
            cycle_start = time.time()
            enabled_channels = [ch for ch, var in self.channel_enabled.items() if var.get()]
            if not enabled_channels:
                time.sleep(0.05)
                continue

            ordered_channels = enabled_channels

            cycle_samples: List[Sample] = []
            cycle_seen: set[int] = set()
            cycle_timestamp = cycle_start

            for ch in ordered_channels:
                req = CHANNEL_REQUESTS[ch]
                try:
                    ser.write(req)
                except serial.SerialException:
                    self.stop_event.set()
                    break
                sample = self._read_one_response(ch, req[3], timeout_s=0.0035)
                if sample is None:
                    continue
                if not self._accept_sample(sample):
                    continue

                # Keep one timestamp per cycle to mirror legacy grouped sampling.
                sample.timestamp = cycle_timestamp
                cycle_samples.append(sample)
                cycle_seen.add(sample.channel)

                self.ui_queue.put(sample)

            # Position-independent recovery: retry only whichever channels are
            # still missing, in bounded passes with slightly longer timeouts.
            for timeout_s in (0.0060, 0.0090):
                still_missing = [ch for ch in ordered_channels if ch not in cycle_seen]
                if not still_missing:
                    break

                for ch in still_missing:
                    req = CHANNEL_REQUESTS[ch]
                    try:
                        ser.write(req)
                    except serial.SerialException:
                        self.stop_event.set()
                        break

                    sample = self._read_one_response(ch, req[3], timeout_s=timeout_s)
                    if sample is None:
                        continue
                    if not self._accept_sample(sample):
                        continue

                    sample.timestamp = cycle_timestamp
                    cycle_samples.append(sample)
                    cycle_seen.add(sample.channel)
                    self.ui_queue.put(sample)

            self.last_cycle_count = len(cycle_samples)

            if self.capture_mode_running and cycle_samples:
                mode = self.log_mode_var.get()
                with self.capture_lock:
                    self.captured_cycles.append(list(cycle_samples))
                    self.take_data_count += 1

                if mode == "Sample":
                    self.sample_captured_cycles += 1
                    if self.sample_captured_cycles >= self.sample_target_cycles:
                        self.capture_mode_running = False
                        self.capture_mode_active = False
                        self.status_var.set(f"Sample capture complete ({self.sample_target_cycles} cycles)")

            elapsed = time.time() - cycle_start

            try:
                target_cycle_s = max(0.0, float(self.cycle_ms_var.get()) / 1000.0)
            except ValueError:
                target_cycle_s = 0.0

            # Continuous polling by default. A positive cycle ms value applies
            # optional pacing when explicitly requested from the UI.
            if target_cycle_s > 0.0:
                sleep_s = target_cycle_s - elapsed
                if sleep_s > 0:
                    time.sleep(sleep_s)

            # Measure effective cycle period including pacing sleep.
            total_cycle_s = time.time() - cycle_start
            self.last_cycle_ms = total_cycle_s * 1000.0

    def _accept_sample(self, sample: Sample) -> bool:
        raw = sample.raw_bar

        # Hard reject impossible float results from framing glitches.
        if not math.isfinite(raw) or abs(raw) > 10000.0:
            return False

        ch = sample.channel
        prev = self.last_accepted_raw_bar.get(ch)
        if prev is None:
            self.last_accepted_raw_bar[ch] = raw
            self.pending_step_raw_bar.pop(ch, None)
            return True

        delta = abs(raw - prev)
        if delta <= self.max_single_step_bar:
            self.last_accepted_raw_bar[ch] = raw
            self.pending_step_raw_bar.pop(ch, None)
            return True

        # Require one repeat for large step changes to avoid one-off spikes.
        pending = self.pending_step_raw_bar.get(ch)
        self.pending_step_raw_bar[ch] = raw
        if pending is not None and abs(raw - pending) <= self.step_confirm_tolerance_bar:
            self.last_accepted_raw_bar[ch] = raw
            self.pending_step_raw_bar.pop(ch, None)
            return True

        return False

    def _read_one_response(self, channel: int, cmd: int, timeout_s: float) -> Optional[Sample]:
        assert self.serial_port is not None
        ser = self.serial_port

        cached = self.pending_samples.pop(channel, None)
        if cached is not None and (time.time() - cached.timestamp) <= 0.8:
            return cached

        deadline = time.time() + timeout_s

        while time.time() < deadline and not self.stop_event.is_set():
            chunk = ser.read(128)
            if chunk:
                self.rx_buffer.extend(chunk)

            # Drain all decodable samples; return the one that matches the
            # request we just sent, but keep other channels updated too.
            while True:
                sample = self._extract_sample_from_buffer()
                if sample is None:
                    break
                if sample.channel == channel:
                    return sample
                self.pending_samples[sample.channel] = sample
                self.ui_queue.put(sample)

            # Non-blocking serial reads can otherwise spin hot when no data yet.
            if not chunk:
                time.sleep(0.0005)

            self._consume_non_measurement_records()

            # Keep buffer bounded in case of noise.
            if len(self.rx_buffer) > 4096:
                del self.rx_buffer[:2048]

        return self.pending_samples.pop(channel, None)

    def _extract_sample_from_buffer(self) -> Optional[Sample]:
        buf = self.rx_buffer
        n = len(buf)

        # Preferred shape observed after successful OG init:
        # [addr,49,01,cmd,chk, addr,49,d0,d1,d2,d3,status,c0,c1]
        for i in range(max(0, n - 13)):
            addr = buf[i] & 0x7F
            if addr < 1 or addr > 24:
                continue
            if (buf[i + 1] & 0x7F) != 0x49 or (buf[i + 2] & 0x7F) != 0x01:
                continue
            if (buf[i + 5] & 0x7F) != addr or (buf[i + 6] & 0x7F) != 0x49:
                continue

            if i > 0:
                del buf[:i]

            rec = bytes(buf[:14])
            del buf[:14]

            cmd = rec[3] & 0x7F
            raw_bar = struct.unpack(">f", rec[7:11])[0]
            status = rec[11]
            raw_hex = " ".join(f"{b:02X}" for b in rec)
            self.rx_full9_count += 1
            return Sample(
                timestamp=time.time(),
                channel=addr,
                addr=addr,
                cmd=cmd,
                raw_bar=raw_bar,
                status=status,
                raw_bytes_hex=raw_hex,
            )

        # Fallback standalone full response shape:
        # [addr,49,d0,d1,d2,d3,status,c0,c1]
        for i in range(max(0, n - 8)):
            addr = buf[i] & 0x7F
            if addr < 1 or addr > 24:
                continue
            if (buf[i + 1] & 0x7F) != 0x49 or (buf[i + 2] & 0x7F) == 0x01:
                continue

            if i > 0:
                del buf[:i]

            rec = bytes(buf[:9])
            del buf[:9]

            raw_bar = struct.unpack(">f", rec[2:6])[0]
            status = rec[6]
            raw_hex = " ".join(f"{b:02X}" for b in rec)
            self.rx_full9_count += 1
            return Sample(
                timestamp=time.time(),
                channel=addr,
                addr=addr,
                cmd=CHANNEL_CMD_BY_ADDR.get(addr, 0),
                raw_bar=raw_bar,
                status=status,
                raw_bytes_hex=raw_hex,
            )

        return None

    def _consume_non_measurement_records(self) -> None:
        buf = self.rx_buffer

        # Consume request+short-reply startup records:
        # [addr,49,01,cmd,chk, addr,C9,xx,xx,chk]
        while len(buf) >= 10:
            addr = buf[0] & 0x7F
            if 1 <= addr <= 24 and (buf[1] & 0x7F) == 0x49 and (buf[2] & 0x7F) == 0x01 and (buf[5] & 0x7F) == addr and (buf[6] & 0x7F) == 0x49:
                self.rx_short5_count += 1
                del buf[:10]
                continue
            break

    def _find_record_start(self, addr: int) -> Optional[int]:
        buf = self.rx_buffer
        max_i = len(buf) - 2
        for i in range(max_i + 1):
            if (buf[i] & 0x7F) == addr and (buf[i + 1] & 0x7F) == 0x49:
                return i
        return None

    def _drain_ui_queue(self) -> None:
        now = time.time()

        while True:
            try:
                sample = self.ui_queue.get_nowait()
            except queue.Empty:
                break

            self.channel_latest[sample.channel] = sample
            self.channel_last_seen[sample.channel] = sample.timestamp
            self.total_samples_received += 1

            acc = self.baseline_acc[sample.channel]
            if self.tare_samples > 0:
                if len(acc) == 0 and sample.channel not in self.baseline_bar:
                    # Provisional tare: apply immediately on first sample.
                    self.baseline_bar[sample.channel] = sample.raw_bar

                if len(acc) < self.tare_samples:
                    acc.append(sample.raw_bar)
                    if len(acc) == self.tare_samples:
                        self.baseline_bar[sample.channel] = sum(acc) / len(acc)

        unit = self.unit_var.get()
        factor = UNIT_FACTORS.get(unit, 1.0)

        try:
            vmin = float(self.min_var.get())
            vmax = float(self.max_var.get())
        except ValueError:
            vmin, vmax = -50.0, 200.0

        self._update_axis_labels(vmin, vmax, unit)

        dynamic_active_window_s = max(1.0, (self.last_cycle_ms / 1000.0) * 3.0)

        for ch, gauge in self.gauges.items():
            last = self.channel_latest.get(ch)
            enabled = self.channel_enabled[ch].get()
            active = self.running and enabled and (now - self.channel_last_seen.get(ch, 0) < dynamic_active_window_s)
            gauge.set_led(enabled=enabled, active=active)

            if last is None:
                gauge.set_text("--")
                continue

            baseline = self.baseline_bar.get(ch, 0.0)
            tared_bar = last.raw_bar - baseline
            disp_value = tared_bar * factor
            gauge.set_value(disp_value, vmin, vmax, unit)

        if self.last_cycle_count > 0:
            hz = 1000.0 / max(1.0, self.last_cycle_ms)
            self.measured_var.set(f"{self.last_cycle_count}ch, {self.last_cycle_ms:.0f}ms ({hz:.1f}Hz)")
        else:
            self.measured_var.set("--")

        self.take_data_label.config(text=f"Captured: {self.take_data_count}")

        if self.running and self.total_samples_received == 0:
            self.status_var.set(
                f"Running (waiting) | init {self.last_init_ms:.0f} ms, sent={self.last_init_sent}, "
                f"full9={self.rx_full9_count}, short5={self.rx_short5_count}, {self.last_init_note}"
            )
        elif self.running and self.capture_mode_running:
            mode = self.log_mode_var.get().lower()
            self.status_var.set(f"Running + {mode} capture")
        elif self.running:
            self.status_var.set(
                f"Running | full9={self.rx_full9_count}, short5={self.rx_short5_count}, "
                f"init {self.last_init_ms:.0f} ms"
            )

        self.root.after(50, self._drain_ui_queue)

    def shutdown(self) -> None:
        self.stop_pta()
        self.capture_mode_active = False
        self.capture_mode_running = False


def main() -> None:
    root = tk.Tk()
    root.state("zoomed")
    app = PTAMonitorApp(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (app.shutdown(), root.destroy()))
    root.mainloop()


if __name__ == "__main__":
    main()
