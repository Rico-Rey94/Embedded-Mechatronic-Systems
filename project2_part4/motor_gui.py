"""
Motor Controller GUI
"""

import os
import sys
import tkinter as tk
import customtkinter as ctk
import serial
import serial.tools.list_ports
import threading
import time
import csv
from datetime import datetime
from collections import deque
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

BAUD_RATE = 115200
PLOT_HISTORY_SECONDS = 30
TELEMETRY_RATE_HZ = 20
PLOT_UPDATE_MS = 100
TABLE_MAX_ROWS = 30
PID_DEBOUNCE_MS = 300

SCRIPT_DIR = os.path.dirname(os.path.abspath(sys.argv[0]))
DEFAULT_LOG_DIR = os.path.join(SCRIPT_DIR, "Logging")

MODES = {
    1: {"name": "Part 1 - Basic Motor Control",
        "desc": "Open-loop. Direct PWM, manual direction. Verify motor + encoder.",
        "color": "#3a8c3a"},
    2: {"name": "Part 2 - RPM Measurement",
        "desc": "Open-loop. Encoder reports measured RPM at fixed PWM.",
        "color": "#3a6c8c"},
    3: {"name": "Part 3 - Closed-Loop PID",
        "desc": "Setpoint RPM + Kp/Ki/Kd. Arduino runs PID, adjusts PWM automatically.",
        "color": "#c08020"},
    4: {"name": "Part 4 - Sensor-Adaptive",
        "desc": "Modulino A=Temp, B=LDR, C=Obstacle. Selected sensor drives the motor.",
        "color": "#8c3a8c"},
}

SENSOR_COLORS = {
    "temp": "#ff6464",
    "ldr":  "#64ff96",
    "ir":   "#6496ff",
}


class MotorControlApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("Motor Control Dashboard - MHEN 5373")
        self.geometry("1600x980")
        self.minsize(1400, 820)

        os.makedirs(DEFAULT_LOG_DIR, exist_ok=True)
        self.log_dir = DEFAULT_LOG_DIR

        self.serial_port = None
        self.serial_thread = None
        self.serial_running = False

        max_points = PLOT_HISTORY_SECONDS * TELEMETRY_RATE_HZ
        self.time_buffer = deque(maxlen=max_points)
        self.rpm_buffer = deque(maxlen=max_points)
        self.setpoint_buffer = deque(maxlen=max_points)
        self.pwm_buffer = deque(maxlen=max_points)
        self.temp_buffer = deque(maxlen=max_points)
        self.ldr_buffer = deque(maxlen=max_points)
        self.ir_buffer = deque(maxlen=max_points)

        self.table_rows = deque(maxlen=TABLE_MAX_ROWS)
        self.last_table_count = 0

        # Latest telemetry
        self.latest_rpm = 0.0
        self.latest_count = 0
        self.latest_pwm = 0
        self.latest_direction = "STOPPED"
        self.latest_setpoint = 0.0
        self.latest_mode = 1
        self.latest_error = 0.0
        self.latest_temp_f = 0.0
        self.latest_humidity = 0.0
        self.latest_dht_status = "INIT"
        self.latest_ldr = 0
        self.latest_ir_raw = 1
        self.latest_obstacle = False
        self.latest_selected_sensor = -1
        self.latest_logic_enabled = False
        self.start_time = time.time()
        self.current_part = 1
        self.pid_running = False
        self.pid_dir_forward = True
        self.part4_dir_forward = True

        self.auto_cycle_running = False
        self.auto_cycle_thread = None

        self._pid_after_id = None
        self._part4_after_id = None

        self.csv_file = None
        self.csv_writer = None

        self._build_ui()
        self.after(PLOT_UPDATE_MS, self._update_plot)
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self._set_part(1)

    # ====================================================
    def _build_ui(self):
        self.grid_columnconfigure(0, weight=0, minsize=380)
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)
        self._build_left_panel()
        self._build_right_panel()

    # ====================================================
    # LEFT PANEL
    # ====================================================
    def _build_left_panel(self):
        self.left = ctk.CTkScrollableFrame(self, corner_radius=0)
        self.left.grid(row=0, column=0, sticky="nsew")
        self.left.grid_columnconfigure(0, weight=1)

        # --- Connection ---
        conn = ctk.CTkFrame(self.left)
        conn.grid(row=0, column=0, sticky="ew", padx=10, pady=(10, 5))
        conn.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(conn, text="Connection",
                     font=ctk.CTkFont(size=16, weight="bold")).grid(
                     row=0, column=0, columnspan=2, pady=(8, 4))
        self.port_dropdown = ctk.CTkComboBox(conn, values=self._get_ports())
        self.port_dropdown.grid(row=1, column=0, sticky="ew", padx=8, pady=4)
        ctk.CTkButton(conn, text="R", width=32,
                      command=self._refresh_ports).grid(row=1, column=1, padx=(0, 8))
        self.connect_btn = ctk.CTkButton(conn, text="Connect",
                                          command=self._toggle_connection,
                                          fg_color="#1f6f3f", hover_color="#155028")
        self.connect_btn.grid(row=2, column=0, columnspan=2, sticky="ew",
                              padx=8, pady=(4, 8))
        self.connection_status = ctk.CTkLabel(conn, text="* Disconnected",
                                               text_color="#888")
        self.connection_status.grid(row=3, column=0, columnspan=2, pady=(0, 8))

        # --- Calibration ---
        cal = ctk.CTkFrame(self.left)
        cal.grid(row=1, column=0, sticky="ew", padx=10, pady=5)
        cal.grid_columnconfigure(1, weight=1)
        ctk.CTkLabel(cal, text="Calibration",
                     font=ctk.CTkFont(size=14, weight="bold")).grid(
                     row=0, column=0, columnspan=3, pady=(8, 4))
        ctk.CTkLabel(cal, text="Pulses per Rev:").grid(
                     row=1, column=0, padx=(8, 4), pady=4, sticky="w")
        self.ppr_var = ctk.StringVar(value="12240")
        ctk.CTkEntry(cal, textvariable=self.ppr_var, width=90).grid(
                     row=1, column=1, padx=(0, 4), pady=4, sticky="ew")
        ctk.CTkButton(cal, text="Send", height=24, width=60,
                      command=self._send_ppr).grid(
                      row=1, column=2, padx=(0, 8), pady=4)
        ctk.CTkLabel(cal,
            text="Hand-rotate output shaft 1 turn,\nuse Delta count as PPR value.",
            text_color="#888", font=ctk.CTkFont(size=10),
            justify="left").grid(row=2, column=0, columnspan=3,
                                  padx=8, pady=(0, 8), sticky="w")

        # --- Active part header ---
        active_header = ctk.CTkFrame(self.left)
        active_header.grid(row=2, column=0, sticky="ew", padx=10, pady=5)
        active_header.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(active_header, text="Active Controls",
                     font=ctk.CTkFont(size=12, weight="bold"),
                     text_color="#888").grid(row=0, column=0, pady=(6, 0))
        self.active_part_label = ctk.CTkLabel(active_header, text="Part 1",
                     font=ctk.CTkFont(size=18, weight="bold"))
        self.active_part_label.grid(row=1, column=0, pady=(0, 8))

        # --- Part panels container ---
        self.part_panel_container = ctk.CTkFrame(self.left, fg_color="transparent")
        self.part_panel_container.grid(row=3, column=0, sticky="nsew",
                                        padx=10, pady=5)
        self.part_panel_container.grid_columnconfigure(0, weight=1)

        self._build_part1_panel()
        self._build_part2_panel()
        self._build_part3_panel()
        self._build_part4_panel()

        # --- Utilities ---
        u = ctk.CTkFrame(self.left)
        u.grid(row=4, column=0, sticky="ew", padx=10, pady=(5, 10))
        u.grid_columnconfigure((0, 1), weight=1)
        ctk.CTkLabel(u, text="Utilities",
                     font=ctk.CTkFont(size=16, weight="bold")).grid(
                     row=0, column=0, columnspan=2, pady=(8, 4))
        ctk.CTkButton(u, text="Reset Encoder",
                      command=lambda: self._send("CMD:RESET")).grid(
                      row=1, column=0, columnspan=2, sticky="ew", padx=8, pady=4)
        self.log_btn = ctk.CTkButton(u, text="Start Logging",
                                      command=self._toggle_logging,
                                      fg_color="#7a5c1c", hover_color="#5a4515")
        self.log_btn.grid(row=2, column=0, columnspan=2, sticky="ew",
                          padx=8, pady=(4, 4))
        self.log_status = ctk.CTkLabel(u, text=f"Logs: {self.log_dir}",
                                        text_color="#888",
                                        font=ctk.CTkFont(size=10),
                                        wraplength=320, justify="left")
        self.log_status.grid(row=3, column=0, columnspan=2,
                              padx=8, pady=(0, 8), sticky="w")

    # ----- Part 1 panel -----
    def _build_part1_panel(self):
        p = ctk.CTkFrame(self.part_panel_container)
        p.grid_columnconfigure(0, weight=1)

        d = ctk.CTkFrame(p)
        d.grid(row=0, column=0, sticky="ew", pady=(0, 6))
        d.grid_columnconfigure((0, 1, 2), weight=1)
        ctk.CTkLabel(d, text="Manual Direction",
                     font=ctk.CTkFont(size=15, weight="bold")).grid(
                     row=0, column=0, columnspan=3, pady=(8, 4))
        ctk.CTkButton(d, text="< BACK", height=44,
                      command=lambda: self._manual_direction("CMD:B"),
                      fg_color="#3a3a8c", hover_color="#2a2a6c").grid(
                      row=1, column=0, padx=(8, 4), pady=(4, 8), sticky="ew")
        ctk.CTkButton(d, text="STOP", height=44,
                      command=lambda: self._manual_direction("CMD:S"),
                      fg_color="#a33", hover_color="#822").grid(
                      row=1, column=1, padx=4, pady=(4, 8), sticky="ew")
        ctk.CTkButton(d, text="FWD >", height=44,
                      command=lambda: self._manual_direction("CMD:F"),
                      fg_color="#1f6f3f", hover_color="#155028").grid(
                      row=1, column=2, padx=(4, 8), pady=(4, 8), sticky="ew")

        s = ctk.CTkFrame(p)
        s.grid(row=1, column=0, sticky="ew", pady=6)
        s.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(s, text="PWM Speed (0-255)",
                     font=ctk.CTkFont(size=15, weight="bold")).grid(
                     row=0, column=0, pady=(8, 4))
        self.speed_value_label = ctk.CTkLabel(s, text="0",
                                               font=ctk.CTkFont(size=24, weight="bold"))
        self.speed_value_label.grid(row=1, column=0, pady=4)
        self.speed_slider = ctk.CTkSlider(s, from_=0, to=255,
                                           number_of_steps=255,
                                           command=self._on_speed_change)
        self.speed_slider.set(0)
        self.speed_slider.grid(row=2, column=0, sticky="ew", padx=8, pady=(4, 8))

        a = ctk.CTkFrame(p)
        a.grid(row=2, column=0, sticky="ew", pady=6)
        a.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(a, text="Automated Test Cycle",
                     font=ctk.CTkFont(size=15, weight="bold")).grid(
                     row=0, column=0, pady=(8, 4))
        ctk.CTkLabel(a, text="FWD 3s -> STOP 2s -> BACK 3s -> STOP 2s, repeat",
                     text_color="#888",
                     font=ctk.CTkFont(size=11)).grid(row=1, column=0, pady=(0, 4))
        self.auto_cycle_btn = ctk.CTkButton(a, text="Start Auto Cycle",
                                             height=44,
                                             command=self._toggle_auto_cycle,
                                             font=ctk.CTkFont(size=14, weight="bold"),
                                             fg_color="#1f6f3f",
                                             hover_color="#155028")
        self.auto_cycle_btn.grid(row=2, column=0, sticky="ew", padx=8, pady=(4, 4))
        self.auto_cycle_status = ctk.CTkLabel(a, text="", text_color="#888",
                                               font=ctk.CTkFont(size=11))
        self.auto_cycle_status.grid(row=3, column=0, pady=(0, 8))

        self.part1_panel = p

    # ----- Part 2 panel -----
    def _build_part2_panel(self):
        p = ctk.CTkFrame(self.part_panel_container)
        p.grid_columnconfigure(0, weight=1)

        d = ctk.CTkFrame(p)
        d.grid(row=0, column=0, sticky="ew", pady=(0, 6))
        d.grid_columnconfigure((0, 1, 2), weight=1)
        ctk.CTkLabel(d, text="Direction",
                     font=ctk.CTkFont(size=15, weight="bold")).grid(
                     row=0, column=0, columnspan=3, pady=(8, 4))
        ctk.CTkButton(d, text="< BACK", height=40,
                      command=lambda: self._manual_direction("CMD:B"),
                      fg_color="#3a3a8c", hover_color="#2a2a6c").grid(
                      row=1, column=0, padx=(8, 4), pady=(4, 8), sticky="ew")
        ctk.CTkButton(d, text="STOP", height=40,
                      command=lambda: self._manual_direction("CMD:S"),
                      fg_color="#a33", hover_color="#822").grid(
                      row=1, column=1, padx=4, pady=(4, 8), sticky="ew")
        ctk.CTkButton(d, text="FWD >", height=40,
                      command=lambda: self._manual_direction("CMD:F"),
                      fg_color="#1f6f3f", hover_color="#155028").grid(
                      row=1, column=2, padx=(4, 8), pady=(4, 8), sticky="ew")

        s = ctk.CTkFrame(p)
        s.grid(row=1, column=0, sticky="ew", pady=6)
        s.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(s, text="PWM Speed (0-255)",
                     font=ctk.CTkFont(size=15, weight="bold")).grid(
                     row=0, column=0, pady=(8, 4))
        self.speed_value_label_p2 = ctk.CTkLabel(s, text="0",
                                                  font=ctk.CTkFont(size=24, weight="bold"))
        self.speed_value_label_p2.grid(row=1, column=0, pady=4)
        self.speed_slider_p2 = ctk.CTkSlider(s, from_=0, to=255,
                                              number_of_steps=255,
                                              command=self._on_speed_change_p2)
        self.speed_slider_p2.set(0)
        self.speed_slider_p2.grid(row=2, column=0, sticky="ew", padx=8, pady=(4, 8))

        self.part2_panel = p

    # ----- Part 3 panel -----
    def _build_part3_panel(self):
        p = ctk.CTkFrame(self.part_panel_container)
        p.grid_columnconfigure(0, weight=1)

        dirf = ctk.CTkFrame(p)
        dirf.grid(row=0, column=0, sticky="ew", pady=(0, 6))
        dirf.grid_columnconfigure((0, 1), weight=1)
        ctk.CTkLabel(dirf, text="PID Direction",
                     font=ctk.CTkFont(size=15, weight="bold")).grid(
                     row=0, column=0, columnspan=2, pady=(8, 4))
        self.pid_dir_segctl = ctk.CTkSegmentedButton(dirf,
                                values=["Forward >", "< Reverse"],
                                command=self._on_pid_direction_change,
                                selected_color="#c08020",
                                selected_hover_color="#a06010")
        self.pid_dir_segctl.set("Forward >")
        self.pid_dir_segctl.grid(row=1, column=0, columnspan=2, sticky="ew",
                                  padx=8, pady=(4, 8))

        sp = ctk.CTkFrame(p)
        sp.grid(row=1, column=0, sticky="ew", pady=6)
        sp.grid_columnconfigure(1, weight=1)
        ctk.CTkLabel(sp, text="Setpoint",
                     font=ctk.CTkFont(size=15, weight="bold")).grid(
                     row=0, column=0, columnspan=2, pady=(8, 4))
        ctk.CTkLabel(sp, text="Target RPM").grid(row=1, column=0,
                                                  padx=(8, 4), pady=4, sticky="w")
        self.setpoint_var = ctk.StringVar(value="120")
        self.setpoint_var.trace_add("write", self._on_pid_value_change)
        ctk.CTkEntry(sp, textvariable=self.setpoint_var, width=80).grid(
                     row=1, column=1, padx=(0, 8), pady=4, sticky="ew")

        g = ctk.CTkFrame(p)
        g.grid(row=2, column=0, sticky="ew", pady=6)
        g.grid_columnconfigure(1, weight=1)
        ctk.CTkLabel(g, text="PID Gains",
                     font=ctk.CTkFont(size=15, weight="bold")).grid(
                     row=0, column=0, columnspan=2, pady=(8, 4))
        self.kp_var = ctk.StringVar(value="2.0")
        self.ki_var = ctk.StringVar(value="1.0")
        self.kd_var = ctk.StringVar(value="0.05")
        self.kp_var.trace_add("write", self._on_pid_value_change)
        self.ki_var.trace_add("write", self._on_pid_value_change)
        self.kd_var.trace_add("write", self._on_pid_value_change)

        for i, (lbl, var) in enumerate([("Kp", self.kp_var),
                                          ("Ki", self.ki_var),
                                          ("Kd", self.kd_var)]):
            ctk.CTkLabel(g, text=lbl).grid(row=i+1, column=0,
                                            padx=(8, 4), pady=2, sticky="w")
            ctk.CTkEntry(g, textvariable=var, width=80).grid(
                         row=i+1, column=1, padx=(0, 8), pady=2, sticky="ew")

        self.pid_pending_label = ctk.CTkLabel(p, text="", text_color="#888",
                                               font=ctk.CTkFont(size=10))
        self.pid_pending_label.grid(row=3, column=0, sticky="w", padx=8)

        ctk.CTkButton(p, text="Apply Now",
                      command=self._apply_pid_settings,
                      fg_color="#5c5c1c", hover_color="#454515",
                      height=30).grid(
                      row=4, column=0, sticky="ew", padx=4, pady=(2, 6))
        self.pid_run_btn = ctk.CTkButton(p, text="Start PID",
                                          height=46, command=self._toggle_pid,
                                          font=ctk.CTkFont(size=15, weight="bold"),
                                          fg_color="#1f6f3f", hover_color="#155028")
        self.pid_run_btn.grid(row=5, column=0, sticky="ew", padx=4, pady=6)

        live = ctk.CTkFrame(p)
        live.grid(row=6, column=0, sticky="ew", pady=6)
        live.grid_columnconfigure((0, 1), weight=1)
        ctk.CTkLabel(live, text="Live Values",
                     font=ctk.CTkFont(size=13, weight="bold"),
                     text_color="#888").grid(row=0, column=0, columnspan=2,
                                              pady=(6, 4))
        ctk.CTkLabel(live, text="Error (RPM):", text_color="#aaa").grid(
                     row=1, column=0, padx=8, pady=2, sticky="w")
        self.error_value = ctk.CTkLabel(live, text="0.0", text_color="#fc6")
        self.error_value.grid(row=1, column=1, padx=8, pady=2, sticky="e")
        ctk.CTkLabel(live, text="Output PWM:", text_color="#aaa").grid(
                     row=2, column=0, padx=8, pady=2, sticky="w")
        self.pwm_value = ctk.CTkLabel(live, text="0", text_color="#9cf")
        self.pwm_value.grid(row=2, column=1, padx=8, pady=(2, 8), sticky="e")

        self.part3_panel = p

    # ----- Part 4 panel -----
    def _build_part4_panel(self):
        p = ctk.CTkFrame(self.part_panel_container)
        p.grid_columnconfigure(0, weight=1)

        # Modulino badge
        m = ctk.CTkFrame(p)
        m.grid(row=0, column=0, sticky="ew", pady=(0, 6))
        m.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(m, text="Modulino Selection",
                     font=ctk.CTkFont(size=15, weight="bold")).grid(
                     row=0, column=0, pady=(8, 4))
        self.modulino_badge = ctk.CTkLabel(m, text="None Selected",
                                            font=ctk.CTkFont(size=14, weight="bold"),
                                            fg_color="#3a3a3a",
                                            corner_radius=8,
                                            text_color="#888",
                                            height=40)
        self.modulino_badge.grid(row=1, column=0, sticky="ew", padx=8, pady=(4, 4))
        ctk.CTkLabel(m, text="Tap A=Temp, B=LDR, C=Obstacle on Modulino Buttons.\nDouble-tap to clear.",
                     text_color="#888",
                     font=ctk.CTkFont(size=10), justify="center").grid(
                     row=2, column=0, padx=8, pady=(0, 8))

        # Direction
        dirf = ctk.CTkFrame(p)
        dirf.grid(row=1, column=0, sticky="ew", pady=6)
        dirf.grid_columnconfigure((0, 1), weight=1)
        ctk.CTkLabel(dirf, text="Motor Direction",
                     font=ctk.CTkFont(size=15, weight="bold")).grid(
                     row=0, column=0, columnspan=2, pady=(8, 4))
        self.part4_dir_segctl = ctk.CTkSegmentedButton(dirf,
                                values=["Forward >", "< Reverse"],
                                command=self._on_part4_dir_change,
                                selected_color="#8c3a8c",
                                selected_hover_color="#6c2a6c")
        self.part4_dir_segctl.set("Forward >")
        self.part4_dir_segctl.grid(row=1, column=0, columnspan=2, sticky="ew",
                                    padx=8, pady=(4, 8))

        # Base PWM
        s = ctk.CTkFrame(p)
        s.grid(row=2, column=0, sticky="ew", pady=6)
        s.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(s, text="Base PWM (0-255)",
                     font=ctk.CTkFont(size=15, weight="bold")).grid(
                     row=0, column=0, pady=(8, 4))
        self.part4_speed_label = ctk.CTkLabel(s, text="150",
                                               font=ctk.CTkFont(size=24, weight="bold"))
        self.part4_speed_label.grid(row=1, column=0, pady=4)
        self.part4_speed_slider = ctk.CTkSlider(s, from_=0, to=255,
                                                 number_of_steps=255,
                                                 command=self._on_part4_speed_change)
        self.part4_speed_slider.set(150)
        self.part4_speed_slider.grid(row=2, column=0, sticky="ew", padx=8, pady=(4, 8))

        # ---- Sensor cards ----

        # Temperature
        tcard = ctk.CTkFrame(p, border_width=2, border_color="#3a3a3a")
        tcard.grid(row=3, column=0, sticky="ew", pady=6)
        tcard.grid_columnconfigure(1, weight=1)
        ctk.CTkLabel(tcard, text="Temperature (A)",
                     font=ctk.CTkFont(size=14, weight="bold"),
                     text_color=SENSOR_COLORS["temp"]).grid(
                     row=0, column=0, columnspan=2, pady=(8, 4), padx=8, sticky="w")

        ctk.CTkLabel(tcard, text="Limit (F):").grid(row=1, column=0,
                                                      padx=8, pady=2, sticky="w")
        self.temp_limit_var = ctk.StringVar(value="95")
        self.temp_limit_var.trace_add("write", self._on_part4_value_change)
        ctk.CTkEntry(tcard, textvariable=self.temp_limit_var, width=70).grid(
                     row=1, column=1, padx=8, pady=2, sticky="e")

        ctk.CTkLabel(tcard, text="Max (F):").grid(row=2, column=0,
                                                    padx=8, pady=2, sticky="w")
        self.temp_max_var = ctk.StringVar(value="110")
        self.temp_max_var.trace_add("write", self._on_part4_value_change)
        ctk.CTkEntry(tcard, textvariable=self.temp_max_var, width=70).grid(
                     row=2, column=1, padx=8, pady=2, sticky="e")

        ctk.CTkLabel(tcard, text="Now:", text_color="#aaa").grid(
                     row=3, column=0, padx=8, pady=(4, 8), sticky="w")
        self.temp_now_label = ctk.CTkLabel(tcard, text="--",
                                            font=ctk.CTkFont(size=18, weight="bold"),
                                            text_color=SENSOR_COLORS["temp"])
        self.temp_now_label.grid(row=3, column=1, padx=8, pady=(4, 8), sticky="e")
        self.temp_card = tcard

        # LDR
        lcard = ctk.CTkFrame(p, border_width=2, border_color="#3a3a3a")
        lcard.grid(row=4, column=0, sticky="ew", pady=6)
        lcard.grid_columnconfigure(1, weight=1)
        ctk.CTkLabel(lcard, text="LDR (B)",
                     font=ctk.CTkFont(size=14, weight="bold"),
                     text_color=SENSOR_COLORS["ldr"]).grid(
                     row=0, column=0, columnspan=2, pady=(8, 4), padx=8, sticky="w")

        ctk.CTkLabel(lcard, text="Mode:").grid(row=1, column=0,
                                                padx=8, pady=2, sticky="w")
        self.ldr_mode_segctl = ctk.CTkSegmentedButton(lcard,
                                values=["Variable", "Threshold"],
                                command=self._on_ldr_mode_change,
                                selected_color="#3a8c5a")
        self.ldr_mode_segctl.set("Variable")
        self.ldr_mode_segctl.grid(row=1, column=1, padx=8, pady=2, sticky="ew")

        ctk.CTkLabel(lcard, text="Polarity:").grid(row=2, column=0,
                                                    padx=8, pady=2, sticky="w")
        self.ldr_polarity_segctl = ctk.CTkSegmentedButton(lcard,
                                values=["High=Bright", "High=Dark"],
                                command=self._on_ldr_polarity_change,
                                selected_color="#3a8c5a")
        self.ldr_polarity_segctl.set("High=Dark")
        self.ldr_polarity_segctl.grid(row=2, column=1, padx=8, pady=2, sticky="ew")

        ctk.CTkLabel(lcard, text="Low (raw):").grid(row=3, column=0,
                                                     padx=8, pady=2, sticky="w")
        self.ldr_lo_var = ctk.StringVar(value="350")
        self.ldr_lo_var.trace_add("write", self._on_part4_value_change)
        ctk.CTkEntry(lcard, textvariable=self.ldr_lo_var, width=70).grid(
                     row=3, column=1, padx=8, pady=2, sticky="e")

        ctk.CTkLabel(lcard, text="High (raw):").grid(row=4, column=0,
                                                      padx=8, pady=2, sticky="w")
        self.ldr_hi_var = ctk.StringVar(value="800")
        self.ldr_hi_var.trace_add("write", self._on_part4_value_change)
        ctk.CTkEntry(lcard, textvariable=self.ldr_hi_var, width=70).grid(
                     row=4, column=1, padx=8, pady=2, sticky="e")

        ctk.CTkLabel(lcard, text="Now:", text_color="#aaa").grid(
                     row=5, column=0, padx=8, pady=(4, 8), sticky="w")
        self.ldr_now_label = ctk.CTkLabel(lcard, text="--",
                                           font=ctk.CTkFont(size=18, weight="bold"),
                                           text_color=SENSOR_COLORS["ldr"])
        self.ldr_now_label.grid(row=5, column=1, padx=8, pady=(4, 8), sticky="e")
        self.ldr_card = lcard

        # Obstacle
        ocard = ctk.CTkFrame(p, border_width=2, border_color="#3a3a3a")
        ocard.grid(row=5, column=0, sticky="ew", pady=6)
        ocard.grid_columnconfigure(1, weight=1)
        ctk.CTkLabel(ocard, text="Obstacle (C)",
                     font=ctk.CTkFont(size=14, weight="bold"),
                     text_color=SENSOR_COLORS["ir"]).grid(
                     row=0, column=0, columnspan=2, pady=(8, 4), padx=8, sticky="w")
        ctk.CTkLabel(ocard, text="(Threshold set on sensor module)",
                     text_color="#888",
                     font=ctk.CTkFont(size=11)).grid(
                     row=1, column=0, columnspan=2, padx=8, pady=2, sticky="w")
        ctk.CTkLabel(ocard, text="Status:", text_color="#aaa").grid(
                     row=2, column=0, padx=8, pady=(4, 8), sticky="w")
        self.ir_now_label = ctk.CTkLabel(ocard, text="CLEAR",
                                          font=ctk.CTkFont(size=16, weight="bold"),
                                          text_color=SENSOR_COLORS["ir"])
        self.ir_now_label.grid(row=2, column=1, padx=8, pady=(4, 8), sticky="e")
        self.ir_card = ocard

        # Sync indicator
        self.part4_pending_label = ctk.CTkLabel(p, text="", text_color="#888",
                                                 font=ctk.CTkFont(size=10))
        self.part4_pending_label.grid(row=6, column=0, sticky="w", padx=8)

        self.part4_panel = p

    # ====================================================
    # RIGHT PANEL
    # ====================================================
    def _build_right_panel(self):
        right = ctk.CTkFrame(self, corner_radius=0, fg_color="transparent")
        right.grid(row=0, column=1, sticky="nsew", padx=10, pady=10)
        right.grid_columnconfigure(0, weight=1)
        right.grid_rowconfigure(2, weight=1)

        # Mode header
        mode_bar = ctk.CTkFrame(right, corner_radius=10)
        mode_bar.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        mode_bar.grid_columnconfigure(0, weight=0)
        mode_bar.grid_columnconfigure(1, weight=1)

        self.part_badge = ctk.CTkLabel(mode_bar, text="PART 1",
                                        font=ctk.CTkFont(size=22, weight="bold"),
                                        fg_color="#3a8c3a", corner_radius=8,
                                        width=140, height=70, text_color="#fff")
        self.part_badge.grid(row=0, column=0, rowspan=2, padx=15, pady=12)
        self.part_title_label = ctk.CTkLabel(mode_bar, text=MODES[1]["name"],
                     font=ctk.CTkFont(size=18, weight="bold"), anchor="w")
        self.part_title_label.grid(row=0, column=1, sticky="w", pady=(12, 0))
        self.part_desc_label = ctk.CTkLabel(mode_bar, text=MODES[1]["desc"],
                     text_color="#aaa", anchor="w",
                     font=ctk.CTkFont(size=12))
        self.part_desc_label.grid(row=1, column=1, sticky="w", pady=(0, 8))

        tab_row = ctk.CTkFrame(mode_bar, fg_color="transparent")
        tab_row.grid(row=2, column=0, columnspan=2, sticky="ew",
                     padx=10, pady=(0, 10))
        for i in range(4):
            tab_row.grid_columnconfigure(i, weight=1)
        self.part_buttons = {}
        for i, part_num in enumerate([1, 2, 3, 4]):
            btn = ctk.CTkButton(tab_row, text=f"Part {part_num}", height=32,
                                command=lambda p=part_num: self._set_part(p))
            btn.grid(row=0, column=i, padx=4, sticky="ew")
            self.part_buttons[part_num] = btn

        # Status readouts
        status = ctk.CTkFrame(right)
        status.grid(row=1, column=0, sticky="ew", pady=(0, 10))
        for i in range(4):
            status.grid_columnconfigure(i, weight=1)
        self.rpm_label = self._make_readout(status, "RPM", "0.0", 0, color="#3fa")
        self.count_label = self._make_readout(status, "Encoder Count", "0", 1, color="#9cf")
        self.pwm_label = self._make_readout(status, "PWM", "0", 2, color="#fc6")
        self.dir_label = self._make_readout(status, "Direction", "STOPPED", 3, color="#fff")

        # Display container
        self.display_container = ctk.CTkFrame(right)
        self.display_container.grid(row=2, column=0, sticky="nsew")
        self.display_container.grid_rowconfigure(0, weight=1)
        self.display_container.grid_columnconfigure(0, weight=1)

        self._build_plot_view()
        self._build_table_view()
        self._build_part4_view()

    def _build_plot_view(self):
        self.plot_frame = ctk.CTkFrame(self.display_container)
        self.plot_frame.grid_rowconfigure(0, weight=1)
        self.plot_frame.grid_columnconfigure(0, weight=1)

        self.fig = Figure(figsize=(8, 5), dpi=100, facecolor="#2b2b2b")
        self.ax_rpm = self.fig.add_subplot(211)
        self.ax_pwm = self.fig.add_subplot(212, sharex=self.ax_rpm)

        for ax in (self.ax_rpm, self.ax_pwm):
            ax.set_facecolor("#1e1e1e")
            ax.tick_params(colors="#ccc")
            for spine in ax.spines.values():
                spine.set_color("#555")
            ax.grid(True, color="#444", linestyle="--", alpha=0.5)

        self.ax_rpm.axhline(0, color="#777", linestyle="-",
                             linewidth=0.8, alpha=0.6)
        self.ax_rpm.set_ylabel("RPM", color="#ccc")
        self.ax_rpm.set_title("Motor Telemetry", color="#fff")
        self.ax_pwm.set_ylabel("PWM", color="#ccc")
        self.ax_pwm.set_xlabel("Time (s)", color="#ccc")
        self.ax_pwm.set_ylim(0, 260)

        self.line_rpm, = self.ax_rpm.plot([], [], color="#3fa", linewidth=2,
                                           label="Measured RPM")
        self.line_setpoint, = self.ax_rpm.plot([], [], color="#fc6", linewidth=1.5,
                                                linestyle="--", label="Setpoint")
        self.line_pwm, = self.ax_pwm.plot([], [], color="#9cf", linewidth=2)

        self.legend = self.ax_rpm.legend(loc="upper right", facecolor="#2b2b2b",
                           edgecolor="#555", labelcolor="#ccc")
        self.fig.tight_layout()

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_frame)
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew",
                                          padx=4, pady=4)

    def _build_table_view(self):
        self.table_frame = ctk.CTkFrame(self.display_container)
        self.table_frame.grid_rowconfigure(1, weight=1)
        self.table_frame.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(self.table_frame, text="Live RPM Data Table",
                     font=ctk.CTkFont(size=16, weight="bold")).grid(
                     row=0, column=0, pady=(8, 4), sticky="w", padx=12)

        from tkinter import ttk
        style = ttk.Style()
        style.theme_use("default")
        style.configure("Dark.Treeview",
                        background="#1e1e1e", foreground="#ddd",
                        fieldbackground="#1e1e1e", rowheight=26,
                        font=("Consolas", 11), borderwidth=0)
        style.configure("Dark.Treeview.Heading",
                        background="#2b2b2b", foreground="#fff",
                        font=("Segoe UI", 11, "bold"), borderwidth=0)
        style.map("Dark.Treeview", background=[("selected", "#3a6c8c")])

        cols = ("time_ms", "pwm", "count", "delta", "rpm")
        col_widths = (110, 70, 110, 90, 100)
        col_headings = ("Time (ms)", "PWM", "Count", "DeltaCount", "RPM")
        self.table = ttk.Treeview(self.table_frame, columns=cols,
                                   show="headings", style="Dark.Treeview",
                                   height=15)
        for c, w, h in zip(cols, col_widths, col_headings):
            self.table.heading(c, text=h, anchor="center")
            self.table.column(c, width=w, anchor="e")
        self.table.grid(row=1, column=0, sticky="nsew", padx=12, pady=(4, 12))
        sb = ttk.Scrollbar(self.table_frame, orient="vertical",
                            command=self.table.yview)
        self.table.configure(yscrollcommand=sb.set)
        sb.grid(row=1, column=1, sticky="ns", pady=(4, 12))

    def _build_part4_view(self):
        self.part4_frame = ctk.CTkFrame(self.display_container)
        self.part4_frame.grid_rowconfigure(0, weight=1)
        self.part4_frame.grid_columnconfigure(0, weight=1)

        self.fig4 = Figure(figsize=(8, 8), dpi=100, facecolor="#2b2b2b")

        gs = self.fig4.add_gridspec(4, 1, hspace=0.5)
        self.ax4_temp = self.fig4.add_subplot(gs[0])
        self.ax4_ldr  = self.fig4.add_subplot(gs[1])
        self.ax4_ir   = self.fig4.add_subplot(gs[2])
        self.ax4_motor = self.fig4.add_subplot(gs[3])

        for ax, ylabel, color in [
            (self.ax4_temp, "Temp (F)", SENSOR_COLORS["temp"]),
            (self.ax4_ldr,  "LDR (raw)", SENSOR_COLORS["ldr"]),
            (self.ax4_ir,   "Obstacle",  SENSOR_COLORS["ir"]),
            (self.ax4_motor,"RPM / PWM", "#fc6"),
        ]:
            ax.set_facecolor("#1e1e1e")
            ax.tick_params(colors="#ccc")
            for spine in ax.spines.values():
                spine.set_color("#555")
            ax.grid(True, color="#444", linestyle="--", alpha=0.5)
            ax.set_ylabel(ylabel, color=color, fontsize=10)

        self.ax4_temp.set_ylim(50, 110)
        self.ax4_ldr.set_ylim(0, 1023)
        self.ax4_ir.set_ylim(-0.2, 1.2)
        self.ax4_ir.set_yticks([0, 1])
        self.ax4_ir.set_yticklabels(["DETECTED", "CLEAR"])
        self.ax4_motor.set_xlabel("Time (s)", color="#ccc")

        self.line4_temp, = self.ax4_temp.plot([], [],
                                color=SENSOR_COLORS["temp"], linewidth=2)
        self.line4_temp_lim = self.ax4_temp.axhline(95, color="#aa6", linestyle="--",
                                                     linewidth=1, alpha=0.7,
                                                     label="Limit")
        self.line4_temp_max = self.ax4_temp.axhline(110, color="#a55", linestyle="--",
                                                     linewidth=1, alpha=0.7,
                                                     label="Max")

        self.line4_ldr, = self.ax4_ldr.plot([], [],
                                color=SENSOR_COLORS["ldr"], linewidth=2)
        self.line4_ldr_lo = self.ax4_ldr.axhline(350, color="#5a5", linestyle="--",
                                                   linewidth=1, alpha=0.7)
        self.line4_ldr_hi = self.ax4_ldr.axhline(800, color="#5a5", linestyle="--",
                                                   linewidth=1, alpha=0.7)

        self.line4_ir, = self.ax4_ir.plot([], [],
                                color=SENSOR_COLORS["ir"], linewidth=2,
                                drawstyle="steps-post")

        self.line4_rpm, = self.ax4_motor.plot([], [], color="#3fa", linewidth=2,
                                                label="RPM")
        self.ax4_motor_pwm = self.ax4_motor.twinx()
        self.ax4_motor_pwm.set_ylabel("PWM", color="#9cf", fontsize=10)
        self.ax4_motor_pwm.tick_params(colors="#9cf")
        self.ax4_motor_pwm.set_ylim(0, 260)
        for spine in self.ax4_motor_pwm.spines.values():
            spine.set_color("#555")
        self.line4_pwm, = self.ax4_motor_pwm.plot([], [], color="#9cf",
                                                    linewidth=1.5, alpha=0.8,
                                                    label="PWM")

        self.fig4.tight_layout()

        self.canvas4 = FigureCanvasTkAgg(self.fig4, master=self.part4_frame)
        self.canvas4.get_tk_widget().grid(row=0, column=0, sticky="nsew",
                                           padx=4, pady=4)

    def _show_view(self, which):
        for f in (self.plot_frame, self.table_frame, self.part4_frame):
            f.grid_forget()
        if which == "plot":
            self.plot_frame.grid(row=0, column=0, sticky="nsew")
        elif which == "table":
            self.table_frame.grid(row=0, column=0, sticky="nsew")
            for item in self.table.get_children():
                self.table.delete(item)
            self.table_rows.clear()
            self.last_table_count = self.latest_count
        elif which == "part4":
            self.part4_frame.grid(row=0, column=0, sticky="nsew")

    def _make_readout(self, parent, label, initial, col, color="#fff"):
        f = ctk.CTkFrame(parent, corner_radius=8)
        f.grid(row=0, column=col, sticky="ew", padx=4, pady=8)
        f.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(f, text=label, font=ctk.CTkFont(size=12),
                     text_color="#888").grid(row=0, column=0, pady=(8, 0))
        v = ctk.CTkLabel(f, text=initial,
                          font=ctk.CTkFont(size=28, weight="bold"),
                          text_color=color)
        v.grid(row=1, column=0, pady=(0, 8))
        return v

    # ====================================================
    # Calibration
    # ====================================================
    def _send_ppr(self):
        try:
            v = float(self.ppr_var.get())
            if v > 0:
                self._send(f"CMD:PPR:{v}")
        except ValueError:
            pass

    # ====================================================
    # Manual direction (Parts 1-2)
    # ====================================================
    def _manual_direction(self, cmd):
        if self.auto_cycle_running:
            self._stop_auto_cycle()
        if cmd == "CMD:S":
            self._send("CMD:S")
        else:
            current_pwm = int(self._get_current_slider_value())
            self._send(f"CMD:SPEED:{current_pwm}")
            self._send(cmd)

    def _get_current_slider_value(self):
        if self.current_part == 2 and hasattr(self, "speed_slider_p2"):
            return self.speed_slider_p2.get()
        return self.speed_slider.get()

    def _on_speed_change(self, value):
        speed = int(value)
        self.speed_value_label.configure(text=str(speed))
        self._send(f"CMD:SPEED:{speed}")

    def _on_speed_change_p2(self, value):
        speed = int(value)
        self.speed_value_label_p2.configure(text=str(speed))
        self._send(f"CMD:SPEED:{speed}")

    # ====================================================
    # Auto-cycle (Part 1)
    # ====================================================
    def _toggle_auto_cycle(self):
        if self.auto_cycle_running:
            self._stop_auto_cycle()
        else:
            self._start_auto_cycle()

    def _start_auto_cycle(self):
        self.auto_cycle_running = True
        self.auto_cycle_btn.configure(text="Stop Auto Cycle",
                                       fg_color="#a33", hover_color="#822")
        self.auto_cycle_thread = threading.Thread(target=self._auto_cycle_loop,
                                                    daemon=True)
        self.auto_cycle_thread.start()

    def _stop_auto_cycle(self):
        self.auto_cycle_running = False
        self.auto_cycle_btn.configure(text="Start Auto Cycle",
                                       fg_color="#1f6f3f", hover_color="#155028")
        self.auto_cycle_status.configure(text="")
        self._send("CMD:S")

    def _auto_cycle_loop(self):
        sequence = [("FORWARD",  "CMD:F", 3.0),
                    ("STOP",     "CMD:S", 2.0),
                    ("BACKWARD", "CMD:B", 3.0),
                    ("STOP",     "CMD:S", 2.0)]
        while self.auto_cycle_running:
            for label, cmd, dur in sequence:
                if not self.auto_cycle_running: break
                pwm = int(self._get_current_slider_value())
                if cmd != "CMD:S":
                    self._send(f"CMD:SPEED:{pwm}")
                self._send(cmd)
                self.after(0, lambda l=label, d=dur:
                            self.auto_cycle_status.configure(
                                text=f"{l} for {int(d)}s",
                                text_color="#3fa"))
                end = time.time() + dur
                while time.time() < end and self.auto_cycle_running:
                    time.sleep(0.05)
        self._send("CMD:S")

    # ====================================================
    # Part switching
    # ====================================================
    def _set_part(self, part):
        if self.auto_cycle_running:
            self._stop_auto_cycle()
        if self.pid_running:
            self._send("CMD:PIDENABLE:0")
            self.pid_running = False
            if hasattr(self, "pid_run_btn"):
                self.pid_run_btn.configure(text="Start PID",
                                            fg_color="#1f6f3f",
                                            hover_color="#155028")
        self._send("CMD:S")

        self.current_part = part
        info = MODES[part]

        self.part_badge.configure(text=f"PART {part}", fg_color=info["color"])
        self.part_title_label.configure(text=info["name"])
        self.part_desc_label.configure(text=info["desc"])
        for p, btn in self.part_buttons.items():
            btn.configure(fg_color=info["color"] if p == part
                           else ("#3a3a3a", "#2a2a2a"))
        self.active_part_label.configure(text=f"Part {part}",
                                          text_color=info["color"])

        self._update_setpoint_visibility(part)

        for panel in (self.part1_panel, self.part2_panel,
                       self.part3_panel, self.part4_panel):
            panel.grid_forget()
        if part == 1:
            self.part1_panel.grid(row=0, column=0, sticky="new")
            self._show_view("plot")
        elif part == 2:
            self.part2_panel.grid(row=0, column=0, sticky="new")
            current_pwm = self.speed_slider.get()
            self.speed_slider_p2.set(current_pwm)
            self.speed_value_label_p2.configure(text=str(int(current_pwm)))
            self._show_view("table")
        elif part == 3:
            self.part3_panel.grid(row=0, column=0, sticky="new")
            self._show_view("plot")
        elif part == 4:
            self.part4_panel.grid(row=0, column=0, sticky="new")
            self._send_all_part4_config()
            self._show_view("part4")

        self._send(f"CMD:MODE:{part}")

    def _update_setpoint_visibility(self, part):
        show = (part == 3)
        self.line_setpoint.set_visible(show)
        if show:
            self.legend = self.ax_rpm.legend(
                handles=[self.line_rpm, self.line_setpoint],
                loc="upper right", facecolor="#2b2b2b",
                edgecolor="#555", labelcolor="#ccc")
        else:
            self.legend = self.ax_rpm.legend(
                handles=[self.line_rpm],
                loc="upper right", facecolor="#2b2b2b",
                edgecolor="#555", labelcolor="#ccc")

    # ====================================================
    # Part 3 actions
    # ====================================================
    def _on_pid_value_change(self, *args):
        self.pid_pending_label.configure(text="* updating...",
                                          text_color="#fc6")
        if self._pid_after_id is not None:
            try: self.after_cancel(self._pid_after_id)
            except Exception: pass
        self._pid_after_id = self.after(PID_DEBOUNCE_MS,
                                         self._do_debounced_pid_send)

    def _do_debounced_pid_send(self):
        self._pid_after_id = None
        ok = self._apply_pid_settings()
        if ok:
            self.pid_pending_label.configure(text="synced", text_color="#3fa")
            self.after(1500, lambda: self.pid_pending_label.configure(text=""))
        else:
            self.pid_pending_label.configure(text="invalid number",
                                              text_color="#f55")

    def _apply_pid_settings(self):
        try:
            kp = float(self.kp_var.get())
            ki = float(self.ki_var.get())
            kd = float(self.kd_var.get())
            sp = float(self.setpoint_var.get())
        except ValueError:
            return False
        self._send(f"CMD:PID:{kp}:{ki}:{kd}")
        self._send(f"CMD:SETPOINT:{sp}")
        return True

    def _on_pid_direction_change(self, value):
        self.pid_dir_forward = (value == "Forward >")
        self._send(f"CMD:PIDDIR:{'F' if self.pid_dir_forward else 'B'}")

    def _toggle_pid(self):
        if self.pid_running:
            self._send("CMD:PIDENABLE:0")
            self.pid_running = False
            self.pid_run_btn.configure(text="Start PID",
                                        fg_color="#1f6f3f", hover_color="#155028")
        else:
            self._apply_pid_settings()
            self._send(f"CMD:PIDDIR:{'F' if self.pid_dir_forward else 'B'}")
            time.sleep(0.05)
            self._send("CMD:PIDENABLE:1")
            self.pid_running = True
            self.pid_run_btn.configure(text="Stop PID",
                                        fg_color="#a33", hover_color="#822")

    # ====================================================
    # Part 4 actions
    # ====================================================
    def _on_part4_value_change(self, *args):
        self.part4_pending_label.configure(text="* updating...",
                                            text_color="#fc6")
        if self._part4_after_id is not None:
            try: self.after_cancel(self._part4_after_id)
            except Exception: pass
        self._part4_after_id = self.after(PID_DEBOUNCE_MS,
                                            self._do_debounced_part4_send)

    def _do_debounced_part4_send(self):
        self._part4_after_id = None
        ok = self._send_all_part4_config()
        if ok:
            self.part4_pending_label.configure(text="synced",
                                                text_color="#3fa")
            self.after(1500, lambda:
                        self.part4_pending_label.configure(text=""))
        else:
            self.part4_pending_label.configure(text="invalid number",
                                                text_color="#f55")

    def _send_all_part4_config(self):
        try:
            tlim = float(self.temp_limit_var.get())
            tmax = float(self.temp_max_var.get())
            llo  = int(self.ldr_lo_var.get())
            lhi  = int(self.ldr_hi_var.get())
        except ValueError:
            return False
        self._send(f"CMD:TEMPLIM:{tlim}")
        self._send(f"CMD:TEMPMAX:{tmax}")
        self._send(f"CMD:LDRLO:{llo}")
        self._send(f"CMD:LDRHI:{lhi}")
        m = "V" if self.ldr_mode_segctl.get() == "Variable" else "T"
        self._send(f"CMD:LDRMODE:{m}")
        p = "B" if self.ldr_polarity_segctl.get() == "High=Bright" else "D"
        self._send(f"CMD:LDRPOLARITY:{p}")
        self._send(f"CMD:PART4BASE:{int(self.part4_speed_slider.get())}")
        self._send(f"CMD:PART4DIR:{'F' if self.part4_dir_forward else 'B'}")
        return True

    def _on_ldr_mode_change(self, value):
        m = "V" if value == "Variable" else "T"
        self._send(f"CMD:LDRMODE:{m}")

    def _on_ldr_polarity_change(self, value):
        p = "B" if value == "High=Bright" else "D"
        self._send(f"CMD:LDRPOLARITY:{p}")

    def _on_part4_dir_change(self, value):
        self.part4_dir_forward = (value == "Forward >")
        self._send(f"CMD:PART4DIR:{'F' if self.part4_dir_forward else 'B'}")

    def _on_part4_speed_change(self, value):
        speed = int(value)
        self.part4_speed_label.configure(text=str(speed))
        self._send(f"CMD:PART4BASE:{speed}")

    def _highlight_active_sensor_card(self):
        sel = self.latest_selected_sensor
        for card, key in [(self.temp_card, "temp"),
                           (self.ldr_card, "ldr"),
                           (self.ir_card, "ir")]:
            card.configure(border_color="#3a3a3a")
        if sel == 0:
            self.temp_card.configure(border_color=SENSOR_COLORS["temp"])
        elif sel == 1:
            self.ldr_card.configure(border_color=SENSOR_COLORS["ldr"])
        elif sel == 2:
            self.ir_card.configure(border_color=SENSOR_COLORS["ir"])

        if sel == -1:
            self.modulino_badge.configure(text="None Selected",
                                           fg_color="#3a3a3a",
                                           text_color="#888")
        elif sel == 0:
            self.modulino_badge.configure(text="A - Temperature",
                                           fg_color=SENSOR_COLORS["temp"],
                                           text_color="#fff")
        elif sel == 1:
            self.modulino_badge.configure(text="B - LDR",
                                           fg_color=SENSOR_COLORS["ldr"],
                                           text_color="#000")
        elif sel == 2:
            self.modulino_badge.configure(text="C - Obstacle",
                                           fg_color=SENSOR_COLORS["ir"],
                                           text_color="#fff")

    # ====================================================
    # Connection
    # ====================================================
    def _get_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        return ports if ports else ["No ports found"]

    def _refresh_ports(self):
        self.port_dropdown.configure(values=self._get_ports())

    def _toggle_connection(self):
        if self.serial_port and self.serial_port.is_open:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_dropdown.get()
        if not port or port == "No ports found":
            self.connection_status.configure(text="* No port selected",
                                              text_color="#f55")
            return
        try:
            self.serial_port = serial.Serial(port, BAUD_RATE, timeout=0.1)
            time.sleep(2)
            self.serial_port.reset_input_buffer()
            self.serial_running = True
            self.serial_thread = threading.Thread(target=self._serial_loop,
                                                    daemon=True)
            self.serial_thread.start()
            self.connect_btn.configure(text="Disconnect",
                                        fg_color="#a33", hover_color="#822")
            self.connection_status.configure(text=f"* Connected to {port}",
                                              text_color="#3fa")
            self.start_time = time.time()
            for buf in (self.time_buffer, self.rpm_buffer, self.setpoint_buffer,
                         self.pwm_buffer, self.temp_buffer, self.ldr_buffer,
                         self.ir_buffer):
                buf.clear()
            # Push current PPR setting to Arduino on connect
            time.sleep(0.1)
            self._send_ppr()
        except Exception as e:
            self.connection_status.configure(text=f"* Error: {e}",
                                              text_color="#f55")

    def _disconnect(self):
        if self.auto_cycle_running:
            self._stop_auto_cycle()
        self.serial_running = False
        if self.serial_port:
            try:
                self.serial_port.write(b"CMD:S\n")
                time.sleep(0.05)
                self.serial_port.close()
            except Exception:
                pass
            self.serial_port = None
        self.connect_btn.configure(text="Connect",
                                    fg_color="#1f6f3f", hover_color="#155028")
        self.connection_status.configure(text="* Disconnected", text_color="#888")

    # ====================================================
    # Serial loop
    # ====================================================
    def _serial_loop(self):
        while self.serial_running and self.serial_port:
            try:
                line = self.serial_port.readline().decode("utf-8",
                                                           errors="ignore").strip()
                if line.startswith("DATA,"):
                    self._parse_telemetry(line)
            except Exception:
                break

    def _parse_telemetry(self, line):
        try:
            parts = line.split(",")
            if len(parts) < 17:
                return
            t_ms = int(parts[1])
            rpm = float(parts[2])
            count = int(parts[3])
            pwm = int(parts[4])
            direction = parts[5]
            setpoint = float(parts[6])
            mode = int(parts[7])
            err = float(parts[8])
            temp_f = float(parts[9])
            humidity = float(parts[10])
            dht_status = parts[11]
            ldr = int(parts[12])
            ir_raw = int(parts[13])
            obstacle = (parts[14] == "YES")
            sel_sensor = int(parts[15])
            logic_enabled = (parts[16] == "1")

            self.latest_rpm = rpm
            self.latest_count = count
            self.latest_pwm = pwm
            self.latest_direction = direction
            self.latest_setpoint = setpoint
            self.latest_mode = mode
            self.latest_error = err
            self.latest_temp_f = temp_f
            self.latest_humidity = humidity
            self.latest_dht_status = dht_status
            self.latest_ldr = ldr
            self.latest_ir_raw = ir_raw
            self.latest_obstacle = obstacle
            self.latest_selected_sensor = sel_sensor
            self.latest_logic_enabled = logic_enabled

            t_sec = time.time() - self.start_time
            self.time_buffer.append(t_sec)
            self.rpm_buffer.append(rpm)
            self.setpoint_buffer.append(setpoint)
            self.pwm_buffer.append(pwm)
            self.temp_buffer.append(temp_f)
            self.ldr_buffer.append(ldr)
            self.ir_buffer.append(0 if obstacle else 1)

            if self.csv_writer:
                delta = count - self.last_table_count
                self.csv_writer.writerow([f"{t_sec:.3f}", t_ms,
                                           f"{rpm:.2f}", count, delta, pwm,
                                           direction, f"{setpoint:.1f}", mode,
                                           f"{err:.2f}", f"{temp_f:.1f}",
                                           f"{humidity:.1f}", dht_status,
                                           ldr, ir_raw,
                                           "YES" if obstacle else "NO",
                                           sel_sensor,
                                           1 if logic_enabled else 0])

            if self.current_part == 2:
                delta = count - self.last_table_count
                self.last_table_count = count
                row = (t_ms, pwm, count, delta, f"{rpm:.1f}")
                self.after(0, self._insert_table_row, row)
        except Exception:
            pass

    def _insert_table_row(self, row):
        self.table.insert("", 0, values=row)
        children = self.table.get_children()
        if len(children) > TABLE_MAX_ROWS:
            for c in children[TABLE_MAX_ROWS:]:
                self.table.delete(c)

    def _send(self, cmd):
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write((cmd + "\n").encode("utf-8"))
            except Exception:
                pass

    # ====================================================
    # CSV logging
    # ====================================================
    def _toggle_logging(self):
        if self.csv_file:
            self._stop_logging()
        else:
            self._start_logging()

    def _start_logging(self):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        fname = os.path.join(self.log_dir, f"motor_log_{ts}.csv")
        try:
            os.makedirs(self.log_dir, exist_ok=True)
            self.csv_file = open(fname, "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(["time_s", "arduino_ms", "rpm", "count",
                                       "delta_count", "pwm", "direction",
                                       "setpoint", "mode", "error",
                                       "temp_f", "humidity", "dht_status",
                                       "ldr_raw", "ir_raw", "obstacle",
                                       "selected_sensor", "logic_enabled"])
            self.last_table_count = self.latest_count
            self.log_btn.configure(text="Stop Logging",
                                    fg_color="#a33", hover_color="#822")
            self.log_status.configure(text=f"-> {fname}", text_color="#3fa")
        except Exception as e:
            self.log_status.configure(text=f"Error: {e}", text_color="#f55")

    def _stop_logging(self):
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
        self.log_btn.configure(text="Start Logging",
                                fg_color="#7a5c1c", hover_color="#5a4515")
        self.log_status.configure(text=f"Logs: {self.log_dir}",
                                   text_color="#888")

    # ====================================================
    # Plot updates
    # ====================================================
    def _update_plot(self):
        if self.latest_rpm > 1:   rpm_color = "#3fa"
        elif self.latest_rpm < -1: rpm_color = "#f88"
        else:                       rpm_color = "#888"
        self.rpm_label.configure(text=f"{self.latest_rpm:.1f}",
                                  text_color=rpm_color)
        self.count_label.configure(text=f"{self.latest_count}")
        self.pwm_label.configure(text=f"{self.latest_pwm}")
        self.dir_label.configure(text=self.latest_direction)

        if hasattr(self, "error_value"):
            self.error_value.configure(text=f"{self.latest_error:.1f}")
            self.pwm_value.configure(text=f"{self.latest_pwm}")

        if hasattr(self, "temp_now_label"):
            self.temp_now_label.configure(
                text=f"{self.latest_temp_f:.1f} F  ({self.latest_humidity:.0f}%)"
                if self.latest_dht_status == "OK" else "FAIL")
            self.ldr_now_label.configure(text=str(self.latest_ldr))
            self.ir_now_label.configure(
                text="DETECTED" if self.latest_obstacle else "CLEAR",
                text_color="#f88" if self.latest_obstacle
                            else SENSOR_COLORS["ir"])

        if self.current_part == 4:
            self._highlight_active_sensor_card()

        t = list(self.time_buffer)
        if len(t) > 1:
            t_max = t[-1]
            t_min = max(0, t_max - PLOT_HISTORY_SECONDS)

            if self.current_part == 4:
                self.line4_temp.set_data(t, list(self.temp_buffer))
                self.line4_ldr.set_data(t, list(self.ldr_buffer))
                self.line4_ir.set_data(t, list(self.ir_buffer))
                self.line4_rpm.set_data(t, list(self.rpm_buffer))
                self.line4_pwm.set_data(t, list(self.pwm_buffer))

                try:
                    self.line4_temp_lim.set_ydata(
                        [float(self.temp_limit_var.get())] * 2)
                    self.line4_temp_max.set_ydata(
                        [float(self.temp_max_var.get())] * 2)
                    self.line4_ldr_lo.set_ydata(
                        [int(self.ldr_lo_var.get())] * 2)
                    self.line4_ldr_hi.set_ydata(
                        [int(self.ldr_hi_var.get())] * 2)
                except ValueError:
                    pass

                for ax in (self.ax4_temp, self.ax4_ldr,
                            self.ax4_ir, self.ax4_motor):
                    ax.set_xlim(t_min, t_max + 0.5)

                rpm_vals = list(self.rpm_buffer)
                if rpm_vals:
                    rmax = max(abs(min(rpm_vals)), abs(max(rpm_vals)), 50) * 1.2
                    self.ax4_motor.set_ylim(-rmax, rmax)

                self.canvas4.draw_idle()

            elif self.current_part != 2:
                self.line_rpm.set_data(t, list(self.rpm_buffer))
                self.line_setpoint.set_data(t, list(self.setpoint_buffer))
                self.line_pwm.set_data(t, list(self.pwm_buffer))
                self.ax_rpm.set_xlim(t_min, t_max + 0.5)
                self.ax_pwm.set_xlim(t_min, t_max + 0.5)

                rpm_values = list(self.rpm_buffer)
                if self.line_setpoint.get_visible():
                    rpm_values += list(self.setpoint_buffer)
                rpm_abs_max = max(abs(min(rpm_values)),
                                    abs(max(rpm_values)), 50)
                self.ax_rpm.set_ylim(-rpm_abs_max * 1.2, rpm_abs_max * 1.2)

                self.canvas.draw_idle()

        self.after(PLOT_UPDATE_MS, self._update_plot)

    # ====================================================
    # Cleanup
    # ====================================================
    def _on_close(self):
        if self.auto_cycle_running:
            self._stop_auto_cycle()
        self._disconnect()
        if self.csv_file:
            self._stop_logging()
        self.destroy()


if __name__ == "__main__":
    app = MotorControlApp()
    app.mainloop()
