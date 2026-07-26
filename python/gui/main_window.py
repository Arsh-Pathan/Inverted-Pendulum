import sys
import os
import time
import math
import numpy as np
from PyQt6.QtWidgets import (QMainWindow, QWidget, QHBoxLayout, QVBoxLayout, 
                             QSplitter, QLabel, QGridLayout, QFrame, QTabWidget)
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QFont
import pyqtgraph as pg

from ..utils.config_loader import load_config, save_config
from ..comms.serial_client import SerialClient
from ..comms.protocol import cmd_motor, cmd_brake, cmd_coast, cmd_zero_tare
from ..controllers.pid_balancer import PIDBalancer
from ..controllers.lqr_balancer import LQRBalancer
from ..controllers.hybrid_balancer import HybridBalancer
from ..controllers.oscillation import OscillationController
try:
    from rl.rl_controller import RLBalancer
except ImportError:
    RLBalancer = None

from .card_widget import CardWidget
from .telemetry_canvas import TelemetryCanvas
from .control_panel import ControlPanel

QSS_STYLE = """
QMainWindow { background-color: #ffffff; }
QWidget { background-color: #ffffff; color: #000000; font-family: 'Inter', 'Segoe UI', sans-serif; }
QLabel { color: #000000; }
QTabWidget::pane { border: 1px solid #cccccc; border-radius: 4px; background: #ffffff; }
QTabBar::tab {
    background: #f0f0f0; border: 1px solid #cccccc; padding: 8px 16px;
    font-weight: bold; font-size: 13px; color: #555555; border-top-left-radius: 4px; border-top-right-radius: 4px;
}
QTabBar::tab:selected { background: #ffffff; color: #000000; border-bottom: 2px solid #3498db; }
QTabBar::tab:hover { background: #e8e8e8; }
"""

class MainWindow(QMainWindow):
    """
    Main Application Window for the Inverted Pendulum HIL Platform.
    Integrates real-time CAD viewport, PyQtGraph telemetry charts (Time-Series & Phase Portrait),
    live gain tuning, and multi-algorithm Python closed-loop balancing (PID, LQR, RL, Hybrid).
    """
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Inverted Pendulum HIL Research Station — Multi-Mode AI Control")
        self.resize(1650, 950)
        self.setStyleSheet(QSS_STYLE)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        # 1) Load Configurations
        self.config = load_config()
        ctrl_cfg = self.config.get("control", {})
        osc_cfg = self.config.get("oscillation", {})

        # 2) Initialize Modular Balancing Algorithms
        self.pid_balancer = PIDBalancer(
            kp=ctrl_cfg.get("kp", 15.0),
            ki=ctrl_cfg.get("ki", 0.0),
            kd=ctrl_cfg.get("kd", 2.5),
            alpha=ctrl_cfg.get("alpha", 0.08),
            min_power=ctrl_cfg.get("min_motor_power", 45),
            max_power=ctrl_cfg.get("max_motor_power", 255),
            deadzone_deg=ctrl_cfg.get("equilibrium_deadzone_deg", 0.4),
            deadzone_vel=ctrl_cfg.get("equilibrium_deadzone_vel", 6.0)
        )
        self.lqr_balancer = LQRBalancer(
            k_theta=ctrl_cfg.get("k_theta", 25.0),
            k_omega=ctrl_cfg.get("k_omega", 3.5),
            alpha=ctrl_cfg.get("alpha", 0.08),
            min_power=ctrl_cfg.get("min_motor_power", 45),
            max_power=ctrl_cfg.get("max_motor_power", 255),
            deadzone_deg=ctrl_cfg.get("equilibrium_deadzone_deg", 0.4),
            deadzone_vel=ctrl_cfg.get("equilibrium_deadzone_vel", 6.0)
        )
        self.hybrid_balancer = HybridBalancer(
            stabilizer=self.pid_balancer
        )
        if RLBalancer is not None:
            self.rl_balancer = RLBalancer(model_path=None, min_power=45, max_power=255)
        else:
            self.rl_balancer = None

        self.oscillation_ctrl = OscillationController(
            speed=osc_cfg.get("speed", 255),
            duration_ms=osc_cfg.get("duration_ms", 400)
        )

        self.active_mode = "PID" # "PID", "LQR", "RL", "HYBRID"
        self.current_action = 0
        self.invert_display = True

        # 3) Telemetry State Variables
        self.theta = 0.0
        self.raw_angle = 0.0
        self.angle_dev = 0.0
        self.vel_deg_s = 0.0
        self.prev_raw = None
        self.last_time = 0.0
        self.elapsed_time = 0.0
        self.start_time = None
        self.peak_angle = 0.0
        self.peak_vel = 0.0
        self.sample_count = 0
        self.sample_rate = 0.0
        self.rate_timer = time.time()
        self.rate_count = 0
        self._data_dirty = False

        # Ring Buffer for multi-channel charting
        self.history_len = 800
        self._buf_time = np.zeros(self.history_len, dtype=np.float64)
        self._buf_angle = np.zeros(self.history_len, dtype=np.float64)
        self._buf_vel = np.zeros(self.history_len, dtype=np.float64)
        self._buf_action = np.zeros(self.history_len, dtype=np.float64)
        self._buf_idx = 0
        self._buf_count = 0

        # 4) Setup UI
        self.init_ui()

        # 5) Serial Connection
        self.serial_client = None
        self.is_connected = False
        self.port_timer = QTimer()
        self.port_timer.timeout.connect(self.auto_connect_serial)
        self.port_timer.start(1500)
        self.auto_connect_serial()

        # 6) Display Update Timers
        self.timer_fast = QTimer()
        self.timer_fast.timeout.connect(self.tick_fast)
        self.timer_fast.start(16) # ~60 FPS

        self.timer_graph = QTimer()
        self.timer_graph.timeout.connect(self.tick_graph)
        self.timer_graph.start(50) # ~20 FPS

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(20)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        main_layout.addWidget(splitter)

        # ── Left Panel: CAD Viewport & Status ──
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(16)
        splitter.addWidget(left_widget)

        # Header Block
        header_widget = QWidget()
        header_layout = QVBoxLayout(header_widget)
        header_layout.setContentsMargins(0, 0, 0, 0)
        header_layout.setSpacing(4)
        lbl_title = QLabel("Inverted Pendulum HIL Research Station")
        lbl_title.setStyleSheet("font-size: 30px; font-weight: 800; color: #000000; letter-spacing: -0.5px;")
        lbl_sub = QLabel("Real-time AI Control (RL/PPO, LQR, Hybrid Swing-Up, PID) & Dynamic Phase Portraits")
        lbl_sub.setStyleSheet("font-size: 13px; font-weight: 600; color: #555555;")
        header_layout.addWidget(lbl_title)
        header_layout.addWidget(lbl_sub)
        left_layout.addWidget(header_widget)

        # Canvas Viewport Card
        sim_card = CardWidget(None)
        self.canvas_widget = TelemetryCanvas()
        sim_card.layout.addWidget(self.canvas_widget, 1)

        # Status row
        status_row = QHBoxLayout()
        self.lbl_telemetry = QLabel("Time: 0.0s | Port: Searching...")
        self.lbl_telemetry.setStyleSheet("color: #555555; font-weight: 600; font-size: 13px;")
        status_row.addWidget(self.lbl_telemetry)
        status_row.addStretch()

        self.status_dot = QFrame()
        self.status_dot.setFixedSize(10, 10)
        self.status_dot.setStyleSheet("border: 1px solid #000000; border-radius: 5px; background-color: #888888;")
        self.status_text = QLabel("Offline")
        self.status_text.setStyleSheet("font-weight: 600; font-size: 13px; color: #000000;")
        status_row.addWidget(self.status_dot)
        status_row.addWidget(self.status_text)
        sim_card.layout.addLayout(status_row)
        left_layout.addWidget(sim_card, 1)

        # ── Right Panel: Controls, Metrics & Tabbed Charts ──
        right_widget = QWidget()
        right_widget.setFixedWidth(640)
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(12)
        splitter.addWidget(right_widget)

        # Control Panel
        self.ctrl_panel = ControlPanel(self.config)
        self._wire_control_panel()
        right_layout.addWidget(self.ctrl_panel)

        # Live Metrics Card
        metrics_card = CardWidget("LIVE HIL TELEMETRY METRICS")
        readout_grid = QGridLayout()
        readout_grid.setSpacing(6)
        big_style = "font-size: 26px; font-family: 'Consolas', monospace; font-weight: bold; color: #000000;"
        small_style = "font-size: 14px; font-family: 'Consolas', monospace; font-weight: bold; color: #000000;"
        lbl_style = "font-size: 10px; font-weight: 800; color: #555555;"

        lbl_dev_title = QLabel("DEVIATION FROM ZERO:")
        lbl_dev_title.setStyleSheet(lbl_style)
        self.lbl_angle_val = QLabel("0.00°")
        self.lbl_angle_val.setStyleSheet(big_style)
        readout_grid.addWidget(lbl_dev_title, 0, 0, 1, 2)
        readout_grid.addWidget(self.lbl_angle_val, 1, 0, 1, 2)

        lbl_vel_title = QLabel("ANGULAR VELOCITY:")
        lbl_vel_title.setStyleSheet(lbl_style)
        self.lbl_vel_val = QLabel("0.0°/s")
        self.lbl_vel_val.setStyleSheet(big_style)
        readout_grid.addWidget(lbl_vel_title, 2, 0, 1, 2)
        readout_grid.addWidget(self.lbl_vel_val, 3, 0, 1, 2)

        lbl_act_title = QLabel("MOTOR ACTION (PWM):")
        lbl_act_title.setStyleSheet(lbl_style)
        self.lbl_action_val = QLabel("0 [COAST]")
        self.lbl_action_val.setStyleSheet(small_style)
        lbl_rate = QLabel("SAMPLE RATE:")
        lbl_rate.setStyleSheet(lbl_style)
        self.lbl_rate_val = QLabel("— Hz")
        self.lbl_rate_val.setStyleSheet(small_style)
        readout_grid.addWidget(lbl_act_title, 4, 0)
        readout_grid.addWidget(lbl_rate, 4, 1)
        readout_grid.addWidget(self.lbl_action_val, 5, 0)
        readout_grid.addWidget(self.lbl_rate_val, 5, 1)

        lbl_pa = QLabel("PEAK ANGLE:")
        lbl_pa.setStyleSheet(lbl_style)
        self.lbl_peak_angle = QLabel("0.0°")
        self.lbl_peak_angle.setStyleSheet(small_style)
        lbl_pv = QLabel("PEAK VELOCITY:")
        lbl_pv.setStyleSheet(lbl_style)
        self.lbl_peak_vel = QLabel("0.0°/s")
        self.lbl_peak_vel.setStyleSheet(small_style)
        readout_grid.addWidget(lbl_pa, 6, 0)
        readout_grid.addWidget(lbl_pv, 6, 1)
        readout_grid.addWidget(self.lbl_peak_angle, 7, 0)
        readout_grid.addWidget(self.lbl_peak_vel, 7, 1)

        metrics_card.layout.addLayout(readout_grid)
        right_layout.addWidget(metrics_card)

        # ── RESEARCH CHARTS TAB WIDGET ──
        self.tab_widget = QTabWidget()
        right_layout.addWidget(self.tab_widget, 1)

        # TAB 1: Time-Series Telemetry (Angle, Vel, PWM Action)
        tab_time = QWidget()
        tab_time_layout = QVBoxLayout(tab_time)
        tab_time_layout.setContentsMargins(4, 8, 4, 4)
        tab_time_layout.setSpacing(8)

        self.angle_card = CardWidget("DEVIATION VS TIME")
        self.angle_plot = pg.PlotWidget()
        self.angle_curve = self._style_chart(self.angle_plot, y_label="Angle", y_unit="°", line_color="#0066cc")
        self.angle_plot.addItem(pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen("#ff3333", width=1.5, style=Qt.PenStyle.DashLine)))
        self.angle_card.layout.addWidget(self.angle_plot)
        tab_time_layout.addWidget(self.angle_card, 1)

        self.vel_card = CardWidget("VELOCITY VS TIME")
        self.vel_plot = pg.PlotWidget()
        self.vel_curve = self._style_chart(self.vel_plot, y_label="Velocity", y_unit="°/s", line_color="#cc3333")
        self.vel_plot.addItem(pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen("#aaaaaa", width=1, style=Qt.PenStyle.DashLine)))
        self.vel_card.layout.addWidget(self.vel_plot)
        tab_time_layout.addWidget(self.vel_card, 1)

        self.action_card = CardWidget("MOTOR ACTION (PWM EFFORT) VS TIME")
        self.action_plot = pg.PlotWidget()
        self.action_curve = self._style_chart(self.action_plot, y_label="PWM Duty", y_unit="", line_color="#27ae60")
        self.action_plot.setYRange(-260, 260)
        self.action_plot.addItem(pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen("#aaaaaa", width=1, style=Qt.PenStyle.DashLine)))
        self.action_card.layout.addWidget(self.action_plot)
        tab_time_layout.addWidget(self.action_card, 1)

        self.tab_widget.addTab(tab_time, "Time-Series Telemetry & Actuation")

        # TAB 2: State-Space Phase Portrait (Angle vs Velocity)
        tab_phase = QWidget()
        tab_phase_layout = QVBoxLayout(tab_phase)
        tab_phase_layout.setContentsMargins(4, 8, 4, 4)

        self.phase_card = CardWidget("STATE-SPACE PHASE PORTRAIT (θ vs dθ/dt)")
        self.phase_plot = pg.PlotWidget()
        self.phase_plot.setBackground("#ffffff")
        p_item = self.phase_plot.getPlotItem()
        p_item.showGrid(x=True, y=True, alpha=0.2)
        p_item.setLabel("bottom", "Angle Deviation (θ)", units="°", **{"font-size": "11px", "color": "#333333"})
        p_item.setLabel("left", "Angular Velocity (dθ/dt)", units="°/s", **{"font-size": "11px", "color": "#333333"})
        p_item.addItem(pg.InfiniteLine(pos=0, angle=90, pen=pg.mkPen("#888888", width=1, style=Qt.PenStyle.DashLine)))
        p_item.addItem(pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen("#888888", width=1, style=Qt.PenStyle.DashLine)))
        
        # Upright origin target indicator (Green dot at 0, 0)
        target_dot = pg.ScatterPlotItem([0], [0], pen=pg.mkPen("#00aa00", width=2), brush=pg.mkBrush("#2ecc71"), size=14)
        p_item.addItem(target_dot)

        self.phase_curve = p_item.plot(pen=pg.mkPen("#8e44ad", width=2.0))
        self.phase_card.layout.addWidget(self.phase_plot)
        tab_phase_layout.addWidget(self.phase_card)

        self.tab_widget.addTab(tab_phase, "State-Space Phase Portrait")

        splitter.setSizes([980, 640])

    def _style_chart(self, plot, y_label="Val", y_unit="", line_color="#000000"):
        plot.setBackground("#ffffff")
        item = plot.getPlotItem()
        item.setContentsMargins(0, 0, 0, 0)
        item.showGrid(x=True, y=True, alpha=0.15)
        item.setClipToView(True)
        item.setDownsampling(mode="peak")
        item.showAxis("right", False)
        item.showAxis("top", False)
        item.enableAutoRange(axis="y")
        font = QFont("Consolas", 8)
        left = item.getAxis("left")
        left.setPen(pg.mkPen("#cccccc", width=1))
        left.setTextPen(pg.mkPen("#555555"))
        left.setTickFont(font)
        left.setLabel(y_label, units=y_unit, **{"font-size": "10px", "color": "#555555"})
        bot = item.getAxis("bottom")
        bot.setPen(pg.mkPen("#cccccc", width=1))
        bot.setTextPen(pg.mkPen("#555555"))
        bot.setTickFont(font)
        bot.setLabel("Time", units="s", **{"font-size": "10px", "color": "#555555"})
        return item.plot(pen=pg.mkPen(line_color, width=1.5))

    def _wire_control_panel(self):
        self.ctrl_panel.start_clicked.connect(self._on_start_oscillation)
        self.ctrl_panel.stop_clicked.connect(self._on_stop_all)
        self.ctrl_panel.balance_toggled.connect(self._on_balance_toggled)
        self.ctrl_panel.tare_clicked.connect(self._on_tare_clicked)
        self.ctrl_panel.mode_selected.connect(self._on_mode_selected)
        self.ctrl_panel.invert_display_toggled.connect(self._on_invert_display_toggled)
        self.ctrl_panel.speed_changed.connect(lambda v: self.oscillation_ctrl.update_params({"speed": v}))
        self.ctrl_panel.duration_changed.connect(lambda v: self.oscillation_ctrl.update_params({"duration_ms": v}))
        self.ctrl_panel.pid_changed.connect(self._on_pid_changed)

    def _on_invert_display_toggled(self, checked: bool):
        self.invert_display = checked
        print(f"[UI DISPLAY] Invert displayed angle set to: {checked}")

    def _on_mode_selected(self, mode: str):
        self.active_mode = mode
        print(f"[HIL MODE] Control algorithm switched to: {mode}")
        if self.ctrl_panel.is_balancing:
            # Switch active controller on the fly
            self.pid_balancer.disable()
            self.lqr_balancer.disable()
            self.hybrid_balancer.disable()
            if self.rl_balancer: self.rl_balancer.disable()
            self._enable_active_controller()

    def _enable_active_controller(self):
        self.oscillation_ctrl.disable()
        if self.active_mode == "PID":
            self.pid_balancer.enable()
        elif self.active_mode == "LQR":
            self.lqr_balancer.enable()
        elif self.active_mode == "RL":
            if self.rl_balancer: self.rl_balancer.enable()
            else: self.pid_balancer.enable()
        elif self.active_mode == "HYBRID":
            self.hybrid_balancer.enable()

    def _on_start_oscillation(self):
        print("[HIL MODE] Enabling Oscillation Controller...")
        self.pid_balancer.disable()
        self.lqr_balancer.disable()
        self.hybrid_balancer.disable()
        if self.rl_balancer: self.rl_balancer.disable()
        self.oscillation_ctrl.enable()

    def _on_stop_all(self):
        print("[HIL MODE] Disabling all controllers. Braking motor...")
        self.pid_balancer.disable()
        self.lqr_balancer.disable()
        self.hybrid_balancer.disable()
        if self.rl_balancer: self.rl_balancer.disable()
        self.oscillation_ctrl.disable()
        self.current_action = 0
        if self.serial_client:
            self.serial_client.send_command(cmd_brake())

    def _on_balance_toggled(self, is_enabled: bool):
        if is_enabled:
            print(f"[HIL MODE] Starting closed-loop balancing with [{self.active_mode}] engine...")
            self._enable_active_controller()
        else:
            print("[HIL MODE] Disabling closed-loop control. Coasting motor...")
            self.pid_balancer.disable()
            self.lqr_balancer.disable()
            self.hybrid_balancer.disable()
            if self.rl_balancer: self.rl_balancer.disable()
            self.current_action = 0
            if self.serial_client:
                self.serial_client.send_command(cmd_coast())

    def _on_tare_clicked(self):
        print("[HIL COMMAND] Triggering hardware tare calibration...")
        if self.serial_client:
            self.serial_client.send_command(cmd_zero_tare())

    def _on_pid_changed(self, params: dict):
        self.pid_balancer.update_params(params)
        if "control" not in self.config:
            self.config["control"] = {}
        self.config["control"].update(params)
        save_config(self.config)

    # ── Serial Connection Management ──
    def auto_connect_serial(self):
        if self.serial_client and self.serial_client.isRunning():
            return

        ports = SerialClient.list_available_ports()
        target = self.config.get("serial", {}).get("preferred_port", "COM3")
        if not ports:
            self.set_status_indicator("Searching...", "gray")
            return

        chosen = target if target in ports else ports[0]
        baud = self.config.get("serial", {}).get("baud_rate", 115200)

        print(f"[COM] Device detected on {chosen}. Launching background HIL client...")
        self.serial_client = SerialClient(chosen, baud)
        self.serial_client.angle_received.connect(self.on_angle_received, Qt.ConnectionType.DirectConnection)
        self.serial_client.status_changed.connect(self.set_status_indicator)
        self.serial_client.start()

    def set_status_indicator(self, text: str, state: str):
        self.status_text.setText(text)
        self.is_connected = (state == "green")
        colors = {"green": "#00aa00", "gray": "#888888", "red": "#ff3333"}
        self.status_dot.setStyleSheet(f"border: 1px solid #000000; border-radius: 5px; background-color: {colors.get(state, '#888888')};")
        if state == "red" and self.serial_client:
            self.serial_client.stop_client()
            self.serial_client = None

    # ── High-Frequency Closed-Loop HIL Telemetry & Actuator Step ──
    def on_angle_received(self, raw_val: float):
        now = time.time()
        dt = now - self.last_time if self.last_time > 0 else 0.01
        self.last_time = now
        if self.start_time is None:
            self.start_time = now

        self.raw_angle = raw_val
        self.angle_dev = raw_val % 360.0

        # Wrap-aware angular velocity
        if self.prev_raw is not None and dt > 0:
            delta = (raw_val - self.prev_raw) % 360.0
            if delta > 180.0: delta -= 360.0
            elif delta < -180.0: delta += 360.0
            self.vel_deg_s = delta / dt
        self.prev_raw = raw_val

        # Update Canvas representation
        self.theta = math.radians(self.angle_dev)
        self.peak_angle = max(self.peak_angle, abs(self.angle_dev))
        self.peak_vel = max(self.peak_vel, abs(self.vel_deg_s))

        # Sample rate calculation
        self.rate_count += 1
        self.sample_count += 1
        if now - self.rate_timer >= 1.0:
            self.sample_rate = self.rate_count / (now - self.rate_timer)
            self.rate_count = 0
            self.rate_timer = now

        # ── HIL CLOSED-LOOP CONTROL EXECUTION ──
        power = 0
        if self.serial_client and self.serial_client.isRunning():
            if self.ctrl_panel.is_balancing:
                if self.active_mode == "PID":
                    power = self.pid_balancer.compute_action(self.angle_dev, dt)
                elif self.active_mode == "LQR":
                    power = self.lqr_balancer.compute_action(self.angle_dev, dt)
                elif self.active_mode == "RL":
                    if self.rl_balancer and self.rl_balancer.enabled:
                        power = self.rl_balancer.compute_action(self.angle_dev, dt)
                    else:
                        power = self.pid_balancer.compute_action(self.angle_dev, dt)
                elif self.active_mode == "HYBRID":
                    power = self.hybrid_balancer.compute_action(self.angle_dev, dt)
                
                self.serial_client.send_command(cmd_motor(power))
                self.current_action = power
            elif self.oscillation_ctrl.enabled:
                power = self.oscillation_ctrl.compute_action(self.angle_dev, dt)
                self.serial_client.send_command(cmd_motor(power))
                self.current_action = power
            else:
                self.current_action = 0

        # Update Ring Buffer
        i = self._buf_idx % self.history_len
        self._buf_time[i] = now - self.start_time
        self._buf_angle[i] = self.angle_dev
        self._buf_vel[i] = self.vel_deg_s
        self._buf_action[i] = float(self.current_action)
        self._buf_idx += 1
        self._buf_count = min(self._buf_count + 1, self.history_len)
        self._data_dirty = True

    def _get_buf_slices(self):
        count = self._buf_count
        idx = self._buf_idx
        if count < self.history_len:
            return self._buf_time[:count], self._buf_angle[:count], self._buf_vel[:count], self._buf_action[:count]
        start = idx % self.history_len
        t = np.concatenate((self._buf_time[start:], self._buf_time[:start]))
        a = np.concatenate((self._buf_angle[start:], self._buf_angle[:start]))
        v = np.concatenate((self._buf_vel[start:], self._buf_vel[:start]))
        u = np.concatenate((self._buf_action[start:], self._buf_action[:start]))
        return t, a, v, u

    # ── GUI Redraw Timers ──
    def tick_fast(self):
        self.elapsed_time = (time.time() - self.start_time) if self.start_time else 0.0
        disp_a = (360.0 - self.angle_dev) % 360.0 if self.invert_display else self.angle_dev
        disp_v = -self.vel_deg_s if self.invert_display else self.vel_deg_s
        self.lbl_angle_val.setText(f"{disp_a:+.2f}°")
        self.lbl_vel_val.setText(f"{disp_v:+.1f}°/s")
        self.lbl_action_val.setText(f"{self.current_action:+} PWM")
        self.lbl_rate_val.setText(f"{self.sample_rate:.0f} Hz")
        self.lbl_peak_angle.setText(f"{self.peak_angle:.1f}°")
        self.lbl_peak_vel.setText(f"{self.peak_vel:.1f}°/s")

        if self.is_connected:
            self.lbl_telemetry.setText(f"Time: {self.elapsed_time:.1f}s | Port: {self.serial_client.port if self.serial_client else 'N/A'} | {self.sample_rate:.0f} Hz")
        else:
            self.lbl_telemetry.setText(f"Time: {self.elapsed_time:.1f}s | Port: Searching for hardware endpoint...")

        self.canvas_widget.update_state(self.theta)

    def tick_graph(self):
        if not self._data_dirty: return
        self._data_dirty = False
        t, a, v, u = self._get_buf_slices()
        
        if self.invert_display:
            a_disp = [(360.0 - x) % 360.0 for x in a]
            v_disp = [-x for x in v]
        else:
            a_disp = a
            v_disp = v

        # Tab 1 Curves
        self.angle_curve.setData(t, a_disp)
        self.vel_curve.setData(t, v_disp)
        self.action_curve.setData(t, u)
        
        # Tab 2 Phase Portrait Curve
        self.phase_curve.setData(a_disp, v_disp)

    def closeEvent(self, event):
        if self.serial_client:
            self.serial_client.stop_client()
        event.accept()

    # ── Keyboard Manual Override ──
    def keyPressEvent(self, event):
        if event.isAutoRepeat() or self.ctrl_panel.is_balancing or self.oscillation_ctrl.enabled:
            return
        if event.key() == Qt.Key.Key_A and self.serial_client:
            self.serial_client.send_command(cmd_motor(255))
            self.current_action = 255
        elif event.key() == Qt.Key.Key_D and self.serial_client:
            self.serial_client.send_command(cmd_motor(-255))
            self.current_action = -255
        elif event.key() == Qt.Key.Key_Space and self.serial_client:
            self.serial_client.send_command(cmd_brake())
            self.current_action = 0
            print("[KEYBOARD] BRAKE applied.")

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat() or self.ctrl_panel.is_balancing or self.oscillation_ctrl.enabled:
            return
        if event.key() in (Qt.Key.Key_A, Qt.Key.Key_D) and self.serial_client:
            self.serial_client.send_command(cmd_coast())
            self.current_action = 0
