import sys
import time
import math
import numpy as np
from PyQt6.QtWidgets import (QMainWindow, QWidget, QHBoxLayout, QVBoxLayout, 
                             QSplitter, QLabel, QGridLayout, QFrame)
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QFont
import pyqtgraph as pg

from ..utils.config_loader import load_config, save_config
from ..comms.serial_client import SerialClient
from ..comms.protocol import cmd_motor, cmd_brake, cmd_coast, cmd_zero_tare
from ..controllers.pid_balancer import PIDBalancer
from ..controllers.oscillation import OscillationController
from .card_widget import CardWidget
from .telemetry_canvas import TelemetryCanvas
from .control_panel import ControlPanel

QSS_STYLE = """
QMainWindow { background-color: #ffffff; }
QWidget { background-color: #ffffff; color: #000000; font-family: 'Inter', 'Segoe UI', sans-serif; }
QLabel { color: #000000; }
"""

class MainWindow(QMainWindow):
    """
    Main Application Window for the Inverted Pendulum HIL Platform.
    Integrates real-time CAD viewport, PyQtGraph telemetry charts, live gain tuning,
    and the Python-hosted closed-loop balancing engine.
    """
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Inverted Pendulum HIL Control Platform")
        self.resize(1600, 900)
        self.setStyleSheet(QSS_STYLE)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        # 1) Load Configurations
        self.config = load_config()

        # 2) Initialize Controllers
        ctrl_cfg = self.config.get("control", {})
        osc_cfg = self.config.get("oscillation", {})
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
        self.oscillation_ctrl = OscillationController(
            speed=osc_cfg.get("speed", 255),
            duration_ms=osc_cfg.get("duration_ms", 400)
        )

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

        # Ring Buffer for charting
        self.history_len = 800
        self._buf_time = np.zeros(self.history_len, dtype=np.float64)
        self._buf_angle = np.zeros(self.history_len, dtype=np.float64)
        self._buf_vel = np.zeros(self.history_len, dtype=np.float64)
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

        # ── Left Panel: Viewport & Status ──
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
        lbl_title = QLabel("Inverted Pendulum HIL Platform")
        lbl_title.setStyleSheet("font-size: 32px; font-weight: 800; color: #000000; letter-spacing: -0.5px;")
        lbl_sub = QLabel("Real-time Python Closed-Loop Control & Reinforcement Learning Station")
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

        # ── Right Panel: Controls & Charts ──
        right_widget = QWidget()
        right_widget.setFixedWidth(620)
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
        big_style = "font-size: 28px; font-family: 'Consolas', monospace; font-weight: bold; color: #000000;"
        small_style = "font-size: 15px; font-family: 'Consolas', monospace; font-weight: bold; color: #000000;"
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

        lbl_raw = QLabel("RAW SENSOR:")
        lbl_raw.setStyleSheet(lbl_style)
        self.lbl_raw_val = QLabel("—")
        self.lbl_raw_val.setStyleSheet(small_style)
        lbl_rate = QLabel("SAMPLE RATE:")
        lbl_rate.setStyleSheet(lbl_style)
        self.lbl_rate_val = QLabel("— Hz")
        self.lbl_rate_val.setStyleSheet(small_style)
        readout_grid.addWidget(lbl_raw, 4, 0)
        readout_grid.addWidget(lbl_rate, 4, 1)
        readout_grid.addWidget(self.lbl_raw_val, 5, 0)
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

        # Deviation Chart
        self.angle_card = CardWidget("DEVIATION VS TIME")
        self.angle_plot = pg.PlotWidget()
        self.angle_curve = self._style_chart(self.angle_plot, y_label="Deviation", y_unit="°", line_color="#0066cc")
        self.angle_plot.addItem(pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen("#ff3333", width=1.5, style=Qt.PenStyle.DashLine)))
        self.angle_card.layout.addWidget(self.angle_plot)
        right_layout.addWidget(self.angle_card, 1)

        # Velocity Chart
        self.vel_card = CardWidget("VELOCITY VS TIME")
        self.vel_plot = pg.PlotWidget()
        self.vel_curve = self._style_chart(self.vel_plot, y_label="Velocity", y_unit="°/s", line_color="#cc3333")
        self.vel_plot.addItem(pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen("#aaaaaa", width=1, style=Qt.PenStyle.DashLine)))
        self.vel_card.layout.addWidget(self.vel_plot)
        right_layout.addWidget(self.vel_card, 1)

        splitter.setSizes([980, 620])

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
        self.ctrl_panel.speed_changed.connect(lambda v: self.oscillation_ctrl.update_params({"speed": v}))
        self.ctrl_panel.duration_changed.connect(lambda v: self.oscillation_ctrl.update_params({"duration_ms": v}))
        self.ctrl_panel.pid_changed.connect(self._on_pid_changed)

    def _on_start_oscillation(self):
        print("[HIL MODE] Enabling Oscillation Controller...")
        self.pid_balancer.disable()
        self.oscillation_ctrl.enable()

    def _on_stop_all(self):
        print("[HIL MODE] Disabling all controllers. Braking motor...")
        self.pid_balancer.disable()
        self.oscillation_ctrl.disable()
        if self.serial_client:
            self.serial_client.send_command(cmd_brake())

    def _on_balance_toggled(self, is_enabled: bool):
        if is_enabled:
            print("[HIL MODE] Enabling Python PID Balancer...")
            self.oscillation_ctrl.disable()
            self.pid_balancer.enable()
        else:
            print("[HIL MODE] Disabling PID Balancer. Coasting motor...")
            self.pid_balancer.disable()
            if self.serial_client:
                self.serial_client.send_command(cmd_coast())

    def _on_tare_clicked(self):
        print("[HIL COMMAND] Triggering hardware tare calibration...")
        if self.serial_client:
            self.serial_client.send_command(cmd_zero_tare())

    def _on_pid_changed(self, params: dict):
        self.pid_balancer.update_params(params)
        # Update config dict and save to disk
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

        # Update Ring Buffer
        i = self._buf_idx % self.history_len
        self._buf_time[i] = now - self.start_time
        self._buf_angle[i] = self.angle_dev
        self._buf_vel[i] = self.vel_deg_s
        self._buf_idx += 1
        self._buf_count = min(self._buf_count + 1, self.history_len)
        self._data_dirty = True

        # ── HIL CLOSED-LOOP CONTROL EXECUTION ──
        if self.serial_client and self.serial_client.isRunning():
            if self.pid_balancer.enabled:
                power = self.pid_balancer.compute_action(self.angle_dev, dt)
                self.serial_client.send_command(cmd_motor(power))
            elif self.oscillation_ctrl.enabled:
                power = self.oscillation_ctrl.compute_action(self.angle_dev, dt)
                self.serial_client.send_command(cmd_motor(power))

    def _get_buf_slices(self):
        count = self._buf_count
        idx = self._buf_idx
        if count < self.history_len:
            return self._buf_time[:count], self._buf_angle[:count], self._buf_vel[:count]
        start = idx % self.history_len
        t = np.concatenate((self._buf_time[start:], self._buf_time[:start]))
        a = np.concatenate((self._buf_angle[start:], self._buf_angle[:start]))
        v = np.concatenate((self._buf_vel[start:], self._buf_vel[:start]))
        return t, a, v

    # ── GUI Redraw Timers ──
    def tick_fast(self):
        self.elapsed_time = (time.time() - self.start_time) if self.start_time else 0.0
        self.lbl_angle_val.setText(f"{self.angle_dev:+.2f}°")
        self.lbl_vel_val.setText(f"{self.vel_deg_s:+.1f}°/s")
        self.lbl_raw_val.setText(f"{self.raw_angle:.2f}°")
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
        t, a, v = self._get_buf_slices()
        self.angle_curve.setData(t, a)
        self.vel_curve.setData(t, v)

    def closeEvent(self, event):
        if self.serial_client:
            self.serial_client.stop_client()
        event.accept()

    # ── Keyboard Manual Override ──
    def keyPressEvent(self, event):
        if event.isAutoRepeat() or self.pid_balancer.enabled or self.oscillation_ctrl.enabled:
            return
        if event.key() == Qt.Key.Key_A and self.serial_client:
            self.serial_client.send_command(cmd_motor(-200))
        elif event.key() == Qt.Key.Key_D and self.serial_client:
            self.serial_client.send_command(cmd_motor(200))

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat() or self.pid_balancer.enabled or self.oscillation_ctrl.enabled:
            return
        if event.key() in (Qt.Key.Key_A, Qt.Key.Key_D) and self.serial_client:
            self.serial_client.send_command(cmd_coast())
