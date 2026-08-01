import time
import math
import numpy as np
import os
import io
from PyQt6.QtWidgets import (QMainWindow, QWidget, QHBoxLayout, QVBoxLayout, 
                             QSplitter, QLabel, QGridLayout, QFrame, QTabWidget, QPushButton,
                             QComboBox, QSpinBox, QLineEdit, QProgressBar, QTextEdit)
from PyQt6.QtCore import QTimer, Qt, QThread, pyqtSignal
from PyQt6.QtGui import QFont
import pyqtgraph as pg

from ..utils.config_loader import load_config, save_config
from ..comms.serial_client import SerialClient
from ..comms.protocol import cmd_motor, cmd_brake, cmd_coast, cmd_zero_tare
from ..math.controllers.pid_balancer import PIDBalancer
from ..math.controllers.lqr_balancer import LQRBalancer
from ..math.controllers.hybrid_balancer import HybridBalancer
from ..math.controllers.oscillation import OscillationController
try:
    from rl.rl_controller import RLBalancer
except ImportError:
    RLBalancer = None

try:
    from stable_baselines3 import PPO, SAC, TD3
    from stable_baselines3.common.callbacks import BaseCallback
    from stable_baselines3.common.monitor import Monitor
    _SB3_AVAILABLE = True
except ImportError:
    _SB3_AVAILABLE = False
    class BaseCallback:
        pass

class GUITrainingCallback(BaseCallback):
    def __init__(self, worker, verbose=0):
        super().__init__(verbose)
        self.worker = worker

    def _on_step(self) -> bool:
        if self.worker.is_stopped:
            return False
            
        self.worker.progress_signal.emit(self.num_timesteps)
        
        if "infos" in self.locals:
            for info in self.locals["infos"]:
                if "episode" in info:
                    ep_reward = info["episode"]["r"]
                    self.worker.reward_signal.emit(self.num_timesteps, float(ep_reward))
                    
        return True

class RLTrainingWorker(QThread):
    progress_signal = pyqtSignal(int)
    reward_signal = pyqtSignal(int, float)
    log_signal = pyqtSignal(str)
    finished_signal = pyqtSignal(str)
    error_signal = pyqtSignal(str)

    def __init__(self, algo, timesteps, save_path):
        super().__init__()
        self.algo = algo
        self.timesteps = timesteps
        self.save_path = save_path
        self.is_stopped = False

    def run(self):
        import sys
        import contextlib

        class StreamLogger(io.StringIO):
            def __init__(self, signal):
                super().__init__()
                self.signal = signal

            def write(self, s):
                if s.strip():
                    self.signal.emit(s.strip())
                pass

        stream = StreamLogger(self.log_signal)
        
        try:
            with contextlib.redirect_stdout(stream), contextlib.redirect_stderr(stream):
                self._train()
        except Exception as e:
            self.error_signal.emit(str(e))
        
    def _train(self):
        if not _SB3_AVAILABLE:
            raise RuntimeError("stable-baselines3 is not installed.")
        
        print(f"Starting {self.algo} training for {self.timesteps} timesteps...")
        os.makedirs(os.path.dirname(self.save_path) or ".", exist_ok=True)
        
        from algorithm.math.envs.inverted_pendulum_env import InvertedPendulumEnv
        
        env = InvertedPendulumEnv(simulated=True, max_episode_steps=1000)
        env = Monitor(env)
        
        if self.algo == "PPO":
            model = PPO("MlpPolicy", env, verbose=1)
        elif self.algo == "SAC":
            model = SAC("MlpPolicy", env, verbose=1)
        elif self.algo == "TD3":
            from stable_baselines3.common.noise import NormalActionNoise
            import numpy as np
            n_actions = env.action_space.shape[-1]
            action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))
            model = TD3("MlpPolicy", env, action_noise=action_noise, verbose=1)
        else:
            raise ValueError(f"Unknown algorithm: {self.algo}")

        callback = GUITrainingCallback(self)
        model.learn(total_timesteps=self.timesteps, callback=callback)
        
        if not self.is_stopped:
            model.save(self.save_path)
            print(f"Model saved to {self.save_path}")
            self.finished_signal.emit(self.save_path)
        else:
            print("Training stopped manually.")
            self.finished_signal.emit("")
            
    def stop(self):
        self.is_stopped = True

from .card_widget import CardWidget
from .telemetry_canvas import TelemetryCanvas
from .control_panel import ControlPanel


QSS_STYLE_DARK = """
QMainWindow { background-color: #121212; }
QWidget { background-color: #121212; color: #e0e0e0; font-family: 'Inter', 'Segoe UI', sans-serif; }
QLabel { color: #e0e0e0; }
CardWidget { background-color: #1e1e1e; border: 1px solid #333333; border-radius: 8px; }
QTabWidget::pane { border: 1px solid #333333; border-radius: 6px; background: #1e1e1e; }
QTabBar::tab {
    background: #1e1e1e; border: 1px solid #333333; padding: 8px 16px;
    font-weight: bold; font-size: 13px; color: #888888; border-top-left-radius: 6px; border-top-right-radius: 6px;
}
QTabBar::tab:selected { background: #2a2a2a; color: #3498db; border-bottom: 2px solid #3498db; }
QTabBar::tab:hover { background: #252525; color: #ffffff; }
QPushButton { background: #2a2a2a; color: #ffffff; border: 1px solid #444444; padding: 6px 12px; border-radius: 4px; font-weight: 600; }
QPushButton:hover { background: #3a3a3a; border-color: #555555; }
QPushButton:checked { background: #3498db; border-color: #2980b9; color: #ffffff; }
QComboBox, QSpinBox, QDoubleSpinBox, QLineEdit {
    background-color: #1e1e1e; color: #ffffff; border: 1px solid #444444; border-radius: 4px; padding: 5px 8px;
    font-family: 'Consolas', monospace; font-size: 12px; font-weight: bold;
}
QComboBox::drop-down { border: none; }
QComboBox QAbstractItemView { background-color: #1e1e1e; color: #ffffff; selection-background-color: #3498db; }
QProgressBar { border: 1px solid #444444; border-radius: 4px; text-align: center; color: white; font-weight: bold; background: #1e1e1e; }
QProgressBar::chunk { background-color: #27ae60; border-radius: 3px; }
QTextEdit { background-color: #181818; color: #2ecc71; border: 1px solid #333333; border-radius: 4px; font-family: 'Consolas', monospace; }
"""

QSS_STYLE = """
QMainWindow { background-color: #f8f9fa; }
QWidget { background-color: #f8f9fa; color: #2c3e50; font-family: 'Inter', 'Segoe UI', sans-serif; }
QLabel { color: #2c3e50; }
CardWidget { background-color: #ffffff; border: 1px solid #e2e8f0; border-radius: 8px; }
QTabWidget::pane { border: 1px solid #e2e8f0; border-radius: 6px; background: #ffffff; }
QTabBar::tab {
    background: #edf2f7; border: 1px solid #e2e8f0; padding: 8px 16px;
    font-weight: bold; font-size: 13px; color: #718096; border-top-left-radius: 6px; border-top-right-radius: 6px;
}
QTabBar::tab:selected { background: #ffffff; color: #3182ce; border-bottom: 2px solid #3182ce; }
QTabBar::tab:hover { background: #e2e8f0; color: #2d3748; }
QPushButton { background: #edf2f7; color: #2d3748; border: 1px solid #cbd5e0; padding: 6px 12px; border-radius: 4px; font-weight: 600; }
QPushButton:hover { background: #e2e8f0; border-color: #a0aec0; }
QPushButton:checked { background: #3182ce; border-color: #2b6cb0; color: #ffffff; }
QComboBox, QSpinBox, QDoubleSpinBox, QLineEdit {
    background-color: #ffffff; color: #2d3748; border: 1px solid #cbd5e0; border-radius: 4px; padding: 5px 8px;
    font-family: 'Consolas', monospace; font-size: 12px; font-weight: bold;
}
QComboBox QAbstractItemView { background-color: #ffffff; color: #2d3748; selection-background-color: #3182ce; selection-color: #ffffff; }
QProgressBar { border: 1px solid #cbd5e0; border-radius: 4px; text-align: center; color: #2d3748; font-weight: bold; background: #edf2f7; }
QProgressBar::chunk { background-color: #38a169; border-radius: 3px; }
QTextEdit { background-color: #1a202c; color: #48bb78; border: 1px solid #cbd5e0; border-radius: 4px; font-family: 'Consolas', monospace; }
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
            kp=ctrl_cfg.get("kp", 20.0),
            ki=ctrl_cfg.get("ki", 0.0),
            kd=ctrl_cfg.get("kd", 2.5),
            alpha=ctrl_cfg.get("alpha", 0.45),
            min_power=ctrl_cfg.get("min_motor_power", 35),
            max_power=ctrl_cfg.get("max_motor_power", 255),
            deadzone_deg=ctrl_cfg.get("equilibrium_deadzone_deg", 0.0),
            deadzone_vel=ctrl_cfg.get("equilibrium_deadzone_vel", 0.0),
            k_cart_v=ctrl_cfg.get("k_cart_v", 150.0),
            k_cart_x=ctrl_cfg.get("k_cart_x", 200.0),
            cart_accel_max=ctrl_cfg.get("cart_accel_max", 6.0),
            cart_damping=ctrl_cfg.get("cart_damping", 7.5),
            dither_power=ctrl_cfg.get("dither_power", 0)
        )
        self.lqr_balancer = LQRBalancer(
            k_theta=ctrl_cfg.get("k_theta", 25.0),
            k_omega=ctrl_cfg.get("k_omega", 3.5),
            alpha=ctrl_cfg.get("alpha", 0.45),
            min_power=ctrl_cfg.get("min_motor_power", 35),
            max_power=ctrl_cfg.get("max_motor_power", 255),
            deadzone_deg=ctrl_cfg.get("equilibrium_deadzone_deg", 0.0),
            deadzone_vel=ctrl_cfg.get("equilibrium_deadzone_vel", 0.0),
            k_cart_v=ctrl_cfg.get("k_cart_v", 150.0),
            k_cart_x=ctrl_cfg.get("k_cart_x", 200.0),
            cart_accel_max=ctrl_cfg.get("cart_accel_max", 6.0),
            cart_damping=ctrl_cfg.get("cart_damping", 7.5),
            dither_power=ctrl_cfg.get("dither_power", 0),
            input_gain_n_per_pwm=ctrl_cfg.get("input_gain_n_per_pwm", 0.008),
            control_loop_rate_hz=ctrl_cfg.get("control_loop_rate_hz", 200.0)
        )
        self.hybrid_balancer = HybridBalancer(
            stabilizer=self.pid_balancer
        )
        if RLBalancer is not None:
            self.rl_balancer = RLBalancer(
                model_path=None,
                min_power=ctrl_cfg.get("min_motor_power", 35),
                max_power=ctrl_cfg.get("max_motor_power", 255)
            )
        else:
            self.rl_balancer = None

        self.oscillation_ctrl = OscillationController(
            speed=osc_cfg.get("speed", 255),
            duration_ms=osc_cfg.get("duration_ms", 400)
        )

        self.active_mode = "PID" # "PID", "LQR", "RL", "HYBRID"
        self.current_action = 0
        self.invert_display = False
        # MUST stay False. The controllers already emit a catch-the-fall command in the
        # canonical +theta convention (see core/state.py), so this must NOT negate it.
        #
        # Verified on hardware: with this True the cart drove opposite to the control law
        # and the rig stabilised the pole HANGING DOWN - the signature of an inverted
        # sign, since negating a catch-the-fall law yields one that damps toward hanging.
        #
        # This flag is only for a physically reversed motor (swapped TB6612 A01/A02).
        self.invert_motor = False

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
        self._buf_reward = np.zeros(self.history_len, dtype=np.float64)
        self._buf_cumreward = np.zeros(self.history_len, dtype=np.float64)
        self._buf_iae = np.zeros(self.history_len, dtype=np.float64)
        self._buf_idx = 0
        self._buf_count = 0

        # Running performance accumulators (control-quality analytics)
        self._sum_sq_err = 0.0      # Σ θ_err²  → RMS error
        self._sum_abs_err = 0.0     # Σ |θ_err| dt → IAE (integral absolute error)
        self._sum_abs_pwm = 0.0     # Σ |PWM| → mean actuator effort
        self._sum_reward = 0.0      # Σ instantaneous reward → cumulative reward
        self._metric_n = 0          # sample count for averaging
        self._balanced_n = 0        # samples within settling tolerance
        self._settle_tol_deg = 2.0  # ±2° settling band

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

        # ── Left Sidebar: Header, CAD Viewport, Controls & Metrics (fixed width) ──
        left_widget = QWidget()
        left_widget.setFixedWidth(600)
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(14)
        splitter.addWidget(left_widget)

        # Header Block
        header_widget = QWidget()
        header_layout = QVBoxLayout(header_widget)
        header_layout.setContentsMargins(0, 0, 0, 0)
        header_layout.setSpacing(4)
        lbl_title = QLabel("Inverted Pendulum HIL Research Station")
        lbl_title.setStyleSheet("font-size: 30px; font-weight: 800; letter-spacing: -0.5px;")
        lbl_sub = QLabel("Real-time AI Control (RL/PPO, LQR, Hybrid Swing-Up, PID) & Dynamic Phase Portraits")
        lbl_sub.setStyleSheet("font-size: 13px; font-weight: 600;")
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
        self.lbl_telemetry.setStyleSheet("font-weight: 600; font-size: 13px;")
        status_row.addWidget(self.lbl_telemetry)
        status_row.addStretch()

        self.btn_dark_mode = QPushButton("Dark Mode")
        self.btn_dark_mode.setCheckable(True)
        self.btn_dark_mode.clicked.connect(self.toggle_dark_mode)
        status_row.addWidget(self.btn_dark_mode)

        self.status_dot = QFrame()
        self.status_dot.setFixedSize(10, 10)
        self.status_dot.setStyleSheet("border: 1px solid #000000; border-radius: 5px; background-color: #888888;")
        self.status_text = QLabel("Offline")
        self.status_text.setStyleSheet("font-weight: 600; font-size: 13px;")
        status_row.addWidget(self.status_dot)
        status_row.addWidget(self.status_text)
        sim_card.layout.addLayout(status_row)
        left_layout.addWidget(sim_card, 1)

        # Control Panel (lives in the left sidebar beneath the viewport)
        self.ctrl_panel = ControlPanel(self.config)
        self._wire_control_panel()
        left_layout.addWidget(self.ctrl_panel)

        # ── Right Panel: Wide Charts & Metrics Area (stretches to fill window) ──
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(12)
        splitter.addWidget(right_widget)

        # ── Live Metrics Card: wide horizontal strip atop the charts area ──
        metrics_card = CardWidget("LIVE HIL TELEMETRY & CONTROL-QUALITY METRICS")
        readout_grid = QGridLayout()
        readout_grid.setSpacing(6)
        readout_grid.setColumnStretch(0, 1)
        readout_grid.setColumnStretch(1, 1)
        readout_grid.setColumnStretch(2, 1)
        readout_grid.setColumnStretch(3, 1)
        big_style = "font-size: 30px; font-family: 'Consolas', monospace; font-weight: bold;"
        small_style = "font-size: 15px; font-family: 'Consolas', monospace; font-weight: bold;"
        lbl_style = "font-size: 10px; font-weight: 800;"

        def add_stat(row, col, title, init):
            """Create a stacked (caption + value) readout cell and return the value label."""
            cap = QLabel(title)
            cap.setStyleSheet(lbl_style)
            val = QLabel(init)
            val.setStyleSheet(small_style)
            box = QVBoxLayout()
            box.setSpacing(1)
            box.addWidget(cap)
            box.addWidget(val)
            readout_grid.addLayout(box, row, col)
            return val

        # Row 0: two large primary readouts spanning two columns each
        dev_cap = QLabel("DEVIATION FROM ZERO")
        dev_cap.setStyleSheet(lbl_style)
        self.lbl_angle_val = QLabel("0.00°")
        self.lbl_angle_val.setStyleSheet(big_style)
        dev_box = QVBoxLayout(); dev_box.setSpacing(1)
        dev_box.addWidget(dev_cap); dev_box.addWidget(self.lbl_angle_val)
        readout_grid.addLayout(dev_box, 0, 0, 1, 2)

        vel_cap = QLabel("ANGULAR VELOCITY")
        vel_cap.setStyleSheet(lbl_style)
        self.lbl_vel_val = QLabel("0.0°/s")
        self.lbl_vel_val.setStyleSheet(big_style)
        vel_box = QVBoxLayout(); vel_box.setSpacing(1)
        vel_box.addWidget(vel_cap); vel_box.addWidget(self.lbl_vel_val)
        readout_grid.addLayout(vel_box, 0, 2, 1, 2)

        # Divider between primary readouts and the analytics grid
        divider = QFrame()
        divider.setFrameShape(QFrame.Shape.HLine)
        divider.setStyleSheet("color: #cccccc; background-color: #cccccc; max-height: 1px;")
        readout_grid.addWidget(divider, 1, 0, 1, 4)

        # Rows 2-3: eight compact secondary stats across four columns
        self.lbl_action_val = add_stat(2, 0, "MOTOR ACTION (PWM)", "0 [COAST]")
        self.lbl_rate_val   = add_stat(2, 1, "SAMPLE RATE", "— Hz")
        self.lbl_peak_angle = add_stat(2, 2, "PEAK ANGLE", "0.0°")
        self.lbl_peak_vel   = add_stat(2, 3, "PEAK VELOCITY", "0.0°/s")
        self.lbl_rms_val    = add_stat(3, 0, "RMS ERROR", "0.00°")
        self.lbl_iae_val    = add_stat(3, 1, "IAE (∫|θ|dt)", "0.0")
        self.lbl_effort_val = add_stat(3, 2, "MEAN |PWM| EFFORT", "0.0")
        self.lbl_uptime_val = add_stat(3, 3, "BALANCE UPTIME", "0.0%")
        self.lbl_reward_val = add_stat(4, 0, "CUMULATIVE REWARD", "0.0")
        self.lbl_settle_val = add_stat(4, 1, "SETTLING BAND", f"±{self._settle_tol_deg:.1f}°")

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

        self.tab_widget.addTab(tab_time, "Time-Series Telemetry and Actuation")

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
        
        # Settling-tolerance ring: dashed circle marking the ±settle_tol_deg band around upright
        theta_ring = np.linspace(0, 2 * math.pi, 90)
        ring_r = self._settle_tol_deg
        ring = pg.PlotCurveItem(
            ring_r * np.cos(theta_ring), ring_r * 20.0 * np.sin(theta_ring),
            pen=pg.mkPen("#2ecc71", width=1.2, style=Qt.PenStyle.DashLine)
        )
        p_item.addItem(ring)

        # Upright origin target indicator (Green dot at 0, 0)
        target_dot = pg.ScatterPlotItem([0], [0], pen=pg.mkPen("#00aa00", width=2), brush=pg.mkBrush("#2ecc71"), size=14)
        p_item.addItem(target_dot)

        # Faded historical trail beneath the live trajectory
        self.phase_trail = p_item.plot(pen=pg.mkPen("#d3b3e6", width=1.0))
        self.phase_curve = p_item.plot(pen=pg.mkPen("#8e44ad", width=2.0))
        # Current-state marker (leading dot)
        self.phase_head = pg.ScatterPlotItem([], [], pen=pg.mkPen("#8e44ad", width=1),
                                             brush=pg.mkBrush("#8e44ad"), size=10)
        p_item.addItem(self.phase_head)
        self.phase_card.layout.addWidget(self.phase_plot)
        tab_phase_layout.addWidget(self.phase_card)

        self.tab_widget.addTab(tab_phase, "State-Space Phase Portrait")

        # TAB 3: Control-Cost / Reward Analytics — side-by-side (mirrors the RL reward function)
        tab_reward = QWidget()
        tab_reward_layout = QHBoxLayout(tab_reward)
        tab_reward_layout.setContentsMargins(4, 8, 4, 4)
        tab_reward_layout.setSpacing(8)

        self.reward_card = CardWidget("INSTANTANEOUS CONTROL REWARD VS TIME")
        self.reward_plot = pg.PlotWidget()
        self.reward_curve = self._style_chart(self.reward_plot, y_label="Reward", y_unit="", line_color="#e67e22")
        self.reward_plot.addItem(pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen("#aaaaaa", width=1, style=Qt.PenStyle.DashLine)))
        self.reward_card.layout.addWidget(self.reward_plot)
        tab_reward_layout.addWidget(self.reward_card, 1)

        self.cumreward_card = CardWidget("CUMULATIVE REWARD (RETURN) VS TIME")
        self.cumreward_plot = pg.PlotWidget()
        self.cumreward_curve = self._style_chart(self.cumreward_plot, y_label="Σ Reward", y_unit="", line_color="#16a085")
        self.cumreward_card.layout.addWidget(self.cumreward_plot)
        tab_reward_layout.addWidget(self.cumreward_card, 1)

        self.tab_widget.addTab(tab_reward, "Control-Cost / Reward")

        # TAB 4: Performance Analytics — two histograms side-by-side, cumulative IAE full-width below
        tab_stats = QWidget()
        tab_stats_layout = QGridLayout(tab_stats)
        tab_stats_layout.setContentsMargins(4, 8, 4, 4)
        tab_stats_layout.setSpacing(8)
        tab_stats_layout.setColumnStretch(0, 1)
        tab_stats_layout.setColumnStretch(1, 1)
        tab_stats_layout.setRowStretch(0, 1)
        tab_stats_layout.setRowStretch(1, 1)

        self.hist_err_card = CardWidget("ANGLE-ERROR DISTRIBUTION (HISTOGRAM)")
        self.hist_err_plot = pg.PlotWidget()
        self.hist_err_plot.setBackground("#ffffff")
        hi = self.hist_err_plot.getPlotItem()
        hi.showGrid(x=True, y=True, alpha=0.15)
        hi.setLabel("bottom", "Angle Deviation (θ)", units="°", **{"font-size": "10px", "color": "#555555"})
        hi.setLabel("left", "Count", **{"font-size": "10px", "color": "#555555"})
        self.hist_err_bars = pg.BarGraphItem(x=[0], height=[0], width=0.8, brush="#0066cc", pen=pg.mkPen("#003f7f"))
        hi.addItem(self.hist_err_bars)
        self.hist_err_card.layout.addWidget(self.hist_err_plot)
        tab_stats_layout.addWidget(self.hist_err_card, 0, 0)

        self.hist_pwm_card = CardWidget("PWM-EFFORT DISTRIBUTION (HISTOGRAM)")
        self.hist_pwm_plot = pg.PlotWidget()
        self.hist_pwm_plot.setBackground("#ffffff")
        pi = self.hist_pwm_plot.getPlotItem()
        pi.showGrid(x=True, y=True, alpha=0.15)
        pi.setLabel("bottom", "PWM Duty", **{"font-size": "10px", "color": "#555555"})
        pi.setLabel("left", "Count", **{"font-size": "10px", "color": "#555555"})
        self.hist_pwm_bars = pg.BarGraphItem(x=[0], height=[0], width=0.8, brush="#27ae60", pen=pg.mkPen("#145a32"))
        pi.addItem(self.hist_pwm_bars)
        self.hist_pwm_card.layout.addWidget(self.hist_pwm_plot)
        tab_stats_layout.addWidget(self.hist_pwm_card, 0, 1)

        self.iae_card = CardWidget("CUMULATIVE ABSOLUTE ERROR (IAE) VS TIME")
        self.iae_plot = pg.PlotWidget()
        self.iae_curve = self._style_chart(self.iae_plot, y_label="∫|θ|dt", y_unit="", line_color="#c0392b")
        self.iae_card.layout.addWidget(self.iae_plot)
        tab_stats_layout.addWidget(self.iae_card, 1, 0, 1, 2)

        self.tab_widget.addTab(tab_stats, "Performance Analytics")

        # TAB 5: RL Training Studio
        tab_rl = QWidget()
        tab_rl_layout = QHBoxLayout(tab_rl)
        tab_rl_layout.setContentsMargins(4, 8, 4, 4)
        tab_rl_layout.setSpacing(12)
        
        rl_ctrl_panel = QWidget()
        rl_ctrl_panel.setFixedWidth(300)
        rl_ctrl_layout = QVBoxLayout(rl_ctrl_panel)
        
        rl_ctrl_layout.addWidget(QLabel("Algorithm:"))
        self.rl_algo_combo = QComboBox()
        self.rl_algo_combo.addItems(["PPO", "SAC", "TD3"])
        rl_ctrl_layout.addWidget(self.rl_algo_combo)
        
        rl_ctrl_layout.addWidget(QLabel("Timesteps:"))
        self.rl_timesteps_spin = QSpinBox()
        self.rl_timesteps_spin.setRange(10000, 500000)
        self.rl_timesteps_spin.setSingleStep(10000)
        self.rl_timesteps_spin.setValue(50000)
        rl_ctrl_layout.addWidget(self.rl_timesteps_spin)
        
        rl_ctrl_layout.addWidget(QLabel("Save Path:"))
        self.rl_save_path_edit = QLineEdit("rl/models/ppo_pendulum.zip")
        rl_ctrl_layout.addWidget(self.rl_save_path_edit)
        
        self.btn_start_rl = QPushButton("START RL TRAINING")
        self.btn_start_rl.setStyleSheet("background-color: #27ae60; font-weight: bold; padding: 10px;")
        self.btn_start_rl.clicked.connect(self._on_start_rl_training)
        rl_ctrl_layout.addWidget(self.btn_start_rl)
        
        self.btn_stop_rl = QPushButton("STOP TRAINING")
        self.btn_stop_rl.setStyleSheet("background-color: #c0392b; font-weight: bold; padding: 10px;")
        self.btn_stop_rl.setEnabled(False)
        self.btn_stop_rl.clicked.connect(self._on_stop_rl_training)
        rl_ctrl_layout.addWidget(self.btn_stop_rl)
        
        self.rl_progress = QProgressBar()
        self.rl_progress.setRange(0, 50000)
        rl_ctrl_layout.addWidget(self.rl_progress)
        
        rl_ctrl_layout.addStretch()
        tab_rl_layout.addWidget(rl_ctrl_panel)
        
        rl_view_panel = QWidget()
        rl_view_layout = QVBoxLayout(rl_view_panel)
        
        self.rl_plot = pg.PlotWidget(title="Training Reward")
        self.rl_curve = self._style_chart(self.rl_plot, y_label="Episode Reward", line_color="#8e44ad")
        rl_view_layout.addWidget(self.rl_plot, 2)
        
        self.rl_log = QTextEdit()
        self.rl_log.setReadOnly(True)
        self.rl_log.setStyleSheet("background-color: #ffffff; color: #000000; font-family: Consolas;")
        rl_view_layout.addWidget(self.rl_log, 1)
        
        tab_rl_layout.addWidget(rl_view_panel, 1)
        self.tab_widget.addTab(tab_rl, "RL Training Studio")

        # Charts area (right) stretches; fixed-width sidebar (left) holds viewport + controls.
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([600, 1050])


    def toggle_dark_mode(self, checked):
        if checked:
            self.setStyleSheet(QSS_STYLE_DARK)
            self.btn_dark_mode.setText("Light Mode")
            # Update plot backgrounds
            self.canvas_widget.is_dark_mode = True
            self.canvas_widget.update()
            self.angle_plot.setBackground("#1e1e1e")
            self.vel_plot.setBackground("#1e1e1e")
            self.action_plot.setBackground("#1e1e1e")
            self.phase_plot.setBackground("#1e1e1e")
            self.reward_plot.setBackground("#1e1e1e")
            self.cumreward_plot.setBackground("#1e1e1e")
            self.hist_err_plot.setBackground("#1e1e1e")
            self.hist_pwm_plot.setBackground("#1e1e1e")
            self.rl_plot.setBackground("#1e1e1e")
            self.rl_log.setStyleSheet("background-color: #1e1e1e; color: #00ff00; font-family: Consolas;")
        else:
            self.setStyleSheet(QSS_STYLE)
            self.btn_dark_mode.setText("Dark Mode")
            self.canvas_widget.is_dark_mode = False
            self.canvas_widget.update()
            self.angle_plot.setBackground("#ffffff")
            self.vel_plot.setBackground("#ffffff")
            self.action_plot.setBackground("#ffffff")
            self.phase_plot.setBackground("#ffffff")
            self.reward_plot.setBackground("#ffffff")
            self.cumreward_plot.setBackground("#ffffff")
            self.hist_err_plot.setBackground("#ffffff")
            self.hist_pwm_plot.setBackground("#ffffff")
            self.rl_plot.setBackground("#ffffff")
            self.rl_log.setStyleSheet("background-color: #ffffff; color: #000000; font-family: Consolas;")

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
        self.ctrl_panel.invert_motor_toggled.connect(self._on_invert_motor_toggled)
        self.ctrl_panel.speed_changed.connect(lambda v: self.oscillation_ctrl.update_params({"speed": v}))
        self.ctrl_panel.duration_changed.connect(lambda v: self.oscillation_ctrl.update_params({"duration_ms": v}))
        self.ctrl_panel.pid_changed.connect(self._on_pid_changed)

    def _on_invert_display_toggled(self, checked: bool):
        self.invert_display = checked
        print(f"[UI DISPLAY] Invert displayed angle set to: {checked}")

    def _on_invert_motor_toggled(self, checked: bool):
        self.invert_motor = checked
        print(f"[MOTOR ACTION] Reverse motor direction set to: {checked}")

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
        self.reset_analytics()
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
        self.reset_analytics()

        # Reset local filter/gate state so the new firmware zero takes effect cleanly.
        self.prev_raw = None
        self.angle_history = []
        self._reject_streak = 0
        if hasattr(self, "_last_good_raw"):
            del self._last_good_raw
        if hasattr(self, "smooth_vel"):
            self.smooth_vel = 0.0

        # Firmware owns zero-referencing: the 'Z' command re-tares on-device.
        if self.serial_client:
            self.serial_client.send_command(cmd_zero_tare())

    def _on_start_rl_training(self):
        algo = self.rl_algo_combo.currentText()
        timesteps = self.rl_timesteps_spin.value()
        save_path = self.rl_save_path_edit.text()
        
        self.rl_progress.setRange(0, timesteps)
        self.rl_progress.setValue(0)
        self.rl_log.clear()
        self.rl_log.append(f"Initializing {algo} training...")
        
        self.rl_times = []
        self.rl_rewards = []
        self.rl_curve.setData(self.rl_times, self.rl_rewards)
        
        self.btn_start_rl.setEnabled(False)
        self.btn_stop_rl.setEnabled(True)
        
        self.rl_worker = RLTrainingWorker(algo, timesteps, save_path)
        self.rl_worker.progress_signal.connect(self._on_rl_progress)
        self.rl_worker.reward_signal.connect(self._on_rl_reward)
        self.rl_worker.log_signal.connect(self._on_rl_log)
        self.rl_worker.finished_signal.connect(self._on_rl_finished)
        self.rl_worker.error_signal.connect(self._on_rl_error)
        self.rl_worker.start()

    def _on_stop_rl_training(self):
        if hasattr(self, "rl_worker") and self.rl_worker.isRunning():
            self.rl_log.append("Stop requested. Waiting for current epoch to finish...")
            self.rl_worker.stop()
            self.btn_stop_rl.setEnabled(False)

    def _on_rl_progress(self, current_step):
        self.rl_progress.setValue(current_step)

    def _on_rl_reward(self, current_step, reward):
        self.rl_times.append(current_step)
        self.rl_rewards.append(reward)
        self.rl_curve.setData(self.rl_times, self.rl_rewards)

    def _on_rl_log(self, text):
        self.rl_log.append(text)
        scrollbar = self.rl_log.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def _on_rl_finished(self, model_path):
        self.btn_start_rl.setEnabled(True)
        self.btn_stop_rl.setEnabled(False)
        
        if model_path and os.path.exists(model_path):
            self.rl_log.append(f"Training completed successfully! Model saved at {model_path}")
            if self.rl_balancer:
                try:
                    self.rl_balancer.load_model(model_path)
                    self.rl_log.append("Success: Model loaded into RL balancer.")
                    self.rl_log.append("You can now select 'RL' and click 'Start Auto-Balance [RL Mode]'.")
                except Exception as e:
                    self.rl_log.append(f"Error loading model into balancer: {e}")
            else:
                self.rl_log.append("Warning: RLBalancer is not available.")
        else:
            self.rl_log.append("Training stopped or failed.")

    def _on_rl_error(self, err_msg):
        self.btn_start_rl.setEnabled(True)
        self.btn_stop_rl.setEnabled(False)
        self.rl_log.append(f"ERROR: {err_msg}")

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

    def reset_analytics(self):
        """Clear peaks and running control-quality accumulators.

        Peaks and Σ-accumulators only ever grow, so a single glitch (or just a long
        session) can leave them showing stale/poisoned values. Reset them on demand
        (STOP / tare) to keep the metrics panel trustworthy.
        """
        self.peak_angle = 0.0
        self.peak_vel = 0.0
        self._sum_sq_err = 0.0
        self._sum_abs_err = 0.0
        self._sum_abs_pwm = 0.0
        self._sum_reward = 0.0
        self._metric_n = 0
        self._balanced_n = 0
        print("[METRICS] Analytics accumulators and peaks reset.")

    # ── High-Frequency Closed-Loop HIL Telemetry & Actuator Step ──
    def on_angle_received(self, raw_val: float):
        now = time.time()
        dt = now - self.last_time if self.last_time > 0 else 0.01
        self.last_time = now
        if self.start_time is None:
            self.start_time = now

        # NOTE: Zero-referencing is owned entirely by the firmware (boot auto-tare + the
        # 'Z' command), which streams an already-calibrated angle. We deliberately do NOT
        # apply a second software offset here — stacking two independent tare references
        # produced a small, accumulating calibration error over time.

        # ── Outlier / Glitch Gate ──
        # The AS5600 occasionally returns corrupt I2C reads (0x000 / 0xFFF or bit-flips)
        # that appear as huge instantaneous jumps. A single glitch over one ~10ms sample
        # implies thousands of deg/s and permanently poisons peaks/stats downstream.
        # Reject any reading that implies physically-impossible motion; hold the last good
        # value. If divergence persists (real motion or a genuine re-position), re-sync so
        # we never get stuck rejecting forever.
        if not hasattr(self, '_last_good_raw'):
            self._last_good_raw = raw_val
            self._reject_streak = 0
        # Max plausible travel between samples (deg): dt * max_slew, clamped for jitter in dt.
        max_slew_deg_s = 3000.0
        max_jump = max(20.0, min(90.0, max_slew_deg_s * dt))
        jump = abs(((raw_val - self._last_good_raw + 180.0) % 360.0) - 180.0)
        if jump > max_jump and self._reject_streak < 5:
            # Treat as a glitch: discard this sample and reuse the last good reading.
            self._reject_streak += 1
            raw_val = self._last_good_raw
        else:
            # Accept: either plausible motion, or sustained divergence forcing a re-sync.
            self._reject_streak = 0
            self._last_good_raw = raw_val

        # ── Noise Rejection: 5-tap Median Filter to reject EMI / I2C glitch spikes ──
        # A wider window kills isolated AS5600 count-glitches better than 3-tap.
        if not hasattr(self, 'angle_history'):
            self.angle_history = []
        self.angle_history.append(raw_val)
        if len(self.angle_history) > 5:
            self.angle_history.pop(0)

        if len(self.angle_history) >= 3:
            # Wrap-aware median: rank samples by circular distance from the newest reading.
            base = self.angle_history[-1]
            sorted_angles = sorted(self.angle_history, key=lambda a: abs(((a - base + 180.0) % 360.0) - 180.0))
            filtered_raw = sorted_angles[len(sorted_angles) // 2]
        else:
            filtered_raw = raw_val

        # ── Adaptive EMA (never freezes) ──
        # The AS5600 is 12-bit (~0.088°/count) and jitters ±1–2 counts at rest. We adapt the
        # smoothing factor to motion magnitude — heavier smoothing when nearly still, near
        # pass-through when slewing — but crucially we NEVER set alpha to 0. A hard freeze
        # blinds the balancing controllers (which read self.angle_dev) to the small, slow
        # corrections needed to catch the pole near upright. Idle spikes are handled by the
        # glitch gate above, so a modest always-on alpha keeps the display calm without
        # starving the control loop.
        if self.prev_raw is None:
            self.smooth_raw = filtered_raw
        else:
            diff = ((filtered_raw - self.smooth_raw + 180.0) % 360.0) - 180.0
            mag = abs(diff)
            # Floor of 0.35 (tracks tiny movements) ramping to 0.9 (near pass-through) when slewing.
            alpha = min(0.9, 0.35 + (mag / 4.0))
            self.smooth_raw = (self.smooth_raw + alpha * diff) % 360.0

        self.raw_angle = self.smooth_raw
        self.angle_dev = self.smooth_raw % 360.0

        # Wrap-aware, heavily-smoothed angular velocity with a rest floor.
        # Velocity is a differentiator (×sample-rate), so it amplifies angle noise the most —
        # apply a stronger EMA and snap tiny residuals to zero.
        if self.prev_raw is not None and dt > 0:
            delta = (self.smooth_raw - self.prev_raw) % 360.0
            if delta > 180.0: delta -= 360.0
            elif delta < -180.0: delta += 360.0
            raw_vel = delta / dt
            if not hasattr(self, 'smooth_vel'):
                self.smooth_vel = raw_vel
            else:
                # alpha=0.55: the controllers apply their own EMA on top of this one, and
                # a 0.2 factor here added ~40 ms of lag which cascaded with the
                # controller's filter to ~155 ms - roughly one pendulum instability time
                # constant (148 ms). The damping term then arrived a full time constant
                # late, which is precisely what produces overshoot at the target.
                self.smooth_vel = (0.55 * raw_vel) + (0.45 * self.smooth_vel)
            # Snap near-zero velocity to exactly zero to stop idle drift on the readout/plots.
            if abs(self.smooth_vel) < 3.0:
                self.smooth_vel = 0.0
            self.vel_deg_s = max(-2000.0, min(2000.0, self.smooth_vel))
        self.prev_raw = self.smooth_raw

        # Shortest-path angle [-180, +180] for clean plotting and peak calculation
        short_a = ((self.angle_dev + 180.0) % 360.0) - 180.0

        # Update Canvas representation
        self.theta = math.radians(short_a)
        self.peak_angle = max(self.peak_angle, abs(short_a))
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
                
                if self.invert_motor:
                    power = -power
                self.serial_client.send_command(cmd_motor(power))
                self.current_action = power
            elif self.oscillation_ctrl.enabled:
                power = self.oscillation_ctrl.compute_action(self.angle_dev, dt)
                if self.invert_motor:
                    power = -power
                self.serial_client.send_command(cmd_motor(power))
                self.current_action = power
            else:
                self.current_action = 0

        # ── Control-Quality Analytics (accumulate performance statistics) ──
        # IMPORTANT: for a balancing task the error must be measured from UPRIGHT (180°),
        # not from the hanging zero. `short_a` above is deviation-from-hanging (used for the
        # "DEVIATION FROM ZERO" display); using it here would reward hanging and punish
        # balancing. `err_upright_deg` is 0 at upright and ±180 when hanging.
        err_upright_deg = (self.angle_dev % 360.0) - 180.0
        err_rad = math.radians(err_upright_deg)
        vel_rad = math.radians(self.vel_deg_s)
        # Instantaneous reward mirrors the RL env: holding bonus + spin penalty + quadratic cost
        in_band = abs(err_upright_deg) <= self._settle_tol_deg
        holding_bonus = 10.0 if in_band else 0.0
        is_spinning = abs(self.vel_deg_s) > 360.0
        spin_penalty = (50.0 + 0.15 * (abs(self.vel_deg_s) - 360.0)) if is_spinning else 0.0
        norm_action = float(self.current_action) / 255.0
        # Cost the NORMALISED action, matching the RL env. Using (norm_action*255)**2 here
        # re-expanded the command back to raw PWM, inflating the effort term by 255^2
        # (~65,000x) and swamping every other reward component.
        inst_reward = holding_bonus - spin_penalty - (err_rad**2 + 0.2 * vel_rad**2 + 0.001 * norm_action**2)

        self._sum_sq_err += err_upright_deg**2
        self._sum_abs_err += abs(err_upright_deg) * dt
        self._sum_abs_pwm += abs(float(self.current_action))
        self._sum_reward += inst_reward
        self._metric_n += 1
        if in_band:
            self._balanced_n += 1

        # Update Ring Buffer
        i = self._buf_idx % self.history_len
        self._buf_time[i] = now - self.start_time
        self._buf_angle[i] = self.angle_dev
        self._buf_vel[i] = self.vel_deg_s
        self._buf_action[i] = float(self.current_action)
        self._buf_reward[i] = inst_reward
        self._buf_cumreward[i] = self._sum_reward
        self._buf_iae[i] = self._sum_abs_err
        self._buf_idx += 1
        self._buf_count = min(self._buf_count + 1, self.history_len)
        self._data_dirty = True

    def _get_buf_slices(self):
        count = self._buf_count
        idx = self._buf_idx
        if count < self.history_len:
            sl = slice(0, count)
            return (self._buf_time[sl], self._buf_angle[sl], self._buf_vel[sl],
                    self._buf_action[sl], self._buf_reward[sl], self._buf_cumreward[sl], self._buf_iae[sl])
        start = idx % self.history_len
        def roll(buf):
            return np.concatenate((buf[start:], buf[:start]))
        return (roll(self._buf_time), roll(self._buf_angle), roll(self._buf_vel),
                roll(self._buf_action), roll(self._buf_reward), roll(self._buf_cumreward), roll(self._buf_iae))

    # ── GUI Redraw Timers ──
    def tick_fast(self):
        self.elapsed_time = (time.time() - self.start_time) if self.start_time else 0.0
        short_a = ((self.angle_dev + 180.0) % 360.0) - 180.0
        disp_a = -short_a if self.invert_display else short_a
        disp_v = -self.vel_deg_s if self.invert_display else self.vel_deg_s
        self.lbl_angle_val.setText(f"{disp_a:+.2f}°")
        self.lbl_vel_val.setText(f"{disp_v:+.1f}°/s")
        self.lbl_action_val.setText(f"{self.current_action:+} PWM")
        self.lbl_rate_val.setText(f"{self.sample_rate:.0f} Hz")
        self.lbl_peak_angle.setText(f"{self.peak_angle:.1f}°")
        self.lbl_peak_vel.setText(f"{self.peak_vel:.1f}°/s")

        # ── Control-Quality Analytics readouts ──
        if self._metric_n > 0:
            rms = math.sqrt(self._sum_sq_err / self._metric_n)
            mean_effort = self._sum_abs_pwm / self._metric_n
            uptime_pct = 100.0 * self._balanced_n / self._metric_n
            self.lbl_rms_val.setText(f"{rms:.2f}°")
            self.lbl_iae_val.setText(f"{self._sum_abs_err:.1f}")
            self.lbl_effort_val.setText(f"{mean_effort:.1f}")
            self.lbl_uptime_val.setText(f"{uptime_pct:.1f}%")
            self.lbl_reward_val.setText(f"{self._sum_reward:.0f}")

        if self.is_connected:
            self.lbl_telemetry.setText(f"Time: {self.elapsed_time:.1f}s | Port: {self.serial_client.port if self.serial_client else 'N/A'} | {self.sample_rate:.0f} Hz")
        else:
            self.lbl_telemetry.setText(f"Time: {self.elapsed_time:.1f}s | Port: Searching for hardware endpoint...")

        self.canvas_widget.update_state(self.theta)

    def tick_graph(self):
        if not self._data_dirty: return
        self._data_dirty = False
        t, a, v, u, r, cr, iae = self._get_buf_slices()

        short_angles = ((np.asarray(a) + 180.0) % 360.0) - 180.0
        v = np.asarray(v)
        if self.invert_display:
            a_disp = -short_angles
            v_disp = -v
        else:
            a_disp = short_angles
            v_disp = v

        # Tab 1: Time-Series Curves
        self.angle_curve.setData(t, a_disp)
        self.vel_curve.setData(t, v_disp)
        self.action_curve.setData(t, u)

        # Tab 2: Phase Portrait — faded full trail, recent bright segment, leading head marker
        self.phase_trail.setData(a_disp, v_disp)
        tail = 120  # highlight the most recent trajectory segment
        self.phase_curve.setData(a_disp[-tail:], v_disp[-tail:])
        if len(a_disp) > 0:
            self.phase_head.setData([a_disp[-1]], [v_disp[-1]])

        # Tab 3: Control-Cost / Reward
        self.reward_curve.setData(t, np.asarray(r))
        self.cumreward_curve.setData(t, np.asarray(cr))

        # Tab 4: Performance Analytics — histograms + cumulative IAE
        if len(a_disp) > 1:
            a_min, a_max = a_disp.min(), a_disp.max()
            if np.isclose(a_min, a_max):
                a_min -= 0.1
                a_max += 0.1
            err_counts, err_edges = np.histogram(a_disp, bins=25, range=(a_min, a_max))
            err_centers = (err_edges[:-1] + err_edges[1:]) / 2.0
            err_width = max(1e-5, (err_edges[1] - err_edges[0]) * 0.9)
            self.hist_err_bars.setOpts(x=err_centers, height=err_counts, width=err_width)

            u_arr = np.asarray(u)
            u_min, u_max = u_arr.min(), u_arr.max()
            if np.isclose(u_min, u_max):
                u_min -= 1.0
                u_max += 1.0
            pwm_counts, pwm_edges = np.histogram(u_arr, bins=25, range=(u_min, u_max))
            pwm_centers = (pwm_edges[:-1] + pwm_edges[1:]) / 2.0
            pwm_width = max(1e-3, (pwm_edges[1] - pwm_edges[0]) * 0.9)
            self.hist_pwm_bars.setOpts(x=pwm_centers, height=pwm_counts, width=pwm_width)

        self.iae_curve.setData(t, np.asarray(iae))

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
