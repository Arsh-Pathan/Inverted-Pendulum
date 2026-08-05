from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, 
                             QPushButton, QLabel, QSpinBox, QDoubleSpinBox, QCheckBox)
from PyQt6.QtCore import pyqtSignal
from .card_widget import CardWidget

class ControlPanel(QWidget):
    """
    Control Panel widget containing motor actuation triggers, HIL control mode selector
    (PID, LQR, RL, Hybrid Swing-Up), auto-balance toggle, and real-time gain tuning spinboxes.
    """
    start_clicked = pyqtSignal()
    stop_clicked = pyqtSignal()
    balance_toggled = pyqtSignal(bool)
    tare_clicked = pyqtSignal()
    mode_selected = pyqtSignal(str) # "PID", "LQR", "RL", "HYBRID"
    invert_display_toggled = pyqtSignal(bool)
    invert_motor_toggled = pyqtSignal(bool)
    
    speed_changed = pyqtSignal(int)
    duration_changed = pyqtSignal(int)
    pid_changed = pyqtSignal(dict) # dictionary of updated PID params

    manual_drive_started = pyqtSignal(int)
    manual_drive_stopped = pyqtSignal()
    manual_brake_clicked = pyqtSignal()

    def __init__(self, initial_config: dict, parent=None):
        super().__init__(parent)
        self.is_balancing = False
        self.active_mode = "PID"
        self.config = initial_config
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)

        ctrl_card = CardWidget("MOTOR & HIL CONTROL STATION")
        ctrl_layout = QVBoxLayout()
        ctrl_layout.setSpacing(10)

        # Row 1: Start / Stop Buttons
        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(10)

        self.btn_start = QPushButton("START OSCILLATION")
        self.btn_start.setStyleSheet("""
            QPushButton {
                background-color: #27ae60; color: white; font-weight: 700; font-size: 13px;
                border: 1px solid #1e8449; border-radius: 6px; padding: 10px 14px;
            }
            QPushButton:hover { background-color: #2ecc71; border-color: #27ae60; }
            QPushButton:pressed { background-color: #1e8449; }
        """)
        self.btn_start.clicked.connect(self.start_clicked.emit)

        self.btn_stop = QPushButton("STOP / BRAKE")
        self.btn_stop.setStyleSheet("""
            QPushButton {
                background-color: #c0392b; color: white; font-weight: 700; font-size: 13px;
                border: 1px solid #922b21; border-radius: 6px; padding: 10px 14px;
            }
            QPushButton:hover { background-color: #e74c3c; border-color: #c0392b; }
            QPushButton:pressed { background-color: #922b21; }
        """)
        self.btn_stop.clicked.connect(self._on_stop_clicked)

        btn_layout.addWidget(self.btn_start)
        btn_layout.addWidget(self.btn_stop)
        ctrl_layout.addLayout(btn_layout)

        # Row 1.5: HIL Controller Mode Selector Bar
        lbl_mode = QLabel("SELECT BALANCING ALGORITHM (HIL ENGINE):")
        lbl_mode.setStyleSheet("font-size: 10px; font-weight: 800; margin-top: 4px;")
        ctrl_layout.addWidget(lbl_mode)

        mode_layout = QHBoxLayout()
        mode_layout.setSpacing(6)

        self.btn_mode_pid = QPushButton("PID Mode")
        self.btn_mode_lqr = QPushButton("LQR Mode")
        self.btn_mode_rl = QPushButton("RL Policy")
        self.btn_mode_hybrid = QPushButton("Hybrid Swing-Up")

        self.mode_buttons = {
            "PID": self.btn_mode_pid,
            "LQR": self.btn_mode_lqr,
            "RL": self.btn_mode_rl,
            "HYBRID": self.btn_mode_hybrid
        }

        for mode, btn in self.mode_buttons.items():
            btn.clicked.connect(lambda _, m=mode: self.select_mode(m))
            mode_layout.addWidget(btn)

        ctrl_layout.addLayout(mode_layout)
        self._update_mode_button_styles()

        # Row 1.7: Display & Motor Transformation Toggles
        self.chk_invert_display = QCheckBox("Invert Displayed Angle (UI & Charts Only - Leaves Control Logic Untouched)")
        self.chk_invert_display.setStyleSheet("font-size: 11px; font-weight: bold; margin-top: 2px;")
        self.chk_invert_display.setChecked(False)
        self.chk_invert_display.toggled.connect(self.invert_display_toggled.emit)
        ctrl_layout.addWidget(self.chk_invert_display)

        self.chk_invert_motor = QCheckBox("Reverse Motor Direction (only if the motor leads are wired backwards)")
        self.chk_invert_motor.setStyleSheet("font-size: 11px; font-weight: bold; color: #d35400; margin-top: 2px; margin-bottom: 4px;")
        # Unchecked by default: the controllers already drive the cart toward the fall.
        # Ticking it inverts control and the rig stabilises the pole hanging DOWN.
        self.chk_invert_motor.setChecked(False)
        self.chk_invert_motor.toggled.connect(self.invert_motor_toggled.emit)
        ctrl_layout.addWidget(self.chk_invert_motor)

        # Row 1.8: Auto-Balance & Tare Buttons
        action_layout = QHBoxLayout()
        action_layout.setSpacing(10)

        self.btn_balance = QPushButton("Start Auto-Balance [PID Mode]")
        self.btn_balance.setStyleSheet("""
            QPushButton {
                background-color: #2980b9; color: white; font-weight: 700; font-size: 14px;
                border: 1px solid #1c5980; border-radius: 6px; padding: 12px;
            }
            QPushButton:hover { background-color: #3498db; border-color: #2980b9; }
        """)
        self.btn_balance.clicked.connect(self._toggle_balance)

        self.btn_tare = QPushButton("Tare Zero")
        self.btn_tare.setStyleSheet("""
            QPushButton {
                background-color: #d35400; color: white; font-weight: 700; font-size: 14px;
                border: 1px solid #a04000; border-radius: 6px; padding: 12px;
            }
            QPushButton:hover { background-color: #e67e22; border-color: #d35400; }
        """)
        self.btn_tare.clicked.connect(self.tare_clicked.emit)

        action_layout.addWidget(self.btn_balance, 3)
        action_layout.addWidget(self.btn_tare, 1)
        ctrl_layout.addLayout(action_layout)

        # Row 2: Settings (Speed & Oscillation Duration)
        settings_layout = QGridLayout()
        settings_layout.setSpacing(6)
        small_title_style = "font-size: 10px; font-weight: 800; color: #555555;"
        spinbox_style = """
            QSpinBox, QDoubleSpinBox {
                font-family: 'Consolas', monospace;
                font-size: 13px; font-weight: bold;
                border: 1px solid #cccccc; border-radius: 4px;
                padding: 4px 6px;
            }
        """

        lbl_speed = QLabel("TEST SPEED (0-255):")
        lbl_speed.setStyleSheet(small_title_style)
        self.spin_speed = QSpinBox()
        self.spin_speed.setRange(0, 255)
        self.spin_speed.setValue(self.config.get("oscillation", {}).get("speed", 255))
        self.spin_speed.setStyleSheet(spinbox_style)
        self.spin_speed.valueChanged.connect(self.speed_changed.emit)

        lbl_dist = QLabel("OSC DURATION (ms):")
        lbl_dist.setStyleSheet(small_title_style)
        self.spin_dist = QSpinBox()
        self.spin_dist.setRange(10, 2000)
        self.spin_dist.setSingleStep(50)
        self.spin_dist.setValue(self.config.get("oscillation", {}).get("duration_ms", 400))
        self.spin_dist.setStyleSheet(spinbox_style)
        self.spin_dist.valueChanged.connect(self.duration_changed.emit)

        settings_layout.addWidget(lbl_speed, 0, 0)
        settings_layout.addWidget(self.spin_speed, 1, 0)
        settings_layout.addWidget(lbl_dist, 0, 1)
        settings_layout.addWidget(self.spin_dist, 1, 1)
        ctrl_layout.addLayout(settings_layout)

        # Row 2.5: Manual Drive Buttons (hold-to-drive, mirrors the A/D keyboard override)
        manual_drive_style = """
            QPushButton {
                background-color: #16a085; color: white; font-weight: 700; font-size: 13px;
                border: 1px solid #0e6655; border-radius: 6px; padding: 10px 12px;
            }
            QPushButton:hover { background-color: #1abc9c; border-color: #16a085; }
            QPushButton:pressed { background-color: #0e6655; }
        """
        manual_brake_style = """
            QPushButton {
                background-color: #c0392b; color: white; font-weight: 700; font-size: 13px;
                border: 1px solid #922b21; border-radius: 6px; padding: 10px 12px;
            }
            QPushButton:hover { background-color: #e74c3c; border-color: #c0392b; }
            QPushButton:pressed { background-color: #922b21; }
        """

        lbl_manual = QLabel("MANUAL DRIVE (HOLD TO MOVE):")
        lbl_manual.setStyleSheet(small_title_style)
        ctrl_layout.addWidget(lbl_manual)

        manual_layout = QHBoxLayout()
        manual_layout.setSpacing(8)

        self.spin_manual_speed = QSpinBox()
        self.spin_manual_speed.setRange(0, 255)
        self.spin_manual_speed.setSingleStep(5)
        self.spin_manual_speed.setValue(self.config.get("control", {}).get("manual_speed", 120))
        self.spin_manual_speed.setStyleSheet(spinbox_style)
        self.spin_manual_speed.setToolTip("Manual drive power (0-255)")
        self.spin_manual_speed.valueChanged.connect(lambda v: self.pid_changed.emit({"manual_speed": v}))
        manual_layout.addWidget(self.spin_manual_speed)

        self.btn_manual_left = QPushButton("LEFT")
        self.btn_manual_left.setStyleSheet(manual_drive_style)
        self.btn_manual_left.setToolTip("Hold to drive the cart left")
        self.btn_manual_left.pressed.connect(lambda: self.manual_drive_started.emit(-self.spin_manual_speed.value()))
        self.btn_manual_left.released.connect(self.manual_drive_stopped.emit)
        manual_layout.addWidget(self.btn_manual_left, 1)

        self.btn_manual_right = QPushButton("RIGHT")
        self.btn_manual_right.setStyleSheet(manual_drive_style)
        self.btn_manual_right.setToolTip("Hold to drive the cart right")
        self.btn_manual_right.pressed.connect(lambda: self.manual_drive_started.emit(self.spin_manual_speed.value()))
        self.btn_manual_right.released.connect(self.manual_drive_stopped.emit)
        manual_layout.addWidget(self.btn_manual_right, 1)

        self.btn_manual_brake = QPushButton("STOP")
        self.btn_manual_brake.setStyleSheet(manual_brake_style)
        self.btn_manual_brake.setToolTip("Hard brake the motor")
        self.btn_manual_brake.clicked.connect(self.manual_brake_clicked.emit)
        manual_layout.addWidget(self.btn_manual_brake, 1)

        ctrl_layout.addLayout(manual_layout)

        # Row 3: PID Tuning Spinboxes
        pid_layout = QHBoxLayout()
        pid_layout.setSpacing(8)

        ctrl_cfg = self.config.get("control", {})

        # KP
        kp_vbox = QVBoxLayout()
        lbl_kp = QLabel("KP (PROP):")
        lbl_kp.setStyleSheet(small_title_style)
        self.spin_kp = QDoubleSpinBox()
        self.spin_kp.setRange(0.0, 100.0)
        self.spin_kp.setSingleStep(0.5)
        self.spin_kp.setValue(ctrl_cfg.get("kp", 15.0))
        self.spin_kp.setStyleSheet(spinbox_style)
        self.spin_kp.valueChanged.connect(lambda v: self.pid_changed.emit({"kp": v}))
        kp_vbox.addWidget(lbl_kp)
        kp_vbox.addWidget(self.spin_kp)
        pid_layout.addLayout(kp_vbox)

        # KI
        ki_vbox = QVBoxLayout()
        lbl_ki = QLabel("KI (INT):")
        lbl_ki.setStyleSheet(small_title_style)
        self.spin_ki = QDoubleSpinBox()
        self.spin_ki.setRange(0.0, 50.0)
        self.spin_ki.setSingleStep(0.05)
        self.spin_ki.setValue(ctrl_cfg.get("ki", 0.0))
        self.spin_ki.setStyleSheet(spinbox_style)
        self.spin_ki.valueChanged.connect(lambda v: self.pid_changed.emit({"ki": v}))
        ki_vbox.addWidget(lbl_ki)
        ki_vbox.addWidget(self.spin_ki)
        pid_layout.addLayout(ki_vbox)

        # KD
        kd_vbox = QVBoxLayout()
        lbl_kd = QLabel("KD (DERIV):")
        lbl_kd.setStyleSheet(small_title_style)
        self.spin_kd = QDoubleSpinBox()
        self.spin_kd.setRange(0.0, 50.0)
        self.spin_kd.setSingleStep(0.1)
        self.spin_kd.setValue(ctrl_cfg.get("kd", 2.5))
        self.spin_kd.setStyleSheet(spinbox_style)
        self.spin_kd.valueChanged.connect(lambda v: self.pid_changed.emit({"kd": v}))
        kd_vbox.addWidget(lbl_kd)
        kd_vbox.addWidget(self.spin_kd)
        pid_layout.addLayout(kd_vbox)

        # ALPHA
        alpha_vbox = QVBoxLayout()
        lbl_alpha = QLabel("ALPHA (EMA):")
        lbl_alpha.setStyleSheet(small_title_style)
        self.spin_alpha = QDoubleSpinBox()
        self.spin_alpha.setRange(0.01, 1.00)
        self.spin_alpha.setSingleStep(0.01)
        self.spin_alpha.setValue(ctrl_cfg.get("alpha", 0.08))
        self.spin_alpha.setStyleSheet(spinbox_style)
        self.spin_alpha.valueChanged.connect(lambda v: self.pid_changed.emit({"alpha": v}))
        alpha_vbox.addWidget(lbl_alpha)
        alpha_vbox.addWidget(self.spin_alpha)
        pid_layout.addLayout(alpha_vbox)

        ctrl_layout.addLayout(pid_layout)
        ctrl_card.layout.addLayout(ctrl_layout)
        layout.addWidget(ctrl_card)

    def select_mode(self, mode: str):
        self.active_mode = mode
        self._update_mode_button_styles()
        if not self.is_balancing:
            self.btn_balance.setText(f"Start Auto-Balance [{mode} Mode]")
        else:
            self.btn_balance.setText(f"Stop Auto-Balance [{mode} Mode]")
        self.mode_selected.emit(mode)

    def _update_mode_button_styles(self):
        colors = {
            "PID": ("#2980b9", "#3498db"),
            "LQR": ("#8e44ad", "#9b59b6"),
            "RL": ("#16a085", "#1abc9c"),
            "HYBRID": ("#d35400", "#e67e22")
        }
        for mode, btn in self.mode_buttons.items():
            dark, light = colors.get(mode, ("#555555", "#777777"))
            if mode == self.active_mode:
                btn.setStyleSheet(f"""
                    QPushButton {{
                        background-color: {dark}; color: white; font-weight: 700; font-size: 12px;
                        border: 1px solid #111111; border-radius: 6px; padding: 8px 10px;
                    }}
                """)
            else:
                btn.setStyleSheet(f"""
                    QPushButton {{
                        background-color: #2a2a2a; color: #bbbbbb; font-weight: 600; font-size: 12px;
                        border: 1px solid #444444; border-radius: 6px; padding: 8px 10px;
                    }}
                    QPushButton:hover {{ background-color: {light}; color: white; border-color: {dark}; }}
                """)

    def _on_stop_clicked(self):
        if self.is_balancing:
            self._toggle_balance()
        self.stop_clicked.emit()

    def _toggle_balance(self):
        self.is_balancing = not self.is_balancing
        self.balance_toggled.emit(self.is_balancing)
        if self.is_balancing:
            self.btn_balance.setText(f"Stop Auto-Balance [{self.active_mode} Mode]")
            self.btn_balance.setStyleSheet("""
                QPushButton {
                    background-color: #c0392b; color: white; font-weight: 700; font-size: 14px;
                    border: 1px solid #922b21; border-radius: 6px; padding: 12px;
                }
                QPushButton:hover { background-color: #e74c3c; border-color: #c0392b; }
            """)
        else:
            self.btn_balance.setText(f"Start Auto-Balance [{self.active_mode} Mode]")
            self.btn_balance.setStyleSheet("""
                QPushButton {
                    background-color: #2980b9; color: white; font-weight: 700; font-size: 14px;
                    border: 1px solid #1c5980; border-radius: 6px; padding: 12px;
                }
                QPushButton:hover { background-color: #3498db; border-color: #2980b9; }
            """)
