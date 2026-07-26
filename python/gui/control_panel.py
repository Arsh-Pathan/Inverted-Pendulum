from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, 
                             QPushButton, QLabel, QSpinBox, QDoubleSpinBox)
from PyQt6.QtCore import pyqtSignal
from .card_widget import CardWidget

class ControlPanel(QWidget):
    """
    Control Panel widget containing motor actuation triggers, auto-balance toggle,
    and real-time PID gain tuning spinboxes.
    """
    start_clicked = pyqtSignal()
    stop_clicked = pyqtSignal()
    balance_toggled = pyqtSignal(bool)
    tare_clicked = pyqtSignal()
    
    speed_changed = pyqtSignal(int)
    duration_changed = pyqtSignal(int)
    pid_changed = pyqtSignal(dict) # dictionary of updated PID params

    def __init__(self, initial_config: dict, parent=None):
        super().__init__(parent)
        self.is_balancing = False
        self.config = initial_config
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)

        ctrl_card = CardWidget("MOTOR & HIL CONTROL")
        ctrl_layout = QVBoxLayout()
        ctrl_layout.setSpacing(10)

        # Row 1: Start / Stop Buttons
        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(10)

        self.btn_start = QPushButton("START OSCILLATION")
        self.btn_start.setStyleSheet("""
            QPushButton {
                background-color: #2ecc71; color: white; font-weight: bold; font-size: 14px;
                border: none; border-radius: 4px; padding: 10px;
            }
            QPushButton:hover { background-color: #27ae60; }
            QPushButton:pressed { background-color: #1e8449; }
        """)
        self.btn_start.clicked.connect(self.start_clicked.emit)

        self.btn_stop = QPushButton("STOP / BRAKE")
        self.btn_stop.setStyleSheet("""
            QPushButton {
                background-color: #e74c3c; color: white; font-weight: bold; font-size: 14px;
                border: none; border-radius: 4px; padding: 10px;
            }
            QPushButton:hover { background-color: #c0392b; }
            QPushButton:pressed { background-color: #922b21; }
        """)
        self.btn_stop.clicked.connect(self._on_stop_clicked)

        btn_layout.addWidget(self.btn_start)
        btn_layout.addWidget(self.btn_stop)
        ctrl_layout.addLayout(btn_layout)

        # Row 1.5: Auto-Balance & Tare Buttons
        action_layout = QHBoxLayout()
        action_layout.setSpacing(10)

        self.btn_balance = QPushButton("Start Auto-Balance (HIL)")
        self.btn_balance.setStyleSheet("""
            QPushButton {
                background-color: #3498db; color: white; font-weight: bold; font-size: 15px;
                border: none; border-radius: 4px; padding: 12px;
            }
            QPushButton:hover { background-color: #2980b9; }
        """)
        self.btn_balance.clicked.connect(self._toggle_balance)

        self.btn_tare = QPushButton("Tare Zero")
        self.btn_tare.setStyleSheet("""
            QPushButton {
                background-color: #f39c12; color: white; font-weight: bold; font-size: 15px;
                border: none; border-radius: 4px; padding: 12px;
            }
            QPushButton:hover { background-color: #e67e22; }
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
                color: #000000; background: #f5f5f5;
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

    def _on_stop_clicked(self):
        if self.is_balancing:
            self._toggle_balance()
        self.stop_clicked.emit()

    def _toggle_balance(self):
        self.is_balancing = not self.is_balancing
        self.balance_toggled.emit(self.is_balancing)
        if self.is_balancing:
            self.btn_balance.setText("Stop Auto-Balance (HIL)")
            self.btn_balance.setStyleSheet("""
                QPushButton {
                    background-color: #9b59b6; color: white; font-weight: bold; font-size: 15px;
                    border: none; border-radius: 4px; padding: 12px;
                }
                QPushButton:hover { background-color: #8e44ad; }
            """)
        else:
            self.btn_balance.setText("Start Auto-Balance (HIL)")
            self.btn_balance.setStyleSheet("""
                QPushButton {
                    background-color: #3498db; color: white; font-weight: bold; font-size: 15px;
                    border: none; border-radius: 4px; padding: 12px;
                }
                QPushButton:hover { background-color: #2980b9; }
            """)
