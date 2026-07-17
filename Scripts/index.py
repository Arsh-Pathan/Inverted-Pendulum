import sys
import math
import time
import random
from collections import deque
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QFrame, QHBoxLayout,
                             QVBoxLayout, QGridLayout, QLabel, QPushButton, QComboBox,
                             QSlider, QSplitter)
from PyQt6.QtCore import QTimer, Qt, QThread, pyqtSignal, QPointF
from PyQt6.QtGui import QPainter, QPen, QColor, QBrush, QFont, QPolygonF
import pyqtgraph as pg
import serial
import serial.tools.list_ports

# ---------------------------------------------------------
# Styling Sheet matching the CAD/monochrome HTML Dashboard
# ---------------------------------------------------------
QSS_STYLE = """
QMainWindow {
    background-color: #000000;
}
QWidget {
    background-color: #000000;
    color: #ffffff;
    font-family: 'Inter', 'Segoe UI', 'Arial', sans-serif;
}
QLabel {
    color: #ffffff;
}
QPushButton {
    background-color: #ffffff;
    color: #000000;
    border: 1px solid #ffffff;
    border-radius: 4px;
    padding: 8px 16px;
    font-size: 13px;
    font-weight: bold;
}
QPushButton:hover {
    background-color: #dddddd;
}
QPushButton:pressed {
    background-color: #aaaaaa;
}
QPushButton#btnReset {
    background-color: transparent;
    color: #ffffff;
    border: 1px solid #ffffff;
}
QPushButton#btnReset:hover {
    background-color: rgba(255, 255, 255, 0.15);
}
QComboBox {
    background-color: #000000;
    color: #ffffff;
    border: 1px solid #ffffff;
    border-radius: 4px;
    padding: 5px 10px;
    font-family: 'Consolas', monospace;
    font-size: 12px;
}
QComboBox::drop-down {
    border: none;
}
QComboBox QAbstractItemView {
    background-color: #000000;
    color: #ffffff;
    selection-background-color: #ffffff;
    selection-color: #000000;
    border: 1px solid #ffffff;
}
QSlider::groove:horizontal {
    border: 1px solid #555555;
    height: 4px;
    background: #000000;
}
QSlider::handle:horizontal {
    background: #ffffff;
    border: 1px solid #ffffff;
    width: 12px;
    height: 12px;
    margin: -4px 0;
    border-radius: 6px;
}
QSlider::handle:horizontal:hover {
    background: #dddddd;
    border-color: #dddddd;
}
"""

# Helper to scan available COM ports
def get_com_ports():
    ports = [p.device for p in serial.tools.list_ports.comports()]
    if not ports:
        # Fallback dummy ports if none exist
        if sys.platform.startswith('win'):
            ports = ["COM3", "COM4", "COM5"]
        else:
            ports = ["/dev/ttyUSB0", "/dev/ttyACM0"]
    return ports

# ---------------------------------------------------------
# Threaded Serial Reader to prevent GUI freezes
# ---------------------------------------------------------
class SerialReader(QThread):
    data_received = pyqtSignal(float)
    status_changed = pyqtSignal(str, str) # text, color (dot state)

    def __init__(self, port, baud=115200):
        super().__init__()
        self.port = port
        self.baud = baud
        self.running = False

    def run(self):
        self.running = True
        try:
            # Short timeout to avoid blocking indefinitely on close/stop
            ser = serial.Serial(self.port, self.baud, timeout=0.01)
            self.status_changed.emit(f"Reading {self.port}", "green")

            # Flush input buffer on startup
            ser.reset_input_buffer()

            while self.running:
                if ser.in_waiting > 0:
                    try:
                        line = ser.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            # Parse float telemetry (assuming it sends degrees around 180)
                            val = float(line)
                            self.data_received.emit(val)
                    except ValueError:
                        pass # skip parsing errors
                self.msleep(2)
            ser.close()
            self.status_changed.emit("Idle", "gray")
        except Exception as e:
            self.status_changed.emit(f"COM Error: {str(e)[:16]}", "red")

    def stop(self):
        self.running = False
        self.wait()

# ---------------------------------------------------------
# Card Container matching dashboard styles
# ---------------------------------------------------------
class CardWidget(QFrame):
    def __init__(self, title, parent=None):
        super().__init__(parent)
        self.setFrameShape(QFrame.Shape.StyledPanel)
        self.setStyleSheet("""
            CardWidget {
                background-color: #000000;
                border: 1px solid #ffffff;
                border-radius: 8px;
            }
        """)
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(16, 16, 16, 16)
        self.layout.setSpacing(8)

        if title:
            self.title_label = QLabel(title)
            self.title_label.setStyleSheet("""
                font-family: 'Inter', sans-serif;
                font-size: 13px;
                font-weight: 800;
                color: #ffffff;
                border: none;
                border-bottom: 1px solid #ffffff;
                padding-bottom: 4px;
                margin-bottom: 4px;
            """)
            self.layout.addWidget(self.title_label)

# ---------------------------------------------------------
# CAD/Engineering Simulation Viewport
# ---------------------------------------------------------
class CanvasWidget(QWidget):
    # Click interaction emits impulse magnitude
    disturbed = pyqtSignal(float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(380)
        self.setMouseTracking(True)
        self.cart_x = 0.0
        self.theta = 0.0  # radians (0 is vertical upright)
        self.force = 0.0
        self.limit = 2.4

    def update_state(self, x, theta, force):
        self.cart_x = x
        self.theta = theta
        self.force = force
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # Clear background to pure black
        painter.fillRect(self.rect(), QColor(0, 0, 0))

        # CAD Grid overlay
        painter.setPen(QPen(QColor(20, 20, 20), 1, Qt.PenStyle.SolidLine))
        grid_size = 40
        for x in range(0, self.width(), grid_size):
            painter.drawLine(x, 0, x, self.height())
        for y in range(0, self.height(), grid_size):
            painter.drawLine(0, y, self.width(), y)

        # Base dimensions
        w, h = self.width(), self.height()
        cy = h / 2.0

        # Pixels-per-meter scaling
        scale = w / 6.0

        # Draw rail line
        painter.setPen(QPen(QColor(255, 255, 255), 2))
        painter.drawLine(0, int(cy + 15), w, int(cy + 15))

        # Rail scale ticks
        painter.setPen(QPen(QColor(150, 150, 150), 1))
        painter.setFont(QFont("Consolas", 8))
        for m in range(-3, 4):
            tx = int(w / 2.0 + m * scale)
            painter.drawLine(tx, int(cy + 15), tx, int(cy + 22))
            painter.drawText(tx - 15, int(cy + 34), f"{m:+.1f}m")

        # Bounds warnings
        painter.setPen(QPen(QColor(255, 50, 50, 90), 1, Qt.PenStyle.DashLine))
        left_bound = int(w / 2.0 - self.limit * scale)
        right_bound = int(w / 2.0 + self.limit * scale)
        painter.drawLine(left_bound, 0, left_bound, h)
        painter.drawLine(right_bound, 0, right_bound, h)

        # Cart position on screen
        cx = w / 2.0 + self.cart_x * scale

        # Draw cart (CAD style: outlined, black filled)
        cart_w, cart_h = 70, 30
        painter.setPen(QPen(QColor(255, 255, 255), 2))
        painter.setBrush(QBrush(QColor(0, 0, 0)))
        painter.drawRect(int(cx - cart_w/2), int(cy - cart_h/2), cart_w, cart_h)

        # Wheels
        wheel_r = 8
        painter.drawEllipse(QPointF(cx - 20, cy + 15), wheel_r, wheel_r)
        painter.drawEllipse(QPointF(cx + 20, cy + 15), wheel_r, wheel_r)

        # Upright reference target line (dotted)
        painter.setPen(QPen(QColor(255, 255, 255, 40), 1, Qt.PenStyle.DotLine))
        painter.drawLine(int(cx), 0, int(cx), int(cy))

        # Rod (drawn pivoting around center of cart)
        pole_len = 130
        px = cx + math.sin(self.theta) * pole_len
        py = cy - math.cos(self.theta) * pole_len

        painter.setPen(QPen(QColor(255, 255, 255), 4, Qt.PenStyle.SolidLine, Qt.PenCapStyle.SquareCap))
        painter.drawLine(QPointF(cx, cy), QPointF(px, py))

        # Bob (heavy mass)
        bob_r = 12
        painter.setBrush(QBrush(QColor(255, 255, 255)))
        painter.drawEllipse(QPointF(px, py), bob_r, bob_r)

        # Hinge pin
        painter.setBrush(QBrush(QColor(0, 0, 0)))
        painter.setPen(QPen(QColor(255, 255, 255), 1.5))
        painter.drawEllipse(QPointF(cx, cy), 4, 4)

        # Draw applied force vector indicator
        if abs(self.force) > 0.05:
            painter.setPen(QPen(QColor(255, 255, 255), 1.5))
            painter.setBrush(QBrush(QColor(255, 255, 255)))

            arrow_y = cy - 25
            arrow_len = min(50, abs(self.force) * 2)

            if self.force > 0:
                end_x = cx - 38
                start_x = end_x - arrow_len
                painter.drawLine(int(start_x), int(arrow_y), int(end_x), int(arrow_y))
                poly = QPolygonF([
                    QPointF(end_x, arrow_y),
                    QPointF(end_x - 6, arrow_y - 4),
                    QPointF(end_x - 6, arrow_y + 4)
                ])
                painter.drawPolygon(poly)
            else:
                end_x = cx + 38
                start_x = end_x + arrow_len
                painter.drawLine(int(start_x), int(arrow_y), int(end_x), int(arrow_y))
                poly = QPolygonF([
                    QPointF(end_x, arrow_y),
                    QPointF(end_x + 6, arrow_y - 4),
                    QPointF(end_x + 6, arrow_y + 4)
                ])
                painter.drawPolygon(poly)

    def mousePressEvent(self, event):
        w = self.width()
        scale = w / 6.0
        cx = w / 2.0 + self.cart_x * scale
        click_x = event.position().x()

        # Apply push in direction clicked relative to cart
        impulse = 2.5 if click_x > cx else -2.5
        self.disturbed.emit(impulse)

# ---------------------------------------------------------
# Dynamic Neural Network Architecture Visualizer
# ---------------------------------------------------------
class NNWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(160)
        self.force = 0.0
        self.time = 0.0

    def update_state(self, force, t):
        self.force = force
        self.time = t
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor(0, 0, 0))

        layers = [4, 8, 8, 1]
        w, h = self.width(), self.height()

        layer_x = w / (len(layers) + 1)
        active_strength = min(1.0, abs(self.force) / 30.0 + 0.1)

        # Coordinate calculation
        nodes = []
        for l_idx, count in enumerate(layers):
            x = layer_x * (l_idx + 1)
            y_gap = h / (count + 1)
            layer_nodes = []
            for n_idx in range(count):
                layer_nodes.append((x, y_gap * (n_idx + 1)))
            nodes.append(layer_nodes)

        # Draw connections/edges
        for l_idx in range(len(layers) - 1):
            for n1_idx, n1 in enumerate(nodes[l_idx]):
                for n2_idx, n2 in enumerate(nodes[l_idx+1]):
                    # Fluctuate transparency to make it look active
                    pulse = (math.sin(self.time * 6.0 + n1_idx * 1.5 + n2_idx) + 1.0) / 2.0

                    if l_idx == len(layers) - 2:
                        # Output connection depends on force direction
                        color = QColor(255, 255, 255, int(active_strength * 255))
                        width = 1.5
                    else:
                        alpha = int((0.04 + 0.14 * pulse * active_strength) * 255)
                        color = QColor(255, 255, 255, alpha)
                        width = 1.0

                    painter.setPen(QPen(color, width))
                    painter.drawLine(QPointF(n1[0], n1[1]), QPointF(n2[0], n2[1]))

        # Draw vertices/nodes
        for l_idx, layer_nodes in enumerate(nodes):
            for n_idx, n in enumerate(layer_nodes):
                painter.setPen(Qt.PenStyle.NoPen)
                if l_idx == len(layers) - 1:
                    # Output node matches output force activation
                    painter.setBrush(QBrush(QColor(255, 255, 255) if self.force >= 0 else QColor(136, 136, 136)))
                else:
                    painter.setBrush(QBrush(QColor(255, 255, 255)))
                painter.drawEllipse(QPointF(n[0], n[1]), 4.0, 4.0)

# ---------------------------------------------------------
# Main Application Window
# ---------------------------------------------------------
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Inverted Pendulum Dashboard")
        self.resize(1400, 850)
        self.setStyleSheet(QSS_STYLE)

        # Physics/Simulation variables (0.0 rad is upright vertical)
        self.g = 9.8
        self.m = 0.8
        self.M = 1.0
        self.L = 0.5
        self.dt = 0.008
        self.x = 0.0
        self.x_dot = 0.0
        self.theta = 0.0 # start upright
        self.theta_dot = 0.0
        self.force = 0.0
        self.fallen_frames = 0
        self.elapsed_time = 0.0

        # State controllers
        self.is_running = False
        self.is_simulation = True # Fallback mode by default

        # Threading reader
        self.serial_thread = None

        # Data history buffers for charts
        self.history_len = 250
        self.angle_history = deque([180.0] * self.history_len, maxlen=self.history_len)
        self.pos_history = deque([0.0] * self.history_len, maxlen=self.history_len)
        self.force_history = deque([0.0] * self.history_len, maxlen=self.history_len)

        # Setup main layouts
        self.init_ui()

        # Configure real-time tick timer (8ms interval matches physics step dt)
        self.timer = QTimer()
        self.timer.timeout.connect(self.tick)
        self.timer.start(8)

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(20)

        # Main splitter (separates control/sim panel on left from graphs on right)
        splitter = QSplitter(Qt.Orientation.Horizontal)
        main_layout.addWidget(splitter)

        # Left side panel container
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(16)
        splitter.addWidget(left_widget)

        # 1. Header label block
        header_widget = QWidget()
        header_layout = QVBoxLayout(header_widget)
        header_layout.setContentsMargins(0, 0, 0, 0)
        header_layout.setSpacing(4)

        lbl_title = QLabel("Inverted Pendulum")
        lbl_title.setStyleSheet("""
            font-size: 32px;
            font-weight: 800;
            color: #ffffff;
            letter-spacing: -0.5px;
        """)
        lbl_subtitle = QLabel("Telemetry / Control CAD Dashboard")
        lbl_subtitle.setStyleSheet("""
            font-size: 13px;
            font-weight: 600;
            color: #aaaaaa;
        """)
        header_layout.addWidget(lbl_title)
        header_layout.addWidget(lbl_subtitle)
        left_layout.addWidget(header_widget)

        # 2. Main Simulation Viewport Card
        self.sim_card = CardWidget(None)
        self.canvas_widget = CanvasWidget()
        self.canvas_widget.disturbed.connect(self.apply_push)
        self.sim_card.layout.addWidget(self.canvas_widget, 1)

        # Row controls underneath the canvas
        ctrl_row = QHBoxLayout()
        ctrl_row.setSpacing(10)

        self.btn_start = QPushButton("Start")
        self.btn_start.clicked.connect(self.toggle_running)

        self.btn_reset = QPushButton("Reset")
        self.btn_reset.setObjectName("btnReset")
        self.btn_reset.clicked.connect(self.reset_all)

        self.btn_push = QPushButton("Push Cart")
        self.btn_push.clicked.connect(lambda: self.apply_push(random.choice([-2.0, 2.0])))

        # Telemetry metrics overlay label
        self.lbl_telemetry = QLabel("Time: 0.0s | Mode: SIM")
        self.lbl_telemetry.setStyleSheet("""
            color: #aaaaaa;
            font-weight: 600;
            font-size: 13px;
            margin-left: 10px;
        """)

        # Status LED indicator dot
        self.status_indicator = QHBoxLayout()
        self.status_indicator.setSpacing(6)
        self.status_dot = QFrame()
        self.status_dot.setFixedSize(10, 10)
        self.status_dot.setStyleSheet("""
            border: 1px solid #ffffff;
            border-radius: 5px;
            background-color: #555555;
        """)
        self.status_text = QLabel("Idle")
        self.status_text.setStyleSheet("font-weight: 600; font-size: 13px;")
        self.status_indicator.addWidget(self.status_dot)
        self.status_indicator.addWidget(self.status_text)

        ctrl_row.addWidget(self.btn_start)
        ctrl_row.addWidget(self.btn_reset)
        ctrl_row.addWidget(self.btn_push)
        ctrl_row.addWidget(self.lbl_telemetry)
        ctrl_row.addStretch()
        ctrl_row.addLayout(self.status_indicator)
        self.sim_card.layout.addLayout(ctrl_row)
        left_layout.addWidget(self.sim_card, 3)

        # 3. Parameters / Configuration Card
        config_card = CardWidget("SYSTEM CONFIGURATION")
        config_grid = QGridLayout()
        config_grid.setSpacing(15)

        # Column A: Connection Parameters
        config_grid.addWidget(QLabel("TELEMETRY SOURCE:"), 0, 0)
        self.cb_mode = QComboBox()
        self.cb_mode.addItems(["Simulation", "Serial COM Port"])
        self.cb_mode.currentTextChanged.connect(self.on_mode_changed)
        config_grid.addWidget(self.cb_mode, 0, 1)

        config_grid.addWidget(QLabel("SERIAL PORT:"), 1, 0)
        self.cb_port = QComboBox()
        self.cb_port.addItems(get_com_ports())
        config_grid.addWidget(self.cb_port, 1, 1)

        config_grid.addWidget(QLabel("BAUD RATE:"), 2, 0)
        self.cb_baud = QComboBox()
        self.cb_baud.addItems(["115200", "57600", "38400", "9600"])
        config_grid.addWidget(self.cb_baud, 2, 1)

        self.btn_connect = QPushButton("Connect")
        self.btn_connect.setEnabled(False) # Only valid in COM Port mode
        self.btn_connect.clicked.connect(self.toggle_connection)
        config_grid.addWidget(self.btn_connect, 3, 0, 1, 2)

        # Column B: PID Parameters (Simulation tuning)
        config_grid.addWidget(QLabel("ANGLE KP:"), 0, 2)
        self.sl_kp_theta = QSlider(Qt.Orientation.Horizontal)
        self.sl_kp_theta.setRange(0, 100)
        self.sl_kp_theta.setValue(60)
        config_grid.addWidget(self.sl_kp_theta, 0, 3)
        self.lbl_kp_theta = QLabel("60")
        self.lbl_kp_theta.setStyleSheet("font-family: 'Consolas'; min-width: 25px;")
        self.sl_kp_theta.valueChanged.connect(lambda v: self.lbl_kp_theta.setText(str(v)))
        config_grid.addWidget(self.lbl_kp_theta, 0, 4)

        config_grid.addWidget(QLabel("ANGLE KD:"), 1, 2)
        self.sl_kd_theta = QSlider(Qt.Orientation.Horizontal)
        self.sl_kd_theta.setRange(0, 30)
        self.sl_kd_theta.setValue(15)
        config_grid.addWidget(self.sl_kd_theta, 1, 3)
        self.lbl_kd_theta = QLabel("15")
        self.lbl_kd_theta.setStyleSheet("font-family: 'Consolas'; min-width: 25px;")
        self.sl_kd_theta.valueChanged.connect(lambda v: self.lbl_kd_theta.setText(str(v)))
        config_grid.addWidget(self.lbl_kd_theta, 1, 4)

        config_grid.addWidget(QLabel("POSITION KP:"), 2, 2)
        self.sl_kp_x = QSlider(Qt.Orientation.Horizontal)
        self.sl_kp_x.setRange(-80, 0)
        self.sl_kp_x.setValue(-25)
        config_grid.addWidget(self.sl_kp_x, 2, 3)
        self.lbl_kp_x = QLabel("-2.5")
        self.lbl_kp_x.setStyleSheet("font-family: 'Consolas'; min-width: 25px;")
        self.sl_kp_x.valueChanged.connect(lambda v: self.lbl_kp_x.setText(f"{v/10.0:.1f}"))
        config_grid.addWidget(self.lbl_kp_x, 2, 4)

        config_grid.addWidget(QLabel("POSITION KD:"), 3, 2)
        self.sl_kd_x = QSlider(Qt.Orientation.Horizontal)
        self.sl_kd_x.setRange(-80, 0)
        self.sl_kd_x.setValue(-35)
        config_grid.addWidget(self.sl_kd_x, 3, 3)
        self.lbl_kd_x = QLabel("-3.5")
        self.lbl_kd_x.setStyleSheet("font-family: 'Consolas'; min-width: 25px;")
        self.sl_kd_x.valueChanged.connect(lambda v: self.lbl_kd_x.setText(f"{v/10.0:.1f}"))
        config_grid.addWidget(self.lbl_kd_x, 3, 4)

        config_grid.setColumnStretch(1, 1)
        config_grid.setColumnStretch(3, 1)
        config_card.layout.addLayout(config_grid)
        left_layout.addWidget(config_card, 1)

        # Right side panel container (Metrics panel)
        right_widget = QWidget()
        right_widget.setFixedWidth(380)
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(16)
        splitter.addWidget(right_widget)

        # 1. Neural Network Policy Card
        nn_card = CardWidget("NEURAL NETWORK POLICY")
        self.nn_widget = NNWidget()
        nn_card.layout.addWidget(self.nn_widget)
        right_layout.addWidget(nn_card)

        # 2. Angle vs Time Card
        self.angle_card = CardWidget("ANGLE VS TIME")
        self.angle_plot = pg.PlotWidget()
        self.angle_curve = self.style_chart(self.angle_plot, 140, 220, (255, 255, 255, 20))
        self.angle_target = pg.InfiniteLine(pos=180.0, angle=0, pen=pg.mkPen('#ff3333', width=1, style=Qt.PenStyle.DashLine))
        self.angle_plot.addItem(self.angle_target)
        self.angle_card.layout.addWidget(self.angle_plot)
        right_layout.addWidget(self.angle_card)

        # 3. Position vs Time Card
        self.pos_card = CardWidget("CART POSITION VS TIME")
        self.pos_plot = pg.PlotWidget()
        self.pos_curve = self.style_chart(self.pos_plot, -2.8, 2.8, (200, 200, 200, 15))
        self.pos_target = pg.InfiniteLine(pos=0.0, angle=0, pen=pg.mkPen('#ff3333', width=1, style=Qt.PenStyle.DashLine))
        self.pos_plot.addItem(self.pos_target)
        self.pos_card.layout.addWidget(self.pos_plot)
        right_layout.addWidget(self.pos_card)

        # 4. Force/Reward vs Time Card
        self.force_card = CardWidget("CONTROL FORCE VS TIME")
        self.force_plot = pg.PlotWidget()
        self.force_curve = self.style_chart(self.force_plot, -35.0, 35.0, (150, 150, 150, 15))
        self.force_target = pg.InfiniteLine(pos=0.0, angle=0, pen=pg.mkPen('#ff3333', width=1, style=Qt.PenStyle.DashLine))
        self.force_plot.addItem(self.force_target)
        self.force_card.layout.addWidget(self.force_plot)
        right_layout.addWidget(self.force_card)

        # Configure layout sizes
        splitter.setSizes([950, 380])

    def style_chart(self, plot, y_min, y_max, fill_color):
        plot.setBackground('#000000')
        plot_item = plot.getPlotItem()

        # Hide standard margins
        plot_item.setContentsMargins(0, 0, 0, 0)
        plot_item.showGrid(x=True, y=True, alpha=0.08)

        # Hide X bottom axis for technical dense look
        plot_item.showAxis('bottom', False)
        plot_item.showAxis('right', False)
        plot_item.showAxis('top', False)

        # Customize Y axis tick marks
        left_axis = plot_item.getAxis('left')
        left_axis.setPen(pg.mkPen('#333333', width=1))
        left_axis.setTextPen(pg.mkPen('#888888'))

        font = QFont("Consolas")
        font.setPointSize(8)
        left_axis.setTickFont(font)
        plot_item.setYRange(y_min, y_max)

        # Connect a white filled path item
        curve = plot_item.plot(
            pen=pg.mkPen('#ffffff', width=1.5),
            fillLevel=y_min - 100.0, # ensures filling to the bottom limit
            fillBrush=pg.mkBrush(fill_color)
        )
        return curve

    # ---------------------------------------------------------
    # Thread Connection Controls
    # ---------------------------------------------------------
    def on_mode_changed(self, mode):
        self.is_simulation = (mode == "Simulation")
        self.btn_connect.setEnabled(not self.is_simulation)
        self.btn_push.setEnabled(self.is_simulation)
        self.lbl_telemetry.setText(f"Time: {self.elapsed_time:.1f}s | Mode: {'SIM' if self.is_simulation else 'HW'}")

        if not self.is_simulation:
            # Pause simulation if switching to HW
            if self.is_running:
                self.toggle_running()
        else:
            # Terminate active serial reader when entering SIM
            self.stop_serial()

    def toggle_connection(self):
        if self.serial_thread and self.serial_thread.isRunning():
            self.stop_serial()
        else:
            self.start_serial()

    def start_serial(self):
        self.stop_serial()
        port = self.cb_port.currentText()
        if not port:
            self.set_status_indicator("No COM Port selected", "red")
            return

        baud = int(self.cb_baud.currentText())
        self.serial_thread = SerialReader(port, baud)
        self.serial_thread.data_received.connect(self.on_serial_data)
        self.serial_thread.status_changed.connect(self.set_status_indicator)
        self.serial_thread.start()
        self.btn_connect.setText("Disconnect")
        self.cb_mode.setEnabled(False)

    def stop_serial(self):
        if self.serial_thread:
            self.serial_thread.stop()
            self.serial_thread = None
        self.btn_connect.setText("Connect")
        self.cb_mode.setEnabled(True)
        self.set_status_indicator("Idle", "gray")

    def set_status_indicator(self, text, state):
        self.status_text.setText(text)

        color_map = {
            "green": "#ffffff", # Running/Active
            "gray": "#555555",  # Idle
            "red": "#ff3333"    # Error
        }
        self.status_dot.setStyleSheet(f"""
            border: 1px solid #ffffff;
            border-radius: 5px;
            background-color: {color_map.get(state, '#555555')};
        """)

        # Change Start button context if COM reading
        if text.startswith("Reading"):
            self.btn_start.setText("Stop")
            self.is_running = True
        elif text == "Idle":
            self.btn_start.setText("Start")
            self.is_running = False

    def on_serial_data(self, value):
        # Update raw values.
        # Expecting value represents angle in degrees (with 180 as vertical upright)
        # Convert to physics domain radians where 0 is upright
        self.theta = math.radians(value - 180.0)
        self.elapsed_time += self.dt

        # In HW mode, we do not calculate force control outputs directly
        self.force = 0.0
        self.x = 0.0 # Cart stays centered since it's hardware
        self.theta_dot = 0.0

    # ---------------------------------------------------------
    # Physics Loop Ticks
    # ---------------------------------------------------------
    def toggle_running(self):
        if not self.is_simulation:
            # If HW mode, clicking start initiates serial connection
            self.toggle_connection()
            return

        self.is_running = not self.is_running
        self.btn_start.setText("Stop" if self.is_running else "Start")
        self.set_status_indicator("Running" if self.is_running else "Paused", "green" if self.is_running else "gray")

    def reset_all(self):
        # Stop
        if self.is_running:
            self.toggle_running()
        self.stop_serial()

        # Reset variables
        self.x = 0.0
        self.x_dot = 0.0
        self.theta = math.radians(random.choice([-1, 1]) * random.uniform(2.0, 10.0)) # start slightly off vertical upright
        self.theta_dot = 0.0
        self.force = 0.0
        self.elapsed_time = 0.0
        self.fallen_frames = 0

        # Clear histories
        self.angle_history = deque([180.0] * self.history_len, maxlen=self.history_len)
        self.pos_history = deque([0.0] * self.history_len, maxlen=self.history_len)
        self.force_history = deque([0.0] * self.history_len, maxlen=self.history_len)

        # Clear plots
        self.angle_curve.setData(list(self.angle_history))
        self.pos_curve.setData(list(self.pos_history))
        self.force_curve.setData(list(self.force_history))

        # Update visuals
        self.canvas_widget.update_state(self.x, self.theta, self.force)
        self.nn_widget.update_state(self.force, self.elapsed_time)
        self.lbl_telemetry.setText(f"Time: {self.elapsed_time:.1f}s | Mode: {'SIM' if self.is_simulation else 'HW'}")

    def apply_push(self, magnitude):
        if self.is_simulation:
            self.x_dot += magnitude

    def reset_simulation_state(self):
        # Soft-reset simulation state when it crashes/falls over (continuity)
        self.x = random.uniform(-1.0, 1.0)
        self.x_dot = 0.0
        self.theta = math.radians(random.choice([-1, 1]) * random.uniform(5.0, 15.0))
        self.theta_dot = 0.0
        self.force = 0.0
        self.fallen_frames = 0

    def tick(self):
        if self.is_running:
            if self.is_simulation:
                # -----------------
                # Physics Step
                # -----------------
                # Fetch controller tuning parameters
                kp_theta = self.sl_kp_theta.value()
                kd_theta = self.sl_kd_theta.value()
                kp_x = self.sl_kp_x.value() / 10.0
                kd_x = self.sl_kd_x.value() / 10.0

                # Border limits safety wall repulsions
                wall_repulsion = 0.0
                margin = 1.2
                if self.x > margin:
                    wall_repulsion = -60.0 * (self.x - margin) - 15.0 * self.x_dot
                elif self.x < -margin:
                    wall_repulsion = -60.0 * (self.x + margin) - 15.0 * self.x_dot

                # Map angle to [-pi, pi] normalized
                norm_theta = ((self.theta % (2 * math.pi)) + 2 * math.pi) % (2 * math.pi)
                if norm_theta > math.pi:
                    norm_theta -= 2 * math.pi

                # Control output calculation
                if abs(norm_theta) < 0.5:
                    # Balance control
                    self.force = (kp_theta * norm_theta) + (kd_theta * self.theta_dot) + (kp_x * self.x) + (kd_x * self.x_dot) + wall_repulsion
                else:
                    # Swing up energy control
                    self.force = -60.0 * math.copysign(1, self.theta_dot * math.cos(norm_theta)) - 2.0 * self.x - 2.0 * self.x_dot + wall_repulsion
                    if abs(self.theta_dot) < 0.01 and abs(norm_theta) > math.pi - 0.1:
                        self.force = 20.0 # start perturbation

                # Cap absolute control force
                self.force = max(-30.0, min(30.0, self.force))

                # Dynamic equations of motion
                sin_t = math.sin(self.theta)
                cos_t = math.cos(self.theta)

                temp = (self.force + self.m * self.L * self.theta_dot * self.theta_dot * sin_t) / (self.M + self.m)
                theta_acc = (self.g * sin_t - cos_t * temp) / (self.L * (4.0/3.0 - self.m * cos_t * cos_t / (self.M + self.m)))
                x_acc = temp - self.m * self.L * theta_acc * cos_t / (self.M + self.m)

                # Integrate Euler-Cromer
                self.x_dot += x_acc * self.dt
                self.x += self.x_dot * self.dt
                self.theta_dot += theta_acc * self.dt
                self.theta_dot *= 0.995 # friction damping

                max_spin = 12.0
                self.theta_dot = max(-max_spin, min(max_spin, self.theta_dot))
                self.theta += self.theta_dot * self.dt

                # Prevent cart from exiting physical track boundaries
                rail_limit = 2.4
                if self.x > rail_limit:
                    self.x = rail_limit
                    if self.x_dot > 0: self.x_dot = 0.0
                elif self.x < -rail_limit:
                    self.x = -rail_limit
                    if self.x_dot < 0: self.x_dot = 0.0

                # Detect fallback reset condition
                if abs(norm_theta) > math.pi / 2.0:
                    self.fallen_frames += 1
                    if self.fallen_frames > 200:
                        self.reset_simulation_state()
                else:
                    self.fallen_frames = 0

                self.elapsed_time += self.dt

            # Update histories at decimation frequency (every 4 ticks ~32ms)
            if int(self.elapsed_time / self.dt) % 4 == 0:
                # Map radians back to display degrees (180 center)
                display_deg = 180.0 + math.degrees(self.theta)
                self.angle_history.append(display_deg)
                self.pos_history.append(self.x)
                self.force_history.append(self.force)

                # Replot curves
                self.angle_curve.setData(list(self.angle_history))
                self.pos_curve.setData(list(self.pos_history))
                self.force_curve.setData(list(self.force_history))

            # Update dashboard overlay readouts
            self.lbl_telemetry.setText(f"Time: {self.elapsed_time:.1f}s | Mode: {'SIM' if self.is_simulation else 'HW'}")

        # Update visuals continuously (smooth animation at ~120fps)
        self.canvas_widget.update_state(self.x, self.theta, self.force)
        self.nn_widget.update_state(self.force, self.elapsed_time)

    def closeEvent(self, event):
        self.stop_serial()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())