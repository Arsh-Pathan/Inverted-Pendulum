import sys
import math
import time
import random
import json
import os
from collections import deque
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QFrame, QHBoxLayout, 
                             QVBoxLayout, QGridLayout, QLabel, QSplitter)
from PyQt6.QtCore import QTimer, Qt, QThread, pyqtSignal, QPointF
from PyQt6.QtGui import QPainter, QPen, QColor, QBrush, QFont, QPolygonF
import pyqtgraph as pg
import serial
import serial.tools.list_ports

# ---------------------------------------------------------
# Configuration loader for plug-and-play digital twin
# ---------------------------------------------------------
def load_config():
    config_path = os.path.join(os.path.dirname(__file__), 'config.json')
    # Default parameters: 180 degrees is upright vertical center
    defaults = {
        "preferred_port": "COM3",
        "baud_rate": 115200,
        "angle_offset": 180.0,
        "angle_scale": 1.0,
        "angle_invert": False
    }
    if os.path.exists(config_path):
        try:
            with open(config_path, 'r') as f:
                data = json.load(f)
                defaults.update(data)
        except Exception as e:
            print(f"Error reading config: {e}")
    else:
        try:
            with open(config_path, 'w') as f:
                json.dump(defaults, f, indent=4)
        except Exception as e:
            print(f"Error writing default config: {e}")
    return defaults

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
"""

# ---------------------------------------------------------
# Multi-variable Serial Reader QThread
# ---------------------------------------------------------
class SerialReader(QThread):
    telemetry_received = pyqtSignal(float, float, float)
    status_changed = pyqtSignal(str, str)

    def __init__(self, port, baud=115200):
        super().__init__()
        self.port = port
        self.baud = baud
        self.running = False
        self.ser = None

    def run(self):
        self.running = True
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.status_changed.emit(f"Connected: {self.port}", "green")
            self.ser.reset_input_buffer()
            
            while self.running:
                if self.ser.in_waiting > 0:
                    try:
                        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            parts = line.split(',')
                            if len(parts) >= 3:
                                theta_deg = float(parts[0])
                                pos = float(parts[1])
                                force = float(parts[2])
                                self.telemetry_received.emit(theta_deg, pos, force)
                            elif len(parts) == 2:
                                theta_deg = float(parts[0])
                                pos = float(parts[1])
                                self.telemetry_received.emit(theta_deg, pos, 0.0)
                            elif len(parts) == 1:
                                theta_deg = float(parts[0])
                                self.telemetry_received.emit(theta_deg, 0.0, 0.0)
                    except ValueError:
                        pass
                self.msleep(2)
            self.ser.close()
            self.ser = None
            self.status_changed.emit("Idle", "gray")
        except Exception as e:
            self.status_changed.emit("Disconnected", "red")
            self.ser = None

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
# CAD/Engineering Telemetry Viewport
# ---------------------------------------------------------
class CanvasWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.cart_x = 0.0
        self.theta = 0.0  # radians (0 is upright)
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
        painter.fillRect(self.rect(), QColor(0, 0, 0))

        # CAD Grid overlay
        painter.setPen(QPen(QColor(20, 20, 20), 1, Qt.PenStyle.SolidLine))
        grid_size = 40
        for x in range(0, self.width(), grid_size):
            painter.drawLine(x, 0, x, self.height())
        for y in range(0, self.height(), grid_size):
            painter.drawLine(0, y, self.width(), y)

        w, h = self.width(), self.height()
        cy = h / 2.0
        scale = w / 6.0

        # Draw rail track
        painter.setPen(QPen(QColor(255, 255, 255), 2))
        painter.drawLine(0, int(cy + 15), w, int(cy + 15))

        # Rail scale ticks
        painter.setPen(QPen(QColor(150, 150, 150), 1))
        painter.setFont(QFont("Consolas", 8))
        for m in range(-3, 4):
            tx = int(w / 2.0 + m * scale)
            painter.drawLine(tx, int(cy + 15), tx, int(cy + 22))
            painter.drawText(tx - 15, int(cy + 34), f"{m:+.1f}m")

        # Track bounds lines
        painter.setPen(QPen(QColor(255, 50, 50, 90), 1, Qt.PenStyle.DashLine))
        left_bound = int(w / 2.0 - self.limit * scale)
        right_bound = int(w / 2.0 + self.limit * scale)
        painter.drawLine(left_bound, 0, left_bound, h)
        painter.drawLine(right_bound, 0, right_bound, h)

        cx = w / 2.0 + self.cart_x * scale

        # Draw cart
        cart_w, cart_h = 75, 32
        painter.setPen(QPen(QColor(255, 255, 255), 2))
        painter.setBrush(QBrush(QColor(0, 0, 0)))
        painter.drawRect(int(cx - cart_w/2), int(cy - cart_h/2), cart_w, cart_h)

        # Wheels
        wheel_r = 8
        painter.drawEllipse(QPointF(cx - 22, cy + 15), wheel_r, wheel_r)
        painter.drawEllipse(QPointF(cx + 22, cy + 15), wheel_r, wheel_r)

        # Upright target reference line
        painter.setPen(QPen(QColor(255, 255, 255, 40), 1, Qt.PenStyle.DotLine))
        painter.drawLine(int(cx), 0, int(cx), int(cy))

        # Rod
        pole_len = 140
        px = cx + math.sin(self.theta) * pole_len
        py = cy - math.cos(self.theta) * pole_len

        painter.setPen(QPen(QColor(255, 255, 255), 4, Qt.PenStyle.SolidLine, Qt.PenCapStyle.SquareCap))
        painter.drawLine(QPointF(cx, cy), QPointF(px, py))

        # Heavy Bob mass
        bob_r = 13
        painter.setBrush(QBrush(QColor(255, 255, 255)))
        painter.drawEllipse(QPointF(px, py), bob_r, bob_r)

        # Center Hinge pin
        painter.setBrush(QBrush(QColor(0, 0, 0)))
        painter.setPen(QPen(QColor(255, 255, 255), 1.5))
        painter.drawEllipse(QPointF(cx, cy), 4, 4)

        # Draw applied force vector indicator arrow
        if abs(self.force) > 0.05:
            painter.setPen(QPen(QColor(255, 255, 255), 1.5))
            painter.setBrush(QBrush(QColor(255, 255, 255)))
            
            arrow_y = cy - 25
            arrow_len = min(50, abs(self.force) * 2)
            
            if self.force > 0:
                end_x = cx - 40
                start_x = end_x - arrow_len
                painter.drawLine(int(start_x), int(arrow_y), int(end_x), int(arrow_y))
                poly = QPolygonF([
                    QPointF(end_x, arrow_y),
                    QPointF(end_x - 6, arrow_y - 4),
                    QPointF(end_x - 6, arrow_y + 4)
                ])
                painter.drawPolygon(poly)
            else:
                end_x = cx + 40
                start_x = end_x + arrow_len
                painter.drawLine(int(start_x), int(arrow_y), int(end_x), int(arrow_y))
                poly = QPolygonF([
                    QPointF(end_x, arrow_y),
                    QPointF(end_x + 6, arrow_y - 4),
                    QPointF(end_x + 6, arrow_y + 4)
                ])
                painter.drawPolygon(poly)

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

        nodes = []
        for l_idx, count in enumerate(layers):
            x = layer_x * (l_idx + 1)
            y_gap = h / (count + 1)
            layer_nodes = []
            for n_idx in range(count):
                layer_nodes.append((x, y_gap * (n_idx + 1)))
            nodes.append(layer_nodes)

        for l_idx in range(len(layers) - 1):
            for n1_idx, n1 in enumerate(nodes[l_idx]):
                for n2_idx, n2 in enumerate(nodes[l_idx+1]):
                    pulse = (math.sin(self.time * 6.0 + n1_idx * 1.5 + n2_idx) + 1.0) / 2.0
                    
                    if l_idx == len(layers) - 2:
                        color = QColor(255, 255, 255, int(active_strength * 255))
                        width = 1.5
                    else:
                        alpha = int((0.04 + 0.14 * pulse * active_strength) * 255)
                        color = QColor(255, 255, 255, alpha)
                        width = 1.0
                        
                    painter.setPen(QPen(color, width))
                    painter.drawLine(QPointF(n1[0], n1[1]), QPointF(n2[0], n2[1]))

        for l_idx, layer_nodes in enumerate(nodes):
            for n_idx, n in enumerate(layer_nodes):
                painter.setPen(Qt.PenStyle.NoPen)
                if l_idx == len(layers) - 1:
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
        self.setWindowTitle("Inverted Pendulum Digital Twin")
        self.resize(1400, 850)
        self.setStyleSheet(QSS_STYLE)

        # Load parameter configs from JSON
        self.config = load_config()

        # Telemetry/State variables
        self.dt = 0.008
        self.x = 0.0
        self.theta = 0.0
        self.theta_dot = 0.0
        self.prev_theta = 0.0
        self.force = 0.0
        self.elapsed_time = 0.0

        self.serial_thread = None
        self.is_connected = False

        # Charts history queues
        self.history_len = 250
        self.angle_history = deque([180.0] * self.history_len, maxlen=self.history_len)
        self.pos_history = deque([0.0] * self.history_len, maxlen=self.history_len)
        self.vel_history = deque([0.0] * self.history_len, maxlen=self.history_len)

        # Initialize Layouts
        self.init_ui()

        # Tick timer for repainting and plot history polling
        self.timer = QTimer()
        self.timer.timeout.connect(self.tick)
        self.timer.start(8)

        # Background plug-and-play port checker (polls every 1.5 seconds)
        self.port_timer = QTimer()
        self.port_timer.timeout.connect(self.auto_connection_handler)
        self.port_timer.start(1500)
        self.auto_connection_handler() # run first scan instantly

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(20)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        main_layout.addWidget(splitter)

        # Left panel: Telemetry Canvas Viewport only (No buttons!)
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(16)
        splitter.addWidget(left_widget)

        # Title Block
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
        lbl_subtitle = QLabel("Real-time Digital Twin Telemetry Monitor")
        lbl_subtitle.setStyleSheet("""
            font-size: 13px;
            font-weight: 600;
            color: #aaaaaa;
        """)
        header_layout.addWidget(lbl_title)
        header_layout.addWidget(lbl_subtitle)
        left_layout.addWidget(header_widget)

        # Canvas Frame (Fills Left Side)
        self.sim_card = CardWidget(None)
        self.canvas_widget = CanvasWidget()
        self.sim_card.layout.addWidget(self.canvas_widget, 1)

        # Integrated Status Info bar underneath canvas
        status_row = QHBoxLayout()
        status_row.setContentsMargins(5, 5, 5, 5)
        
        self.lbl_telemetry = QLabel("Time: 0.0s | Angle: 180.00° | Port: Searching...")
        self.lbl_telemetry.setStyleSheet("""
            color: #aaaaaa;
            font-weight: 600;
            font-size: 13px;
        """)
        status_row.addWidget(self.lbl_telemetry)
        status_row.addStretch()

        self.status_dot = QFrame()
        self.status_dot.setFixedSize(10, 10)
        self.status_dot.setStyleSheet("""
            border: 1px solid #ffffff;
            border-radius: 5px;
            background-color: #555555;
        """)
        self.status_text = QLabel("Offline")
        self.status_text.setStyleSheet("font-weight: 600; font-size: 13px;")
        status_row.addWidget(self.status_dot)
        status_row.addWidget(self.status_text)
        
        self.sim_card.layout.addLayout(status_row)
        left_layout.addWidget(self.sim_card, 1)

        # Right Panel: Telemetry Charts
        right_widget = QWidget()
        right_widget.setFixedWidth(380)
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(16)
        splitter.addWidget(right_widget)

        # 1. Neural Network Policy Card (infers actions based on angle error)
        nn_card = CardWidget("INFERRED CONTROL POLICY")
        self.nn_widget = NNWidget()
        nn_card.layout.addWidget(self.nn_widget)
        right_layout.addWidget(nn_card)

        # 2. Angle vs Time
        self.angle_card = CardWidget("ANGLE VS TIME")
        self.angle_plot = pg.PlotWidget()
        self.angle_curve = self.style_chart(self.angle_plot, 140, 220, (255, 255, 255, 20))
        self.angle_target = pg.InfiniteLine(pos=180.0, angle=0, pen=pg.mkPen('#ff3333', width=1, style=Qt.PenStyle.DashLine))
        self.angle_plot.addItem(self.angle_target)
        self.angle_card.layout.addWidget(self.angle_plot)
        right_layout.addWidget(self.angle_card)

        # 3. Angular Velocity vs Time
        self.vel_card = CardWidget("ANGULAR VELOCITY VS TIME")
        self.vel_plot = pg.PlotWidget()
        self.vel_curve = self.style_chart(self.vel_plot, -200.0, 200.0, (200, 200, 200, 15))
        self.vel_target = pg.InfiniteLine(pos=0.0, angle=0, pen=pg.mkPen('#ff3333', width=1, style=Qt.PenStyle.DashLine))
        self.vel_plot.addItem(self.vel_target)
        self.vel_card.layout.addWidget(self.vel_plot)
        right_layout.addWidget(self.vel_card)

        # 4. Cart Position vs Time
        self.pos_card = CardWidget("CART POSITION VS TIME")
        self.pos_plot = pg.PlotWidget()
        self.pos_curve = self.style_chart(self.pos_plot, -2.8, 2.8, (150, 150, 150, 15))
        self.pos_target = pg.InfiniteLine(pos=0.0, angle=0, pen=pg.mkPen('#ff3333', width=1, style=Qt.PenStyle.DashLine))
        self.pos_plot.addItem(self.pos_target)
        self.pos_card.layout.addWidget(self.pos_plot)
        right_layout.addWidget(self.pos_card)

        splitter.setSizes([950, 380])

    def style_chart(self, plot, y_min, y_max, fill_color):
        plot.setBackground('#000000')
        plot_item = plot.getPlotItem()
        plot_item.setContentsMargins(0, 0, 0, 0)
        plot_item.showGrid(x=True, y=True, alpha=0.08)

        plot_item.showAxis('bottom', False)
        plot_item.showAxis('right', False)
        plot_item.showAxis('top', False)

        left_axis = plot_item.getAxis('left')
        left_axis.setPen(pg.mkPen('#333333', width=1))
        left_axis.setTextPen(pg.mkPen('#888888'))
        
        font = QFont("Consolas")
        font.setPointSize(8)
        left_axis.setTickFont(font)
        plot_item.setYRange(y_min, y_max)

        curve = plot_item.plot(
            pen=pg.mkPen('#ffffff', width=1.5),
            fillLevel=y_min - 100.0,
            fillBrush=pg.mkBrush(fill_color)
        )
        return curve

    # ---------------------------------------------------------
    # Automatic plug-and-play connection scanner
    # ---------------------------------------------------------
    def auto_connection_handler(self):
        if self.serial_thread and self.serial_thread.isRunning():
            return # Already connected or trying to connect

        # Scan ports
        ports = [p.device for p in serial.tools.list_ports.comports()]
        target_port = self.config.get("preferred_port", "COM3")
        
        if not ports:
            self.set_status_indicator("Searching...", "gray")
            return

        # Connect to preferred port if available, else pick first found
        chosen_port = target_port if target_port in ports else ports[0]
        baud = self.config.get("baud_rate", 115200)

        self.serial_thread = SerialReader(chosen_port, baud)
        self.serial_thread.telemetry_received.connect(self.on_hardware_telemetry)
        self.serial_thread.status_changed.connect(self.set_status_indicator)
        self.serial_thread.start()

    def set_status_indicator(self, text, state):
        self.status_text.setText(text)
        self.is_connected = (state == "green")
        
        color_map = {
            "green": "#ffffff", # Connected
            "gray": "#555555",  # Scanning
            "red": "#ff3333"    # Lost / Disconnected
        }
        self.status_dot.setStyleSheet(f"""
            border: 1px solid #ffffff;
            border-radius: 5px;
            background-color: {color_map.get(state, '#555555')};
        """)

        if state == "red":
            # Clean thread reference on failure so scanner can retry
            if self.serial_thread:
                self.serial_thread.stop()
                self.serial_thread = None

    def on_hardware_telemetry(self, theta_deg, pos, force):
        offset = self.config.get("angle_offset", 180.0)
        scale = self.config.get("angle_scale", 1.0)
        invert = -1.0 if self.config.get("angle_invert", False) else 1.0

        # Calibrate offset and scaling
        calibrated_diff = (theta_deg - offset) * scale * invert
        
        self.theta = math.radians(calibrated_diff)
        self.x = pos
        # Inferred force (calculates control output representation for the neural net visualizer)
        self.force = force if force != 0.0 else (50.0 * self.theta + 12.0 * self.theta_dot)

    # ---------------------------------------------------------
    # Core update tick
    # ---------------------------------------------------------
    def tick(self):
        display_deg = 180.0
        
        if self.is_connected:
            self.elapsed_time += self.dt
            
            # Calculate angular velocity
            diff = self.theta - self.prev_theta
            # Normalize diff to [-pi, pi]
            diff = (diff + math.pi) % (2 * math.pi) - math.pi
            self.theta_dot = diff / self.dt
            self.prev_theta = self.theta

            display_deg = 180.0 + math.degrees(self.theta)
            vel_deg_s = math.degrees(self.theta_dot)

            # Decimated buffer updates (~32ms)
            if int(self.elapsed_time / self.dt) % 4 == 0:
                self.angle_history.append(display_deg)
                self.pos_history.append(self.x)
                self.vel_history.append(vel_deg_s)

                self.angle_curve.setData(list(self.angle_history))
                self.pos_curve.setData(list(self.pos_history))
                self.vel_curve.setData(list(self.vel_history))

            self.lbl_telemetry.setText(
                f"Time: {self.elapsed_time:.1f}s | Angle: {display_deg:.2f}° | "
                f"Vel: {vel_deg_s:.1f}°/s | Port: {self.serial_thread.port if self.serial_thread else 'N/A'}"
            )
        else:
            self.lbl_telemetry.setText("Time: 0.0s | Angle: 180.00° | Port: Searching for hardware...")

        # Visual Repaints
        self.canvas_widget.update_state(self.x, self.theta, self.force)
        self.nn_widget.update_state(self.force, self.elapsed_time)

    def closeEvent(self, event):
        if self.serial_thread:
            self.serial_thread.stop()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())