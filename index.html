import sys
import math
import time
import random
import json
import os
from collections import deque
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QFrame, QHBoxLayout, 
                             QVBoxLayout, QGridLayout, QLabel, QSplitter)
from PyQt6.QtCore import QTimer, Qt, QPointF
from PyQt6.QtGui import QPainter, QPen, QColor, QBrush, QFont, QPolygonF
import pyqtgraph as pg
import serial
import serial.tools.list_ports

# ---------------------------------------------------------
# Configuration loader for plug-and-play digital twin
# ---------------------------------------------------------
def load_config():
    config_path = os.path.join(os.path.dirname(__file__), 'config.json')
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
# Styling Sheet: CAD high-contrast white aesthetic
# ---------------------------------------------------------
QSS_STYLE = """
QMainWindow {
    background-color: #ffffff;
}
QWidget {
    background-color: #ffffff;
    color: #000000;
    font-family: 'Inter', 'Segoe UI', 'Arial', sans-serif;
}
QLabel {
    color: #000000;
}
"""

# ---------------------------------------------------------
# Card Container matching dashboard styles (White Theme)
# ---------------------------------------------------------
class CardWidget(QFrame):
    def __init__(self, title, parent=None):
        super().__init__(parent)
        self.setFrameShape(QFrame.Shape.StyledPanel)
        self.setStyleSheet("""
            CardWidget {
                background-color: #ffffff;
                border: 1px solid #000000;
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
                color: #000000;
                border: none;
                border-bottom: 1px solid #000000;
                padding-bottom: 4px;
                margin-bottom: 4px;
            """)
            self.layout.addWidget(self.title_label)

# ---------------------------------------------------------
# CAD/Engineering Telemetry Viewport (White Theme)
# ---------------------------------------------------------
class CanvasWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.cart_x = 0.0
        self.theta = 0.0
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
        painter.fillRect(self.rect(), QColor(255, 255, 255))

        # CAD Grid overlay (Light gray)
        painter.setPen(QPen(QColor(240, 240, 240), 1, Qt.PenStyle.SolidLine))
        grid_size = 40
        for x in range(0, self.width(), grid_size):
            painter.drawLine(x, 0, x, self.height())
        for y in range(0, self.height(), grid_size):
            painter.drawLine(0, y, self.width(), y)

        w, h = self.width(), self.height()
        cy = h / 2.0
        scale = w / 6.0

        # Draw rail track (Black)
        painter.setPen(QPen(QColor(0, 0, 0), 2))
        painter.drawLine(0, int(cy + 15), w, int(cy + 15))

        # Rail scale ticks
        painter.setPen(QPen(QColor(100, 100, 100), 1))
        painter.setFont(QFont("Consolas", 8))
        for m in range(-3, 4):
            tx = int(w / 2.0 + m * scale)
            painter.drawLine(tx, int(cy + 15), tx, int(cy + 22))
            painter.drawText(tx - 15, int(cy + 34), f"{m:+.1f}m")

        # Track bounds lines (Red dashed)
        painter.setPen(QPen(QColor(255, 50, 50, 120), 1, Qt.PenStyle.DashLine))
        left_bound = int(w / 2.0 - self.limit * scale)
        right_bound = int(w / 2.0 + self.limit * scale)
        painter.drawLine(left_bound, 0, left_bound, h)
        painter.drawLine(right_bound, 0, right_bound, h)

        cx = w / 2.0 + self.cart_x * scale

        # Draw cart (White fill, Black border)
        cart_w, cart_h = 75, 32
        painter.setPen(QPen(QColor(0, 0, 0), 2))
        painter.setBrush(QBrush(QColor(255, 255, 255)))
        painter.drawRect(int(cx - cart_w/2), int(cy - cart_h/2), cart_w, cart_h)

        # Wheels
        wheel_r = 8
        painter.drawEllipse(QPointF(cx - 22, cy + 15), wheel_r, wheel_r)
        painter.drawEllipse(QPointF(cx + 22, cy + 15), wheel_r, wheel_r)

        # Upright target reference line
        painter.setPen(QPen(QColor(0, 0, 0, 60), 1, Qt.PenStyle.DotLine))
        painter.drawLine(int(cx), 0, int(cx), int(cy))

        # Rod (Black)
        pole_len = 140
        px = cx + math.sin(self.theta) * pole_len
        py = cy - math.cos(self.theta) * pole_len

        painter.setPen(QPen(QColor(0, 0, 0), 4, Qt.PenStyle.SolidLine, Qt.PenCapStyle.SquareCap))
        painter.drawLine(QPointF(cx, cy), QPointF(px, py))

        # Heavy Bob mass (Black)
        bob_r = 13
        painter.setBrush(QBrush(QColor(0, 0, 0)))
        painter.drawEllipse(QPointF(px, py), bob_r, bob_r)

        # Center Hinge pin
        painter.setBrush(QBrush(QColor(255, 255, 255)))
        painter.setPen(QPen(QColor(0, 0, 0), 1.5))
        painter.drawEllipse(QPointF(cx, cy), 4, 4)

        # Draw applied force vector indicator arrow
        if abs(self.force) > 0.05:
            painter.setPen(QPen(QColor(0, 0, 0), 1.5))
            painter.setBrush(QBrush(QColor(0, 0, 0)))
            
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
# Main Application Window
# ---------------------------------------------------------
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Inverted Pendulum Digital Twin")
        self.resize(1400, 850)
        self.setStyleSheet(QSS_STYLE)

        # Load configurations
        self.config = load_config()

        # Telemetry variables
        self.dt = 0.008
        self.x = 0.0
        self.theta = 0.0
        self.theta_dot = 0.0
        self.prev_theta = 0.0
        self.force = 0.0
        self.elapsed_time = 0.0

        self.ser = None
        self.is_connected = False

        # Charts history
        self.history_len = 250
        self.angle_history = deque([180.0] * self.history_len, maxlen=self.history_len)
        self.pos_history = deque([0.0] * self.history_len, maxlen=self.history_len)
        self.vel_history = deque([0.0] * self.history_len, maxlen=self.history_len)

        # Initialize Layouts
        self.init_ui()

        # Tick timer (unconditional clock loop)
        self.timer = QTimer()
        self.timer.timeout.connect(self.tick)
        self.timer.start(8)

        # Background connection checker (polls every 1.5 seconds)
        self.port_timer = QTimer()
        self.port_timer.timeout.connect(self.try_open_serial)
        self.port_timer.start(1500)
        self.try_open_serial()

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(20)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        main_layout.addWidget(splitter)

        # Left panel: Telemetry Canvas Viewport only
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
            color: #000000;
            letter-spacing: -0.5px;
        """)
        lbl_subtitle = QLabel("Real-time Digital Twin Telemetry Monitor")
        lbl_subtitle.setStyleSheet("""
            font-size: 13px;
            font-weight: 600;
            color: #555555;
        """)
        header_layout.addWidget(lbl_title)
        header_layout.addWidget(lbl_subtitle)
        left_layout.addWidget(header_widget)

        # Canvas Frame (Fills Left Side)
        self.sim_card = CardWidget(None)
        self.canvas_widget = CanvasWidget()
        self.sim_card.layout.addWidget(self.canvas_widget, 1)

        # Integrated Status Info bar
        status_row = QHBoxLayout()
        status_row.setContentsMargins(5, 5, 5, 5)
        
        self.lbl_telemetry = QLabel("Time: 0.0s | Angle: 180.00° | Port: Searching...")
        self.lbl_telemetry.setStyleSheet("""
            color: #555555;
            font-weight: 600;
            font-size: 13px;
        """)
        status_row.addWidget(self.lbl_telemetry)
        status_row.addStretch()

        self.status_dot = QFrame()
        self.status_dot.setFixedSize(10, 10)
        self.status_dot.setStyleSheet("""
            border: 1px solid #000000;
            border-radius: 5px;
            background-color: #888888;
        """)
        self.status_text = QLabel("Offline")
        self.status_text.setStyleSheet("font-weight: 600; font-size: 13px; color: #000000;")
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

        # 1. Angle vs Time
        self.angle_card = CardWidget("ANGLE VS TIME")
        self.angle_plot = pg.PlotWidget()
        self.angle_curve = self.style_chart(self.angle_plot, 140, 220, (0, 0, 0, 10))
        self.angle_target = pg.InfiniteLine(pos=180.0, angle=0, pen=pg.mkPen('#ff3333', width=1, style=Qt.PenStyle.DashLine))
        self.angle_plot.addItem(self.angle_target)
        self.angle_card.layout.addWidget(self.angle_plot)
        right_layout.addWidget(self.angle_card, 1)

        # 2. Angular Velocity vs Time
        self.vel_card = CardWidget("ANGULAR VELOCITY VS TIME")
        self.vel_plot = pg.PlotWidget()
        self.vel_curve = self.style_chart(self.vel_plot, -200.0, 200.0, (0, 0, 0, 8))
        self.vel_target = pg.InfiniteLine(pos=0.0, angle=0, pen=pg.mkPen('#ff3333', width=1, style=Qt.PenStyle.DashLine))
        self.vel_plot.addItem(self.vel_target)
        self.vel_card.layout.addWidget(self.vel_plot)
        right_layout.addWidget(self.vel_card, 1)

        # 3. Cart Position vs Time
        self.pos_card = CardWidget("CART POSITION VS TIME")
        self.pos_plot = pg.PlotWidget()
        self.pos_curve = self.style_chart(self.pos_plot, -2.8, 2.8, (0, 0, 0, 8))
        self.pos_target = pg.InfiniteLine(pos=0.0, angle=0, pen=pg.mkPen('#ff3333', width=1, style=Qt.PenStyle.DashLine))
        self.pos_plot.addItem(self.pos_target)
        self.pos_card.layout.addWidget(self.pos_plot)
        right_layout.addWidget(self.pos_card, 1)

        splitter.setSizes([950, 380])

    def style_chart(self, plot, y_min, y_max, fill_color):
        plot.setBackground('#ffffff')
        plot_item = plot.getPlotItem()
        plot_item.setContentsMargins(0, 0, 0, 0)
        plot_item.showGrid(x=True, y=True, alpha=0.1)

        plot_item.showAxis('bottom', False)
        plot_item.showAxis('right', False)
        plot_item.showAxis('top', False)

        left_axis = plot_item.getAxis('left')
        left_axis.setPen(pg.mkPen('#cccccc', width=1))
        left_axis.setTextPen(pg.mkPen('#555555'))
        
        font = QFont("Consolas")
        font.setPointSize(8)
        left_axis.setTickFont(font)
        plot_item.setYRange(y_min, y_max)

        curve = plot_item.plot(
            pen=pg.mkPen('#000000', width=1.5),
            fillLevel=y_min - 100.0,
            fillBrush=pg.mkBrush(fill_color)
        )
        return curve

    # ---------------------------------------------------------
    # Automatic plug-and-play connection scanner
    # ---------------------------------------------------------
    def try_open_serial(self):
        if self.ser and self.ser.is_open:
            return True

        ports = [p.device for p in serial.tools.list_ports.comports()]
        target_port = self.config.get("preferred_port", "COM3")
        
        if not ports:
            self.set_status_indicator("Searching...", "gray")
            return False

        chosen_port = target_port if target_port in ports else ports[0]
        baud = self.config.get("baud_rate", 115200)

        try:
            # Open with timeout=0.001 exactly like your working code script
            self.ser = serial.Serial(chosen_port, baud, timeout=0.001)
            self.set_status_indicator(f"Connected: {chosen_port}", "green")
            self.ser.reset_input_buffer()
            print(f"[COM] Successfully connected to {chosen_port} at {baud} baud.")
            return True
        except Exception as e:
            print(f"[COM] Connection failed on {chosen_port}: {e}")
            self.set_status_indicator("Disconnected", "red")
            self.ser = None
            return False

    def set_status_indicator(self, text, state):
        self.status_text.setText(text)
        self.is_connected = (state == "green")
        
        color_map = {
            "green": "#00aa00",
            "gray": "#888888",
            "red": "#ff3333"
        }
        self.status_dot.setStyleSheet(f"""
            border: 1px solid #000000;
            border-radius: 5px;
            background-color: {color_map.get(state, '#888888')};
        """)

    def update_telemetry(self, theta_deg, pos, force):
        offset = self.config.get("angle_offset", 180.0)
        scale = self.config.get("angle_scale", 1.0)
        invert = -1.0 if self.config.get("angle_invert", False) else 1.0

        calibrated_diff = (theta_deg - offset) * scale * invert
        self.theta = math.radians(calibrated_diff)
        self.x = pos
        self.force = force

    # ---------------------------------------------------------
    # Core update tick (unconditional execution)
    # ---------------------------------------------------------
    def tick(self):
        self.elapsed_time += self.dt

        # Read serial data directly from port if connected
        if self.ser and self.ser.is_open:
            try:
                while self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        parts = line.split(',')
                        if len(parts) >= 3:
                            theta_deg = float(parts[0])
                            pos = float(parts[1])
                            force = float(parts[2])
                            self.update_telemetry(theta_deg, pos, force)
                        elif len(parts) == 2:
                            theta_deg = float(parts[0])
                            pos = float(parts[1])
                            self.update_telemetry(theta_deg, pos, 0.0)
                        elif len(parts) == 1:
                            theta_deg = float(parts[0])
                            self.update_telemetry(theta_deg, 0.0, 0.0)
            except Exception as e:
                print(f"[COM] Serial read error: {e}")
                try:
                    self.ser.close()
                except:
                    pass
                self.ser = None
                self.set_status_indicator("Disconnected", "red")

        # Calculate angular velocity
        diff = self.theta - self.prev_theta
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

            # Live console logs (active immediately on start)
            print(f"[Telemetry] Time: {self.elapsed_time:.2f}s | Angle: {display_deg:.2f}° | Vel: {vel_deg_s:.1f}°/s | Pos: {self.x:.2f}m")

        if self.is_connected:
            self.lbl_telemetry.setText(
                f"Time: {self.elapsed_time:.1f}s | Angle: {display_deg:.2f}° | "
                f"Vel: {vel_deg_s:.1f}°/s | Port: {self.ser.port if self.ser else 'N/A'}"
            )
        else:
            self.lbl_telemetry.setText(
                f"Time: {self.elapsed_time:.1f}s | Angle: 180.00° | Port: Searching for hardware..."
            )

        # Repaint views
        self.canvas_widget.update_state(self.x, self.theta, self.force)

    def closeEvent(self, event):
        if self.ser:
            try:
                self.ser.close()
            except:
                pass
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())