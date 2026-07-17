import sys
import math
import time
import json
import os
from collections import deque
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QFrame, QHBoxLayout, 
                             QVBoxLayout, QGridLayout, QLabel, QSplitter)
from PyQt6.QtCore import QTimer, Qt, QThread, pyqtSignal, QPointF
from PyQt6.QtGui import QPainter, QPen, QColor, QBrush, QFont
import pyqtgraph as pg
import serial
import serial.tools.list_ports

# ---------------------------------------------------------
# Configuration loader
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
# Single-variable ESP32 Serial Reader QThread
# ---------------------------------------------------------
class SerialReader(QThread):
    angle_received = pyqtSignal(float)
    status_changed = pyqtSignal(str, str)

    def __init__(self, port, baud=115200):
        super().__init__()
        self.port = port
        self.baud = baud
        self.running = False

    def run(self):
        self.running = True
        try:
            # Open with 0.001 timeout matching user's working configuration
            ser = serial.Serial(self.port, self.baud, timeout=0.001)
            ser.dtr = True
            ser.rts = True
            self.status_changed.emit(f"Connected: {self.port}", "green")
            ser.reset_input_buffer()
            
            while self.running:
                while ser.in_waiting and self.running:
                    try:
                        line = ser.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            value = float(line)
                            self.angle_received.emit(value)
                    except ValueError:
                        pass
                self.msleep(5) # short sleep to keep CPU usage low
            ser.close()
            self.status_changed.emit("Idle", "gray")
        except Exception as e:
            print(f"[COM] Connection failed on {self.port}: {e}")
            self.status_changed.emit("Disconnected", "red")

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
# CAD/Engineering Telemetry Viewport (Fixed Pivot Pendulum)
# ---------------------------------------------------------
class CanvasWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.theta = 0.0  # radians (0 is vertical upright)

    def update_state(self, theta):
        self.theta = theta
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
        cx = w / 2.0
        cy = h / 2.0

        # Draw pivot mount (CAD Style circle/crosshair)
        painter.setPen(QPen(QColor(100, 100, 100), 1, Qt.PenStyle.DotLine))
        painter.drawLine(int(cx - 30), int(cy), int(cx + 30), int(cy))
        painter.drawLine(int(cx), int(cy - 30), int(cx), int(cy + 30))
        painter.drawEllipse(QPointF(cx, cy), 15.0, 15.0)

        # Draw reference upright dotted target line
        painter.setPen(QPen(QColor(255, 50, 50, 120), 1, Qt.PenStyle.DashLine))
        painter.drawLine(int(cx), 0, int(cx), h)

        # Pendulum Rod (Black)
        pole_len = 160
        px = cx + math.sin(self.theta) * pole_len
        py = cy - math.cos(self.theta) * pole_len

        painter.setPen(QPen(QColor(0, 0, 0), 4, Qt.PenStyle.SolidLine, Qt.PenCapStyle.SquareCap))
        painter.drawLine(QPointF(cx, cy), QPointF(px, py))

        # Bob mass (Black)
        bob_r = 15
        painter.setBrush(QBrush(QColor(0, 0, 0)))
        painter.drawEllipse(QPointF(px, py), bob_r, bob_r)

        # Inner center pin
        painter.setBrush(QBrush(QColor(255, 255, 255)))
        painter.setPen(QPen(QColor(0, 0, 0), 1.5))
        painter.drawEllipse(QPointF(cx, cy), 4, 4)

# ---------------------------------------------------------
# Main Application Window
# ---------------------------------------------------------
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Inverted Pendulum Telemetry Monitor")
        self.resize(1400, 850)
        self.setStyleSheet(QSS_STYLE)

        # Load configurations
        self.config = load_config()

        # Telemetry state variables (ESP32 is only source of truth)
        self.dt = 0.008
        self.theta = 0.0
        self.angle_deg = 180.0
        self.vel_deg_s = 0.0
        self.prev_angle = 180.0
        self.last_time = 0.0
        self.elapsed_time = 0.0

        self.serial_thread = None
        self.is_connected = False

        # Charts history (Angle Plot only)
        self.history_len = 250
        self.angle_history = deque([180.0] * self.history_len, maxlen=self.history_len)

        # Initialize Layouts
        self.init_ui()

        # Tick timer (unconditional clock loop at 60+ FPS)
        self.timer = QTimer()
        self.timer.timeout.connect(self.tick)
        self.timer.start(8)

        # Background connection scanner
        self.port_timer = QTimer()
        self.port_timer.timeout.connect(self.auto_connection_handler)
        self.port_timer.start(1500)
        self.auto_connection_handler()

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
        lbl_subtitle = QLabel("Real-time Microcontroller Telemetry Monitor")
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
        
        self.lbl_telemetry = QLabel("Time: 0.0s | Port: Searching...")
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

        # Right Panel: Telemetry Charts and Text Displays
        right_widget = QWidget()
        right_widget.setFixedWidth(380)
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(16)
        splitter.addWidget(right_widget)

        # Value Readouts Card
        readout_card = CardWidget("LIVE METRICS")
        readout_layout = QGridLayout()
        readout_layout.setSpacing(10)
        
        lbl_angle_title = QLabel("PENDULUM ANGLE:")
        lbl_angle_title.setStyleSheet("font-size: 11px; font-weight: 800; color: #555555;")
        self.lbl_angle_val = QLabel("180.00°")
        self.lbl_angle_val.setStyleSheet("font-size: 38px; font-family: 'Consolas', monospace; font-weight: bold; color: #000000;")
        
        lbl_vel_title = QLabel("ANGULAR VELOCITY:")
        lbl_vel_title.setStyleSheet("font-size: 11px; font-weight: 800; color: #555555;")
        self.lbl_vel_val = QLabel("0.0°/s")
        self.lbl_vel_val.setStyleSheet("font-size: 38px; font-family: 'Consolas', monospace; font-weight: bold; color: #000000;")

        readout_layout.addWidget(lbl_angle_title, 0, 0)
        readout_layout.addWidget(self.lbl_angle_val, 1, 0)
        readout_layout.addWidget(lbl_vel_title, 2, 0)
        readout_layout.addWidget(self.lbl_vel_val, 3, 0)
        readout_card.layout.addLayout(readout_layout)
        right_layout.addWidget(readout_card)

        # Angle vs Time
        self.angle_card = CardWidget("ANGLE VS TIME")
        self.angle_plot = pg.PlotWidget()
        self.angle_curve = self.style_chart(self.angle_plot, 150.0, 210.0, (0, 0, 0, 10))
        self.angle_target = pg.InfiniteLine(pos=180.0, angle=0, pen=pg.mkPen('#ff3333', width=1.5, style=Qt.PenStyle.DashLine))
        self.angle_plot.addItem(self.angle_target)
        self.angle_card.layout.addWidget(self.angle_plot)
        right_layout.addWidget(self.angle_card, 1)

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
    # Background Serial Thread auto-connections
    # ---------------------------------------------------------
    def auto_connection_handler(self):
        if self.serial_thread and self.serial_thread.isRunning():
            return

        ports = [p.device for p in serial.tools.list_ports.comports()]
        target_port = self.config.get("preferred_port", "COM3")
        
        if not ports:
            self.set_status_indicator("Searching...", "gray")
            return

        chosen_port = target_port if target_port in ports else ports[0]
        baud = self.config.get("baud_rate", 115200)

        print(f"[COM] Device detected. Starting serial reader thread on {chosen_port} ({baud} baud)...")
        self.serial_thread = SerialReader(chosen_port, baud)
        self.serial_thread.angle_received.connect(self.on_angle_received)
        self.serial_thread.status_changed.connect(self.set_status_indicator)
        self.serial_thread.start()

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

        if state == "red":
            if self.serial_thread:
                self.serial_thread.stop()
                self.serial_thread = None

    def on_angle_received(self, raw_angle):
        current_time = time.time()
        dt = current_time - self.last_time if self.last_time > 0 else 0.008
        self.last_time = current_time

        offset = self.config.get("angle_offset", 180.0)
        scale = self.config.get("angle_scale", 1.0)
        invert = -1.0 if self.config.get("angle_invert", False) else 1.0

        # Calibrate the angle
        self.angle_deg = (raw_angle - offset) * scale * invert + 180.0
        
        # Calculate velocity by differentiating consecutive angle measurements
        diff = self.angle_deg - self.prev_angle
        # handle circular wrap-around
        diff = (diff + 180.0) % 360.0 - 180.0
        self.vel_deg_s = diff / dt if dt > 0 else 0.0
        self.prev_angle = self.angle_deg

        # Store radians for the canvas animation (where 0 rad is vertical upright)
        self.theta = math.radians(self.angle_deg - 180.0)

        # Print to console
        print(f"[Telemetry] Time: {self.elapsed_time:.2f}s | Angle: {self.angle_deg:.2f}° | Vel: {self.vel_deg_s:.1f}°/s")

    # ---------------------------------------------------------
    # Core update tick (unconditional execution)
    # ---------------------------------------------------------
    def tick(self):
        self.elapsed_time += self.dt

        # Decimated buffer updates (~32ms)
        if int(self.elapsed_time / self.dt) % 4 == 0:
            self.angle_history.append(self.angle_deg)
            self.angle_curve.setData(list(self.angle_history))

        # Update text labels
        self.lbl_angle_val.setText(f"{self.angle_deg:.2f}°")
        self.lbl_vel_val.setText(f"{self.vel_deg_s:.1f}°/s")

        if self.is_connected:
            self.lbl_telemetry.setText(
                f"Time: {self.elapsed_time:.1f}s | Port: {self.serial_thread.port if self.serial_thread else 'N/A'}"
            )
        else:
            self.lbl_telemetry.setText(
                f"Time: {self.elapsed_time:.1f}s | Port: Searching for hardware..."
            )

        # Repaint canvas
        self.canvas_widget.update_state(self.theta)

    def closeEvent(self, event):
        if self.serial_thread:
            self.serial_thread.stop()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())