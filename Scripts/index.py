import sys
import math
import time
import json
import os
import numpy as np
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
            # Open WITHOUT asserting DTR/RTS to prevent the ESP32 auto-reset circuit
            # from rebooting the microcontroller (DTR high → EN pin reset pulse).
            # This matches Arduino Serial Monitor behaviour: connect to an already-running board.
            ser = serial.Serial()
            ser.write_timeout = 0
            ser.port = self.port
            ser.baudrate = self.baud
            ser.timeout = 0.05
            ser.dtr = False
            ser.rts = False
            ser.open()
            print(f"[COM] Serial port {self.port} opened successfully (baud={self.baud}, timeout=0.1s, DTR=False)")
            self.status_changed.emit(f"Connected: {self.port}", "green")

            # Flush any partial data sitting in the OS buffer and discard the first
            # (potentially incomplete) line so we start on a clean line boundary.
            ser.reset_input_buffer()
            ser.readline()
            print("[COM] Input buffer reset, first partial line discarded. Entering read loop...")

            while self.running:
                raw = ser.readline()          # blocks up to 0.1s for a '\n'
                if not raw:
                    continue                  # timeout with no data – just retry

                try:
                    line = raw.decode('utf-8', errors='ignore').strip()
                except Exception as e:
                    print(f"[COM] Decode error: {e} | raw bytes: {raw!r}")
                    continue

                if not line:
                    continue                  # blank line (e.g. lone \r\n)

                try:
                    value = float(line)
                except ValueError:
                    print(f"[COM] Parse error – not a float: '{line}'")
                    continue

                self.angle_received.emit(value)

            ser.close()
            print("[COM] Serial port closed normally.")
            self.status_changed.emit("Idle", "gray")

        except serial.SerialException as e:
            print(f"[COM] Serial error on {self.port}: {e}")
            self.status_changed.emit("Disconnected", "red")
        except Exception as e:
            print(f"[COM] Unexpected error on {self.port}: {e}")
            self.status_changed.emit("Disconnected", "red")

    def stop(self):
        self.running = False
        self.wait(2000)  # wait up to 2s so readline() can finish its current timeout

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

        # ── Protractor: angle markings around pivot ──
        protractor_r = 130  # radius of the tick circle
        tick_font = QFont("Consolas", 8)
        painter.setFont(tick_font)

        for deg in range(0, 360, 10):
            rad = math.radians(deg)
            # tick direction: 0° = straight up, clockwise positive
            sin_a = math.sin(rad)
            cos_a = math.cos(rad)

            is_major = (deg % 30 == 0)
            tick_inner = protractor_r - (12 if is_major else 6)
            tick_outer = protractor_r

            x1 = cx + sin_a * tick_inner
            y1 = cy - cos_a * tick_inner
            x2 = cx + sin_a * tick_outer
            y2 = cy - cos_a * tick_outer

            if is_major:
                painter.setPen(QPen(QColor(160, 160, 160), 1.2, Qt.PenStyle.SolidLine))
            else:
                painter.setPen(QPen(QColor(200, 200, 200), 0.8, Qt.PenStyle.SolidLine))
            painter.drawLine(QPointF(x1, y1), QPointF(x2, y2))

            # Labels at every 30°
            if is_major:
                label_r = protractor_r + 14
                lx = cx + sin_a * label_r
                ly = cy - cos_a * label_r
                painter.setPen(QPen(QColor(140, 140, 140)))
                text = f"{deg}°"
                fm = painter.fontMetrics()
                tw = fm.horizontalAdvance(text)
                th = fm.height()
                painter.drawText(int(lx - tw / 2), int(ly + th / 4), text)

        # Faint protractor arc
        painter.setPen(QPen(QColor(220, 220, 220), 0.8, Qt.PenStyle.SolidLine))
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawEllipse(QPointF(cx, cy), protractor_r, protractor_r)

        # ── Draw Cart and Rail ──
        # Rail (thick light gray line)
        rail_y = cy + 20
        painter.setPen(QPen(QColor(220, 220, 220), 8, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
        painter.drawLine(int(cx - 200), int(rail_y), int(cx + 200), int(rail_y))

        # Cart Body
        cart_w = 90
        cart_h = 40
        painter.setPen(QPen(QColor(60, 60, 60), 2))
        painter.setBrush(QBrush(QColor(245, 245, 245)))
        painter.drawRoundedRect(int(cx - cart_w / 2), int(cy - cart_h / 2), cart_w, cart_h, 6, 6)

        # Wheels
        painter.setBrush(QBrush(QColor(80, 80, 80)))
        painter.drawEllipse(QPointF(cx - 25, rail_y), 6, 6)
        painter.drawEllipse(QPointF(cx + 25, rail_y), 6, 6)

        # Pivot mount crosshair on cart
        painter.setPen(QPen(QColor(100, 100, 100), 1, Qt.PenStyle.DotLine))
        painter.drawLine(int(cx - 20), int(cy), int(cx + 20), int(cy))
        painter.drawLine(int(cx), int(cy - 20), int(cx), int(cy + 20))
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawEllipse(QPointF(cx, cy), 12.0, 12.0)

        # Reference upright dotted target line
        painter.setPen(QPen(QColor(255, 50, 50, 120), 1, Qt.PenStyle.DashLine))
        painter.drawLine(int(cx), int(cy - 180), int(cx), int(cy + 40))

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
        self.theta = 0.0
        self.raw_angle = 0.0       # raw 0-360 from sensor
        self.angle_dev = 0.0       # deviation from offset (centered on 0)
        self.vel_deg_s = 0.0
        self.prev_raw = None       # for wrap-aware velocity
        self.last_time = 0.0
        self.elapsed_time = 0.0
        self.start_time = None     # real clock reference for X axis
        self.peak_angle = 0.0
        self.peak_vel = 0.0
        self.sample_count = 0
        self.sample_rate = 0.0
        self.rate_timer = time.time()
        self.rate_count = 0
        self._data_dirty = False   # flag: new data arrived since last graph redraw

        self.serial_thread = None
        self.is_connected = False

        # Pre-allocated numpy ring buffer for chart data
        self.history_len = 800
        self._buf_time = np.zeros(self.history_len, dtype=np.float64)
        self._buf_angle = np.zeros(self.history_len, dtype=np.float64)
        self._buf_vel = np.zeros(self.history_len, dtype=np.float64)
        self._buf_idx = 0      # write cursor
        self._buf_count = 0    # total samples stored (capped at history_len)

        # Initialize Layouts
        self.init_ui()

        # Fast timer: canvas + labels at ~60 FPS
        self.timer = QTimer()
        self.timer.timeout.connect(self.tick_fast)
        self.timer.start(16)

        # Slow timer: graph redraws at ~20 FPS
        self.graph_timer = QTimer()
        self.graph_timer.timeout.connect(self.tick_graph)
        self.graph_timer.start(50)

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
        right_widget.setFixedWidth(420)
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(12)
        splitter.addWidget(right_widget)

        # ── Metrics Card ──
        readout_card = CardWidget("LIVE METRICS")
        readout_grid = QGridLayout()
        readout_grid.setSpacing(6)
        big_val_style = "font-size: 32px; font-family: 'Consolas', monospace; font-weight: bold; color: #000000;"
        small_title_style = "font-size: 10px; font-weight: 800; color: #555555;"
        small_val_style = "font-size: 16px; font-family: 'Consolas', monospace; font-weight: bold; color: #000000;"
        spinbox_style = """
            QDoubleSpinBox {
                font-family: 'Consolas', monospace;
                font-size: 14px; font-weight: bold;
                color: #000000; background: #f5f5f5;
                border: 1px solid #cccccc; border-radius: 4px;
                padding: 2px 6px;
            }
            QDoubleSpinBox::up-button, QDoubleSpinBox::down-button { width: 18px; }
        """

        # Row 0-1: Deviation angle (big)
        lbl_dev_title = QLabel("DEVIATION FROM ZERO:")
        lbl_dev_title.setStyleSheet(small_title_style)
        self.lbl_angle_val = QLabel("0.00°")
        self.lbl_angle_val.setStyleSheet(big_val_style)
        readout_grid.addWidget(lbl_dev_title, 0, 0, 1, 2)
        readout_grid.addWidget(self.lbl_angle_val, 1, 0, 1, 2)

        # Row 2-3: Velocity (big)
        lbl_vel_title = QLabel("ANGULAR VELOCITY:")
        lbl_vel_title.setStyleSheet(small_title_style)
        self.lbl_vel_val = QLabel("0.0°/s")
        self.lbl_vel_val.setStyleSheet(big_val_style)
        readout_grid.addWidget(lbl_vel_title, 2, 0, 1, 2)
        readout_grid.addWidget(self.lbl_vel_val, 3, 0, 1, 2)

        # Row 4: Raw angle | Sample rate (side by side)
        lbl_raw_title = QLabel("RAW SENSOR:")
        lbl_raw_title.setStyleSheet(small_title_style)
        self.lbl_raw_val = QLabel("—")
        self.lbl_raw_val.setStyleSheet(small_val_style)
        lbl_rate_title = QLabel("SAMPLE RATE:")
        lbl_rate_title.setStyleSheet(small_title_style)
        self.lbl_rate_val = QLabel("— Hz")
        self.lbl_rate_val.setStyleSheet(small_val_style)
        readout_grid.addWidget(lbl_raw_title, 4, 0)
        readout_grid.addWidget(lbl_rate_title, 4, 1)
        readout_grid.addWidget(self.lbl_raw_val, 5, 0)
        readout_grid.addWidget(self.lbl_rate_val, 5, 1)

        # Row 6: Peak angle | Peak velocity (side by side)
        lbl_peak_a_title = QLabel("PEAK ANGLE:")
        lbl_peak_a_title.setStyleSheet(small_title_style)
        self.lbl_peak_angle = QLabel("0.0°")
        self.lbl_peak_angle.setStyleSheet(small_val_style)
        lbl_peak_v_title = QLabel("PEAK VELOCITY:")
        lbl_peak_v_title.setStyleSheet(small_title_style)
        self.lbl_peak_vel = QLabel("0.0°/s")
        self.lbl_peak_vel.setStyleSheet(small_val_style)
        readout_grid.addWidget(lbl_peak_a_title, 6, 0)
        readout_grid.addWidget(lbl_peak_v_title, 6, 1)
        readout_grid.addWidget(self.lbl_peak_angle, 7, 0)
        readout_grid.addWidget(self.lbl_peak_vel, 7, 1)

        readout_card.layout.addLayout(readout_grid)
        right_layout.addWidget(readout_card)

        # ── Angle deviation graph (centered on 0) ──
        self.angle_card = CardWidget("DEVIATION VS TIME")
        self.angle_plot = pg.PlotWidget()
        self.angle_curve = self.style_chart(self.angle_plot, y_label='Deviation', y_unit='°', line_color='#0066cc')
        self.angle_zero_line = pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen('#ff3333', width=1.5, style=Qt.PenStyle.DashLine))
        self.angle_plot.addItem(self.angle_zero_line)
        self.angle_card.layout.addWidget(self.angle_plot)
        right_layout.addWidget(self.angle_card, 1)

        # ── Velocity graph ──
        self.vel_card = CardWidget("VELOCITY VS TIME")
        self.vel_plot = pg.PlotWidget()
        self.vel_curve = self.style_chart(self.vel_plot, y_label='Velocity', y_unit='°/s', line_color='#cc3333')
        self.vel_zero_line = pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen('#aaaaaa', width=1, style=Qt.PenStyle.DashLine))
        self.vel_plot.addItem(self.vel_zero_line)
        self.vel_card.layout.addWidget(self.vel_plot)
        right_layout.addWidget(self.vel_card, 1)

        splitter.setSizes([950, 420])

    def style_chart(self, plot, y_label='Value', y_unit='', line_color='#000000'):
        plot.setBackground('#ffffff')
        plot_item = plot.getPlotItem()
        plot_item.setContentsMargins(0, 0, 0, 0)
        plot_item.showGrid(x=True, y=True, alpha=0.15)
        plot_item.setClipToView(True)          # only render visible points
        plot_item.setDownsampling(mode='peak')  # auto-downsample dense data

        plot_item.showAxis('right', False)
        plot_item.showAxis('top', False)
        plot_item.enableAutoRange(axis='y')

        font = QFont("Consolas")
        font.setPointSize(8)

        left_axis = plot_item.getAxis('left')
        left_axis.setPen(pg.mkPen('#cccccc', width=1))
        left_axis.setTextPen(pg.mkPen('#555555'))
        left_axis.setTickFont(font)
        left_axis.setLabel(y_label, units=y_unit, **{'font-size': '10px', 'color': '#555555'})

        bottom_axis = plot_item.getAxis('bottom')
        bottom_axis.setPen(pg.mkPen('#cccccc', width=1))
        bottom_axis.setTextPen(pg.mkPen('#555555'))
        bottom_axis.setTickFont(font)
        bottom_axis.setLabel('Time', units='s', **{'font-size': '10px', 'color': '#555555'})

        curve = plot_item.plot(pen=pg.mkPen(line_color, width=1.5))
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
        self.serial_thread.angle_received.connect(
            self.on_angle_received, Qt.ConnectionType.DirectConnection)
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

    @staticmethod
    def _wrap_180(deg):
        """Wrap an angle into the -180 … +180 range."""
        return (deg + 180.0) % 360.0 - 180.0

    def on_angle_received(self, raw_angle):
        current_time = time.time()
        dt = current_time - self.last_time if self.last_time > 0 else 0.008
        self.last_time = current_time
        if self.start_time is None:
            self.start_time = current_time

        self.raw_angle = raw_angle

        # Deviation from offset, wrapped to -180…+180 → clean sine wave
        offset = self.config.get("angle_offset", 180.0)
        self.angle_dev = self._wrap_180(raw_angle - offset)

        # Wrap-aware velocity: handles the 0/360 boundary correctly
        if self.prev_raw is not None and dt > 0:
            delta = self._wrap_180(raw_angle - self.prev_raw)
            self.vel_deg_s = delta / dt
        self.prev_raw = raw_angle

        # Canvas angle (theta=0 is UP)
        self.theta = math.radians(self.angle_dev)

        # Track peaks
        self.peak_angle = max(self.peak_angle, abs(self.angle_dev))
        self.peak_vel = max(self.peak_vel, abs(self.vel_deg_s))

        # Sample rate counter
        self.rate_count += 1
        self.sample_count += 1
        rate_elapsed = current_time - self.rate_timer
        if rate_elapsed >= 1.0:
            self.sample_rate = self.rate_count / rate_elapsed
            self.rate_count = 0
            self.rate_timer = current_time

        # Write into numpy ring buffer (zero-copy, no allocation)
        i = self._buf_idx % self.history_len
        self._buf_time[i] = current_time - self.start_time
        self._buf_angle[i] = self.angle_dev
        self._buf_vel[i] = self.vel_deg_s
        self._buf_idx += 1
        self._buf_count = min(self._buf_count + 1, self.history_len)
        self._data_dirty = True

    def _get_buf_slices(self):
        """Return atomic slices of the ring buffer to prevent race conditions during updates."""
        count = self._buf_count
        idx = self._buf_idx
        
        if count < self.history_len:
            return self._buf_time[:count], self._buf_angle[:count], self._buf_vel[:count]
            
        start = idx % self.history_len
        t = np.concatenate((self._buf_time[start:], self._buf_time[:start]))
        a = np.concatenate((self._buf_angle[start:], self._buf_angle[:start]))
        v = np.concatenate((self._buf_vel[start:], self._buf_vel[:start]))
        return t, a, v

    # ---------------------------------------------------------
    # Fast tick: canvas + labels (~60 FPS)
    # ---------------------------------------------------------
    def tick_fast(self):
        self.elapsed_time = (time.time() - self.start_time) if self.start_time else 0.0

        # Update text labels
        self.lbl_angle_val.setText(f"{self.angle_dev:+.2f}°")
        self.lbl_vel_val.setText(f"{self.vel_deg_s:+.1f}°/s")
        self.lbl_raw_val.setText(f"{self.raw_angle:.2f}°")
        self.lbl_rate_val.setText(f"{self.sample_rate:.0f} Hz")
        self.lbl_peak_angle.setText(f"{self.peak_angle:.1f}°")
        self.lbl_peak_vel.setText(f"{self.peak_vel:.1f}°/s")

        if self.is_connected:
            self.lbl_telemetry.setText(
                f"Time: {self.elapsed_time:.1f}s | Port: {self.serial_thread.port if self.serial_thread else 'N/A'} | {self.sample_rate:.0f} Hz"
            )
        else:
            self.lbl_telemetry.setText(
                f"Time: {self.elapsed_time:.1f}s | Port: Searching for hardware..."
            )

        # Repaint canvas (very cheap)
        self.canvas_widget.update_state(self.theta)

    # ---------------------------------------------------------
    # Slow tick: graph redraws (~20 FPS, only when new data)
    # ---------------------------------------------------------
    def tick_graph(self):
        if not self._data_dirty:
            return
        self._data_dirty = False

        t, a, v = self._get_buf_slices()
        self.angle_curve.setData(t, a)
        self.vel_curve.setData(t, v)

    def closeEvent(self, event):
        if self.serial_thread:
            self.serial_thread.stop()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())