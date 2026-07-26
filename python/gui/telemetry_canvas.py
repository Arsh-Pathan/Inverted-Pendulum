import math
from PyQt6.QtWidgets import QWidget
from PyQt6.QtCore import Qt, QPointF
from PyQt6.QtGui import QPainter, QPen, QColor, QBrush, QFont

class TelemetryCanvas(QWidget):
    """
    CAD/Engineering Telemetry Viewport (Fixed Pivot Pendulum).
    Renders high-contrast protractor markings, track rail, cart body, and
    real-time pendulum rod orientation.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.theta = 0.0  # radians (0 is vertical upright)

    def update_state(self, theta_rad: float):
        self.theta = theta_rad
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
        painter.drawEllipse(QPointF(cx, cy), float(protractor_r), float(protractor_r))

        # ── Draw Cart and Rail ──
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
        painter.drawEllipse(QPointF(cx - 25, rail_y), 6.0, 6.0)
        painter.drawEllipse(QPointF(cx + 25, rail_y), 6.0, 6.0)

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
        bob_r = 15.0
        painter.setBrush(QBrush(QColor(0, 0, 0)))
        painter.drawEllipse(QPointF(px, py), bob_r, bob_r)

        # Inner center pin
        painter.setBrush(QBrush(QColor(255, 255, 255)))
        painter.setPen(QPen(QColor(0, 0, 0), 1.5))
        painter.drawEllipse(QPointF(cx, cy), 4.0, 4.0)
