from PyQt6.QtWidgets import QFrame, QVBoxLayout, QLabel

class CardWidget(QFrame):
    """
    Custom styled QFrame container with a CAD high-contrast border and optional title header.
    Matches clean engineering aesthetics.
    """
    def __init__(self, title: str = None, parent=None):
        super().__init__(parent)
        self.setFrameShape(QFrame.Shape.StyledPanel)
        self.setStyleSheet("""
            CardWidget {
                border: 1px solid #777777;
                border-radius: 8px;
            }
        """)
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(16, 16, 16, 16)
        self.layout.setSpacing(8)

        if title:
            self.title_label = QLabel(title)
            self.title_label.setStyleSheet("""
                font-family: 'Inter', 'Segoe UI', sans-serif;
                font-size: 13px;
                font-weight: 800;
                border: none;
                border-bottom: 1px solid #777777;
                padding-bottom: 4px;
                margin-bottom: 4px;
            """)
            self.layout.addWidget(self.title_label)
