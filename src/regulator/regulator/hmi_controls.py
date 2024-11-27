from PyQt5.QtWidgets import (
    QWidget,
    QLabel,
    QDial,
    QLCDNumber,
    QVBoxLayout,
    QSizePolicy,
)
from PyQt5.QtCore import Qt, pyqtSignal


class HeadingControlWidget(QWidget):
    headingChanged = pyqtSignal(float)  # Signal emitted when heading changes

    def __init__(self, parent=None):
        super().__init__(parent)
        # Create widgets
        heading_label = QLabel("Heading Control")
        heading_label.setAlignment(Qt.AlignCenter)
        heading_label.setStyleSheet("color: white;")

        self.heading_display = QLCDNumber()
        self.heading_display.setSegmentStyle(QLCDNumber.Flat)
        self.heading_display.setDigitCount(3)
        self.heading_display.setStyleSheet(
            """
            QLCDNumber {
                background-color: #1E1E1E;
                color: #7DF9FF;
            }
        """
        )

        self.heading_dial = QDial()
        self.heading_dial.setMinimum(0)
        self.heading_dial.setMaximum(360)
        self.heading_dial.setWrapping(True)
        self.heading_dial.setValue(180)  # Set initial value to 180 (north)
        self.heading_dial.setStyleSheet(
            """
            QDial {
                background-color: #333333;
            }
            QDial::handle {
                background-color: #7DF9FF;
            }
        """
        )

        # Initial display value remapped to north (0)
        self.heading_display.display(self.remap_dial_value(self.heading_dial.value()))

        # Connect valueChanged signal to update display and emit headingChanged
        self.heading_dial.valueChanged.connect(self.on_dial_value_changed)

        # Set size policies to allow horizontal expansion
        heading_label.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        self.heading_display.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.heading_dial.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        # Layout
        layout = QVBoxLayout()
        layout.addWidget(heading_label)
        layout.addWidget(self.heading_display)
        layout.addWidget(self.heading_dial)

        # Adjust stretch factors
        layout.setStretch(0, 1)  # heading_label
        layout.setStretch(1, 1)  # heading_display
        layout.setStretch(2, 1)  # heading_dial

        # Adjust margins and spacing
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(10)

        self.setLayout(layout)

    def remap_dial_value(self, value):
        remapped_value = (value - 180) % 360
        return remapped_value

    def on_dial_value_changed(self, value):
        remapped_value = self.remap_dial_value(value)
        self.heading_display.display(remapped_value)
        self.headingChanged.emit(remapped_value)


class SpeedControlWidget(QWidget):
    speedChanged = pyqtSignal(float)

    def __init__(self, parent=None):
        super().__init__(parent)
        speed_label = QLabel("Speed Control")
        speed_label.setAlignment(Qt.AlignCenter)
        speed_label.setStyleSheet("color: white;")

        self.speed_display = QLCDNumber()
        self.speed_display.setSegmentStyle(QLCDNumber.Flat)
        self.speed_display.setDigitCount(3)
        self.speed_display.setStyleSheet(
            """
            QLCDNumber {
                background-color: #1E1E1E;
                color: #7DF9FF;
            }
        """
        )

        self.speed_dial = QDial()
        self.speed_dial.setMinimum(-3)
        self.speed_dial.setMaximum(3)
        self.speed_dial.setValue(0)
        self.speed_dial.setStyleSheet(
            """
            QDial {
                background-color: #333333;
            }
            QDial::handle {
                background-color: #7DF9FF;
            }
        """
        )

        self.speed_display.display(self.speed_dial.value())
        self.speed_dial.valueChanged.connect(self.on_dial_value_changed)

        # Set size policies to allow horizontal expansion
        speed_label.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        self.speed_display.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.speed_dial.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        # Layout
        layout = QVBoxLayout()
        layout.addWidget(speed_label)
        layout.addWidget(self.speed_display)
        layout.addWidget(self.speed_dial)

        # Adjust stretch factors
        layout.setStretch(0, 1)  # speed_label
        layout.setStretch(1, 1)  # speed_display
        layout.setStretch(2, 1)  # speed_dial

        # Adjust margins and spacing
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(10)

        self.setLayout(layout)

    def on_dial_value_changed(self, value):
        self.speed_display.display(value)
        self.speedChanged.emit(value)
