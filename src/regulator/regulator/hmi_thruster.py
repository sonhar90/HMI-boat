from PyQt5.QtCore import Qt, QRectF
from PyQt5.QtGui import QPainter, QPen, QColor
from PyQt5.QtWidgets import QFrame, QSizePolicy


class ThrusterBar(QFrame):
    def __init__(self, thruster_id, max_forward_value, max_reverse_value, parent=None):
        super().__init__(parent)
        self.thruster_id = thruster_id
        self.max_forward_value = max_forward_value  # Maximum positive value
        self.max_reverse_value = (
            max_reverse_value  # Maximum negative value (negative number)
        )
        self.setpoint = 0.0  # Setpoint value
        self.current_value = 0.0  # Current value
        self.opacity_level = 0.9  # Default opacity
        self.init_ui()

    def init_ui(self):
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMinimumHeight(50)
        self.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        self.setStyleSheet("background-color: #1E1E1E;")  # Match overall theme

    def set_values(self, setpoint, current_value):
        self.setpoint = setpoint
        self.current_value = current_value
        self.update()  # Trigger repaint

    def paintEvent(self, event):
        # print(f"ThrusterBar {self.thruster_id}: paintEvent called")
        # print(
        #    f"ThrusterBar {self.thruster_id}: size - width={self.width()}, height={self.height()}"
        # )
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        rect = self.rect()

        # Apply padding
        padding = 10
        rect = rect.adjusted(padding, padding, -padding, -padding)

        # Get dimensions
        width = rect.width()
        height = rect.height()
        center_y = rect.top() + height / 2

        # Set opacity
        painter.setOpacity(self.opacity_level)

        # Draw the background
        painter.fillRect(self.rect(), Qt.lightGray)

        # Draw the center line (zero line)
        pen = QPen(Qt.white, 2)
        painter.setPen(pen)
        painter.drawLine(
            int(rect.left()), int(center_y), int(rect.right()), int(center_y)
        )

        # Calculate current thrust percentage and bar length
        if self.current_value >= 0:
            thrust_percentage = self.current_value / self.max_forward_value
            bar_length = (height / 2) * thrust_percentage
            # Draw forward thrust bar (upwards)
            rect_bar = QRectF(rect.left(), center_y - bar_length, width, bar_length)
            painter.fillRect(rect_bar, QColor("green"))
        else:
            thrust_percentage = abs(self.current_value) / abs(self.max_reverse_value)
            bar_length = (height / 2) * thrust_percentage
            # Draw reverse thrust bar (downwards)
            rect_bar = QRectF(rect.left(), center_y, width, bar_length)
            painter.fillRect(rect_bar, QColor("red"))

        # Calculate setpoint position
        if self.setpoint >= 0:
            setpoint_percentage = self.setpoint / self.max_forward_value
            setpoint_position = center_y - (height / 2) * setpoint_percentage
        else:
            setpoint_percentage = abs(self.setpoint) / abs(self.max_reverse_value)
            setpoint_position = center_y + (height / 2) * setpoint_percentage

        # Draw setpoint line
        pen = QPen(Qt.blue, 2, Qt.DashLine)
        painter.setPen(pen)
        painter.drawLine(
            int(rect.left()),
            int(setpoint_position),
            int(rect.right()),
            int(setpoint_position),
        )

        # Draw outline
        pen = QPen(Qt.white, 2)
        painter.setPen(pen)
        painter.drawRect(rect)

        # Draw thruster ID at the top
        painter.setPen(QPen(Qt.white))
        painter.drawText(
            rect, Qt.AlignTop | Qt.AlignHCenter, f"Thruster {self.thruster_id}"
        )

        # Draw current value at the bottom
        painter.drawText(
            rect, Qt.AlignBottom | Qt.AlignHCenter, f"{self.current_value:.2f}"
        )

        painter.end()

    def set_opacity(self, opacity_level):
        # Store the opacity level
        self.opacity_level = opacity_level
        self.update()  # Trigger a repaint
