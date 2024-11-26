import os
import numpy as np
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPixmap, QPainter, QPen
from PyQt5.QtWidgets import QWidget, QGridLayout, QLabel, QSlider, QVBoxLayout
from PyQt5.QtCore import QPoint
from ament_index_python.packages import get_package_share_directory


class CompassWidget(QWidget):
    def __init__(self):
        super().__init__()

        # Get the share directory of the package
        package_share_directory = get_package_share_directory("regulator")

        # Construct the full paths to the images
        compass_image_path = os.path.join(
            package_share_directory, "images", "compass_background.png"
        )
        otter_image_path = os.path.join(package_share_directory, "images", "otter.png")
        arrow_image_path = os.path.join(package_share_directory, "images", "arrow.png")

        print(f"Compass image path: {compass_image_path}")
        print(f"Otter image path: {otter_image_path}")
        print(f"Arrow image path: {arrow_image_path}")

        # Load the compass background image
        self.compass_pixmap = QPixmap(compass_image_path)
        if self.compass_pixmap.isNull():
            print("Failed to load compass background image.")

        # Load the feedback image (otter)
        self.feedback_pixmap = QPixmap(otter_image_path)
        if self.feedback_pixmap.isNull():
            print("Failed to load otter image.")

        # Load the setpoint image (arrow)
        self.setpoint_pixmap = QPixmap(arrow_image_path)
        if self.setpoint_pixmap.isNull():
            print("Failed to load arrow image.")

        self.setpoint = 0
        self.feedback = 0

        # Timer for updating the feedback (optional)
        self.timer = QTimer()
        self.timer.timeout.connect(self.set_feedback)
        self.timer.start(100)  # Update every 100 ms

    def set_setpoint(self, value):
        self.setpoint = value
        self.update()  # Trigger repaint

    def set_feedback(self, value):
        self.feedback = value
        self.update()  # Trigger repaint

    def paintEvent(self, event):
        try:
            painter = QPainter(self)

            painter.fillRect(self.rect(), Qt.lightGray)
            # Draw the compass background
            compass_size = int(min(self.width(), self.height()))
            if not self.compass_pixmap.isNull():
                compass_scaled = self.compass_pixmap.scaled(
                    compass_size,
                    compass_size,
                    Qt.KeepAspectRatio,
                    Qt.SmoothTransformation,
                )
                compass_rect = compass_scaled.rect()
                compass_rect.moveCenter(self.rect().center())

                shift_x_1 = 2
                compass_rect.translate(-shift_x_1, 0)

                painter.drawPixmap(compass_rect.topLeft(), compass_scaled)
            else:
                print("Compass pixmap is null. Drawing placeholder.")
                painter.drawEllipse(
                    -100, -100, 200, 200
                )  # Simple circle as placeholder

            # Move the painter coordinate system to the center
            center = self.rect().center()
            painter.translate(center)

            # --- Draw Setpoint Indicator ---
            if not self.setpoint_pixmap.isNull():
                painter.save()
                painter.rotate(self.setpoint)  # Rotate to setpoint angle

                # Draw setpoint indicator using an image
                setpoint_size = int(compass_size * 0.75)
                setpoint_scaled = self.setpoint_pixmap.scaled(
                    setpoint_size,
                    setpoint_size,
                    Qt.KeepAspectRatio,
                    Qt.SmoothTransformation,
                )
                setpoint_rect = setpoint_scaled.rect()
                setpoint_rect.moveCenter(QPoint(0, 0))
                painter.drawPixmap(setpoint_rect.topLeft(), setpoint_scaled)

                painter.restore()
            else:
                print("Setpoint pixmap is null. Drawing setpoint as a red line.")
                painter.save()
                painter.rotate(self.setpoint)  # Rotate to setpoint angle
                pen = QPen(Qt.red, 3, Qt.SolidLine)
                painter.setPen(pen)
                painter.drawLine(0, 0, 0, -int(compass_size * 0.4))  # Adjust as needed
                painter.restore()

            # --- Draw Feedback Indicator ---
            if not self.feedback_pixmap.isNull():
                painter.save()
                painter.rotate(self.feedback)  # Rotate to feedback angle

                # Draw feedback indicator (otter image)
                feedback_size = int(compass_size * 0.7)
                feedback_scaled = self.feedback_pixmap.scaled(
                    feedback_size,
                    feedback_size,
                    Qt.KeepAspectRatio,
                    Qt.SmoothTransformation,
                )
                feedback_rect = feedback_scaled.rect()
                feedback_rect.moveCenter(QPoint(-2, 0))  # Adjust position as needed
                painter.drawPixmap(feedback_rect.topLeft(), feedback_scaled)

                painter.restore()
            else:
                print("Feedback pixmap is null. Skipping drawing feedback.")

        except Exception as e:
            print(f"An error occurred in paintEvent: {e}")
