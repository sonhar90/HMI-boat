from PyQt5.QtWidgets import QPushButton, QSlider, QLCDNumber
from PyQt5.QtCore import Qt

def add_speed_control(self):
    speed_label = QPushButton('Speed Control')
    speed_label.setEnabled(False)  # Disable the button so it behaves like a label
    speed_slider = QSlider(Qt.Horizontal)
    speed_slider.setMinimum(0)
    speed_slider.setMaximum(100)
    speed_slider.setValue(50)
    speed_slider.setTickPosition(QSlider.TicksBelow)
    speed_slider.setTickInterval(10)
    speed_slider.valueChanged.connect(self.update_speed)

    speed_slider.setStyleSheet("""
        QSlider::groove:horizontal {
            border: 1px solid #999;
            height: 20px;
            background: #b0c4de;  /* Light Steel Blue */
        }

        QSlider::handle:horizontal {
            background: #4682b4;  /* Steel Blue handle */
            border: 1px solid #333;
            width: 20px;
            margin: -10px 0;
        }

        QSlider::handle:horizontal:pressed {
            background: #00bfff;  /* Light up handle when pressed */
            box-shadow: 0px 0px 15px #00bfff;  /* Add glow effect */
        }

        QSlider::add-page:horizontal {
            background: #D3D3D3;  /* Groove before the handle */
        }

        QSlider::sub-page:horizontal {
            background: #4682b4;  /* Default filled groove color */
        }

        QSlider::groove:horizontal:pressed {
            background: #00bfff;  /* Light up the filled groove when pressed */
            box-shadow: 0px 0px 10px #00bfff;  /* Add glow to the filled groove */
        }
    """)

    speed_display = QLCDNumber()
    speed_display.setDigitCount(3)
    speed_display.display(speed_slider.value())

    # Sett fast størrelse for speed-slider og display
    speed_slider.setFixedSize(200, 50)  # Eksempelstørrelse for slider
    speed_display.setFixedSize(100, 50)  # Eksempelstørrelse for display

    return speed_label, speed_slider, speed_display
