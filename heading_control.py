import math
from PyQt5.QtWidgets import QDial, QLCDNumber, QLabel, QSpacerItem, QSizePolicy, QVBoxLayout, QPushButton
from PyQt5.QtCore import Qt

def add_heading_control(self):
    # Create a layout for the heading control
    wheel_layout = QVBoxLayout()

    heading_label = QPushButton('Heading Control')
    heading_label.setEnabled(False)  # Disable the button so it behaves like a label
    #heading_button.setEnabled(False)  # Disable the button so it behaves like a label

    #heading_label.setEnabled(True)  # Disable the button so it behaves like a label

    # Add label for Heading Control
    #heading_label = QPushButton('Heading (°)')
    #heading_label.setAlignment(Qt.AlignCenter)
    wheel_layout.addWidget(heading_label)

    # Create and configure the dial
    self.heading_dial = QDial()
    self.heading_dial.setMinimum(0)
    self.heading_dial.setMaximum(360)
    self.heading_dial.setWrapping(True)  # Enable wrapping to allow continuous rotation
    self.heading_dial.setValue(180)  # Start at 180 degrees (north)
    self.heading_dial.setNotchesVisible(True)
    self.heading_dial.valueChanged.connect(self.update_heading)  # Use the correct method
    wheel_layout.addWidget(self.heading_dial)

    # Create LCD display for the heading value
    self.heading_display = QLCDNumber()
    self.heading_display.setSegmentStyle(QLCDNumber.Flat)
    self.heading_display.setDigitCount(6)
    self.heading_display.display(self.remap_dial_value(self.heading_dial.value()))
    wheel_layout.addWidget(self.heading_display)

    # Update the LCD display when the dial value changes
    self.heading_dial.valueChanged.connect(lambda value: self.heading_display.display(self.remap_dial_value(value)))

    # Add a spacer for layout consistency
    wheel_layout.addItem(QSpacerItem(20, 20, QSizePolicy.Minimum, QSizePolicy.Expanding))

    return heading_label, self.heading_dial, self.heading_display




