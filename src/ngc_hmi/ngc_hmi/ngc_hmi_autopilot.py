import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QFormLayout, QHBoxLayout, QPushButton, QLabel, QSpacerItem, QSizePolicy, QLCDNumber, QDial
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont
import rclpy
from rclpy.node import Node
from ngc_interfaces.msg import Eta, Nu
from PyQt5.QtWidgets import QSlider
from ngc_utils.qos_profiles import default_qos_profile

class AutopilotHMI(Node):
    def __init__(self):
        super().__init__('ship_autopilot_hmi')

        # Initialize the previous heading value
        self.previous_heading_value = 0

        # Subscribers to eta_sim and nu_sim
        self.create_subscription(Eta, 'eta_sim', self.update_eta_feedback, default_qos_profile)
        self.create_subscription(Nu, 'nu_sim', self.update_nu_feedback, default_qos_profile)

        # Publishers for eta_setpoint and nu_setpoint
        self.eta_publisher = self.create_publisher(Eta, 'eta_setpoint', default_qos_profile)
        self.nu_publisher = self.create_publisher(Nu, 'nu_setpoint', default_qos_profile)

        # Initialize the UI
        self.init_ui()

    def init_ui(self):
        self.app = QApplication(sys.argv)
        self.window = QWidget()
        self.window.setWindowTitle("Ship Autopilot HMI")
        self.layout = QVBoxLayout()

        # Heading setpoint with a continuous wheel (QDial)
        self.add_wheel_layout('Heading', 0, 360, '°', self.update_heading_setpoint)

        # Surge speed setpoint with finer granularity in knots
        self.add_slider_layout('Surge Speed', 0, 10, 'knots', self.update_surge_setpoint, resolution=0.1)

        self.window.setLayout(self.layout)
        self.window.show()

        # Process ROS 2 callbacks regularly
        self.timer = QTimer()
        self.timer.timeout.connect(self.ros_spin)
        self.timer.start(100)  # 100 ms interval

        sys.exit(self.app.exec_())

    def ros_spin(self):
        rclpy.spin_once(self, timeout_sec=0.1)

    def add_wheel_layout(self, label_text, min_val, max_val, unit, update_method):
        wheel_layout = QVBoxLayout()

        label = QLabel(f'{label_text} ({unit})')
        label.setAlignment(Qt.AlignCenter)
        wheel_layout.addWidget(label)

        dial = QDial()
        dial.setMinimum(min_val)
        dial.setMaximum(max_val)
        dial.setWrapping(True)  # Enable wrapping to allow continuous rotation
        dial.setValue(180)  # Start at 90 degrees so that the indicator points up and maps to 0 degrees
        dial.setNotchesVisible(True)
        dial.valueChanged.connect(lambda value: self.handle_heading_wraparound(self.remap_dial_value(value), update_method))
        wheel_layout.addWidget(dial)

        lcd_display = QLCDNumber()
        lcd_display.setSegmentStyle(QLCDNumber.Flat)
        lcd_display.setDigitCount(6)
        dial.valueChanged.connect(lambda value: lcd_display.display(self.remap_dial_value(value)))
        wheel_layout.addWidget(lcd_display)

        self.layout.addLayout(wheel_layout)
        self.layout.addItem(QSpacerItem(20, 20, QSizePolicy.Minimum, QSizePolicy.Expanding))

    def remap_dial_value(self, value):
        # Remap the dial so that 90 is 0 degrees (top), 180 is 90 degrees (right), 270 is 180 degrees (bottom), and 0 is 270 degrees (left)
        remapped_value = (value - 180) % 360
        return remapped_value

    def add_slider_layout(self, label_text, min_val, max_val, unit, update_method, resolution=1.0):
        slider_layout = QVBoxLayout()

        label = QLabel(f'{label_text} ({unit})')
        label.setAlignment(Qt.AlignCenter)
        slider_layout.addWidget(label)

        slider = QSlider(Qt.Horizontal)
        slider.setMinimum(int(min_val / resolution))
        slider.setMaximum(int(max_val / resolution))
        slider.setValue((min_val + max_val) // 2)
        slider.setTickPosition(QSlider.TicksBelow)
        slider.setTickInterval(int(1 / resolution))
        slider.valueChanged.connect(lambda value: update_method(value * resolution))
        slider_layout.addWidget(slider)

        lcd_display = QLCDNumber()
        lcd_display.setSegmentStyle(QLCDNumber.Flat)
        lcd_display.setDigitCount(6)
        lcd_display.display(slider.value() * resolution)
        slider.valueChanged.connect(lambda value: lcd_display.display(value * resolution))
        slider_layout.addWidget(lcd_display)

        self.layout.addLayout(slider_layout)
        self.layout.addItem(QSpacerItem(20, 20, QSizePolicy.Minimum, QSizePolicy.Expanding))

    def handle_heading_wraparound(self, current_value, update_method):
        
        update_method(current_value)

    def update_eta_feedback(self, msg):
        # Handle the incoming Eta message from eta_sim
        pass  # Visualization of the feedback can be added here

    def update_nu_feedback(self, msg):
        # Handle the incoming Nu message from nu_sim
        pass  # Visualization of the feedback can be added here

    def update_heading_setpoint(self, value):
        eta_msg = Eta()
        eta_msg.psi = float(value)
        self.eta_publisher.publish(eta_msg)
        self.get_logger().info(f'Published heading setpoint: {value}°')

    def update_surge_setpoint(self, value):
        # Convert knots to m/s (1 knot = 0.514444 m/s)
        value_mps = float(value) * 0.514444
        nu_msg = Nu()
        nu_msg.u = value_mps
        self.nu_publisher.publish(nu_msg)
        self.get_logger().info(f'Published surge speed setpoint: {value} knots ({value_mps:.2f} m/s)')

def main(args=None):
    rclpy.init(args=args)
    autopilot_hmi = AutopilotHMI()
    rclpy.spin(autopilot_hmi)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
