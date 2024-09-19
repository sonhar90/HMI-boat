import sys
import os
import signal
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QFormLayout, QHBoxLayout, QPushButton, QLabel, QSpacerItem, QSizePolicy, QLCDNumber
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont
import rclpy
from rclpy.node import Node
from ngc_interfaces.msg import ThrusterSignals
from ngc_utils.thruster_objects_loader import load_thrusters_from_yaml
import yaml
from ament_index_python.packages import get_package_share_directory
from ngc_utils.qos_profiles import default_qos_profile
from ngc_hmi.custom_slider_widget import CustomSliderWidget


class HMI(Node):
    def __init__(self):
        super().__init__('ngc_hmi')
        self.declare_parameter('yaml_package_name', 'ngc_bringup')
        self.declare_parameter('propulsion_config_file', 'config/propulsion_config.yaml')
        self.declare_parameter('simulation_config_file', 'config/simulator_config.yaml')
        self.declare_parameter('vessel_config_file', 'config/vessel_config.yaml')

        # Dynamically find the config files in the NGC bringup package
        yaml_package_name = self.get_parameter('yaml_package_name').get_parameter_value().string_value
        yaml_package_path = get_package_share_directory(yaml_package_name)
        simulation_config_path = os.path.join(yaml_package_path, self.get_parameter('simulation_config_file').get_parameter_value().string_value)
        vessel_config_path = os.path.join(yaml_package_path, self.get_parameter('vessel_config_file').get_parameter_value().string_value)

        # Load configurations and initialize VesselModel
        self.simulation_config = self.load_yaml_file(simulation_config_path)
        self.vessel_config = self.load_yaml_file(vessel_config_path)

        # Create thruster objects from yaml file
        self.thrusters = load_thrusters_from_yaml(self.get_logger(), yaml_package_name, self.get_parameter('propulsion_config_file').get_parameter_value().string_value, self.simulation_config['physical_parameters'])

        self.create_thruster_publishers()
        self.create_feedback_subscribers()
        self.init_ui()

    def init_ui(self):
        self.app = QApplication(sys.argv)
        self.window = QWidget()
        self.window.setWindowTitle(self.vessel_config['vessel']['name'] + " thruster controls")
        self.layout = QVBoxLayout()

        self.sliders = {}
        self.buttons = {}
        self.lcds = {}

        enable_all_button = QPushButton('Enable All Thrusters')
        enable_all_button.setCheckable(True)
        enable_all_button.clicked.connect(self.toggle_all_thrusters)
        self.layout.addWidget(enable_all_button)
        self.enable_all_button = enable_all_button

        for thruster in self.thrusters:
            thruster_id = thruster.id

            thruster_layout = QVBoxLayout()

            enable_button = QPushButton(f'Enable Thruster {thruster_id}')
            enable_button.setCheckable(True)
            enable_button.clicked.connect(lambda state, thr=thruster: self.toggle_thruster(state, thr))
            thruster_layout.addWidget(enable_button)
            self.buttons[thruster_id] = enable_button

            form_layout = QFormLayout()

            self.add_slider_layout(thruster, thruster.propeller.min_rpm, thruster.propeller.max_rpm, 'RPM', self.update_thruster_rpm, form_layout)
            if hasattr(thruster, 'rudder_angle_rad'):
                if thruster.has_rudder:
                    self.add_slider_layout(thruster, -thruster.max_rudder_angle_deg, thruster.max_rudder_angle_deg, 'Rudder', self.update_thruster_azimuth, form_layout)
            if hasattr(thruster, 'azimuth_angle_rad'):
                self.add_slider_layout(thruster, -180, 180, 'Azimuth', self.update_thruster_azimuth, form_layout)
            if hasattr(thruster.propeller, 'pitch'):
                self.add_slider_layout(thruster, thruster.propeller.min_pitch, thruster.propeller.max_pitch, 'Pitch', self.update_thruster_pitch, form_layout)

            thruster_layout.addLayout(form_layout)
            self.layout.addLayout(thruster_layout)
            self.layout.addItem(QSpacerItem(20, 20, QSizePolicy.Minimum, QSizePolicy.Expanding))

        self.window.setLayout(self.layout)
        self.window.show()

        # Timer for handling ROS2 callbacks
        self.timer = QTimer()
        self.timer.timeout.connect(self.ros_spin)
        self.timer.start(100)  # Spin ROS every 100ms

    def ros_spin(self):
        rclpy.spin_once(self, timeout_sec=0.1)

    def add_slider_layout(self, thruster, min_val, max_val, label_text, update_method, layout):
        scale_factor = 1000  # Scale factor to convert float to int

        slider_widget = CustomSliderWidget(min_val, max_val, scale_factor, label_text, update_method, thruster)
        slider_widget.setFixedHeight(50)  # Ensure consistent slider height
        self.sliders[f'{thruster.id}_{label_text.lower()}'] = slider_widget

        lcd_display = QLCDNumber()
        lcd_display.setSegmentStyle(QLCDNumber.Flat)
        lcd_display.setStyleSheet("background-color: #DFDFDF; color: #1E90FF; border: 1px solid grey;")
        lcd_display.setDigitCount(8)

        self.lcds[f'{thruster.id}_{label_text.lower()}'] = lcd_display

        label = QLabel(label_text)
        label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        label.setFixedHeight(50)  # Ensure consistent label height

        h_layout = QHBoxLayout()
        h_layout.addWidget(label)
        h_layout.addWidget(slider_widget)
        h_layout.addWidget(lcd_display)

        layout.addRow(h_layout)

    def create_thruster_publishers(self):
        self.thruster_publishers = {}
        for thruster in self.thrusters:
            topic_name = f'thruster_{thruster.id}_setpoints'
            self.thruster_publishers[thruster.id] = self.create_publisher(ThrusterSignals, topic_name, default_qos_profile)
            self.get_logger().info(f'Created publisher for {topic_name}')

    def create_feedback_subscribers(self):
        self.feedback_subscribers = {}
        for thruster in self.thrusters:
            topic_name = f'thruster_{thruster.id}_feedback'
            self.feedback_subscribers[thruster.id] = self.create_subscription(
                ThrusterSignals,
                topic_name,
                lambda msg, thr=thruster: self.update_feedback(msg, thr),
                default_qos_profile
            )
            self.get_logger().info(f'Created subscriber for {topic_name}')

    def update_feedback(self, msg, thruster):
        thruster_id = thruster.id

        slider_keys = {
            'rps': 'rpm',
            'azimuth_deg': 'rudder',
            'pitch': 'pitch'
        }

        for attr, key in slider_keys.items():
            slider_key = f'{thruster_id}_{key}'
            if slider_key in self.sliders:
                value = getattr(msg, attr)
                if key == 'rpm':
                    value *= 60  # Convert RPS to RPM
                    value = round(value, 0)
                elif key == 'pitch':
                    value = round(value, 2)
                else:
                    value = round(value, 1)
                self.sliders[slider_key].set_feedback_value(value)
                self.lcds[slider_key].display(value)

        # Update button color based on active signal
        if thruster_id in self.buttons:
            button = self.buttons[thruster_id]
            if msg.active:
                button.setStyleSheet("background-color: #1E90FF")  # Custom blue color
            else:
                button.setStyleSheet("")

        # Check if all thrusters are active to update the "Enable All Thrusters" button
        all_active = all(self.buttons[thr.id].isChecked() for thr in self.thrusters)
        if all_active:
            self.enable_all_button.setStyleSheet("background-color: #1E90FF")
        else:
            self.enable_all_button.setStyleSheet("")

    def toggle_thruster(self, state, thruster):
        thruster.setpoints.active = state
        msg = thruster.setpoints
        self.thruster_publishers[thruster.id].publish(msg)

    def toggle_all_thrusters(self, state):
        for thruster in self.thrusters:
            self.buttons[thruster.id].setChecked(state)
            self.toggle_thruster(state, thruster)

    def update_thruster_rpm(self, value, thruster):
        thruster.setpoints.rps = value / 60  # Convert RPM to RPS
        msg = thruster.setpoints
        self.thruster_publishers[thruster.id].publish(msg)

    def update_thruster_azimuth(self, value, thruster):
        thruster.setpoints.azimuth_deg = value
        msg = thruster.setpoints
        self.thruster_publishers[thruster.id].publish(msg)

    def update_thruster_pitch(self, value, thruster):
        thruster.setpoints.pitch = value
        msg = thruster.setpoints
        self.thruster_publishers[thruster.id].publish(msg)

    def load_yaml_file(self, file_path):
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)


def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)

    # Create the HMI node
    hmi = HMI()

    # Set up a QTimer to spin the ROS2 node and handle callbacks
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(hmi, timeout_sec=0.1))
    timer.start(100)  # Call every 100 ms

    # Handle signal for graceful shutdown
    def signal_handler(sig, frame):
        print("SIGINT received, shutting down...")
        hmi.destroy_node()
        rclpy.shutdown()
        QApplication.quit()

    signal.signal(signal.SIGINT, signal_handler)

    # Start the PyQt5 application event loop
    sys.exit(hmi.app.exec_())


if __name__ == '__main__':
    main()
