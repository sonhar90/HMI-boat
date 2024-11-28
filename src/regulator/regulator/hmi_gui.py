from PyQt5.QtWidgets import (
    QMainWindow,
    QApplication,
    QGridLayout,
    QWidget,
    QSizePolicy,
    QHBoxLayout,
    QVBoxLayout,
    QButtonGroup,
    QPushButton,
)
from PyQt5.QtCore import Qt, QTimer
import numpy as np
from regulator.hmi_compass import CompassWidget
from regulator.hmi_thruster import ThrusterBar
from regulator.hmi_speed import SpeedBar
from regulator.hmi_controls import HeadingControlWidget, SpeedControlWidget
from regulator.hmi_ros import ROSNode
from regulator.hmi_graph import GraphManager
import sys
import subprocess
import time
import rclpy
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas


class HMIWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # Initialize ROS Node
        self.ros_node = ROSNode()

        # GUI setup
        self.setWindowTitle("Ship Control HMI")
        self.setGeometry(0, 0, 1850, 1000)

        # Central widget and layout with dark background
        self.central_widget = QWidget()
        self.central_widget.setStyleSheet("background-color: #1E1E1E;")
        self.setCentralWidget(self.central_widget)
        self.grid_layout = QGridLayout(self.central_widget)

        # Set stretch factors for columns (0 to 6)
        for i in range(7):
            self.grid_layout.setColumnStretch(i, 1)

        # Set stretch factors for rows (0 to 5)
        for i in range(6):
            self.grid_layout.setRowStretch(i, 1)

        # Heading control widget
        self.heading_control = HeadingControlWidget()
        self.heading_control.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.grid_layout.addWidget(self.heading_control, 4, 4, 2, 1)
        self.heading_control.headingChanged.connect(self.update_heading_setpoint)

        # Speed control widget
        self.speed_control = SpeedControlWidget()
        self.speed_control.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.grid_layout.addWidget(self.speed_control, 4, 2, 2, 1)
        self.speed_control.speedChanged.connect(self.update_surge_setpoint)

        # Graphs
        self.graph_manager = GraphManager()
        self.graph_canvas_1 = FigureCanvas(self.graph_manager.fig1)
        self.graph_canvas_2 = FigureCanvas(self.graph_manager.fig2)
        self.graph_canvas_3 = FigureCanvas(self.graph_manager.fig3)
        self.graph_canvas_4 = FigureCanvas(self.graph_manager.fig4)

        self.graph_canvas_1.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.graph_canvas_2.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.graph_canvas_3.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.graph_canvas_4.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.grid_layout.addWidget(self.graph_canvas_1, 0, 5, 2, 2)
        self.grid_layout.addWidget(self.graph_canvas_2, 2, 5, 2, 2)
        self.grid_layout.addWidget(self.graph_canvas_3, 0, 0, 2, 2)
        self.grid_layout.addWidget(self.graph_canvas_4, 2, 0, 2, 2)

        # Buttons
        thruster_buttons_layout = self.add_thruster_buttons()
        self.grid_layout.addLayout(thruster_buttons_layout, 4, 3, 2, 1)

        # Create ThrusterBar widgets
        max_thrust_forward_value = 30.0  # Adjust according to your system
        max_thrust_reverse_value = -30.0  # Negative value

        self.thruster1_bar = ThrusterBar(
            thruster_id=1,
            max_forward_value=max_thrust_forward_value,
            max_reverse_value=max_thrust_reverse_value,
        )
        self.thruster2_bar = ThrusterBar(
            thruster_id=2,
            max_forward_value=max_thrust_forward_value,
            max_reverse_value=max_thrust_reverse_value,
        )
        self.thruster1_bar.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.thruster2_bar.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # Create SpeedBar widget
        max_speed_value = 3.5  # Adjust according to your system
        self.speed_bar = SpeedBar(
            max_forward_value=max_speed_value, max_reverse_value=-max_speed_value
        )
        self.speed_bar.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        thrust_and_speed_layout = QHBoxLayout()
        thrust_and_speed_layout.addWidget(self.thruster1_bar)
        thrust_and_speed_layout.addWidget(self.speed_bar)
        thrust_and_speed_layout.addWidget(self.thruster2_bar)
        thrust_and_speed_layout.setStretch(0, 3)  # For thruster1_bar (60% of speed_bar)
        thrust_and_speed_layout.setStretch(1, 5)  # For speed_bar
        thrust_and_speed_layout.setStretch(2, 3)  # For thruster2_bar (60% of speed_bar)
        thrust_and_speed_layout.setContentsMargins(10, 10, 10, 10)
        self.grid_layout.addLayout(thrust_and_speed_layout, 4, 0, 2, 2)

        # Compass widget
        self.compass_widget = CompassWidget()
        self.compass_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.compass_layout = QVBoxLayout()
        self.compass_layout.addWidget(self.compass_widget)
        self.compass_layout.setContentsMargins(10, 10, 10, 10)
        self.grid_layout.addLayout(self.compass_layout, 4, 5, 2, 2)

        # Timer for continuous updates
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_graphs)
        self.timer.start(10)

        # Connect ROSNode callbacks
        self.ros_node.on_eta_received = self.update_eta_feedback
        self.ros_node.on_nu_received = self.update_nu_feedback
        self.ros_node.on_thruster1_setpoint_received = self.update_thruster1_setpoint
        self.ros_node.on_thruster2_setpoint_received = self.update_thruster2_setpoint
        self.ros_node.on_thruster1_feedback_received = self.update_thruster1_feedback
        self.ros_node.on_thruster2_feedback_received = self.update_thruster2_feedback

        # Publishing timer with continuous setpoint update
        self.publish_timer = QTimer()
        self.publish_timer.timeout.connect(self.publish_setpoints_continuous)
        self.publish_timer.start(10)  # Publishing every 10 ms

    # Methods for handling setpoints
    def update_heading_setpoint(self, value):
        self.ros_node.heading_setpoint = value

    def update_surge_setpoint(self, value):
        self.ros_node.surge_setpoint = value

    def publish_setpoints_continuous(self):
        # Convert heading to radians and map to [-pi, pi]
        psi_radians = np.deg2rad(self.ros_node.heading_setpoint)
        psi_mapped = ((psi_radians + np.pi) % (2 * np.pi)) - np.pi

        self.ros_node.publish_eta_setpoint(psi_mapped)
        self.ros_node.publish_nu_setpoint(
            self.ros_node.surge_setpoint * 0.514444
        )  # Convert knots to m/s

        current_time = time.time() - self.graph_manager.start_time
        self.graph_manager.eta_setpoints_HMI.append(
            (current_time, self.ros_node.heading_setpoint)
        )
        self.graph_manager.nu_setpoints_HMI.append(
            (current_time, self.ros_node.surge_setpoint * 0.514444)
        )

        # Limit storage length
        self.graph_manager.eta_setpoints_HMI = self.graph_manager.eta_setpoints_HMI[
            -100:
        ]
        self.graph_manager.nu_setpoints_HMI = self.graph_manager.nu_setpoints_HMI[-100:]

        self.speed_bar.set_setpoint(self.ros_node.surge_setpoint)

    # Methods for updating feedback
    def update_eta_feedback(self, msg):
        current_time = time.time() - self.graph_manager.start_time
        psi_in_degrees = np.degrees(msg.psi)
        self.graph_manager.eta_hat_values.append((current_time, psi_in_degrees))
        self.graph_manager.eta_hat_values = self.graph_manager.eta_hat_values[-100:]

        # Update CompassWidget
        print(f"Calling set_feedback with value: {psi_in_degrees}")
        self.compass_widget.set_feedback(psi_in_degrees)
        self.compass_widget.set_setpoint(self.ros_node.heading_setpoint)

    def update_nu_feedback(self, msg):
        current_time = time.time() - self.graph_manager.start_time
        self.graph_manager.nu_hat_values.append((current_time, msg.u))
        self.graph_manager.nu_hat_values = self.graph_manager.nu_hat_values[-100:]
        self.speed_bar.set_nu(msg.u / 0.514444)  # Convert m/s to knots

    def update_graphs(self):
        self.graph_manager.update_graphs()

    # Thruster updates
    def update_thruster1_setpoint(self, msg):
        current_time = time.time() - self.graph_manager.start_time
        self.graph_manager.thruster_1_setpoints.append((current_time, msg.rps))
        self.graph_manager.thruster_1_setpoints = (
            self.graph_manager.thruster_1_setpoints[-100:]
        )
        self.thruster1_bar.set_values(
            setpoint=msg.rps,
            current_value=(
                self.graph_manager.thruster_1_feedback[-1][1]
                if self.graph_manager.thruster_1_feedback
                else 0
            ),
        )

    def update_thruster2_setpoint(self, msg):
        current_time = time.time() - self.graph_manager.start_time
        self.graph_manager.thruster_2_setpoints.append((current_time, msg.rps))
        self.graph_manager.thruster_2_setpoints = (
            self.graph_manager.thruster_2_setpoints[-100:]
        )
        self.thruster2_bar.set_values(
            setpoint=msg.rps,
            current_value=(
                self.graph_manager.thruster_2_feedback[-1][1]
                if self.graph_manager.thruster_2_feedback
                else 0
            ),
        )

    def update_thruster1_feedback(self, msg):
        current_time = time.time() - self.graph_manager.start_time
        self.graph_manager.thruster_1_feedback.append((current_time, msg.rps))
        self.graph_manager.thruster_1_feedback = self.graph_manager.thruster_1_feedback[
            -100:
        ]
        self.thruster1_bar.set_values(
            setpoint=(
                self.graph_manager.thruster_1_setpoints[-1][1]
                if self.graph_manager.thruster_1_setpoints
                else 0
            ),
            current_value=msg.rps,
        )

    def update_thruster2_feedback(self, msg):
        current_time = time.time() - self.graph_manager.start_time
        self.graph_manager.thruster_2_feedback.append((current_time, msg.rps))
        self.graph_manager.thruster_2_feedback = self.graph_manager.thruster_2_feedback[
            -100:
        ]
        self.thruster2_bar.set_values(
            setpoint=(
                self.graph_manager.thruster_2_setpoints[-1][1]
                if self.graph_manager.thruster_2_setpoints
                else 0
            ),
            current_value=msg.rps,
        )

    def add_thruster_buttons(self):
        button_layout = QVBoxLayout()
        buttons = {"Standby": 0, "Position": 1, "Sail": 2, "Track": 3}
        self.button_group = QButtonGroup()
        self.button_group.setExclusive(True)
        for label, mode in buttons.items():
            button = QPushButton(label)
            button.setCheckable(True)
            button.clicked.connect(
                lambda _, m=mode: self.ros_node.publish_button_mode(m)
            )
            button.setStyleSheet(
                """
                QPushButton {
                    background-color: #1E90FF;
                    border-radius: 10px;
                    border: 2px solid #ffffff;
                    color: white;
                    padding: 10px;
                    font-size: 14px;
                    font-weight: bold;
                }
                QPushButton:hover {
                    background-color: #00BFFF;
                }
                QPushButton:pressed {
                    background-color: #8CFF33;
                }
                QPushButton:checked {
                    background-color: #8CFF33;
                }
            """
            )
            button_layout.addWidget(button)
            self.button_group.addButton(button)

        exit_button = QPushButton("Exit")
        exit_button.setStyleSheet(
            """
            QPushButton {
            background-color: red;
            color: white;
            border-radius: 10px;
            border: 2px solid #ffffff;
            padding: 10px;
            font-size: 14px;
            font-weight: bold;
            }
            QPushButton:hover {
            background-color: darkred;
            }
            """
        )
        exit_button.clicked.connect(self.close)
        button_layout.addWidget(exit_button)

        return button_layout


def keep_opencpn_on_top():
    # Use wmctrl to ensure OpenCPN stays on top
    subprocess.run(["wmctrl", "-r", "opencpn", "-b", "add,above"])


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    hmi_window = HMIWindow()
    hmi_window.show()

    # Launch OpenCPN
    opencpn_process = subprocess.Popen(["opencpn"])

    # Allow time for OpenCPN to start
    time.sleep(3)

    # Ensure OpenCPN overlays the HMI window
    hmi_window_title = "Ship Control HMI"
    subprocess.run(["wmctrl", "-r", hmi_window_title, "-b", "remove,above"])
    subprocess.run(["wmctrl", "-r", "opencpn", "-b", "add,above"])

    # Timer to keep OpenCPN on top
    opencpn_timer = QTimer()
    opencpn_timer.timeout.connect(keep_opencpn_on_top)
    opencpn_timer.start(1000)  # Check every 1 second

    # Spin the ROS node from HMIWindow's ROSNode instance
    ros_timer = QTimer()
    ros_timer.timeout.connect(
        lambda: rclpy.spin_once(hmi_window.ros_node, timeout_sec=0.1)
    )
    ros_timer.start(10)

    try:
        sys.exit(app.exec_())
    finally:
        opencpn_process.terminate()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
