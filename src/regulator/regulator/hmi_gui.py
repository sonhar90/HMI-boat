# hmi_gui.py
from PyQt5.QtWidgets import (
    QMainWindow,
    QApplication,
    QGridLayout,
    QWidget,
    QLabel,
    QVBoxLayout,
    QPushButton,
    QDial,
    QLCDNumber,
    QSlider,
    QSizePolicy,
    QHBoxLayout,
    QFrame,
    QButtonGroup,
)
from PyQt5.QtCore import Qt, QTimer
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import rclpy
from ngc_interfaces.msg import Eta, Nu, ButtonControl, ThrusterSignals
from ngc_interfaces.msg import Eta, Nu, ButtonControl, SystemMode
import sys
from ngc_utils.qos_profiles import default_qos_profile
import ngc_utils.math_utils as mu
import numpy as np
from ament_index_python.packages import get_package_share_directory
from regulator.hmi_compass import CompassWidget
from regulator.hmi_thruster import ThrusterBar
import subprocess
import os
import time



class HMIWindow(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()

        # Lagre ROS-noden som en klasseattributt
        self.node = ros_node

        # GUI-oppsett
        self.setWindowTitle("Ship Control HMI")
        self.setGeometry(0, 0, 1850, 1000)

        # Sentral widget og layout med mørk bakgrunn
        self.central_widget = QWidget()
        self.central_widget.setStyleSheet("background-color: #1E1E1E;")
        self.setCentralWidget(self.central_widget)
        self.grid_layout = QGridLayout(self.central_widget)

        # Set stretch factors for columns (0 to 7)
        for i in range(8):
            self.grid_layout.setColumnStretch(i, 1)

        # Set stretch factors for rows (0 to 5)
        for i in range(6):
            self.grid_layout.setRowStretch(i, 1)

        # Initialize lists to store data for graphs
        self.eta_setpoints_HMI = []
        self.eta_sim_values = []
        self.nu_setpoints_HMI = []
        self.nu_sim_values = []
        self.thruster_1_setpoints = []
        self.thruster_1_feedback = []

        # Heading og speed control
        heading_label, self.heading_dial, self.heading_display = (
            self.add_heading_control()
        )
        self.heading_dial.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.heading_display.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        # self.grid_layout.addWidget(heading_label, 3, 4)
        self.grid_layout.addWidget(self.heading_display, 4, 4)
        self.grid_layout.addWidget(self.heading_dial, 5, 4)

        speed_label, self.speed_slider, self.speed_display = self.add_speed_control()
        self.speed_slider.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.speed_display.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        # self.grid_layout.addWidget(speed_label, 3, 3)
        self.grid_layout.addWidget(self.speed_display, 4, 3)
        self.grid_layout.addWidget(self.speed_slider, 5, 3)

        # Tre grafer for kontinuerlig data
        self.graph_canvas_1 = self.add_plotjuggler_graph(
            "Eta Setpoint vs Sim", "Heading (degrees)"
        )
        self.graph_canvas_2 = self.add_plotjuggler_graph(
            "Nu Setpoint vs Sim", "Speed (m/s)"
        )
        self.graph_canvas_3 = self.add_plotjuggler_graph(
            "Thruster 1 Setpoint vs Sim", "Thruster Output"
        )

        self.graph_canvas_1.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.graph_canvas_2.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.graph_canvas_3.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.grid_layout.addWidget(self.graph_canvas_1, 0, 6, 2, 2)
        self.grid_layout.addWidget(self.graph_canvas_2, 2, 6, 2, 2)
        self.grid_layout.addWidget(self.graph_canvas_3, 4, 6, 2, 2)

        # Thruster-knapper
        thruster_buttons_layout = self.add_thruster_buttons()
        self.grid_layout.addLayout(thruster_buttons_layout, 4, 5, 2, 1)

        # Create ThrusterBar widgets
        max_forward_value = 30.0  # Adjust according to your system
        max_reverse_value = -30.0  # Negative value

        self.thruster1_bar = ThrusterBar(
            thruster_id=1,
            max_forward_value=max_forward_value,
            max_reverse_value=max_reverse_value,
        )
        self.thruster2_bar = ThrusterBar(
            thruster_id=2,
            max_forward_value=max_forward_value,
            max_reverse_value=max_reverse_value,
        )

        # Set size policies for the ThrusterBar widgets
        self.thruster1_bar.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.thruster2_bar.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # Create a horizontal layout for the thruster bars
        thruster_layout = QHBoxLayout()
        thruster_layout.addWidget(self.thruster1_bar)
        thruster_layout.addWidget(self.thruster2_bar)

        # Adjust stretch factors within the thruster_layout
        thruster_layout.setStretch(0, 1)  # For thruster1_bar
        thruster_layout.setStretch(1, 1)  # For thruster2_bar

        # Add the thruster_layout to the grid layout at row 5, column 0
        self.grid_layout.addLayout(thruster_layout, 4, 0, 2, 1)  # Adjusted column span

        # Compass widget
        self.compass_widget = CompassWidget()
        self.grid_layout.addWidget(self.compass_widget, 4, 1, 2, 2)

        # Timer for kontinuerlig oppdatering
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_graphs)
        self.timer.start(100)

        # ROS2 abonnenter og publisenter
        # Initialize setpoints
        self.heading_setpoint = 0.5  # Initial heading setpoint
        self.surge_setpoint = 0.0  # Initial surge speed setpoint in knots

        # Subscribers to eta_sim and nu_sim
        self.node.create_subscription(
            Eta, "eta_hat", self.update_eta_feedback, default_qos_profile
        )
        self.node.create_subscription(
            Nu, "nu_hat", self.update_nu_feedback, default_qos_profile
        )

        # Publishers for eta_setpoint and nu_setpoint
        self.eta_publisher_HMI = self.node.create_publisher(Eta, 'eta_setpoint_HMI', default_qos_profile)
        self.nu_publisher_HMI = self.node.create_publisher(Nu, 'nu_setpoint_HMI', default_qos_profile)
        
        # Publiser ButtonControl meldinger for knappetrykk
        self.button_publisher = self.node.create_publisher(ButtonControl, 'button_control', default_qos_profile)
        self.systemmode_publisher = self.node.create_publisher(SystemMode, 'system_mode', default_qos_profile)   
        

        # Subscribers for thruster setpoints
        self.node.create_subscription(
            ThrusterSignals,
            "thruster_1_setpoints",
            self.update_thruster1_setpoint,
            default_qos_profile,
        )
        self.node.create_subscription(
            ThrusterSignals,
            "thruster_2_setpoints",
            self.update_thruster2_setpoint,
            default_qos_profile,
        )

        # Subscribers for thruster simulated values
        self.node.create_subscription(
            ThrusterSignals,
            "thruster_1_feedback",
            self.update_thruster1_feedback,
            default_qos_profile,
        )
        self.node.create_subscription(
            ThrusterSignals,
            "thruster_2_feedback",
            self.update_thruster2_feedback,
            default_qos_profile,
        )

        # Initialize variables to store the setpoint and feedback values
        self.thruster1_setpoint = 0.0
        self.thruster1_feedback = 0.0
        self.thruster2_setpoint = 0.0
        self.thruster2_feedback = 0.0

        # Publishing timer with continuous setpoint update
        self.publish_timer = QTimer()
        self.publish_timer.timeout.connect(self.publish_setpoints_continuous)
        self.publish_timer.start(100)  # Publishing every 100 ms

    def add_heading_control(self):
        heading_label = QLabel("Heading Control")
        heading_label.setAlignment(Qt.AlignCenter)
        heading_label.setStyleSheet("color: white;")

        self.heading_dial = QDial()
        self.heading_dial.setMinimum(0)
        self.heading_dial.setMaximum(360)
        self.heading_dial.setWrapping(True)
        self.heading_dial.setValue(
            180
        )  # Sett startverdien til 180 for å vise nord som 0

        self.heading_display = QLCDNumber()
        self.heading_display.setSegmentStyle(QLCDNumber.Flat)
        self.heading_display.setDigitCount(6)
        self.heading_display.setStyleSheet("color: #7DF9FF;")
        # Initial display value remapped to north (0)
        self.heading_display.display(self.remap_dial_value(self.heading_dial.value()))

        # Connect valueChanged signal to display the remapped dial value and update setpoint
        self.heading_dial.valueChanged.connect(
            lambda value: (
                self.heading_display.display(self.remap_dial_value(value)),
                self.update_heading_setpoint(self.remap_dial_value(value)),
            )
        )

        return heading_label, self.heading_dial, self.heading_display

    def remap_dial_value(self, value):
        remapped_value = (value - 180) % 360
        return remapped_value

    def add_speed_control(self):
        speed_label = QLabel("Speed Control")
        speed_label.setAlignment(Qt.AlignCenter)
        speed_label.setStyleSheet("color: white;")

        self.speed_slider = QSlider(Qt.Vertical)
        self.speed_slider.setMinimum(-6)
        self.speed_slider.setMaximum(6)
        self.speed_slider.setValue(0)
        self.speed_display = QLCDNumber()
        self.speed_display.setSegmentStyle(QLCDNumber.Flat)
        self.speed_display.setDigitCount(2)
        self.speed_display.setStyleSheet("color: lightgrey;")
        self.speed_display.display(self.speed_slider.value())

        self.speed_slider.valueChanged.connect(
            lambda value: (
                self.speed_display.display(value),
                self.update_surge_setpoint(value),
            )
        )
        return speed_label, self.speed_slider, self.speed_display

    def add_plotjuggler_graph(self, title, ylabel):
        fig = Figure(facecolor="#2E2E2E")
        canvas = FigureCanvas(fig)
        ax = fig.add_subplot(111)
        ax.set_facecolor("#333333")
        ax.grid(True, color="gray")
        ax.set_title(title, color="white")
        ax.set_ylabel(ylabel, color="white")
        ax.tick_params(axis="x", colors="white")
        ax.tick_params(axis="y", colors="white")
        return canvas

    def publish_button_mode(self, mode):
        """Publiser en ButtonControl melding med gitt modus"""
        button_msg = ButtonControl()
        button_msg.mode = mode
        self.button_publisher.publish(button_msg)
        self.node.get_logger().info(f"Knapp trykket: Publiserer melding {button_msg} til button_control.")

        msg_sys = SystemMode()

        if mode == 0:
            
            msg_sys.standby_mode = True
            msg_sys.auto_mode    = False
        else:
            msg_sys.standby_mode = False
            msg_sys.auto_mode    = True

        self.systemmode_publisher.publish(msg_sys)  
        

    def update_eta_feedback(self, msg):
        # Sjekk om psi-verdi er gyldig før konvertering
        if not np.isfinite(msg.psi):
            self.node.get_logger().error(f"Invalid psi value received: {msg.psi}")
            return

        # Konverter psi-verdi fra radianer til grader
        psi_in_degrees = np.degrees(msg.psi)
        self.eta_sim_values.append(psi_in_degrees)

        # Update CompassWidget's feedback
        self.compass_widget.set_feedback(psi_in_degrees)
        self.compass_widget.set_setpoint(self.heading_setpoint)

        # Begrens lagringslengde til 100 elementer
        if len(self.eta_sim_values) > 100:
            self.eta_sim_values.pop(0)

        # Logging av mottatt psi-verdi
        # self.node.get_logger().info(f"Received eta_sim psi (degrees): {psi_in_degrees}")

    def update_nu_feedback(self, msg):
        self.nu_sim_values.append(msg.u)
        if len(self.nu_sim_values) > 100:
            self.nu_sim_values.pop(0)

    def update_heading_setpoint(self, value):
        self.heading_setpoint = value

    def update_surge_setpoint(self, value):
        self.surge_setpoint = value

    def publish_setpoints_continuous(self):
        # Sjekk at heading_setpoint og surge_setpoint er gyldige tall
        if not np.isfinite(self.heading_setpoint) or not np.isfinite(
            self.surge_setpoint
        ):
            self.node.get_logger().error("Invalid setpoint values detected.")
            return

        eta_msg = Eta()

        # Map heading_setpoint til [-pi, pi] og sjekk gyldighet
        mapped_psi = mu.mapToPiPi(np.deg2rad(self.heading_setpoint))
        if not np.isfinite(mapped_psi):
            self.node.get_logger().error(f"Mapped psi is invalid: {mapped_psi}")
            return

        eta_msg.psi = float(mapped_psi)
        self.eta_publisher_HMI.publish(eta_msg)
        self.eta_setpoints_HMI.append(self.heading_setpoint)
        if len(self.eta_setpoints_HMI) > 100:
            self.eta_setpoints_HMI.pop(0)

        # Logging av publisert eta_setpoint
        # self.node.get_logger().info(f"Published eta_setpoint_HMI (psi in degrees): {self.heading_setpoint}")

        nu_msg = Nu()
        nu_msg.u = float(self.surge_setpoint) * 0.514444
        self.nu_publisher_HMI.publish(nu_msg)
        self.nu_setpoints_HMI.append(nu_msg.u)
        if len(self.nu_setpoints_HMI) > 100:
            self.nu_setpoints_HMI.pop(0)

        # Logging av publisert surge_setpoint
        # self.node.get_logger().info(f"Published nu_setpoint_HMI (u in m/s): {nu_msg.u}")

    def update_graphs(self):
        if self.eta_setpoints_HMI and self.eta_sim_values:
            ax1 = self.graph_canvas_1.figure.get_axes()[0]
            ax1.clear()
            ax1.plot(
                range(len(self.eta_setpoints_HMI)),
                self.eta_setpoints_HMI,
                label="Setpoint",
                color="#39FF14",
            )
            ax1.plot(
                range(len(self.eta_sim_values)),
                self.eta_sim_values,
                label="Simulated",
                color="#FF073A",
            )
            ax1.legend(loc="upper right")
            ax1.set_title("Eta Setpoint vs Sim", color="white")
            self.graph_canvas_1.draw()

        if self.nu_setpoints_HMI and self.nu_sim_values:
            ax2 = self.graph_canvas_2.figure.get_axes()[0]
            ax2.clear()
            ax2.plot(
                range(len(self.nu_setpoints_HMI)),
                self.nu_setpoints_HMI,
                label="Setpoint",
                color="#39FF14",
            )
            ax2.plot(
                range(len(self.nu_sim_values)),
                self.nu_sim_values,
                label="Simulated",
                color="#FF073A",
            )
            ax2.legend(loc="upper right")
            ax2.set_title("Nu Setpoint vs Sim", color="white")
            self.graph_canvas_2.draw()

        if self.thruster_1_setpoints and self.thruster_1_feedback:
            ax3 = self.graph_canvas_3.figure.get_axes()[0]
            ax3.clear()
            ax3.plot(
                range(len(self.thruster_1_setpoints)),
                self.thruster_1_setpoints,
                label="Setpoint",
                color="#39FF14",
            )
            ax3.plot(
                range(len(self.thruster_1_feedback)),
                self.thruster_1_feedback,
                label="Feedback",
                color="#FF073A",
            )
            ax3.legend(loc="upper right")
            ax3.set_title("Thruster 1 Setpoint vs Sim", color="white")
            ax3.set_ylabel("Thruster Output", color="white")
            ax3.set_xlabel("Time Steps", color="white")
            ax3.tick_params(axis="x", colors="white")
            ax3.tick_params(axis="y", colors="white")
            self.graph_canvas_3.draw()

    def add_thruster_buttons(self):
        button_layout = QVBoxLayout()
        buttons = {"Standby": 0, "Position": 1, "Sail": 2, "Track": 3}
        self.button_group = QButtonGroup()
        self.button_group.setExclusive(True)
        for label, mode in buttons.items():
            button = QPushButton(label)
            button.setCheckable(True)
            button.clicked.connect(lambda _, m=mode: self.publish_button_mode(m))
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
        exit_button.clicked.connect(self.close)
        button_layout.addWidget(exit_button)

        return button_layout

    def update_thruster1_setpoint(self, msg):
        self.thruster1_setpoint = msg.rps
        self.thruster_1_setpoints.append(self.thruster1_setpoint)
        if len(self.thruster_1_setpoints) > 100:
            self.thruster_1_setpoints.pop(0)
        self.thruster1_bar.set_values(
            setpoint=self.thruster1_setpoint, current_value=self.thruster1_feedback
        )

    def update_thruster2_setpoint(self, msg):
        self.thruster2_setpoint = msg.rps
        self.thruster2_bar.set_values(
            setpoint=self.thruster2_setpoint, current_value=self.thruster2_feedback
        )

    def update_thruster1_feedback(self, msg):
        self.thruster1_feedback = msg.rps
        self.thruster_1_feedback.append(self.thruster1_feedback)
        if len(self.thruster_1_feedback) > 100:
            self.thruster_1_feedback.pop(0)
        self.thruster1_bar.set_values(
            setpoint=self.thruster1_setpoint, current_value=self.thruster1_feedback
        )

    def update_thruster2_feedback(self, msg):
        self.thruster2_feedback = msg.rps
        self.thruster2_bar.set_values(
            setpoint=self.thruster2_setpoint, current_value=self.thruster2_feedback
        )


def keep_opencpn_on_top():
    # Use wmctrl to ensure OpenCPN stays on top
    subprocess.run(["wmctrl", "-r", "opencpn", "-b", "add,above"])


def main():
    rclpy.init()
    node = rclpy.create_node("hmi_gui")
    app = QApplication(sys.argv)
    hmi_window = HMIWindow(node)
    hmi_window.show()

    # Launch OpenCPN
    opencpn_process = subprocess.Popen(["opencpn"])

    # Allow time for OpenCPN to start
    time.sleep(3)

    # Use wmctrl to ensure OpenCPN overlays the HMI window initially
    hmi_window_title = "Ship Control HMI"
    subprocess.run(["wmctrl", "-r", hmi_window_title, "-b", "remove,above"])
    subprocess.run(["wmctrl", "-r", "opencpn", "-b", "add,above"])

    # Timer to keep OpenCPN on top
    opencpn_timer = QTimer()
    opencpn_timer.timeout.connect(keep_opencpn_on_top)
    opencpn_timer.start(1000)  # Check every 1 second

    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.1))
    ros_timer.start(10)

    try:
        sys.exit(app.exec_())
    finally:
        opencpn_process.terminate()  # Ensure OpenCPN is closed when the app exits
        rclpy.shutdown()


if __name__ == "__main__":
    main()
