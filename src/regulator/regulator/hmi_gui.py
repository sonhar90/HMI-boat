# hmi_gui.py
from PyQt5.QtWidgets import QMainWindow, QApplication, QGridLayout, QWidget, QLabel, QVBoxLayout, QPushButton, QDial, QLCDNumber, QSlider, QSpacerItem, QSizePolicy, QHBoxLayout
from PyQt5.QtCore import Qt, QTimer
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import rclpy
from ngc_interfaces.msg import Eta, Nu, ButtonControl
import sys
import math
from ngc_utils.qos_profiles import default_qos_profile
import ngc_utils.math_utils as mu
import numpy as np


class HMIWindow(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()

        # Lagre ROS-noden som en klasseattributt
        self.node = ros_node

        # GUI-oppsett
        self.setWindowTitle('Ship Control HMI')
        self.setGeometry(100, 100, 1200, 800)
        self.setFixedSize(1200, 800)

        # Sentral widget og layout med mørk bakgrunn
        self.central_widget = QWidget()
        self.central_widget.setStyleSheet("background-color: #1E1E1E;")
        self.setCentralWidget(self.central_widget)
        self.grid_layout = QGridLayout(self.central_widget)

        # Initialize lists to store data for graphs
        self.eta_setpoints_HMI = []
        self.eta_sim_values = []
        self.nu_setpoints_HMI = []
        self.nu_sim_values = []

        # Heading og speed control
        heading_label, self.heading_dial, self.heading_display = self.add_heading_control()
        self.heading_dial.setFixedSize(200, 200)
        self.heading_display.setFixedSize(200, 80)
        self.grid_layout.addWidget(heading_label, 0, 0)
        self.grid_layout.addWidget(self.heading_dial, 1, 0)
        self.grid_layout.addWidget(self.heading_display, 2, 0)

        speed_label, self.speed_slider, self.speed_display = self.add_speed_control()
        self.grid_layout.addWidget(speed_label, 3, 0)
        self.grid_layout.addWidget(self.speed_slider, 4, 0)
        self.grid_layout.addWidget(self.speed_display, 5, 0)

        # Placeholder for gray background
        gray_background = QLabel()
        gray_background.setStyleSheet("background-color: gray;")
        gray_background.setFixedSize(950, 400)
        self.grid_layout.addWidget(gray_background, 2, 1, 4, 3)

        # Tre grafer for kontinuerlig data
        self.graph_canvas_1 = self.add_plotjuggler_graph("Eta Setpoint vs Sim", 'Heading (psi)')
        self.graph_canvas_2 = self.add_plotjuggler_graph("Nu Setpoint vs Sim", 'Speed (u)')
        self.graph_canvas_3 = self.add_plotjuggler_graph("Tau Propulsion", 'Surge (x)')

        self.grid_layout.addWidget(self.graph_canvas_1, 0, 1, 2, 1)
        self.grid_layout.addWidget(self.graph_canvas_2, 0, 2, 2, 1)
        self.grid_layout.addWidget(self.graph_canvas_3, 0, 3, 2, 1)

        # Thruster-knapper
        thruster_layout = self.add_thruster_buttons()
        self.grid_layout.addLayout(thruster_layout, 7, 0, 1, 4)

        # Timer for kontinuerlig oppdatering
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_graphs)
        self.timer.start(100)

        # ROS2 abonnenter og publisenter
        # Initialize setpoints
        self.heading_setpoint = 0.5  # Initial heading setpoint
        self.surge_setpoint = 0.0  # Initial surge speed setpoint in knots

        # Subscribers to eta_sim and nu_sim
        self.node.create_subscription(Eta, 'eta_sim', self.update_eta_feedback, default_qos_profile)
        self.node.create_subscription(Nu, 'nu_sim', self.update_nu_feedback, default_qos_profile)

        # Publishers for eta_setpoint and nu_setpoint
        self.eta_publisher_HMI = self.node.create_publisher(Eta, 'eta_setpoint_HMI', default_qos_profile)
        self.nu_publisher_HMI = self.node.create_publisher(Nu, 'nu_setpoint_HMI', default_qos_profile)
        
        # Publiser ButtonControl meldinger for knappetrykk
        self.button_publisher = self.node.create_publisher(ButtonControl, 'button_control', default_qos_profile)


        # Publishing timer with continuous setpoint update
        self.publish_timer = QTimer()
        self.publish_timer.timeout.connect(self.publish_setpoints_continuous)
        self.publish_timer.start(100)  # Publishing every 100 ms

    def add_heading_control(self):
        heading_label = QLabel('Heading Control')
        heading_label.setAlignment(Qt.AlignCenter)
        heading_label.setStyleSheet("color: white;")

        self.heading_dial = QDial()
        self.heading_dial.setMinimum(0)
        self.heading_dial.setMaximum(360)
        self.heading_dial.setWrapping(True)
        self.heading_dial.setValue(180)  # Sett startverdien til 180 for å vise nord som 0
        
        self.heading_display = QLCDNumber()
        self.heading_display.setSegmentStyle(QLCDNumber.Flat)
        self.heading_display.setDigitCount(6)
        # Initial display value remapped to north (0)
        self.heading_display.display(self.remap_dial_value(self.heading_dial.value()))

        # Connect valueChanged signal to display the remapped dial value and update setpoint
        self.heading_dial.valueChanged.connect(lambda value: (
            self.heading_display.display(self.remap_dial_value(value)), 
            self.update_heading_setpoint(self.remap_dial_value(value))
        ))

        return heading_label, self.heading_dial, self.heading_display

    def remap_dial_value(self, value):
        remapped_value = (value - 180) % 360
        return remapped_value

    def add_speed_control(self):
        speed_label = QLabel('Speed Control')
        speed_label.setAlignment(Qt.AlignCenter)
        speed_label.setStyleSheet("color: white;")
        
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setMinimum(0)
        self.speed_slider.setMaximum(100)
        self.speed_slider.setValue(1)
        self.speed_display = QLCDNumber()
        self.speed_display.setSegmentStyle(QLCDNumber.Flat)
        self.speed_display.setDigitCount(3)
        self.speed_display.display(self.speed_slider.value())
        
        self.speed_slider.valueChanged.connect(lambda value: (self.speed_display.display(value), self.update_surge_setpoint(value)))
        return speed_label, self.speed_slider, self.speed_display

    def add_plotjuggler_graph(self, title, ylabel):
        fig = Figure(facecolor='#2E2E2E')
        canvas = FigureCanvas(fig)
        ax = fig.add_subplot(111)
        ax.set_facecolor('#333333')
        ax.grid(True, color='gray')
        ax.set_title(title, color='white')
        ax.set_ylabel(ylabel, color='white')
        ax.tick_params(axis='x', colors='white')
        ax.tick_params(axis='y', colors='white')
        return canvas
    
    def publish_button_mode(self, mode):
        """Publiser en ButtonControl melding med gitt modus"""
        button_msg = ButtonControl()
        button_msg.mode = mode
        self.button_publisher.publish(button_msg)
        self.node.get_logger().info(f"Knapp trykket: Publiserer melding {button_msg} til button_control.")


    def update_eta_feedback(self, msg):
        # Sjekk om psi-verdi er gyldig før konvertering
        if not np.isfinite(msg.psi):
            self.node.get_logger().error(f"Invalid psi value received: {msg.psi}")
            return
        
        # Konverter psi-verdi fra radianer til grader
        psi_in_degrees = np.degrees(msg.psi)
        self.eta_sim_values.append(psi_in_degrees)
        
        # Begrens lagringslengde til 100 elementer
        if len(self.eta_sim_values) > 100:
            self.eta_sim_values.pop(0)

        # Logging av mottatt psi-verdi
        #self.node.get_logger().info(f"Received eta_sim psi (degrees): {psi_in_degrees}")


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
        if not np.isfinite(self.heading_setpoint) or not np.isfinite(self.surge_setpoint):
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
        #self.node.get_logger().info(f"Published eta_setpoint_HMI (psi in degrees): {self.heading_setpoint}")

        nu_msg = Nu()
        nu_msg.u = float(self.surge_setpoint) * 0.514444
        self.nu_publisher_HMI.publish(nu_msg)
        self.nu_setpoints_HMI.append(nu_msg.u)
        if len(self.nu_setpoints_HMI) > 100:
            self.nu_setpoints_HMI.pop(0)

        # Logging av publisert surge_setpoint
        #self.node.get_logger().info(f"Published nu_setpoint_HMI (u in m/s): {nu_msg.u}")


    def update_graphs(self):
        if self.eta_setpoints_HMI and self.eta_sim_values:
            ax1 = self.graph_canvas_1.figure.get_axes()[0]
            ax1.clear()
            ax1.plot(range(len(self.eta_setpoints_HMI)), self.eta_setpoints_HMI, label="Setpoint", color='#39FF14')
            ax1.plot(range(len(self.eta_sim_values)), self.eta_sim_values, label="Simulated", color='#FF073A')
            ax1.legend(loc="upper right")
            ax1.set_title("Eta Setpoint vs Sim", color="white")
            self.graph_canvas_1.draw()

        if self.nu_setpoints_HMI and self.nu_sim_values:
            ax2 = self.graph_canvas_2.figure.get_axes()[0]
            ax2.clear()
            ax2.plot(range(len(self.nu_setpoints_HMI)), self.nu_setpoints_HMI, label="Setpoint", color='#39FF14')
            ax2.plot(range(len(self.nu_sim_values)), self.nu_sim_values, label="Simulated", color='#FF073A')
            ax2.legend(loc="upper right")
            ax2.set_title("Nu Setpoint vs Sim", color="white")
            self.graph_canvas_2.draw()

    def add_thruster_buttons(self):
        button_layout = QHBoxLayout()
        buttons = {'Standby': 0, 'Position': 1, 'Sail': 2, 'Track': 3}

        for label, mode in buttons.items():
            button = QPushButton(label)
            button.setCheckable(True)
            button.clicked.connect(lambda _, m=mode: self.publish_button_mode(m))
            button.setStyleSheet("""
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
                    background-color: #007ACC;
                }
            """)
            button_layout.addWidget(button)

        exit_button = QPushButton('Exit')
        exit_button.clicked.connect(self.close)
        button_layout.addWidget(exit_button)

        return button_layout

def main():
    rclpy.init()
    node = rclpy.create_node('hmi_gui')
    app = QApplication(sys.argv)
    hmi_window = HMIWindow(node)
    hmi_window.show()
    
    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.1))
    ros_timer.start(10)

    sys.exit(app.exec_())
    rclpy.shutdown()

if __name__ == '__main__':
    main()