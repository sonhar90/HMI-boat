# ros_node.py

from rclpy.node import Node
from ngc_interfaces.msg import Eta, Nu, ButtonControl, ThrusterSignals, SystemMode
from ngc_utils.qos_profiles import default_qos_profile
import numpy as np


class ROSNode(Node):
    def __init__(self):
        super().__init__("hmi_ros_node")

        # Publishers
        self.eta_publisher_HMI = self.create_publisher(
            Eta, "eta_setpoint_HMI", default_qos_profile
        )
        self.nu_publisher_HMI = self.create_publisher(
            Nu, "nu_setpoint_HMI", default_qos_profile
        )
        self.button_publisher = self.create_publisher(
            ButtonControl, "button_control", default_qos_profile
        )
        self.systemmode_publisher = self.create_publisher(
            SystemMode, "system_mode", default_qos_profile
        )

        # Subscribers
        self.create_subscription(Eta, "eta_hat", self.eta_callback, default_qos_profile)
        self.create_subscription(Nu, "nu_hat", self.nu_callback, default_qos_profile)
        self.create_subscription(
            ThrusterSignals,
            "thruster_1_setpoints",
            self.thruster1_setpoint_callback,
            default_qos_profile,
        )
        self.create_subscription(
            ThrusterSignals,
            "thruster_2_setpoints",
            self.thruster2_setpoint_callback,
            default_qos_profile,
        )
        self.create_subscription(
            ThrusterSignals,
            "thruster_1_feedback",
            self.thruster1_feedback_callback,
            default_qos_profile,
        )
        self.create_subscription(
            ThrusterSignals,
            "thruster_2_feedback",
            self.thruster2_feedback_callback,
            default_qos_profile,
        )

        # Initialize variables to store data
        self.heading_setpoint = 0.0
        self.surge_setpoint = 0.0
        self.current_button_mode = None  # Add this line

        # Callbacks to be set by HMIWindow
        self.on_eta_received = None
        self.on_nu_received = None
        self.on_thruster1_setpoint_received = None
        self.on_thruster2_setpoint_received = None
        self.on_thruster1_feedback_received = None
        self.on_thruster2_feedback_received = None

    # Callbacks for subscriptions
    def eta_callback(self, msg):
        if self.on_eta_received:
            self.on_eta_received(msg)

    def nu_callback(self, msg):
        if self.on_nu_received:
            self.on_nu_received(msg)

    def thruster1_setpoint_callback(self, msg):
        if self.on_thruster1_setpoint_received:
            self.on_thruster1_setpoint_received(msg)

    def thruster2_setpoint_callback(self, msg):
        if self.on_thruster2_setpoint_received:
            self.on_thruster2_setpoint_received(msg)

    def thruster1_feedback_callback(self, msg):
        if self.on_thruster1_feedback_received:
            self.on_thruster1_feedback_received(msg)

    def thruster2_feedback_callback(self, msg):
        if self.on_thruster2_feedback_received:
            self.on_thruster2_feedback_received(msg)

    # Methods to publish messages
    def publish_eta_setpoint(self, psi):
        eta_msg = Eta()
        eta_msg.psi = float(psi)
        self.eta_publisher_HMI.publish(eta_msg)

    def publish_nu_setpoint(self, u):
        nu_msg = Nu()
        nu_msg.u = float(u)
        self.nu_publisher_HMI.publish(nu_msg)

    def publish_button_mode(self, mode):
        self.current_button_mode = mode  # Update current button mode
        button_msg = ButtonControl()
        button_msg.mode = mode
        self.button_publisher.publish(button_msg)

        system_mode_msg = SystemMode()
        if mode == 0:
            system_mode_msg.standby_mode = True
            system_mode_msg.auto_mode = False
        else:
            system_mode_msg.standby_mode = False
            system_mode_msg.auto_mode = True

        self.systemmode_publisher.publish(system_mode_msg)
