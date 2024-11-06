import rclpy
from rclpy.node import Node
import numpy as np
import os
import yaml
from ngc_interfaces.msg import Eta, Nu, ButtonControl
from ament_index_python.packages import get_package_share_directory
from ngc_utils.qos_profiles import default_qos_profile

class Guide(Node):
    def __init__(self):
        super().__init__('guide')
        self.get_logger().info('Guide node is initialized.')

        # Load configuration parameters from YAML files
        self.declare_parameter('yaml_package_name', 'ngc_bringup')
        self.declare_parameter('simulation_config_file', 'config/simulator_config.yaml')

        yaml_package_name = self.get_parameter('yaml_package_name').get_parameter_value().string_value
        yaml_package_path = get_package_share_directory(yaml_package_name)
        simulation_config_path = os.path.join(yaml_package_path, self.get_parameter('simulation_config_file').get_parameter_value().string_value)

        self.simulation_config = self.load_yaml_file(simulation_config_path)
        self.step_size = self.simulation_config['simulation_settings']['step_size']

        # Subscribe to eta and nu setpoints from HMI
        self.eta_setpoint_HMI_sub = self.create_subscription(Eta, 'eta_setpoint_HMI', self.eta_setpoint_HMI_callback, default_qos_profile)
        self.nu_setpoint_HMI_sub = self.create_subscription(Nu, 'nu_setpoint_HMI', self.nu_setpoint_HMI_callback, default_qos_profile)
        
        # Subscribe to button mode messages from HMI
        self.button_control_sub = self.create_subscription(ButtonControl, 'button_control', self.button_control_callback, default_qos_profile)

        # Publishers for forwarding selected setpoints based on mode
        self.eta_setpoint_pub = self.create_publisher(Eta, "eta_setpoint", default_qos_profile) 
        self.nu_setpoint_pub = self.create_publisher(Nu, "nu_setpoint", default_qos_profile)

        # Initialize storage for setpoints and mode
        self.eta_setpoint = Eta()
        self.nu_setpoint = Nu()
        self.current_mode = None  # Will hold the current mode from button presses

        # Start control loop with the same timestep as the simulator
        self.timer = self.create_timer(self.step_size, self.step_control)

    def eta_setpoint_HMI_callback(self, msg):
        """Callback for eta_setpoint subscription from HMI."""
        self.eta_setpoint = msg
        self.get_logger().info(f"Received eta setpoint from HMI: psi = {msg.psi}")

    def nu_setpoint_HMI_callback(self, msg):
        """Callback for nu_setpoint subscription from HMI."""
        self.nu_setpoint = msg
        self.get_logger().info(f"Received nu setpoint from HMI: u = {msg.u}")

    def button_control_callback(self, msg):
        """Callback for button control messages indicating mode selection."""
        self.current_mode = msg.mode
        self.get_logger().info(f"Button control mode set to: {self.current_mode}")

    def step_control(self):
        """Control loop that runs based on the button mode from HMI."""
        if self.current_mode == 0:  # Standby
            # Stop forwarding eta and nu setpoints
            self.get_logger().info("Standby: Stopping eta and nu setpoints.")

        elif self.current_mode == 1:  # Position
            # Placeholder for Position mode handling
            self.get_logger().info("Position: Position-hold function activated (placeholder).")

        elif self.current_mode == 2:  # Sail
            # Forward eta and nu setpoints from HMI directly to control nodes
            self.eta_setpoint_pub.publish(self.eta_setpoint)
            self.nu_setpoint_pub.publish(self.nu_setpoint)
            self.get_logger().info("Sail: Forwarding eta and nu setpoints from HMI.")

        elif self.current_mode == 3:  # Track
            # Placeholder for Track mode handling
            self.get_logger().info("Track: Tracking function activated (placeholder).")

    def load_yaml_file(self, file_path):
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)

def main(args=None):
    rclpy.init(args=args)
    node = Guide()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
