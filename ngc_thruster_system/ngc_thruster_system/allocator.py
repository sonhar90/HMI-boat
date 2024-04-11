#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import yaml
import numpy as np
from ngc_thruster_system.thruster_model import FixedPitchPropeller, Azimuth, Tunnel
from ngc_interfaces.msg import Tau

"""
The Allocator class is the manager class in the thruster system and is repsonible to 
"""
class Allocator(Node):
    def __init__(self):
        super().__init__("allocator")
        self.declare_parameter("thruster_config_file", "")
        self.get_logger().info("Waiting for thruster config file path...")
        self.config_file = self.get_parameter("thruster_config_file").value
        #TODO:pass this as an argument from the launch file? 
        self.thrusters = self.load_thrusters_from_yaml("/home/nupix/ngc_ws/src/ngc_bringup/config/arbeidsbaat/thrusters.yaml")
        print("Loaded thruster objects:")
        for thruster in self.thrusters:
            print(f"  - Name: {thruster.name}, Type: {type(thruster).__name__}")
        self.get_logger().info("Allocator node started!")
        self.ff_tau_pub = self.create_publisher(Tau, "ff_tau", 1)
        self.timer = self.create_timer(0.05, self.get_tau)

    def get_tau(self):
        sum_tau=  np.array([0,0,0])
        for thruster in self.thrusters:
            sum_tau = sum_tau + np.array(thruster.get_tau())
        return sum_tau 

    def load_thrusters_from_yaml(self, filename):
        """
        This function reads a YAML file containing thruster configurations and returns a list of Thruster objects.

        Args:
            filename: Path to the YAML file.

        Returns:
            A list of Thruster objects.
        """
        with open(filename, 'r') as f:
            data = yaml.safe_load(f)

        thrusters = []
        for name, config in data.items():
            # Extract configuration details
            position = config.get('position', [0, 0, 0])
            thruster_type = config.get('type')
            thrust_torque_curve = config.get('thrust_torque_curve')
            prop_diameter = config.get('prop_diameter')
            rudder_config = config.get('rudder', {})

            # Create thruster object based on type
            if thruster_type == 'propeller_fixed':
                thruster = FixedPitchPropeller(name, thrust_torque_curve, prop_diameter)
            elif thruster_type == 'azimuth':
                thruster = Azimuth(name, thrust_torque_curve, prop_diameter)
            elif thruster_type == 'tunnel_thruster':
                thruster = Tunnel(name, thrust_torque_curve, prop_diameter)
            else:
                raise ValueError(f"Unknown thruster type: {thruster_type}")

            # Set common attributes
            thruster.name = name
            thruster.position = position

            # Set rudder attributes if defined
            if rudder_config:
                thruster.rudder_height = rudder_config.get('height', 0.0)
                thruster.rudder_length = rudder_config.get('length')
                thruster.rudder_block_coeff = rudder_config.get('block_coeff', 0.5)

            # Add thruster to the list
            thrusters.append(thruster)

        return thrusters


def main(args = None):
    rclpy.init(args=args)
    node = Allocator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()





