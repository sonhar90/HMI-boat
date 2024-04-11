#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import yaml
import numpy as np
from ngc_thruster_system.thruster_model import FixedPitchPropeller, Azimuth, Tunnel, ControllablePitchPropellerAndRudder
from ngc_interfaces.msg import Tau, SetPoints, Nu
import math

import rclpy.time


"""
The Thruster System class is the manager class in the thruster system and is repsonible to derive the thrust forces
"""
class ThrusterSystem(Node):
    def __init__(self):
        super().__init__("thruster_system")
        self.declare_parameter("thruster_config_file_name", "")
        self.get_logger().info("Waiting for thruster config file path...")
        self.config_file_name = self.get_parameter("thruster_config_file_name").value
        self.get_logger().info(f"thruster system config filename: {self.config_file_name}")
        #TODO:pass this as an argument from the launch file? 
        self.thrusters = self.load_thrusters_from_yaml(self.config_file_name)
        print("Loaded thruster objects:")
        for thruster in self.thrusters:
            print(f"  - Name: {thruster.name}, Type: {type(thruster).__name__}")
        self.get_logger().info("Thruster system node started!")
        self.set_points = SetPoints()
        self.nu = Nu()
        self.nu.u = 0.0
        self.nu.v = 0.0
        self.set_point_subscriber = self.create_subscription(SetPoints, "set_points_thrusters", self.set_point_callback, 2)
        self.nu_subscriber = self.create_subscription(Nu, "nu_sim", self.nu_callback, 10)
        self.tau_publisher = self.create_publisher(Tau, "tau_prop", 1)        
        self.timer = self.create_timer(0.05, self.publish_tau_callback)
    
    def publish_tau_callback(self):
        #test paramas:
        """n= 20.0
        pitch = 1.4 #1.4
        angle = 0.0 #0.35 #20deg = 0.35 rads, 10 deg: 0.17:rads
        nu = Nu()
        nu.u= 10.0
        self.nu = nu
        test = SetPoints()
        test.setpoints = [n, pitch, angle, n, pitch, angle]
        self.set_points = test"""
        #####test params done

        thruster_forces =  []

        sp = np.array(self.set_points.setpoints)
        sp = np.split(sp, len(self.thrusters))
        
        for i, thruster in enumerate(self.thrusters):
            thruster_sp = sp[i]
            n = thruster_sp[0]
            pitch = thruster_sp[1]
            angle = thruster_sp[2]
            name = thruster.name
            thruster_forces.append(thruster.get_tau(nu= self.nu, n= n, pitch = pitch, angle = angle))
        
        sum_thruster_forces = np.sum(thruster_forces, axis = 0)
        tau_msg = Tau()
        tau_msg.surge_x, tau_msg.sway_y, tau_msg.yaw_n = map(lambda x:x,sum_thruster_forces )
        self.tau_publisher.publish(tau_msg)

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
            elif thruster_type == "rudder_propeller_variable":
                thruster = ControllablePitchPropellerAndRudder(name, thrust_torque_curve, prop_diameter)
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
                thruster.rudder_thikness = rudder_config.get('thikness', 0.0)
                thruster.distance_propeller_rudder = rudder_config.get('l_prop_to_rudder', 0.0)

            # Add thruster to the list
            #print(thruster.position)
            thrusters.append(thruster)

        return thrusters

    def set_point_callback(self, msg:SetPoints):
        self.set_points = msg

    def nu_callback(self, msg:Nu):
        self.nu = msg

def main(args = None):
    rclpy.init(args=args)
    node = ThrusterSystem()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()





