#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import yaml
import os
from ament_index_python.packages import get_package_share_directory
import numpy as np
from ngc_utils.thruster_models import AzimuthThruster, PropellerRudderUnit, TunnelThruster
from ngc_interfaces.msg import Tau, ThrusterSignals, Nu
import math
from ngc_utils.qos_profiles import default_qos_profile
import ngc_utils.thruster_objects_loader as tl
import rclpy.time

class PropulsionSimulator(Node):
    def __init__(self):
        super().__init__("ngc_propulsion_sim")
        self.declare_parameter('yaml_package_name', 'ngc_bringup')
        self.declare_parameter('propulsion_config_file', 'config/propulsion_config.yaml')
        self.declare_parameter('simulation_config_file', 'config/simulator_config.yaml')

        # Dynamically find the config files in the NGC bringup package
        yaml_package_name      = self.get_parameter('yaml_package_name').get_parameter_value().string_value
        yaml_package_path      = get_package_share_directory(yaml_package_name)
        simulation_config_path = os.path.join(yaml_package_path, self.get_parameter('simulation_config_file').get_parameter_value().string_value)

        # Load configurations and initialize VesselModel
        simulation_config = self.load_yaml_file(simulation_config_path)

        self.get_logger().info("Configuration files loaded.")

        # Create thruster objects from yaml file
        self.thrusters = tl.load_thrusters_from_yaml(self.get_logger(), yaml_package_name,self.get_parameter('propulsion_config_file').get_parameter_value().string_value, simulation_config['physical_parameters'])
        
        # Subscriptions
        self.nu_sub  = self.create_subscription(Nu, "nu_sim",  self.nu_callback, default_qos_profile)
        self.tau_pub = self.create_publisher(Tau, "tau_propulsion", default_qos_profile)

        self.thruster_subscribers = {}
        self.thruster_publishers  = {}
        
        for thruster in self.thrusters:
            topic_name_sp = f"thruster_{thruster.id}_setpoints"
            topic_name_fb = f"thruster_{thruster.id}_feedback"
            self.thruster_subscribers[thruster.id] = self.create_subscription(ThrusterSignals, topic_name_sp, self.create_thruster_callback(thruster), default_qos_profile)
            self.thruster_publishers[thruster.id] = self.create_publisher(ThrusterSignals, topic_name_fb, default_qos_profile)

        self.nu = np.array([0.0, 0.0, 0.0])

        # Set up simulation step size and callback
        self.step_size = simulation_config['simulation_settings']['step_size']
        self.timer     = self.create_timer(self.step_size, self.step_simulation)

        self.get_logger().info("Setup finished.")

    def step_simulation(self):

        T     = np.empty((5,0))
        u     = np.empty((2 * len(self.thrusters), 1), dtype=float)
        index = 0

        for thruster in self.thrusters:

            thruster.active = thruster.setpoints.active

            if thruster.active == False:
                thruster.setpoints.rps = 0.0    # Override
                
            if thruster.active == True:

                # Update pitch
                if thruster.propeller.type == 'cpp':
                    thruster.propeller.pitch += self.step_size*(1.0/thruster.propeller.pitch_time_constant)*(thruster.setpoints.pitch - thruster.propeller.pitch)

                # Update rudder angle
                if thruster.type == 'propeller_with_rudder':
                    thruster.rudder_angle_rad += self.step_size*(1.0/thruster.rudder_time_constant)*(math.radians(thruster.setpoints.azimuth_deg) - thruster.rudder_angle_rad)

                # Update azimuth angle
                if thruster.type == 'azimuth_thruster':
                    thruster.rudder_angle_rad += self.step_size*(1.0/thruster.azimuth_time_constant)*(math.radians(thruster.setpoints.azimuth_deg) - thruster.azimuth_angle_rad)


            # Update rps - always do this
            thruster.propeller.rps += self.step_size*(1/thruster.propeller.rpm_time_constant)*(thruster.setpoints.rps - thruster.propeller.rps)

            # Publish feedback from thrusters
            thruster_feedback_message             =  ThrusterSignals()
            thruster_feedback_message.thruster_id = int(thruster.id)   
            thruster_feedback_message.rps         = float(thruster.propeller.rps)  
            thruster_feedback_message.active      = thruster.active     
            thruster_feedback_message.error       = False   
            
            if thruster.propeller.type == 'cpp':
                thruster_feedback_message.pitch = float(thruster.propeller.pitch)
            else:
                thruster_feedback_message.pitch = 0.0
            
            if thruster.type == 'propeller_with_rudder':
                thruster_feedback_message.azimuth_deg = float(math.degrees(thruster.rudder_angle_rad))
            elif thruster.type == 'azimuth_thruster':
                thruster_feedback_message.azimuth_deg = float(math.degrees(thruster.azimuth_angle_rad))
            else: 
                thruster_feedback_message.azimuth_deg = 0.0                               

            self.thruster_publishers[thruster.id].publish(thruster_feedback_message)

            T_vector_x = np.array([1.0,0.0,0.0,thruster.position[2],-thruster.position[1]]).reshape(5, 1)
            T_vector_y = np.array([0.0,1.0,-thruster.position[2],0.0,thruster.position[0]]).reshape(5, 1)
          
            if thruster.type == 'propeller_with_rudder':
                if thruster.propeller.type == 'cpp':
                    F_x, F_y = thruster.get_force(thruster.propeller.rps, self.nu, thruster.rudder_angle_rad, thruster.propeller.pitch)
                else:
                    F_x, F_y = thruster.get_force(thruster.propeller.rps, self.nu, thruster.rudder_angle_rad)

            elif thruster.type == 'tunnel_thruster':
                if thruster.propeller.type == 'cpp':
                    F_x, F_y = thruster.get_force(thruster.propeller.rps, self.nu, thruster.propeller.pitch)
                else:
                    F_x, F_y = thruster.get_force(thruster.propeller.rps, self.nu)

            elif thruster.type == 'azimuth_thruster':
                if thruster.propeller.type == 'cpp':
                    F_x, F_y = thruster.get_force(thruster.propeller.rps, self.nu, thruster.rudder_angle_rad, thruster.propeller.pitch)
                else:
                    F_x, F_y = thruster.get_force(thruster.propeller.rps, self.nu, thruster.rudder_angle_rad)

            u[2*index]   = F_x
            u[2*index+1] = F_y
            index += 1

            T = np.hstack((T, T_vector_x))
            T = np.hstack((T, T_vector_y))

        # Compute forces and moments
        tau = T @ u

        # Publish data
        tau_message = Tau()
        
        tau_message.surge_x = float(tau[0])
        tau_message.sway_y  = float(tau[1])
        tau_message.heave_z = 0.0
        tau_message.roll_k  = float(tau[2])
        tau_message.pitch_m = float(tau[3])
        tau_message.yaw_n   = float(tau[4])
        
        self.tau_pub.publish(tau_message)   

        return
    
    def load_yaml_file(self, file_path):
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
   
    def create_thruster_callback(self, thruster):
        def callback(msg: ThrusterSignals):
            thruster.setpoints = msg
            #self.get_logger().info(f"Received setpoint for thruster {thruster.id}")
        return callback

    def nu_callback(self, msg:Nu):
        self.nu = np.array([msg.u, msg.v, msg.r])


def main(args = None):
    rclpy.init(args=args)
    node = PropulsionSimulator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()





