import yaml
import os
import numpy as np
import math
from ament_index_python.packages import get_package_share_directory
from ngc_utils.thruster_models import AzimuthThruster, PropellerRudderUnit, TunnelThruster
import ngc_utils.thruster_objects_loader as tl
import rclpy

class PropulsionModel:
    def __init__(self, node):
        self.node = node
        self.node.declare_parameter('yaml_package_name', 'ngc_bringup')
        self.node.declare_parameter('propulsion_config_file', 'config/propulsion_config.yaml')
        self.node.declare_parameter('simulation_config_file', 'config/simulator_config.yaml')

        yaml_package_name = self.node.get_parameter('yaml_package_name').get_parameter_value().string_value
        propulsion_config_file = self.node.get_parameter('propulsion_config_file').get_parameter_value().string_value
        simulation_config_file = self.node.get_parameter('simulation_config_file').get_parameter_value().string_value
        
        yaml_package_path = get_package_share_directory(yaml_package_name)
        simulation_config_path = os.path.join(yaml_package_path, simulation_config_file)

        # Load configurations
        self.simulation_config = self.load_yaml_file(simulation_config_path)
        self.step_size = self.simulation_config['simulation_settings']['step_size']
        self.thrusters = self.load_thrusters(yaml_package_name, propulsion_config_file)
        
    def load_yaml_file(self, file_path):
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)

    def load_thrusters(self, yaml_package_name, propulsion_config_file):
        return tl.load_thrusters_from_yaml(self.node.get_logger(), yaml_package_name, propulsion_config_file, self.simulation_config['physical_parameters'])

    def step_simulation(self, nu):
        T = np.empty((5, 0))
        u = np.empty((2 * len(self.thrusters), 1), dtype=float)
        index = 0

        for thruster in self.thrusters:
            thruster.active = thruster.setpoints.active

            if not thruster.active:
                thruster.setpoints.rps = 0.0  # Override

            if thruster.active:
                if thruster.propeller.type == 'cpp':
                    thruster.propeller.pitch += self.step_size * (1.0 / thruster.propeller.pitch_time_constant) * (thruster.setpoints.pitch - thruster.propeller.pitch)
                if thruster.type == 'propeller_with_rudder':
                    thruster.rudder_angle_rad += self.step_size * (1.0 / thruster.rudder_time_constant) * (math.radians(thruster.setpoints.azimuth_deg) - thruster.rudder_angle_rad)
                if thruster.type == 'azimuth_thruster':
                    thruster.rudder_angle_rad += self.step_size * (1.0 / thruster.azimuth_time_constant) * (math.radians(thruster.setpoints.azimuth_deg) - thruster.azimuth_angle_rad)

            thruster.propeller.rps += self.step_size * (1 / thruster.propeller.rpm_time_constant) * (thruster.setpoints.rps - thruster.propeller.rps)

            thruster_feedback_message = ThrusterSignals()
            thruster_feedback_message.thruster_id = int(thruster.id)
            thruster_feedback_message.rps = float(thruster.propeller.rps)
            thruster_feedback_message.active = thruster.active
            thruster_feedback_message.error = False

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

            self.node.get_publisher(f"thruster_{thruster.id}_feedback").publish(thruster_feedback_message)

            T_vector_x = np.array([1.0, 0.0, 0.0, thruster.position[2], -thruster.position[1]]).reshape(5, 1)
            T_vector_y = np.array([0.0, 1.0, -thruster.position[2], 0.0, thruster.position[0]]).reshape(5, 1)

            if thruster.type == 'propeller_with_rudder':
                if thruster.propeller.type == 'cpp':
                    F_x, F_y = thruster.get_force(thruster.propeller.rps, nu, thruster.rudder_angle_rad, thruster.propeller.pitch)
                else:
                    F_x, F_y = thruster.get_force(thruster.propeller.rps, nu, thruster.rudder_angle_rad)

            elif thruster.type == 'tunnel_thruster':
                if thruster.propeller.type == 'cpp':
                    F_x, F_y = thruster.get_force(thruster.propeller.rps, nu, thruster.propeller.pitch)
                else:
                    F_x, F_y = thruster.get_force(thruster.propeller.rps, nu)

            elif thruster.type == 'azimuth_thruster':
                if thruster.propeller.type == 'cpp':
                    F_x, F_y = thruster.get_force(thruster.propeller.rps, nu, thruster.rudder_angle_rad, thruster.propeller.pitch)
                else:
                    F_x, F_y = thruster.get_force(thruster.propeller.rps, nu, thruster.rudder_angle_rad)

            u[2 * index] = F_x
            u[2 * index + 1] = F_y
            index += 1

            T = np.hstack((T, T_vector_x))
            T = np.hstack((T, T_vector_y))

        tau = T @ u
        return tau