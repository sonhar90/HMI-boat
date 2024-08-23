#!/usr/bin/env python3
from typing import List
import rclpy
from rclpy.node import Node
import numpy as np
from builtin_interfaces.msg import Time
from ngc_interfaces.msg import Eta, Nu, NuDot, Tau, Wind
from ngc_utils.geo_utils import add_distance_to_lat_lon
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from ngc_utils.vessel_model import VesselModel
import math
import ngc_utils.math_utils as mu
from ngc_utils.qos_profiles import default_qos_profile
import time

class HullSimulator(Node):
    def __init__(self):
        super().__init__('ngc_hull_simulator')
        self.declare_parameter('yaml_package_name', 'ngc_bringup')
        self.declare_parameter('vessel_config_file', 'config/vessel_config.yaml')
        self.declare_parameter('simulation_config_file', 'config/simulator_config.yaml')

        # Dynamically find the config files in the NGC bringup package
        yaml_package_name      = self.get_parameter('yaml_package_name').get_parameter_value().string_value
        yaml_package_path      = get_package_share_directory(yaml_package_name)
        vessel_config_path     = os.path.join(yaml_package_path, self.get_parameter('vessel_config_file').get_parameter_value().string_value)
        simulation_config_path = os.path.join(yaml_package_path, self.get_parameter('simulation_config_file').get_parameter_value().string_value)

        # Load configurations and initialize VesselModel
        vessel_config     = self.load_yaml_file(vessel_config_path)
        simulation_config = self.load_yaml_file(simulation_config_path)

        self.vessel_model = VesselModel(vessel_config)  # Initialize the vessel model
        self.vessel_model.set_physical_parameters(simulation_config['physical_parameters'])
        self.vessel_model.display_info()

        self.get_logger().info("Configuration files loaded.")

        # Set up the input and output data
        self.eta_pub        = self.create_publisher(Eta, "eta_sim", default_qos_profile)
        self.nu_pub         = self.create_publisher(Nu, "nu_sim", default_qos_profile)
        self.nu_dot_pub     = self.create_publisher(NuDot, "nu_dot_sim", default_qos_profile)
        self.wind_pub       = self.create_publisher(Wind, "wind_sim", default_qos_profile)
        
        self.tau_sub        = self.create_subscription(Tau, "tau_propulsion",  self.force_cmd_callback, default_qos_profile)
        self.tau            = Tau()

        # Setup inital values of the simlator
        self.eta     = np.array([0,0,0,0,0,0], dtype= float)
        self.nu      = np.array([0,0,0,0,0,0], dtype= float)
        self.nu_dot  = np.array([0,0,0,0,0,0], dtype= float)

        self.latitude = simulation_config['initial_conditions']['latitude']
        self.longitude = simulation_config['initial_conditions']['longitude']
        self.eta[5] = math.radians(simulation_config['initial_conditions']['heading'])
        self.nu = np.array(simulation_config['initial_conditions']['nu'], dtype=float)

        # Setup wind and current disturbances
        self.wind_magnitude    = simulation_config['environmental_disturbances']['wind_magnitude']
        self.wind_direction    = simulation_config['environmental_disturbances']['wind_direction']
        self.wind_variation    = simulation_config['environmental_disturbances']['wind_variation']
        self.current_magnitude = simulation_config['environmental_disturbances']['current_magnitude']
        self.current_direction = simulation_config['environmental_disturbances']['current_direction']
        self.current_variation = simulation_config['environmental_disturbances']['current_variation']
        self.nu_current = np.array([0,0,0,0,0,0], dtype= float)

        # Set up simulation step size and callback
        self.step_size = simulation_config['simulation_settings']['step_size']
        self.timer     = self.create_timer(self.step_size, self.step_simulation)

        # Setup timing variable for monitoring
        self.prev_run_time = time.perf_counter()
        self.run_dt        = 0.0

        self.get_logger().info("Setup finished.")

    def load_yaml_file(self, file_path):
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
        
    def force_cmd_callback(self, msg:Tau):
        self.tau = msg

    def step_simulation(self):
        
        time_now = time.perf_counter()
        self.run_dt = time_now - self.prev_run_time
        self.prev_run_time = time_now

        # Setup the output topic messages
        eta_message    = Eta()
        nu_message     = Nu()
        nu_dot_message = NuDot()
        wind_message   = Wind()

        # Update wind and current variation models
        self.update_environment_disturbances()

        # Get the Coreolis and centripital force components
        tau_coreolis_centripital = self.vessel_model.get_6DOF_coreolis_centripital_forces(self.nu)

        # Compute the different force components for the integration
        tau_propulsion = np.array([self.tau.surge_x,self.tau.sway_y,self.tau.heave_z,self.tau.roll_k,self.tau.pitch_m,self.tau.yaw_n], dtype= float)
        tau_damping    = self.vessel_model.get_6dof_hydrodynamic_damping(self.nu - self.nu_current)
        tau_restoring  = self.vessel_model.get_6dof_restoring_forces(self.eta)
        tau_wind       = self.vessel_model.get_6Dof_wind_forces(self.eta, self.nu, self.wind_direction, self.wind_magnitude)

        # Get the inverse mass matix
        M_inv = self.vessel_model.MassInv6Dof

        # Compute the velocity derivative and integrate
        self.nu_dot = M_inv @ (- 0*tau_coreolis_centripital + tau_propulsion + tau_damping - tau_restoring + 0*tau_wind)
        self.nu += self.step_size*self.nu_dot

        # Reset north and east positions in order to get the differential from last timestep
        self.eta[0] = 0
        self.eta[1] = 0

        # Integrate to get the position
        eta_dot      = np.zeros(6, dtype=float)
        eta_dot[0:3] = mu.RotationMatrix(self.eta[3],self.eta[4],self.eta[5]) @ self.nu[0:3]
        eta_dot[3:6] = mu.TranslationMatrix(self.eta[3],self.eta[4]) @ self.nu[3:6]

        self.eta += self.step_size*eta_dot

        # Map the heading to -pi to pi to avoid any problems with high or low values
        self.eta[5] = mu.mapToPiPi(self.eta[5])

        #Update latitude and longitude positions from the differetial change in north and east positions
        self.latitude, self.longitude = add_distance_to_lat_lon(self.latitude, self.longitude, self.eta[0], self.eta[1])
        
        # Construct and publish the output topic messages
        eta_message.lat, eta_message.lon, eta_message.z, eta_message.psi = self.latitude, self.longitude,self.eta[2], self.eta[5] 
        nu_message.u, nu_message.v, nu_message.w, nu_message.p, nu_message.q, nu_message.r = map(lambda x:x, self.nu)
        nu_dot_message.u_dot, nu_dot_message.v_dot, nu_dot_message.w_dot, nu_dot_message.p_dot, nu_dot_message.q_dot, nu_dot_message.r_dot = map(lambda x:x, self.nu_dot)

        relative_wind_speed, relative_wind_direction = self.vessel_model.get_relative_wind_speed_and_direction(self.eta, self.nu, self.wind_direction, self.wind_magnitude)

        wind_message.direction_relative_deg = relative_wind_direction
        wind_message.magnitude_ms           = relative_wind_speed

        # TEMP for monitoring
        eta_message.z = self.run_dt

        # Publish the data
        self.eta_pub.publish(eta_message)
        self.nu_pub.publish(nu_message)
        self.nu_dot_pub.publish(nu_dot_message)
        self.wind_pub.publish(wind_message)

    def update_environment_disturbances(self):

        #Update the wind and current models if variation is specified
        if self.current_variation:
            curent_mag_variation = np.random.normal(0, 1.0, 1)
            curent_dir_variation = np.random.normal(0, 5.0, 1)
            self.current_magnitude += self.step_size*0.1*curent_mag_variation[0]
            self.current_direction += self.step_size*0.1*curent_dir_variation[0]

            # Ensure positive magnitude and well defined direction
            self.current_magnitude = np.abs(self.current_magnitude)
            self.current_direction = np.degrees(mu.mapToPiPi(np.radians(self.current_direction)))

        if self.wind_variation:
            wind_mag_variation = np.random.normal(0, 0.5, 1)
            wind_dir_variation = np.random.normal(0, 5.0, 1)
            self.wind_magnitude += self.step_size*0.1*wind_mag_variation[0]
            self.wind_direction += self.step_size*0.1*wind_dir_variation[0]

            # Ensure positive magnitude and well defined direction
            self.wind_magnitude = np.abs(self.wind_magnitude)
            self.wind_direction = np.degrees(mu.mapToPiPi(np.radians(self.wind_direction)))

        NED_current     = np.array([self.current_magnitude*math.cos(math.radians(self.current_direction)),self.current_magnitude*math.sin(math.radians(self.current_direction)),0], dtype= float)
        body_current    = mu.RotationMatrix(0,0,self.eta[5]).T @ NED_current
        self.nu_current = np.array([body_current[0],body_current[1],0,0,0,0])

def main(args=None):
    rclpy.init(args=args)
    simulation_node = HullSimulator()
    rclpy.spin(simulation_node)
    simulation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
