import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from ngc_interfaces.msg import Eta, Nu
from ngc_utils.geo_utils import add_body_frame_pos_to_lat_lon, add_distance_to_lat_lon
from ngc_utils.nmea_utils import create_gga_message, create_vtg_message
import socket
import math
from ngc_utils.qos_profiles import default_qos_profile
import ngc_utils.math_utils as mu

class GNSSSimulator(Node):
    def __init__(self):
        super().__init__('gnss_simulator')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.get_logger().info('Starting GNSS simulator')

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('ID',1),
                ('fix_frequency', 10.0),  # Default update frequency
                ('x_pos', 0.0),
                ('y_pos', 0.0),
                ('z_pos', 0.0),
                ('position_std_dev', 1.0),
                ('velocity_std_dev', 0.1),
                ('udp_ip', '127.0.0.1'),
                ('udp_port', 55555),
            ]
        )

        # Initialize noise state and other variables
        self.position_noise_state = np.array([0.0, 0.0])
        self.velocity_noise_state = np.array([0.0, 0.0])

        # Fetch parameters from parameter server
        self.fix_frequency = self.get_parameter('fix_frequency').get_parameter_value().double_value
        
        # Sanity check to avoid zero division in timer setup
        if self.fix_frequency < 0.05:
            self.get_logger().warning('Fix frequency is too high, or out of bounds, constraining to 20Hz')
            self.fix_frequency = 0.05
        
        self.x_pos = self.get_parameter('x_pos').get_parameter_value().double_value
        self.y_pos = self.get_parameter('y_pos').get_parameter_value().double_value
        self.z_pos = self.get_parameter('z_pos').get_parameter_value().double_value
        self.position_std_dev = self.get_parameter('position_std_dev').get_parameter_value().double_value
        self.velocity_std_dev = self.get_parameter('velocity_std_dev').get_parameter_value().double_value
        self.udp_ip = self.get_parameter('udp_ip').get_parameter_value().string_value
        self.udp_port = self.get_parameter('udp_port').get_parameter_value().integer_value

        # Setup callback group
        self.callback_group = ReentrantCallbackGroup()

        # Create timer for updating noise
        self.noise_timer = self.create_timer(0.1, self.update_noise_model, callback_group=self.callback_group)

        # Create timer for main callback
        self.timer = self.create_timer(1.0 / self.fix_frequency, self.timer_callback, callback_group=self.callback_group)

        # Create subscriptions for Eta and Nu messages
        self.eta_subscription = self.create_subscription(Eta, 'eta_sim', self.eta_callback, default_qos_profile, callback_group=self.callback_group)
        self.nu_subscription = self.create_subscription(Nu, 'nu_sim', self.nu_callback, default_qos_profile, callback_group=self.callback_group)
        
        # Initialize message variables
        self.latest_eta_msg = None
        self.latest_nu_msg = None

        self.get_logger().info('Setup done')

    def eta_callback(self, msg):
        self.latest_eta_msg = msg

    def nu_callback(self, msg):
        self.latest_nu_msg = msg

    def update_noise_model(self):
        # Simple position noise model
        white_noise = np.random.normal(0, self.position_std_dev, 2)
        self.position_noise_state[0] += 0.1*(-0.1*self.position_noise_state[0] + white_noise[0])
        self.position_noise_state[1] += 0.1*(-0.1*self.position_noise_state[1] + white_noise[1])

        self.velocity_noise_state = np.random.normal(0, self.velocity_noise_state, 2)

    def timer_callback(self):

        # Make local copies to avoide the object chaning during routine
        local_latest_eta_msg = self.latest_eta_msg
        local_latest_nu_msg = self.latest_nu_msg

        if (local_latest_eta_msg is not None) and (local_latest_nu_msg is not None):
            # Calculate velocity considering noise and movement due to roll and pitch rates
            # For simplicity, roll_rate affects sway (v) and pitch_rate affects surge (u)
            # But with SNAME conventions, positive roll increases sway and positive pitch increases surge

            antenna_movement_due_to_pitch = -local_latest_nu_msg.q * self.z_pos
            antenna_movement_due_to_roll = -local_latest_nu_msg.p * self.z_pos
            
            u_with_noise_and_movement = local_latest_nu_msg.u + self.velocity_noise_state[0] + antenna_movement_due_to_pitch
            v_with_noise_and_movement = local_latest_nu_msg.v + self.velocity_noise_state[1] + antenna_movement_due_to_roll
            
            antenna_lat, antenna_lon = add_body_frame_pos_to_lat_lon(local_latest_eta_msg.lat, local_latest_eta_msg.lon, self.x_pos, self.y_pos, self.z_pos, math.degrees(local_latest_eta_msg.phi), math.degrees(local_latest_eta_msg.theta), math.degrees(local_latest_eta_msg.psi))
            noisy_lat, noisy_lon = add_distance_to_lat_lon(antenna_lat, antenna_lon, self.position_noise_state[0], self.position_noise_state[1])

            # Rotate velocities to NED for COG calculation
            nu_3_dof       = np.array([u_with_noise_and_movement,v_with_noise_and_movement,0.0])
            R              = mu.RotationMatrix(0.0,0.0,local_latest_eta_msg.psi)
            NED_velocities = R @ nu_3_dof

            # Generate and send GGA and VTG NMEA messages
            gga_message = create_gga_message(noisy_lat, noisy_lon)
            vtg_message = create_vtg_message(NED_velocities[0], NED_velocities[1])
            
            #self.get_logger().info(f'Sending GGA NMEA Message: "{gga_message}"\nSending VTG NMEA Message: "{vtg_message}"')

            self.sock.sendto(gga_message.encode(), (self.udp_ip, self.udp_port))
            self.sock.sendto(vtg_message.encode(), (self.udp_ip, self.udp_port))

            local_latest_eta_msg = None
            local_latest_nu_msg = None

def main(args=None):
    rclpy.init(args=args)
    gnss_simulator = GNSSSimulator()
    executor = MultiThreadedExecutor()
    executor.add_node(gnss_simulator)
    
    try:
        executor.spin()
    finally:
        gnss_simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
