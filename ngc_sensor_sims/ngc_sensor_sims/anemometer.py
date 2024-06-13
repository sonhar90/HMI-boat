import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from ngc_interfaces.msg import Nu, Wind
from ngc_utils.nmea_utils import create_mwv_message
import socket
import math
import ngc_utils.math_utils as mu
from ngc_utils.qos_profiles import default_qos_profile

class AnemometerSimulator(Node):
    def __init__(self):
        super().__init__('Anemometer_sensor_simulator')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.get_logger().info('Starting Anemometer sensor simulator')

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('ID',1),
                ('fix_frequency', 10.0),  # Default update frequency
                ('x_pos', 0.0),
                ('y_pos', 0.0),
                ('z_pos', 0.0),
                ('direction_deg_std_dev', 1.0),
                ('magnitude_ms_std_dev', 0.1),
                ('udp_ip', '127.0.0.1'),
                ('udp_port', 55555),
            ]
        )

        # Initialize noise state and other variables
        self.direction_deg_noise_state = 0.0
        self.magnitude_ms_noise_state = 0.0

        # Fetch parameters from parameter server
        self.fix_frequency = self.get_parameter('fix_frequency').get_parameter_value().double_value
        
        # Sanity check to avoid zero division in timer setup
        if self.fix_frequency < 0.05:
            self.get_logger().warning('Fix frequency is too high, or out of bounds, constraining to 20Hz')
            self.fix_frequency = 0.05
        
        self.direction_deg_std_dev = self.get_parameter('direction_deg_std_dev').get_parameter_value().double_value
        self.magnitude_ms_std_dev = self.get_parameter('magnitude_ms_std_dev').get_parameter_value().double_value
        self.udp_ip = self.get_parameter('udp_ip').get_parameter_value().string_value
        self.udp_port = self.get_parameter('udp_port').get_parameter_value().integer_value
        self.x_pos = self.get_parameter('x_pos').get_parameter_value().double_value
        self.y_pos = self.get_parameter('y_pos').get_parameter_value().double_value
        self.z_pos = self.get_parameter('z_pos').get_parameter_value().double_value

        # Setup callback group
        self.callback_group = ReentrantCallbackGroup()

        # Create timer for updating noise
        self.noise_timer = self.create_timer(0.1, self.update_noise_model, callback_group=self.callback_group)

        # Create timer for main callback
        self.timer = self.create_timer(1.0 / self.fix_frequency, self.timer_callback, callback_group=self.callback_group)

        # Create subscriptions for Eta and Nu messages
        self.nu_subscription = self.create_subscription(Nu, 'nu_sim', self.nu_callback, default_qos_profile, callback_group=self.callback_group)
        self.wind_subscription = self.create_subscription(Wind, 'wind_sim', self.wind_callback, default_qos_profile, callback_group=self.callback_group)
        
        # Initialize message variables
        self.latest_nu_msg   = None
        self.latest_wind_msg = None

        self.get_logger().info('Setup done')

    def nu_callback(self, msg):
        self.latest_nu_msg = msg

    def wind_callback(self, msg):
        self.latest_wind_msg = msg

    def update_noise_model(self):
        # Simple position noise model
        self.direction_deg_noise_state = np.random.normal(0, self.direction_deg_std_dev, 1)
        self.magnitude_ms_noise_state = np.random.normal(0, self.magnitude_ms_noise_state, 1)

    def timer_callback(self):

        # Make local copies to avoide the object chaning during routine
        local_latest_nu_msg = self.latest_nu_msg
        local_latest_wind_msg = self.latest_wind_msg

        if (local_latest_nu_msg is not None) and (local_latest_wind_msg is not None):
            
            lever_arm_pitch = np.sqrt(self.x_pos**2 + self.z_pos**2)
            lever_arm_roll  = np.sqrt(self.y_pos**2 + self.z_pos**2)
            u_rw            = local_latest_wind_msg.magnitude_ms*np.cos(np.radians(local_latest_wind_msg.direction_relative_deg)) - lever_arm_pitch*local_latest_nu_msg.q
            v_rw           = local_latest_wind_msg.magnitude_ms*np.sin(np.radians(local_latest_wind_msg.direction_relative_deg)) - lever_arm_roll*local_latest_nu_msg.p

            relative_wind_direction = mu.mapToPiPi(-np.arctan2(v_rw,u_rw))
            relative_wind_speed     = np.sqrt(u_rw**2 + v_rw**2)

            # Generate and send MWV NMEA messages
            mwv_message = create_mwv_message(relative_wind_direction + self.direction_deg_noise_state[0],relative_wind_speed + self.magnitude_ms_noise_state[0])
            
            #self.get_logger().info(f'Sending HDT NMEA Message: "{hdt_message}"\nSending ROT NMEA Message: "{hdt_message}"')
            
            self.sock.sendto(mwv_message.encode(), (self.udp_ip, self.udp_port))

            local_latest_nu_msg = None
            local_latest_wind_msg = None

def main(args=None):
    rclpy.init(args=args)
    anemometer_simulator = AnemometerSimulator()
    executor = MultiThreadedExecutor()
    executor.add_node(anemometer_simulator)
    
    try:
        executor.spin()
    finally:
        anemometer_simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
