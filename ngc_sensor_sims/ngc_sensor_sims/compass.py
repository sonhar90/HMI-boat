import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from ngc_interfaces.msg import Eta, Nu
from ngc_utils.nmea_utils import create_hdt_message, create_rot_message
import socket
import math

class CompassSimulator(Node):
    def __init__(self):
        super().__init__('heading_sensor_simulator')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.get_logger().info('Starting Heading sensor simulator')

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('ID',1),
                ('fix_frequency', 10.0),  # Default update frequency
                ('heading_std_dev', 0.01),
                ('rot_std_dev', 0.01),
                ('udp_ip', '127.0.0.1'),
                ('udp_port', 55555),
            ]
        )

        # Initialize noise state and other variables
        self.heading_noise_state = 0.0
        self.rot_noise_state = 0.0

        # Fetch parameters from parameter server
        self.fix_frequency = self.get_parameter('fix_frequency').get_parameter_value().double_value
        
        # Sanity check to avoid zero division in timer setup
        if self.fix_frequency < 0.05:
            self.get_logger().warning('Fix frequency is too high, or out of bounds, constraining to 20Hz')
            self.fix_frequency = 0.05
        
        self.position_std_dev = self.get_parameter('heading_std_dev').get_parameter_value().double_value
        self.velocity_std_dev = self.get_parameter('rot_std_dev').get_parameter_value().double_value
        self.udp_ip = self.get_parameter('udp_ip').get_parameter_value().string_value
        self.udp_port = self.get_parameter('udp_port').get_parameter_value().integer_value

        # Setup callback group
        self.callback_group = ReentrantCallbackGroup()

        # Create timer for updating noise
        self.noise_timer = self.create_timer(0.1, self.update_noise_model, callback_group=self.callback_group)

        # Create timer for main callback
        self.timer = self.create_timer(1.0 / self.fix_frequency, self.timer_callback, callback_group=self.callback_group)

        # Create subscriptions for Eta and Nu messages
        self.eta_subscription = self.create_subscription(Eta, 'eta_sim', self.eta_callback, 10, callback_group=self.callback_group)
        self.nu_subscription = self.create_subscription(Nu, 'nu_sim', self.nu_callback, 10, callback_group=self.callback_group)
        
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
        self.heading_noise_state = np.random.normal(0, self.position_std_dev, 1)
        self.rot_noise_state = np.random.normal(0, self.rot_noise_state, 1)

    def timer_callback(self):

        # Make local copies to avoide the object chaning during routine
        local_latest_eta_msg = self.latest_eta_msg
        local_latest_nu_msg = self.latest_nu_msg

        if (local_latest_eta_msg is not None) and (local_latest_nu_msg is not None):
            
            # Generate and send HDT and ROT NMEA messages
            hdt_message = create_hdt_message(math.degrees(local_latest_eta_msg.psi) + self.heading_noise_state[0])
            rot_message = create_rot_message((math.degrees(local_latest_nu_msg.r) + self.rot_noise_state[0])/60.0)
            
            #self.get_logger().info(f'Sending HDT NMEA Message: "{hdt_message}"\nSending ROT NMEA Message: "{hdt_message}"')
            
            self.sock.sendto(hdt_message.encode(), (self.udp_ip, self.udp_port))
            self.sock.sendto(rot_message.encode(), (self.udp_ip, self.udp_port))

            local_latest_eta_msg = None
            local_latest_nu_msg = None

def main(args=None):
    rclpy.init(args=args)
    compass_simulator = CompassSimulator()
    executor = MultiThreadedExecutor()
    executor.add_node(compass_simulator)
    
    try:
        executor.spin()
    finally:
        compass_simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
