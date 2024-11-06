# hmi.py
import rclpy
from rclpy.node import Node
from ngc_interfaces.msg import Eta, Nu
from ngc_utils.qos_profiles import default_qos_profile

class HMI(Node):
    def __init__(self):
        super().__init__('hmi')
        
        # Log at node startup
        self.get_logger().info("HMI ROS node started successfully.")
        
        # Set up ROS2 publishers and subscribers
        self.eta_publisher = self.create_publisher(Eta, 'eta_setpoint', default_qos_profile)
        self.nu_publisher = self.create_publisher(Nu, 'nu_setpoint', default_qos_profile)
        self.create_subscription(Eta, 'eta_sim', self.update_eta_feedback, default_qos_profile)
        self.create_subscription(Nu, 'nu_sim', self.update_nu_feedback, default_qos_profile)

    def update_eta_feedback(self, msg):
    # Logging er fjernet for å unngå utskrift
        pass

    def update_nu_feedback(self, msg):
        # Logging er fjernet for å unngå utskrift
        pass


def main():
    rclpy.init()
    
    # Start ROS-node
    ros_node = HMI()
    
    # Log that the node initialization is complete
    ros_node.get_logger().info("HMI ROS node main loop started.")
    
    rclpy.spin(ros_node)
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
