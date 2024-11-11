import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import socket
import json
from std_msgs.msg import String

class UDPListenerNode(Node):
    def __init__(self):
        super().__init__('udp_listener_node')
        
        # Create a UDP socket to listen for data from OpenCPN
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', 5005))  # Bind to all interfaces on port 5005

        # ROS2 Publisher to send the target point
        self.waypoint_pub = self.create_publisher(PointStamped, 'position_setpoint', 10)
        # Subscriber to listen to control mode changes from HMI (currently not used, position mode always active)
        # self.mode_sub = self.create_subscription(String, 'control_mode', self.mode_callback, 10)

        # Timer to periodically check for UDP data
        self.create_timer(0.1, self.receive_udp_data)
        
        self.position_mode_active = True  # Position mode always active for testing purposes
        self.get_logger().info("UDP Listener Node initialized, waiting for OpenCPN data...")

    def receive_udp_data(self):
        # Only proceed if "Position Mode" is active
        if not self.position_mode_active:
            return
        
        try:
            data, _ = self.sock.recvfrom(1024)  # Buffer size of 1024 bytes
            # Assume data is received in JSON format for simplicity
            waypoint = json.loads(data)

            # Extract latitude and longitude from the received data
            lat = waypoint['lat']
            lon = waypoint['lon']

            # Create a PointStamped message to publish the waypoint
            point_msg = PointStamped()
            point_msg.header.stamp = self.get_clock().now().to_msg()
            point_msg.header.frame_id = "map"  # Assuming a map frame
            point_msg.point.x = lat
            point_msg.point.y = lon
            point_msg.point.z = 0.0  # Assuming the vessel stays at sea level

            self.waypoint_pub.publish(point_msg)
            self.get_logger().info(f"Received and published waypoint: lat={lat}, lon={lon}")

        except socket.error:
            pass  # No data received, ignore


def main(args=None):
    rclpy.init(args=args)

    # Run the ROS2 UDP listener node
    udp_listener_node = UDPListenerNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(udp_listener_node)

    try:
        # Spin the node to keep it active
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        udp_listener_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
