import rclpy
from rclpy.node import Node
import socket
from ngc_interfaces.msg import Waypoint, Route

class WaypointListener(Node):
    def __init__(self):
        super().__init__('waypoint_listener')
        self.declare_parameter('udp_port', 55556)  # Adjust the port as necessary
        self.port = self.get_parameter('udp_port').value

        # Create a UDP socket to listen to NMEA0183 messages
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', self.port))
        self.get_logger().info(f'Listening for NMEA data on 127.0.0.1:{self.port}')

        # Variables for filtering duplicate messages and handling routes
        self.last_message = None   # Store the last message to filter duplicates
        self.route_name = None     # Store the current route name
        self.route_waypoints = []  # Store the waypoints for the current route
        self.total_parts = 0       # Total number of route segments
        self.current_part = 0      # Current part number

        # Create publishers for waypoint and route messages
        self.waypoint_publisher = self.create_publisher(Waypoint, 'waypoint', 10)
        self.route_publisher = self.create_publisher(Route, 'route', 10)

    def receive_nmea_messages(self):
        """ Continuously receive and process NMEA messages from the socket """
        while rclpy.ok():
            data, _ = self.sock.recvfrom(4096)  # Buffer size of 4096 bytes
            message = data.decode('ascii').strip()

            # Filter out duplicate messages
            if message == self.last_message:
                continue  # Skip if it's a duplicate message

            self.last_message = message
            self.get_logger().info(f"Received message: {message}")
            self.process_nmea_message(message)

    def process_nmea_message(self, message):
        """ Process the received NMEA message """
        if message.startswith('$ECWPL'):
            self.get_logger().info(f'ECWPL Waypoint received: {message}')
            self.parse_ecwpl(message)
        elif message.startswith('$ECRTE'):
            self.get_logger().info(f'ECRTE Route received: {message}')
            self.parse_ecrte(message)

    def parse_ecwpl(self, message):
    
        fields = message.split(',')
        if len(fields) >= 6:
            lat_raw = fields[1]  # Raw latitude (NMEA format)
            lat_dir = fields[2]  # Latitude direction (N or S)
            lon_raw = fields[3]  # Raw longitude (NMEA format)
            lon_dir = fields[4]  # Longitude direction (E or W)
            waypoint_name = fields[5].split('*')[0]

            # Convert raw NMEA latitude (DDMM.MMMM) to decimal degrees
            lat_deg = int(lat_raw[:2])  # First two digits are degrees
            lat_min = float(lat_raw[2:])  # The rest are minutes
            lat = lat_deg + (lat_min / 60.0)  # Decimal degrees conversion

            # Convert raw NMEA longitude (DDDMM.MMMM) to decimal degrees
            lon_deg = int(lon_raw[:3])  # First three digits are degrees
            lon_min = float(lon_raw[3:])  # The rest are minutes
            lon = lon_deg + (lon_min / 60.0)  # Decimal degrees conversion

            # Convert to negative values if in the southern or western hemispheres
            if lat_dir == 'S':
                lat = -lat
            if lon_dir == 'W':
                lon = -lon

            self.get_logger().info(f'Parsed waypoint: {lat} {lat_dir}, {lon} {lon_dir} (Name: {waypoint_name})')

            # Publish the waypoint message
            waypoint_msg = Waypoint()
            waypoint_msg.latitude = lat  # Decimal degrees
            waypoint_msg.longitude = lon  # Decimal degrees
            waypoint_msg.name = waypoint_name
            self.waypoint_publisher.publish(waypoint_msg)

    def parse_ecrte(self, message):
        """ Parse the ECRTE (Route) message, reconstruct the route if split into multiple parts """
        fields = message.split(',')

        if len(fields) >= 5:
            current_part = int(fields[1])  # Current part of the route message
            total_parts = int(fields[2])   # Total number of parts
            route_name = fields[4]         # Route name
            
            # Handle the waypoints and the last waypoint with the checksum
            waypoints = fields[5:-1]  # Extract all waypoints except the last one
            
            # Now handle the last waypoint, which includes the checksum
            last_waypoint_with_checksum = fields[-1]
            last_waypoint, _ = last_waypoint_with_checksum.split('*')  # Split by '*' to separate the checksum

            waypoints.append(last_waypoint)  # Add the last waypoint to the list

            # If this is the first part of the route (or a new route), reset the current route data
            if self.route_name != route_name or current_part == 1:
                self.route_name = route_name
                self.route_waypoints = []  # Reset waypoint list for the new route
                self.total_parts = total_parts
                self.current_part = 0

            # Ensure all waypoints in this segment are added without overwriting previous segments
            self.route_waypoints.extend(waypoints)
            self.current_part = current_part

            self.get_logger().info(f'Parsed route segment {current_part}/{total_parts} for {route_name} with waypoints: {waypoints}')

            # If we've received all parts, process the full route
            if current_part == total_parts:
                self.process_full_route()

    def process_full_route(self):
        """ Handle the full route after all parts have been received """
        self.get_logger().info(f'Full route "{self.route_name}" received with waypoints: {self.route_waypoints}')
        
        # Create and publish the Route message
        route_msg = Route()
        route_msg.route_name = self.route_name

        for wp in self.route_waypoints:
            waypoint_msg = Waypoint()
            waypoint_msg.name = wp
            # You'll need to provide lat/lon for each waypoint, use some lookup method or extend ECRTE for lat/lon
            # Dummy values used here as lat/lon are not directly available in ECRTE
            waypoint_msg.latitude = 0.0  # Dummy value, adjust this
            waypoint_msg.longitude = 0.0  # Dummy value, adjust this
            route_msg.waypoints.append(waypoint_msg)

        self.route_publisher.publish(route_msg)

        # Reset the route data for the next route
        self.route_name = None
        self.route_waypoints = []
        self.total_parts = 0
        self.current_part = 0


def main(args=None):
    rclpy.init(args=args)
    node = WaypointListener()

    try:
        node.receive_nmea_messages()
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()