import rclpy
from rclpy.node import Node
import numpy as np
import os
import yaml
from ngc_interfaces.msg import Eta, Nu, ButtonControl, Waypoint, GNSS, Route
from ament_index_python.packages import get_package_share_directory
from ngc_utils.qos_profiles import default_qos_profile

class Guide(Node):
    def __init__(self):
        super().__init__('guide')
        self.get_logger().info('Guide node is initialized.')

        # Load configuration parameters from YAML files
        self.declare_parameter('yaml_package_name', 'ngc_bringup')
        self.declare_parameter('simulation_config_file', 'config/simulator_config.yaml')

        yaml_package_name = self.get_parameter('yaml_package_name').get_parameter_value().string_value
        yaml_package_path = get_package_share_directory(yaml_package_name)
        simulation_config_path = os.path.join(yaml_package_path, self.get_parameter('simulation_config_file').get_parameter_value().string_value)

        self.simulation_config = self.load_yaml_file(simulation_config_path)
        self.step_size = self.simulation_config['simulation_settings']['step_size']

        # Subscribe to eta and nu setpoints from HMI
        self.eta_setpoint_HMI_sub = self.create_subscription(Eta, 'eta_setpoint_HMI', self.eta_setpoint_HMI_callback, default_qos_profile)
        self.nu_setpoint_HMI_sub = self.create_subscription(Nu, 'nu_setpoint_HMI', self.nu_setpoint_HMI_callback, default_qos_profile)
        
        # Subscribe to button mode messages from HMI
        self.button_control_sub = self.create_subscription(ButtonControl, 'button_control', self.button_control_callback, default_qos_profile)

        # Subscribe to target waypoint from waypoint_controller
        self.waypoint_sub = self.create_subscription(Waypoint, 'waypoint', self.waypoint_callback, default_qos_profile)
        # Subscribe to route from waypoint_controller
        self.route_sub = self.create_subscription(Route, 'route', self.route_callback, default_qos_profile)

        # Subscribe to current position from GNSS node
        self.gnss_sub = self.create_subscription(GNSS, 'gnss_measurement', self.gnss_callback, default_qos_profile)

        # Publishers for forwarding selected setpoints based on mode
        self.eta_setpoint_pub = self.create_publisher(Eta, "eta_setpoint", default_qos_profile) 
        self.nu_setpoint_pub = self.create_publisher(Nu, "nu_setpoint", default_qos_profile)

        # Initialize storage for setpoints and mode
        self.eta_setpoint = Eta()
        self.nu_setpoint = Nu()
        self.current_mode = None  # Will hold the current mode from button presses

        # Initialize variables for LOS calculation
        self.target_waypoint = None
        self.current_position = None

        # Initialize variables for Track mode
        self.route = []
        self.current_waypoint_index = 0
        self.route_received = False
        self.track_completed = False  # Flag to indicate route completion

        # Waypoint tolerance in meters
        self.waypoint_tolerance = 5.0  # Adjust as needed

        # Start control loop with the same timestep as the simulator
        self.timer = self.create_timer(self.step_size, self.step_control)

    def eta_setpoint_HMI_callback(self, msg):
        """Callback for eta_setpoint subscription from HMI."""
        self.eta_setpoint = msg
        self.get_logger().info(f"Received eta setpoint from HMI: psi = {msg.psi}")

    def nu_setpoint_HMI_callback(self, msg):
        """Callback for nu_setpoint subscription from HMI."""
        self.nu_setpoint = msg
        self.get_logger().info(f"Received nu setpoint from HMI: u = {msg.u}")

    def button_control_callback(self, msg):
        """Callback for button control messages indicating mode selection."""
        self.current_mode = msg.mode
        self.get_logger().info(f"Button control mode set to: {self.current_mode}")

        # Reset track variables when switching modes
        if self.current_mode != 3:
            self.current_waypoint_index = 0
            self.track_completed = False

    def waypoint_callback(self, msg):
        """Callback for receiving the target waypoint."""
        self.target_waypoint = msg
        self.get_logger().info(f"Received target waypoint: lat = {msg.latitude}, lon = {msg.longitude}")

    def route_callback(self, msg):
        """Callback for receiving the route (list of waypoints)."""
        self.route = msg.waypoints  # Assuming msg.waypoints is a list of Waypoint messages
        self.current_waypoint_index = 0  # Reset index when a new route is received
        self.route_received = True
        self.track_completed = False
        self.get_logger().info(f"Received new route with {len(self.route)} waypoints.")

    def gnss_callback(self, msg):
        """Callback for receiving the current GNSS position."""
        self.current_position = msg
        #self.get_logger().info(f"Current position: lat = {msg.lat}, lon = {msg.lon}")

    def calculate_los_setpoints(self):
        """Calculate heading and speed setpoints using LOS algorithm for Position mode."""
        if self.current_position is None or self.target_waypoint is None:
            self.get_logger().warning("Current position or target waypoint is not available.")
            return None, None

        # Get current and target positions
        current_lat = self.current_position.lat
        current_lon = self.current_position.lon
        target_lat = self.target_waypoint.latitude
        target_lon = self.target_waypoint.longitude

        # Calculate distance and bearing to the target
        distance, bearing = self.calculate_distance_and_bearing(current_lat, current_lon, target_lat, target_lon)

        # Check if the vessel has reached the waypoint
        if distance <= self.waypoint_tolerance:
            self.get_logger().info("Waypoint reached. Stopping vessel.")
            desired_speed = 0.0
            # Optionally switch mode or handle waypoint completion
        else:
            # Dynamic speed profile
            desired_speed = self.calculate_desired_speed(distance)

        return bearing, desired_speed

    def calculate_track_setpoints(self):
        """Calculate heading and speed setpoints to follow the route."""
        if not self.route_received or not self.current_position:
            self.get_logger().warning("Track Mode: Route or current position not available.")
            return None, None

        # Check if all waypoints have been reached
        if self.current_waypoint_index >= len(self.route):
            self.get_logger().info("Track Mode: All waypoints reached. Switching to position hold.")
            self.track_completed = True
            return None, None

        # Get the current waypoint
        target_waypoint = self.route[self.current_waypoint_index]

        # Calculate distance and bearing to the current waypoint
        current_lat = self.current_position.lat
        current_lon = self.current_position.lon
        target_lat = target_waypoint.latitude
        target_lon = target_waypoint.longitude

        distance, bearing = self.calculate_distance_and_bearing(
            current_lat, current_lon, target_lat, target_lon)

        # Check if we have reached the waypoint
        if distance <= self.waypoint_tolerance:
            self.get_logger().info(f"Track Mode: Waypoint {self.current_waypoint_index + 1} reached.")
            self.current_waypoint_index += 1  # Move to the next waypoint
            if self.current_waypoint_index >= len(self.route):
                self.get_logger().info("Track Mode: Final waypoint reached. Holding position.")
                self.track_completed = True
                return None, None
            else:
                # Update target_waypoint for the next iteration
                target_waypoint = self.route[self.current_waypoint_index]
                distance, bearing = self.calculate_distance_and_bearing(
                    current_lat, current_lon, target_waypoint.latitude, target_waypoint.longitude)

        # Calculate desired speed
        desired_speed = self.calculate_desired_speed(distance)

        return bearing, desired_speed

    def calculate_desired_speed(self, distance):
        """Calculate desired speed based on distance to waypoint."""
        max_speed = 2.0  # Maximum speed in m/s
        min_speed = 0.5  # Minimum speed in m/s
        # Simple linear reduction of speed when approaching waypoint
        desired_speed = max(min_speed, min(distance * 0.1, max_speed))
        return desired_speed

    def calculate_distance_and_bearing(self, lat1, lon1, lat2, lon2):
        """Calculate distance and bearing from current position to target."""
        # Convert to radians
        lat1_rad = np.deg2rad(lat1)
        lon1_rad = np.deg2rad(lon1)
        lat2_rad = np.deg2rad(lat2)
        lon2_rad = np.deg2rad(lon2)

        # Calculate differences
        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad

        # Haversine formula for distance
        a = np.sin(dlat/2)**2 + np.cos(lat1_rad) * np.cos(lat2_rad) * np.sin(dlon/2)**2
        c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
        earth_radius = 6371000  # in meters
        distance = earth_radius * c

        # Calculate bearing
        x = np.sin(dlon) * np.cos(lat2_rad)
        y = np.cos(lat1_rad) * np.sin(lat2_rad) - (np.sin(lat1_rad) * np.cos(lat2_rad) * np.cos(dlon))
        initial_bearing = np.arctan2(x, y)
        initial_bearing_deg = (np.rad2deg(initial_bearing) + 360) % 360

        return distance, initial_bearing_deg

    def step_control(self):
        """Control loop that runs based on the button mode from HMI."""
        if self.current_mode == 0:  # Standby
            self.get_logger().info("Standby: Stopping eta and nu setpoints.")
            # Optionally, set speed to zero or send stop command
            self.publish_zero_setpoints()

        elif self.current_mode == 1:  # Position
            # Calculate LOS setpoints and publish them
            desired_heading, desired_speed = self.calculate_los_setpoints()
            if desired_heading is not None and desired_speed is not None:
                self.publish_setpoints(desired_heading, desired_speed)
                self.get_logger().info(f"Position Mode: Publishing setpoints - Heading: {desired_heading:.2f} deg, Speed: {desired_speed:.2f} m/s")
            else:
                self.get_logger().warning("Position Mode: Unable to calculate setpoints due to missing data.")

        elif self.current_mode == 2:  # Sail
            # Forward eta and nu setpoints from HMI directly to control nodes
            self.eta_setpoint_pub.publish(self.eta_setpoint)
            self.nu_setpoint_pub.publish(self.nu_setpoint)
            self.get_logger().info("Sail: Forwarding eta and nu setpoints from HMI.")

        elif self.current_mode == 3:  # Track
            # Implement Track mode handling
            if self.track_completed:
                self.get_logger().info("Track Mode: Route completed. Holding position.")
                # Switch to Position mode to hold position at the last waypoint
                self.current_mode = 1  # Switch to Position mode
                self.target_waypoint = Waypoint()
                self.target_waypoint.latitude = self.current_position.lat
                self.target_waypoint.longitude = self.current_position.lon
            else:
                desired_heading, desired_speed = self.calculate_track_setpoints()
                if desired_heading is not None and desired_speed is not None:
                    self.publish_setpoints(desired_heading, desired_speed)
                    self.get_logger().info(f"Track Mode: Publishing setpoints - Heading: {desired_heading:.2f} deg, Speed: {desired_speed:.2f} m/s")
                else:
                    self.get_logger().warning("Track Mode: Unable to calculate setpoints due to missing data.")

    def publish_setpoints(self, heading_deg, speed_mps):
        """Helper method to publish setpoints."""
        # Create and publish eta_setpoint
        eta_msg = Eta()
        eta_msg.psi = np.deg2rad(heading_deg)  # Convert to radians
        self.eta_setpoint_pub.publish(eta_msg)

        # Create and publish nu_setpoint
        nu_msg = Nu()
        nu_msg.u = speed_mps  # Assume speed is in m/s
        self.nu_setpoint_pub.publish(nu_msg)

    def publish_zero_setpoints(self):
        """Helper method to publish zero setpoints."""
        eta_msg = Eta()
        eta_msg.psi = 0.0
        self.eta_setpoint_pub.publish(eta_msg)

        nu_msg = Nu()
        nu_msg.u = 0.0
        self.nu_setpoint_pub.publish(nu_msg)

    def load_yaml_file(self, file_path):
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)

def main(args=None):
    rclpy.init(args=args)
    node = Guide()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
