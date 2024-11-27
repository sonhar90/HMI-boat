import rclpy
from rclpy.node import Node
import numpy as np
import os
import yaml
from ngc_interfaces.msg import (
    Eta,
    Nu,
    ButtonControl,
    Waypoint,
    GNSS,
    Route,
    ThrusterSignals,
)
from ament_index_python.packages import get_package_share_directory
from ngc_utils.qos_profiles import default_qos_profile
from std_msgs.msg import Bool


class Guide(Node):
    def __init__(self):
        super().__init__("guide")
        self.get_logger().info("Guide node is initialized.")

        # Load configuration parameters from YAML files
        self.declare_parameter("yaml_package_name", "ngc_bringup")
        self.declare_parameter("simulation_config_file", "config/simulator_config.yaml")

        yaml_package_name = (
            self.get_parameter("yaml_package_name").get_parameter_value().string_value
        )
        yaml_package_path = get_package_share_directory(yaml_package_name)
        simulation_config_path = os.path.join(
            yaml_package_path,
            self.get_parameter("simulation_config_file")
            .get_parameter_value()
            .string_value,
        )

        self.simulation_config = self.load_yaml_file(simulation_config_path)
        self.step_size = self.simulation_config["simulation_settings"]["step_size"]

        # Subscribe to eta and nu setpoints from HMI
        self.eta_setpoint_HMI_sub = self.create_subscription(
            Eta, "eta_setpoint_HMI", self.eta_setpoint_HMI_callback, default_qos_profile
        )
        self.nu_setpoint_HMI_sub = self.create_subscription(
            Nu, "nu_setpoint_HMI", self.nu_setpoint_HMI_callback, default_qos_profile
        )

        # New publisher for stop command
        self.stop_command_pub = self.create_publisher(
            Bool, "stop_command", default_qos_profile
        )

        # Subscribe to button mode messages from HMI
        self.button_control_sub = self.create_subscription(
            ButtonControl,
            "button_control",
            self.button_control_callback,
            default_qos_profile,
        )

        # Subscribe to target waypoint from waypoint_controller
        self.waypoint_sub = self.create_subscription(
            Waypoint, "waypoint", self.waypoint_callback, default_qos_profile
        )

        # Subscribe to current position from GNSS node
        self.gnss_sub = self.create_subscription(
            GNSS, "gnss_measurement", self.gnss_callback, default_qos_profile
        )

        # Publishers for forwarding setpoints to control nodes
        ################self.thruster_signals_pub = self.create_publisher(ThrusterSignals, 'thruster_signals', default_qos_profile)
        # Publishers for forwarding selected setpoints based on mode
        self.eta_setpoint_pub = self.create_publisher(
            Eta, "eta_setpoint", default_qos_profile
        )
        self.nu_setpoint_pub = self.create_publisher(
            Nu, "nu_setpoint", default_qos_profile
        )

        # Initialize storage for setpoints and mode
        self.eta_setpoint = Eta()
        self.nu_setpoint = Nu()
        self.current_mode = None  # Will hold the current mode from button presses

        # Initialize variables for LOS calculation
        self.target_waypoint = None
        self.current_position = None

        # Initialize variables for Track mode
        self.route = []  # Store waypoints for track mode
        self.track_completed = False  # Flag to indicate route completion
        self.current_waypoint_index = 0  # Index of the current waypoint in the route

        # Waypoint tolerance in meters
        self.waypoint_tolerance = 8.0  # Adjust as needed

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
        previous_mode = self.current_mode
        self.current_mode = msg.mode
        self.get_logger().info(f"Button control mode set to: {self.current_mode}")

        # Reset track variables when switching modes
        if self.current_mode != previous_mode:
            if self.current_mode == 3:  # Entering Track Mode
                self.track_completed = False
            else:
                # Reset track variables when leaving Track Mode
                self.track_completed = False
                self.current_waypoint_index = 0

    def waypoint_callback(self, msg):
        """Callback for receiving waypoints."""
        # Append waypoint to the route for track mode
        if msg.name == "001":
            self.route = []
        self.route.append(msg)
        # Update target waypoint for position mode
        self.target_waypoint = msg
        self.get_logger().info(
            f"Added waypoint to route: lat = {msg.latitude}, lon = {msg.longitude}, name = {msg.name}"
        )

    def gnss_callback(self, msg):
        """Callback for receiving the current GNSS position."""
        self.current_position = msg

    def calculate_los_setpoints(self):
        """Calculate heading and speed setpoints using LOS algorithm for Position mode."""
        if self.current_position is None or self.target_waypoint is None:
            self.get_logger().warning(
                "Current position or target waypoint is not available."
            )
            return None, None

        # Get current and target positions
        current_lat = self.current_position.lat
        current_lon = self.current_position.lon
        target_lat = self.target_waypoint.latitude
        target_lon = self.target_waypoint.longitude

        # Calculate distance and bearing to the target
        distance, bearing = self.calculate_distance_and_bearing(
            current_lat, current_lon, target_lat, target_lon
        )

        # Check if the vessel is within the desired distance from the waypoint
        if distance <= self.waypoint_tolerance:
            self.get_logger().info(
                "Within desired distance from waypoint. Holding position."
            )
            return bearing, 0.0
        else:
            # Dynamic speed profile
            desired_speed = self.calculate_desired_speed(
                distance - self.waypoint_tolerance
            )
            return bearing, desired_speed

    def calculate_track_setpoints(self):
        """Calculate heading and speed setpoints for Track mode."""
        if not self.current_position or self.current_waypoint_index >= len(self.route):
            self.get_logger().info(
                "Track Mode: Route completed or position unavailable."
            )
            self.track_completed = True
            return None, None

        # Navigate to the current waypoint
        target_waypoint = self.route[self.current_waypoint_index]
        current_lat = self.current_position.lat
        current_lon = self.current_position.lon
        target_lat = target_waypoint.latitude
        target_lon = target_waypoint.longitude

        distance, bearing = self.calculate_distance_and_bearing(
            current_lat, current_lon, target_lat, target_lon
        )

        if distance <= self.waypoint_tolerance:
            self.get_logger().info(
                f"Reached waypoint {self.current_waypoint_index + 1}."
            )
            self.current_waypoint_index += 1  # Move to the next waypoint
            return self.calculate_track_setpoints()  # Recalculate for the next waypoint
        else:
            desired_speed = self.calculate_desired_speed(distance)
            return bearing, desired_speed

    def calculate_desired_speed(self, distance):
        """Calculate desired speed based on distance to waypoint."""
        max_speed = 0.5  # Maximum speed in m/s
        min_speed = 0.1  # Minimum speed in m/s
        return max(min_speed, min(distance * 0.1, max_speed))

    def calculate_distance_and_bearing(self, lat1, lon1, lat2, lon2):
        """Calculate distance and bearing from current position to target."""
        # Convert to radians
        lat1_rad = np.deg2rad(lat1)
        lon1_rad = np.deg2rad(lon1)
        lat2_rad = np.deg2rad(lat2)
        lon2_rad = np.deg2rad(lon2)

        # Haversine formula
        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad
        a = (
            np.sin(dlat / 2) ** 2
            + np.cos(lat1_rad) * np.cos(lat2_rad) * np.sin(dlon / 2) ** 2
        )
        c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
        distance = 6371000 * c  # Earth's radius in meters

        # Bearing calculation
        x = np.sin(dlon) * np.cos(lat2_rad)
        y = np.cos(lat1_rad) * np.sin(lat2_rad) - np.sin(lat1_rad) * np.cos(
            lat2_rad
        ) * np.cos(dlon)
        initial_bearing = np.arctan2(x, y)
        return distance, (np.rad2deg(initial_bearing) + 360) % 360

    def step_control(self):
        """Control loop that runs based on the button mode from HMI."""
        if self.current_mode == 0:  # Standby
            self.publish_zero_setpoints()
            stop_msg = Bool()
            stop_msg.data = True
            self.stop_command_pub.publish(stop_msg)

        elif self.current_mode == 1:  # Position
            # Calculate LOS setpoints and publish them
            desired_heading, desired_speed = self.calculate_los_setpoints()
            stop_msg = Bool()
            stop_msg.data = False  # Ensure stop command is disabled
            self.stop_command_pub.publish(stop_msg)
            if desired_heading is not None and desired_speed is not None:
                self.publish_setpoints(desired_heading, desired_speed)
                self.get_logger().info(
                    f"Position Mode: Publishing setpoints - Heading: {desired_heading:.2f} deg, Speed: {desired_speed:.2f} m/s"
                )
            else:
                self.get_logger().warning(
                    "Position Mode: Unable to calculate setpoints due to missing data."
                )

        elif self.current_mode == 2:  # Sail
            # Forward eta and nu setpoints from HMI directly to control nodes
            stop_msg = Bool()
            stop_msg.data = False  # Ensure stop command is disabled
            self.stop_command_pub.publish(stop_msg)
            self.eta_setpoint_pub.publish(self.eta_setpoint)
            self.nu_setpoint_pub.publish(self.nu_setpoint)
            self.get_logger().info("Sail: Forwarding eta and nu setpoints from HMI.")

        elif self.current_mode == 3:  # Track
            # Implement Track mode handling
            stop_msg = Bool()
            stop_msg.data = False  # Ensure stop command is disabled
            self.stop_command_pub.publish(stop_msg)
            if self.track_completed:
                self.publish_zero_setpoints()
                # feil, skal ha kopi av mode 1
            else:
                heading, speed = self.calculate_track_setpoints()
                if heading is not None and speed is not None:
                    self.publish_setpoints(heading, speed)

    def publish_setpoints(self, heading_deg, speed_mps):
        eta_msg = Eta()
        eta_msg.psi = np.deg2rad(heading_deg)
        self.eta_setpoint_pub.publish(eta_msg)

        nu_msg = Nu()
        nu_msg.u = speed_mps
        self.nu_setpoint_pub.publish(nu_msg)

    def publish_zero_setpoints(self):
        self.publish_setpoints(0.0, 0.0)

    def load_yaml_file(self, file_path):
        with open(file_path, "r") as file:
            return yaml.safe_load(file)


def main(args=None):
    rclpy.init(args=args)
    node = Guide()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
