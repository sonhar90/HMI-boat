import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from ngc_interfaces.msg import (
    ThrusterSignals,
    SystemMode,
    GNSS,
    HeadingDevice,
    OtterStatus,
)
from ngc_utils.qos_profiles import default_qos_profile
import time
import socket
import select
import numpy as np
from numpy import pi
from copy import copy
import ngc_utils.math_utils as mu
import csv
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from scipy.interpolate import griddata
from ngc_utils.nmea_utils import (
    create_gga_message,
    create_vtg_message,
    create_hdt_message,
    create_rot_message,
    create_sog_cog_vtg_message,
)


def checksum(message):
    checksum = 0
    for character in message:
        checksum ^= ord(character)
    checksum = hex(checksum)
    checksum = checksum[2:]
    if len(checksum) == 1:
        checksum = "0" + checksum
    return checksum


# Main class for connecting to the Otter.
class otter_connector:
    def __init__(self, ros_node):

        # Reference to the ros2 node for printing info, warnings and errors
        self.node = ros_node

        # This enables the printing of messages. Used for debugging. Slows down the software a bit.
        self.verbose = self.node.simulation_config["otter_interface"][
            "connection_debug_messages"
        ]

        # Keeping track of the connection status
        self.connection_status = False

        # Stores the last message received from the Otter
        self.last_message_received = ""

        # Variables
        self.current_position = [0.0, 0.0, 0.0]
        self.previous_position = [0.0, 0.0, 0.0]
        self.last_speed_update = time.time()
        self.current_course_over_ground = 0.0
        self.current_speed = 0.0
        self.current_fuel_capacity = "0.0"
        self.current_orientation = [0.0, 0.0, 0.0]  # roll, pitch, yaw
        self.current_rotational_velocities = [0.0, 0.0, 0.0]  # roll, pitch, yaw
        self.current_mode = "-1"
        self.rpm_port = 0.0
        self.rpm_strb = 0.0
        self.power_port = 0.0
        self.power_strb = 0.0

    # Establishes a socket communication to the Otter with ip and port. Default ip and port is for Wifi connection. Pass other parameters to alter these.
    def establish_connection(self, ip, port):

        try:
            self.node.get_logger().info(f"Connecting with ip {ip} and port {port}")

            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((ip, port))

            self.connection_status = True
            self.node.get_logger().info("Connected to Otter")

            return True
        except:
            self.node.get_logger().info("Could not connect to Otter")
            return False

    # Sends a message through the socket connection to the Otter. Calculates checksum and adds \r\n to the message.     ---CHECKSUM IS NMEA STANDARD---
    def send_message(self, message, checksum_needed):
        try:
            if checksum_needed:
                if self.verbose:
                    self.node.get_logger().info("Adding checksum")

                message += "*"
                message += checksum(message[1:-1]).upper()

            message += "\r\n"
            self.sock.sendall(message.encode())

            # self.node.get_logger().info(f"Sending message: {message}")

            if self.verbose:
                self.node.get_logger().info(f"Sending message: {message}")
                self.node.get_logger().info("Message sendt OK")

            return True

        except:
            self.node.get_logger().info("Couldn't send message to Otter")
            return False

    # Closes the socket connection.
    def close_connection(self):
        try:
            self.sock.close()
            self.connection_status = False
            return True

        except:
            self.node.get_logger().info("Error when disconnecting from Otter")
            return False

    # Checks if a socket connection is established and returns a boolean.
    def check_connection(self):
        if self.verbose:
            self.node.get_logger().info(
                f"Connection status is {self.connection_status}"
            )

        return self.connection_status

    # Reads a message from the Otter and returns it and stores it in "last_message_recieved".
    def read_message(self, timeout=10):
        if self.verbose:
            self.node.get_logger().info("Listening to message from Otter")

        self.sock.setblocking(0)
        ready = select.select([self.sock], [], [], timeout)

        if ready[0]:
            try:
                received_message = self.sock.recv(1024).decode()
                self.last_message_received = received_message
                return received_message
            except:
                if self.verbose:
                    self.node.get_logger().info(
                        "Error in recieving message. This is going to fast"
                    )

        else:
            return None

    # Updates all the values for the Otter with the messages that are sendt from the Otter. This needs to be called every time the values should be updated. Timeout for the read message is by default 10, but can be changed as an argument.
    def update_values(self, timeout=10):

        self.previous_position = copy(self.current_position)
        msg = self.read_message(timeout)

        if msg is None:
            self.node.get_logger().info("No message received from Otter")
            self.node.get_logger().info("Check communication")
            return

        list = msg.split()

        # We skip the last one because it is usually incomplete
        list = list[:-1]

        # Get the newest messages
        gps_message = ""
        imu_message = ""
        mod_message = ""
        status_message = ""

        for message in list:
            if message[:8] == "$PMARGPS":
                gps_message = message
            elif message[:8] == "$PMARIMU":
                imu_message = message
            elif message[:8] == "$PMARMOD":
                mod_message = message
            elif message[:8] == "$PMAROTR":
                status_message = message
            elif message[:8] == "$PMARERR":
                error_message = message
                self.node.get_logger().warning(error_message)

            # self.node.get_logger().info(f"{message}")

        # GNSS
        if gps_message != "":

            # self.node.get_logger().info(gps_message)

            if checksum(gps_message[1:-3]) != gps_message[-2:].lower():
                self.node.get_logger().warning("Checksum error in $PMARGPS message")

            else:
                gps_message = gps_message.split("*")[0]  # Removing checksum
                gps_message = gps_message.split(",")

                # Update position
                lat_msg = gps_message[2]
                lon_msg = gps_message[4]
                lat_deg = lat_msg[:2]
                lon_deg = lon_msg[:3]
                lat_min = lat_msg[2:]
                lon_min = lon_msg[3:]

                if gps_message[2] != "" and gps_message[4] != "":

                    try:
                        lat = float(lat_deg) + ((float(lat_min) / 100) / 0.6)
                        lon = float(lon_deg) + ((float(lon_min) / 100) / 0.6)
                    except:
                        if self.verbose:
                            self.node.get_logger().info(
                                "Could not get GPS coordintes. Check GPS coverage!"
                            )
                        lat = 0
                        lon = 0

                    if gps_message[3] == "S":
                        lat *= -1
                    if gps_message[5] == "W":
                        lon *= -1

                    # Creates current position. Height is set as 0.0 as this is not implemented yet.
                    self.current_position = [lat, lon, 0.0]

                    # Update speed
                    if gps_message[7] != "":
                        self.speed_over_ground = float(gps_message[7])
                        self.last_speed_update = time.time()

                    # Update course over ground
                    if gps_message[8] != "":
                        self.current_course_over_ground = float(gps_message[8])

        if imu_message != "":

            # self.node.get_logger().info(imu_message)

            if checksum(imu_message[1:-3]) != imu_message[-2:].lower():
                self.node.get_logger().info("Checksum error in $PMARIMU message")
            else:
                # Update orientation
                imu_message = imu_message.split("*")[0]  # Removing checksum
                imu_message = imu_message.split(",")

                if imu_message[1] != "":
                    self.current_orientation[0] = float(imu_message[1])
                if imu_message[2] != "":
                    self.current_orientation[1] = float(imu_message[2])
                if imu_message[3] != "":
                    self.current_orientation[2] = float(imu_message[3])

                # Update rotational velocities
                if imu_message[4] != "":
                    self.current_rotational_velocities[0] = float(imu_message[4])
                if imu_message[5] != "":
                    self.current_rotational_velocities[1] = float(imu_message[5])
                if imu_message[6] != "":
                    self.current_rotational_velocities[2] = float(imu_message[6])

        if mod_message != "":

            # self.node.get_logger().info(mod_message)

            if checksum(mod_message[1:-3]) != mod_message[-2:].lower():
                self.node.get_logger().info("Checksum error in $PMARMOD message")
            else:
                # Update fuel capacity
                mod_message = mod_message.split("*")[0]  # Removing checksum
                mod_message = mod_message.split(",")

                self.current_mode = mod_message[1]
                self.current_fuel_capacity = mod_message[2]

        if status_message != "":

            # self.node.get_logger().info(status_message)

            if checksum(status_message[1:-3]) != status_message[-2:].lower():
                self.node.get_logger().info("Checksum error in $PMAROTR message")
            else:
                status_message = status_message.split("*")[0]  # Removing checksum
                status_message = status_message.split(",")

                if status_message[1] != "":
                    self.rpm_port = float(status_message[1])
                if status_message[2] != "":
                    self.rpm_strb = float(status_message[2])
                if status_message[5] != "":
                    self.power_port = float(status_message[5])
                if status_message[6] != "":
                    self.power_strb = float(status_message[6])

        return

    # Sets the Otter in drift mode with zero trust
    def set_drift_mode(self):

        if self.verbose:
            self.node.get_logger().info("Otter entering drift mode")

        return self.send_message("$PMARABT", False)

    # Sets the Otter in manual mode with specific forces and torques - this message must be repeated every 3 seconds or else the otter will enter drift mode.
    def set_manual_control_mode(self, force_x, force_y, torque_z):

        # Check if the connection is still active before sending the message
        if not self.check_connection():

            self.node.get_logger().warning(
                "Connection lost. Attempting to reconnect..."
            )
            connected = self.establish_connection(
                self.node.simulation_config["otter_interface"]["ip"],
                self.node.simulation_config["otter_interface"]["port"],
            )

            if not connected:
                self.node.get_logger().error(
                    "Reconnection failed. Skipping command send."
                )
                return False

        # Force y is fixed at 0.0 according to the manual
        force_y = 0.0

        # Format the force and torque values
        force_x_str = f"{force_x:.4f}"
        force_y_str = f"{force_y:.4f}"
        torque_z_str = f"{torque_z:.4f}"

        # Create the message according to the manual specification
        message_to_send = f"$PMARMAN,{force_x_str},{force_y_str},{torque_z_str}"

        if self.verbose:
            self.node.get_logger().info(
                f"Sending manual control message: {message_to_send}"
            )

        # Send the message with checksum calculation
        send_success = self.send_message(message_to_send, checksum_needed=True)

        if send_success:
            return True
        else:
            self.node.get_logger().error("Failed to send manual control message")
            return False


class OtterUSVNode(Node):
    def __init__(self):
        super().__init__("otter_usv_node")

        self.declare_parameter("yaml_package_name", "ngc_bringup")
        self.declare_parameter(
            "propulsion_config_file", "config/propulsion_config.yaml"
        )
        self.declare_parameter("simulation_config_file", "config/simulator_config.yaml")

        # Henter konfigurasjonsfilenes stier ved hjelp av pakkenavnet
        yaml_package_name = (
            self.get_parameter("yaml_package_name").get_parameter_value().string_value
        )
        yaml_package_path = get_package_share_directory(yaml_package_name)
        propulsion_config_path = os.path.join(
            yaml_package_path,
            self.get_parameter("propulsion_config_file")
            .get_parameter_value()
            .string_value,
        )
        simulation_config_path = os.path.join(
            yaml_package_path,
            self.get_parameter("simulation_config_file")
            .get_parameter_value()
            .string_value,
        )

        # Laster YAML-konfigurasjonsfiler som inneholder informasjon om fartøyet og simuleringen
        self.propulsion_config = self.load_yaml_file(propulsion_config_path)
        self.simulation_config = self.load_yaml_file(simulation_config_path)

        self.simulation = self.simulation_config["simulator_in_the_loop"]

        # Thrusters
        self.thruster_1_sub = self.create_subscription(
            ThrusterSignals,
            "thruster_1_setpoints",
            self.thruster_1_setpoints_callback,
            default_qos_profile,
        )
        self.thruster_2_sub = self.create_subscription(
            ThrusterSignals,
            "thruster_2_setpoints",
            self.thruster_2_setpoints_callback,
            default_qos_profile,
        )
        self.thruster_1_pub = self.create_publisher(
            ThrusterSignals, "thruster_1_feedback", default_qos_profile
        )
        self.thruster_2_pub = self.create_publisher(
            ThrusterSignals, "thruster_2_feedback", default_qos_profile
        )

        # Sensors
        self.gnss_pub = self.create_publisher(
            GNSS, "gnss_measurement", default_qos_profile
        )
        self.heading_pub = self.create_publisher(
            HeadingDevice, "heading_measurement", default_qos_profile
        )

        # Mode
        self.system_mode_sub = self.create_subscription(
            SystemMode, "system_mode", self.system_mode_callback, default_qos_profile
        )

        # Subscribers for simulated data if in simulation mode
        if self.simulation:

            # Thrusters
            self.thruster_1_sim_sub = self.create_subscription(
                ThrusterSignals,
                "thruster_1_feedback_sim",
                self.thruster_1_sim_callback,
                default_qos_profile,
            )
            self.thruster_2_sim_sub = self.create_subscription(
                ThrusterSignals,
                "thruster_2_feedback_sim",
                self.thruster_2_sim_callback,
                default_qos_profile,
            )
            self.thruster_1_sim_pub = self.create_publisher(
                ThrusterSignals, "thruster_1_setpoints_sim", default_qos_profile
            )
            self.thruster_2_sim_pub = self.create_publisher(
                ThrusterSignals, "thruster_2_setpoints_sim", default_qos_profile
            )

            # Sensors
            self.sim_gnss_sub = self.create_subscription(
                GNSS,
                "gnss_measurement_sim",
                self.gnss_sim_callback,
                default_qos_profile,
            )
            self.sim_heading_sub = self.create_subscription(
                HeadingDevice,
                "heading_measurement_sim",
                self.heading_sim_callback,
                default_qos_profile,
            )

        else:
            self.otter = otter_connector(self)
            self.otter_status_pub = self.create_publisher(
                OtterStatus, "otter_status", default_qos_profile
            )

            package_share_directory = get_package_share_directory("ngc_otter_interface")
            csv_file_path = os.path.join(
                package_share_directory, "resource", "parsed_data.csv"
            )

            self.get_logger().info(f"File path to otter data: {csv_file_path}")

            self.data = self.load_parsed_data(csv_file_path)

            self.get_logger().info(f"1")

            self.rpm_values = np.array(
                [(entry[0], entry[1]) for entry in self.data]
            )  # Starboard and Port RPM pairs
            self.Fx_values = np.array(
                [entry[2] for entry in self.data]
            )  # Corresponding F_x
            self.Mz_values = np.array(
                [entry[3] for entry in self.data]
            )  # Corresponding M_z

            # CSV file for logging
            self.log_file = open("otter_status_log.csv", "a", newline="")
            self.csv_writer = csv.writer(self.log_file)

            self.get_logger().info(f"2")

            # Check if file is empty to write headers
            if (
                os.stat("otter_status_log.csv").st_size == 0
                and self.simulation_config["otter_interface"]["log_measurements_to_csv"]
            ):
                self.csv_writer.writerow(
                    [
                        "time",
                        "latitude",
                        "longitude",
                        "heading",
                        "rate of turn",
                        "sog",
                        "cog",
                        "rpm_port_setpoint",
                        "rpm_port_fb",
                        "rpm_stb_setpoint",
                        "rpm_stb_fb",
                        "current_mode",
                        "current_fuel_capacity",
                        "fx",
                        "fz",
                    ]
                )

            self.sock_nmea = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            self.get_logger().info(f"3")

        # Timer to publish data at 10Hz
        self.timer = self.create_timer(0.1, self.publish_measurements)

        # Store the latest messages from simulator
        self.latest_thruster_1_setpoints = None
        self.latest_thruster_2_setpoints = None
        self.latest_thruster_1_feedback = None
        self.latest_thruster_2_feedback = None
        self.latest_system_mode = None
        self.latest_gnss_data = None
        self.latest_heading_data = None

        # For testing
        self.fx = 0.0
        self.fz = 0.0

    # Funksjon for å laste inn YAML-konfigurasjonsfiler
    def load_yaml_file(self, file_path):

        with open(file_path, "r") as file:
            return yaml.safe_load(file)

    def thruster_1_setpoints_callback(self, msg):
        self.latest_thruster_1_setpoints = msg

    def thruster_2_setpoints_callback(self, msg):
        self.latest_thruster_2_setpoints = msg

    def system_mode_callback(self, msg):
        self.latest_system_mode = msg

    def thruster_1_sim_callback(self, msg):
        self.latest_thruster_1_feedback = msg

    def thruster_2_sim_callback(self, msg):
        self.latest_thruster_2_feedback = msg

    def gnss_sim_callback(self, msg):
        self.latest_gnss_data = msg

    def heading_sim_callback(self, msg):
        self.latest_heading_data = msg

    def load_parsed_data(self, csv_file):
        data = []
        with open(csv_file, newline="") as csvfile:
            reader = csv.reader(csvfile)
            next(reader)  # Skip the header
            for row in reader:
                F_x = float(row[0])
                M_z = float(row[1])
                rpm_starboard = float(row[2])
                rpm_port = float(row[3])
                data.append((rpm_starboard, rpm_port, F_x, M_z))
        return data

    def map_rpm_to_force_moment(self, rpm_starboard, rpm_port):

        rpm_starboard = round(rpm_starboard)
        rpm_port = round(rpm_port)

        # Saturate the input RPMs within the bounds of the dataset
        rpm_starboard = np.clip(
            rpm_starboard, self.rpm_values[:, 0].min(), self.rpm_values[:, 0].max()
        )
        rpm_port = np.clip(
            rpm_port, self.rpm_values[:, 1].min(), self.rpm_values[:, 1].max()
        )

        # Interpolate Fx and Mz for the given RPM_starboard and RPM_port values
        fx = griddata(
            self.rpm_values, self.Fx_values, (rpm_starboard, rpm_port), method="linear"
        )
        mz = griddata(
            self.rpm_values, self.Mz_values, (rpm_starboard, rpm_port), method="linear"
        )

        if np.isnan(fx) or np.isnan(mz):
            fx = griddata(
                self.rpm_values,
                self.Fx_values,
                (rpm_starboard, rpm_port),
                method="nearest",
            )
            mz = griddata(
                self.rpm_values,
                self.Mz_values,
                (rpm_starboard, rpm_port),
                method="nearest",
            )

        return np.round(fx, 4), np.round(mz, 4)

    def publish_measurements(self):

        if self.latest_system_mode is None:
            return

        if self.simulation == False:

            if self.otter.check_connection() == False:

                connected = self.otter.establish_connection(
                    self.simulation_config["otter_interface"]["ip"],
                    self.simulation_config["otter_interface"]["port"],
                )

                if not connected:
                    self.get_logger().error("Reconnection failed. Retrying next cycle.")
                    return

            # Update Otter values
            self.otter.update_values()

            # Publishing the extended GNSS data
            if self.latest_gnss_data is None:
                self.latest_gnss_data = GNSS()

            self.latest_gnss_data.lat = self.otter.current_position[0]
            self.latest_gnss_data.lon = self.otter.current_position[1]
            self.latest_gnss_data.sog = self.otter.current_speed
            self.latest_gnss_data.cog = self.otter.current_course_over_ground
            self.latest_gnss_data.valid_signal = True

            # Publishing the extended HeadingDevice data
            if self.latest_heading_data is None:
                self.latest_heading_data = HeadingDevice()

            self.latest_heading_data.heading = np.deg2rad(
                float(self.otter.current_orientation[2])
            )
            self.latest_heading_data.rot = np.deg2rad(
                float(self.otter.current_rotational_velocities[2])
            )
            self.latest_heading_data.valid_signal = True

            if self.latest_thruster_1_feedback is None:
                self.latest_thruster_1_feedback = ThrusterSignals()

            self.latest_thruster_1_feedback.thruster_id = 1
            self.latest_thruster_1_feedback.rps = self.otter.rpm_port / 60.0
            self.latest_thruster_1_feedback.pitch = 0.0
            self.latest_thruster_1_feedback.azimuth_deg = 0.0
            self.latest_thruster_1_feedback.active = True
            self.latest_thruster_1_feedback.error = False

            if self.latest_thruster_2_feedback is None:
                self.latest_thruster_2_feedback = ThrusterSignals()

            self.latest_thruster_2_feedback.thruster_id = 2
            self.latest_thruster_2_feedback.rps = self.otter.rpm_strb / 60.0
            self.latest_thruster_2_feedback.pitch = 0.0
            self.latest_thruster_2_feedback.azimuth_deg = 0.0
            self.latest_thruster_2_feedback.active = True
            self.latest_thruster_2_feedback.error = False

            otter_status_msg = OtterStatus()
            otter_status_msg.rpm_port = self.otter.rpm_port
            otter_status_msg.rpm_stb = self.otter.rpm_strb
            otter_status_msg.power_port = self.otter.power_port
            otter_status_msg.power_stb = self.otter.power_strb
            otter_status_msg.current_mode = self.otter.current_mode
            otter_status_msg.current_fuel_capacity = self.otter.current_fuel_capacity
            otter_status_msg.normalized_fx = self.fx
            otter_status_msg.normalized_mz = self.fz

            # Log the otter status and control forces to CSV
            if self.simulation_config["otter_interface"]["log_measurements_to_csv"]:

                rpm_port_setpoint = 0.0
                rpm_strb_setpoint = 0.0

                if (
                    self.latest_thruster_1_setpoints is not None
                    and self.latest_thruster_2_setpoints is not None
                ):
                    rpm_port_setpoint = self.latest_thruster_1_setpoints.rps * 60.0
                    rpm_strb_setpoint = self.latest_thruster_2_setpoints.rps * 60.0

                self.csv_writer.writerow(
                    [
                        time.time(),
                        self.otter.current_position[0],
                        self.otter.current_position[1],
                        self.otter.current_orientation[2],
                        self.otter.current_rotational_velocities[2],
                        self.otter.current_speed,
                        self.otter.current_course_over_ground,
                        rpm_port_setpoint,
                        self.otter.rpm_port,
                        rpm_strb_setpoint,
                        self.otter.rpm_strb,
                        self.otter.current_mode,
                        self.otter.current_fuel_capacity,
                        self.fx,
                        self.fz,
                    ]
                )
                self.log_file.flush()

            # Publish data to ros system
            self.thruster_1_pub.publish(self.latest_thruster_1_feedback)
            self.thruster_2_pub.publish(self.latest_thruster_2_feedback)
            self.gnss_pub.publish(self.latest_gnss_data)
            self.heading_pub.publish(self.latest_heading_data)
            self.otter_status_pub.publish(otter_status_msg)

            if self.simulation_config["otter_interface"]["publish_nmea_to_localhost"]:
                gga_message = create_gga_message(
                    self.otter.current_position[0], self.otter.current_position[1]
                )
                vtg_message = create_sog_cog_vtg_message(
                    self.otter.current_speed, self.otter.current_course_over_ground
                )
                hdt_message = create_hdt_message(self.otter.current_orientation[2])
                rot_message = create_rot_message(
                    self.otter.current_rotational_velocities[2]
                )

                self.sock_nmea.sendto(gga_message.encode(), ("127.0.0.1", 55555))
                self.sock_nmea.sendto(vtg_message.encode(), ("127.0.0.1", 55555))
                self.sock_nmea.sendto(hdt_message.encode(), ("127.0.0.1", 55555))
                self.sock_nmea.sendto(rot_message.encode(), ("127.0.0.1", 55555))

            if self.latest_system_mode.standby_mode == True:
                self.otter.set_drift_mode()

            elif self.latest_system_mode.test_normalized_force_mode == True:

                self.fx = mu.saturate(float(self.latest_system_mode.fx_test), -1.0, 1.0)
                self.fz = mu.saturate(float(self.latest_system_mode.fz_test), -1.0, 1.0)

                self.otter.set_manual_control_mode(self.fx, 0.0, self.fz)

            elif self.latest_system_mode.test_rpm_output_mode == True:

                self.fx, self.fz = self.map_rpm_to_force_moment(
                    self.latest_system_mode.rpm_strb_test,
                    self.latest_system_mode.rpm_port_test,
                )
                self.otter.set_manual_control_mode(self.fx, 0.0, self.fz)

            elif self.latest_system_mode.auto_mode == True:

                if (
                    self.latest_thruster_1_setpoints is not None
                    and self.latest_thruster_2_setpoints is not None
                ):

                    rpm_port_setpoint = mu.saturate(
                        self.latest_thruster_1_setpoints.rps * 60.0, -800, 1100
                    )
                    rpm_strb_setpoint = mu.saturate(
                        self.latest_thruster_2_setpoints.rps * 60.0, -800, 1100
                    )

                    self.fx, self.fz = self.map_rpm_to_force_moment(
                        rpm_strb_setpoint, rpm_port_setpoint
                    )
                    self.otter.set_manual_control_mode(self.fx, 0.0, self.fz)
        else:
            # From interface to control system
            if self.latest_gnss_data:
                self.gnss_pub.publish(self.latest_gnss_data)
                self.latest_gnss_data = None

            # From interface to control system
            if self.latest_heading_data:
                self.heading_pub.publish(self.latest_heading_data)
                self.latest_heading_data = None

            # From allocator setpoints to simulator
            if self.latest_thruster_1_setpoints:
                self.thruster_1_sim_pub.publish(self.latest_thruster_1_setpoints)
                self.latest_thruster_1_setpoints = None

            # From allocator setpoints to simulator
            if self.latest_thruster_2_setpoints:
                self.thruster_2_sim_pub.publish(self.latest_thruster_2_setpoints)
                self.latest_thruster_2_setpoints = None

            # From sim to system
            if self.latest_thruster_1_feedback:
                self.thruster_1_pub.publish(self.latest_thruster_1_feedback)
                self.latest_thruster_1_feedback = None

            # From sim to system
            if self.latest_thruster_2_feedback:
                self.thruster_2_pub.publish(self.latest_thruster_2_feedback)
                self.latest_thruster_2_feedback = None


def main(args=None):
    rclpy.init(args=args)
    node = OtterUSVNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
