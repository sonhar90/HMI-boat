#Importerer nødvendige biblioteker og moduler for ROS2
import rclpy
from rclpy.node import Node
import numpy as np
import os
import yaml
from ngc_interfaces.msg import HeadingDevice, GNSS, Eta, Nu, Tau
from ament_index_python.packages import get_package_share_directory
from ngc_utils.vessel_model import VesselModel
from ngc_utils.qos_profiles import default_qos_profile
import time
import ngc_utils.math_utils as mu
from std_msgs.msg import String
import ngc_utils.geo_utils as geos

class estimator(Node):
    def __init__(self):
        super().__init__("estimator")

        ##### YAML CONFIGURATION #####

        # Deklarerer og laster konfigurasjonsparametere fra YAML-filer
        self.declare_parameter('yaml_package_name', 'ngc_bringup')
        self.declare_parameter('vessel_config_file', 'config/vessel_config.yaml')
        self.declare_parameter('simulation_config_file', 'config/simulator_config.yaml')
        self.declare_parameter('control_config_file', 'config/control_config.yaml')

        # Henter konfigurasjonsfilenes stier ved hjelp av pakkenavnet
        yaml_package_name        = self.get_parameter('yaml_package_name').get_parameter_value().string_value
        yaml_package_path        = get_package_share_directory(yaml_package_name)
        vessel_config_path       = os.path.join(yaml_package_path, self.get_parameter('vessel_config_file').get_parameter_value().string_value)
        simulation_config_path   = os.path.join(yaml_package_path, self.get_parameter('simulation_config_file').get_parameter_value().string_value)
        self.control_config_path = os.path.join(yaml_package_path, self.get_parameter('control_config_file').get_parameter_value().string_value)

        # Laster YAML-konfigurasjonsfiler som inneholder informasjon om fartøyet og simuleringen
        self.vessel_config     = self.load_yaml_file(vessel_config_path)
        self.simulation_config = self.load_yaml_file(simulation_config_path)
        self.control_config    = self.load_yaml_file(self.control_config_path)

        # Initialiserer fartøymodellen basert på konfigurasjonen
        self.vessel_model = VesselModel(self.vessel_config)
    

        ##### SUBSCRIBER AND PUBLISHER #####

        # Abonnenter
        self.reload_config_sub          = self.create_subscription(String, 'reload_configs', self.reload_configs_callback, default_qos_profile)
        self.position_measurement_sub   = self.create_subscription(GNSS, 'gnss_measurement', self.position_measurement_callback, default_qos_profile)
        self.heading_measurement_sub    = self.create_subscription(HeadingDevice, 'heading_measurement', self.heading_measurement_callback, default_qos_profile)
        self.tau_propulsion_sub         = self.create_subscription(Tau, 'tau_propulsion', self.tau_propulsion_callback, default_qos_profile)

        # Publisher to eta_hat and nu_hat
        self.eta_hat_pub    = self.create_publisher(Eta, 'eta_hat', default_qos_profile)
        self.nu_hat_pub     = self.create_publisher(Nu, 'nu_hat', default_qos_profile)

        ##### Data variabler #####
        self.heading_measured = 0.0
        

        self.eta_hat        = np.zeros(3)
        self.nu_hat         = np.zeros(3)
        self.bias           = np.zeros(3)
        self.tau            = np.zeros(3)

        self.surge_x            = None
        self.sway_y             = None
        self.heave_z            = None
        self.roll_k             = None
        self.pitch_m            = None
        self.yaw_n              = None

        self.rot_deg_per_sec    = None
        self.heading_deg        = None
        self.heading_valid      = False

        self.gnss_lat           = None
        self.gnss_lon           = None
        self.gnss_valid         = False

        self.estimator_pos_initialized = False
        self.estimator_hdg_initialized = False

        ##### Globale variabler #####

        self.lat_measured       = None
        self.lon_measured       = None
        self.lat_hat            = None
        self.lon_hat            = None

        ##### Input fra yaml hmi #####
        self.L1         = self.control_config['estimator']['L1']
        self.Omega_E    = self.control_config['estimator']['omega_e']
        self.Bias_Gain  = self.control_config['estimator']['bias_gain']
        self.X_uu       = self.control_config['estimator']['X_uu']
        self.Y_vv       = self.control_config['estimator']['Y_vv']
        self.N_rr       = self.control_config['estimator']['N_rr']


        # Set up a timer to call the estimator function at each simulation timestep
        self.step_size = self.simulation_config['simulation_settings']['step_size']
        self.timer = self.create_timer(self.step_size, self.estimator)  # Calls estimator every 0.1 seconds

    # Callback-funksjon for å motta posisjonsmåling
    def position_measurement_callback(self, msg: GNSS):
        # Lagre posisjonsmåling fra meldingen
        self.lat_measured   = msg.lat
        self.lon_measured   = msg.lon
        self.gnss_valid     = msg.valid_signal

        if self.lat_hat is None and self.lon_hat is None:
            self.lat_hat = self.lat_measured
            self.lon_hat = self.lon_measured
                
        #OVER RIDE IDK
        if self.gnss_valid:
            self.estimator_pos_initialized = True

    # Callback-funksjon for å motta heading-måling
    def heading_measurement_callback(self, msg: HeadingDevice):
        self.heading_measured   = msg.heading
        self.rot_deg_per_sec    = msg.rot  
        self.heading_valid      = msg.valid_signal

        #OVER RIDE IDK
        if self.heading_valid:
            self.estimator_hdg_initialized = True

    # Callback-funksjon for å motta tau_propulsion data
    def tau_propulsion_callback(self, msg: Tau):
        self.surge_x    = msg.surge_x
        self.sway_y     = msg.sway_y
        self.heave_z    = msg.heave_z
        self.roll_k     = msg.roll_k
        self.pitch_m    = msg.pitch_m
        self.yaw_n      = msg.yaw_n
        self.tau        = np.array([self.surge_x, self.sway_y, self.yaw_n])


    def reload_configs_callback(self, msg: String):
        self.control_config = self.load_yaml_file(self.control_config_path)
        self.get_logger().info("Estimator-node har lastet in config!")


    def load_yaml_file(self, file_path):
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)



    # Funksjon som kjøres på hver simuleringstidssteg og implementerer kontrollalgoritmen
    def estimator(self):
        ################## ESTIMATOR #####################
        
        if self.estimator_pos_initialized and self.estimator_hdg_initialized:

            eta_tilde = np.zeros(3)

            if self.gnss_valid:
                eta_tilde[0], eta_tilde[1] = geos.calculate_distance_north_east(self.lat_hat, self.lon_hat, self.lat_measured, self.lon_measured)

            if self.heading_valid:
                eta_tilde[2] = mu.mapToPiPi(self.heading_measured - self.eta_hat[2])

             ##### Input fra yaml hmi #####
            self.L1         = self.control_config['estimator']['L1']
            self.Omega_E    = self.control_config['estimator']['omega_e']
            self.Bias_Gain  = self.control_config['estimator']['bias_gain']
            self.X_uu       = self.control_config['estimator']['X_uu']
            self.Y_vv       = self.control_config['estimator']['Y_vv']
            self.N_rr       = self.control_config['estimator']['N_rr']

            L1 = self.L1 * np.diag([1,1,1])
            L2 = (self.Omega_E ** 2) * np.diag([self.vessel_model.M[0][0], self.vessel_model.M[1][1], self.vessel_model.M[5][5]])
            L3 = self.Bias_Gain * L2

            Q = np.diag([0.5 * self.vessel_model.dimensions['width'], self.vessel_model.dimensions['length'], 1])

            self.bias += self.step_size * L3 @ self.bias

            R = mu.RotationMatrix(0, 0, self.eta_hat[2])

            M_inv = np.linalg.inv(np.diag([self.vessel_model.M[0][0], self.vessel_model.M[1][1], self.vessel_model.M[5][5]]))
            bias = Q @ R.T @ eta_tilde
            drag = np.zeros(3)
            drag[0] = -self.X_uu * abs(self.nu_hat[0]) * self.nu_hat[0]
            drag[1] = -self.Y_vv * abs(self.nu_hat[1]) * self.nu_hat[1]
            drag[2] = -self.N_rr * abs(self.nu_hat[2]) * self.nu_hat[2]


            self.eta_hat[0] = 0
            self.eta_hat[1] = 0

            self.nu_hat     += self.step_size * M_inv @ (drag + self.tau + bias + L2 @ R.T @ eta_tilde)
            self.eta_hat    += self.step_size * (R @ self.nu_hat + L1 @ eta_tilde)


            self.eta_hat[2] = mu.mapToPiPi(self.eta_hat[2])

            self.lat_hat, self.lon_hat = geos.add_distance_to_lat_lon(self.lat_hat, self.lon_hat, self.eta_hat[0], self.eta_hat[1])


            eta_hat_message         = Eta()
            eta_hat_message.lat     = float(self.lat_hat)
            eta_hat_message.lon     = float(self.lon_hat)
            eta_hat_message.z       = float(0)
            eta_hat_message.phi     = float(0)
            eta_hat_message.theta   = float(0)
            eta_hat_message.psi     = float(self.eta_hat[2])

            nu_hat_message          = Nu()
            nu_hat_message.u        = float(self.nu_hat[0])
            nu_hat_message.v        = float(self.nu_hat[1])
            nu_hat_message.w        = float(0)
            nu_hat_message.p        = float(0)
            nu_hat_message.q        = float(0)
            nu_hat_message.r        = float(self.nu_hat[2])
            
            self.eta_hat_pub.publish(eta_hat_message)
            self.nu_hat_pub.publish(nu_hat_message)
            
        
    
        

# Hovedfunksjonen som starter ROS2-noden
def main(args=None):
    # Initialiserer ROS2-kommunikasjon
    rclpy.init(args=args)
    node = estimator()

    # Kjører noden til den stoppes
    rclpy.spin(node)

    # Når noden avsluttes, frigjør ressurser
    node.destroy_node()
    rclpy.shutdown()

# Starter hovedfunksjonen hvis denne filen kjøres som et skript
if __name__ == '__main__':
    main()