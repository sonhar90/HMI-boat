# Importerer nødvendige biblioteker og moduler for ROS2
import rclpy
from rclpy.node import Node
import numpy as np
import os
import yaml
from ngc_interfaces.msg import Eta, Nu, Tau
from ament_index_python.packages import get_package_share_directory
from ngc_utils.vessel_model import VesselModel
from ngc_utils.qos_profiles import default_qos_profile
import time
import ngc_utils.math_utils as mu
from std_msgs.msg import String

# Klassen Kontroller definerer en ROS2-node
class Kontroller(Node):
    # Initialiserer kontrolleren og setter opp abonnementer og publiseringer
    def __init__(self):
        # Kaller på superklassen Node for å initialisere noden med navnet "kontroller"
        super().__init__('kontroller')
        
        # Deklarerer og laster konfigurasjonsparametere fra YAML-filer
        self.declare_parameter('yaml_package_name', 'ngc_bringup')
        self.declare_parameter('vessel_config_file', 'config/vessel_config.yaml')
        self.declare_parameter('simulation_config_file', 'config/simulator_config.yaml')
        self.declare_parameter('control_config_file', 'config/control_config.yaml')

        # Henter konfigurasjonsfilenes stier ved hjelp av pakkenavnet
        yaml_package_name = self.get_parameter('yaml_package_name').get_parameter_value().string_value
        yaml_package_path = get_package_share_directory(yaml_package_name)
        vessel_config_path = os.path.join(yaml_package_path, self.get_parameter('vessel_config_file').get_parameter_value().string_value)
        simulation_config_path = os.path.join(yaml_package_path, self.get_parameter('simulation_config_file').get_parameter_value().string_value)
        self.control_config_path = os.path.join(yaml_package_path, self.get_parameter('control_config_file').get_parameter_value().string_value)

        # Laster YAML-konfigurasjonsfiler som inneholder informasjon om fartøyet og simuleringen
        self.vessel_config = self.load_yaml_file(vessel_config_path)
        self.simulation_config = self.load_yaml_file(simulation_config_path)
        self.control_config = self.load_yaml_file(self.control_config_path)

        # Initialiserer fartøymodellen basert på konfigurasjonen
        self.vessel_model = VesselModel(self.vessel_config)
        
        # Henter simulasjonens tidssteg fra konfigurasjonsfilen
        self.step_size = self.simulation_config['simulation_settings']['step_size']

        # Setter opp abonnenter for å lytte på data fra simulatoren (eta og nu) og settpunkter (eta_setpoint, nu_setpoint)
        self.eta_sub           = self.create_subscription(Eta, 'eta_sim', self.eta_callback, default_qos_profile)
        self.nu_sub            = self.create_subscription(Nu, 'nu_sim', self.nu_callback, default_qos_profile)
        self.eta_setpoint_sub  = self.create_subscription(Eta, 'eta_setpoint', self.eta_setpoint_callback, default_qos_profile)
        self.nu_setpoint_sub   = self.create_subscription(Nu, 'nu_setpoint', self.nu_setpoint_callback, default_qos_profile)
        self.reload_config_sub = self.create_subscription(String, 'reload_configs', self.reload_configs_callback, default_qos_profile)

        # Setter opp en publisher for å publisere kontrollsignalene (tau_propulsion)
        self.tau_pub = self.create_publisher(Tau, 'tau_propulsion', default_qos_profile)

        # Initialiserer variabler for å lagre data fra simulatoren og settpunkter
        self.eta = np.zeros(6)
        self.nu = np.zeros(6)
        self.eta_setpoint = np.zeros(6)
        self.nu_setpoint = np.zeros(6)

        # Integraltilstander i PID kontroll
        self.qi_psi = 0.0
        self.qi_u   = 0.0

        # Starter kontroll-løkken som kjører med samme tidssteg som simulatoren
        self.timer = self.create_timer(self.step_size, self.step_control)

        self.get_logger().info("Kontroller-node er initialisert.")

    # Funksjon for å laste inn YAML-konfigurasjonsfiler
    def load_yaml_file(self, file_path):
        
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)

    def reload_configs_callback(self, msg: String):

        self.control_config = self.load_yaml_file(self.control_config_path)
        self.get_logger().info("Kontroller-node har lastet in config!")

    # Callback-funksjon for å motta eta_sim-data fra simulatoren
    def eta_callback(self, msg: Eta):
        
        # Mottar posisjons- og orienteringsdata fra simulatoren
        self.eta = np.array([msg.lat, msg.lon, msg.z, msg.phi, msg.theta, msg.psi])

    # Callback-funksjon for å motta nu_sim-data fra simulatoren
    def nu_callback(self, msg: Nu):
        
        # Mottar hastighetsdata fra simulatoren
        self.nu = np.array([msg.u, msg.v, msg.w, msg.p, msg.q, msg.r])

    # Callback-funksjon for å motta eta_setpoint-data (settpunkt for posisjon og orientering)
    def eta_setpoint_callback(self, msg: Eta):
        
        # Mottar settpunktdata for posisjon og orientering
        self.eta_setpoint = np.array([msg.lat, msg.lon, msg.z, msg.phi, msg.theta, msg.psi])

    # Callback-funksjon for å motta nu_setpoint-data (settpunkt for hastighet)
    def nu_setpoint_callback(self, msg: Nu):
       
        # Mottar settpunktdata for hastigheter
        self.nu_setpoint = np.array([msg.u, msg.v, msg.w, msg.p, msg.q, msg.r])

    # Funksjon som kjøres på hver simuleringstidssteg og implementerer kontrollalgoritmen
    def step_control(self):
        
        # Denne funksjonen vil inneholde kontrolllogikk for å beregne de nødvendige kreftene (tau)
        # Foreløpig brukes en enkel PD-kontroll (Propositional-Derivative) som eksempel

        ################## PID Heading #####################
        # Beregn avvik (feil) mellom settpunkt og faktisk verdi for både posisjon (eta) og hastighet (nu)
        e_psi     = mu.mapToPiPi(self.eta_setpoint[5] - self.eta[5])
        e_psi_dot = self.nu_setpoint[5] - self.nu[5]

        zeta      = self.control_config['heading_control']['zeta']
        omega_psi = self.control_config['heading_control']['omega']
        ki_scale  = self.control_config['heading_control']['ki_scale']
        ki_limit  = self.control_config['heading_control']['ki_saturation_limit']

        d_star = self.control_config['heading_control']['N_rr']*self.control_config['heading_control']['linearization_point']

        K_p_psi = self.vessel_model.M[5][5]*omega_psi**2
        K_d_psi = 2*zeta*omega_psi*self.vessel_model.M[5][5] - d_star
        K_i_psi = K_p_psi / (abs(ki_scale) + np.rad2deg(e_psi)**2)   

        self.qi_psi += self.step_size*K_i_psi*mu.saturate(e_psi,-np.deg2rad(ki_limit),np.deg2rad(ki_limit))

        tau_N = K_p_psi*e_psi + K_d_psi*e_psi_dot + self.qi_psi

        ################## PI Fart #####################
        e_u = self.nu_setpoint[0] - self.nu[0]

        ki_scale = self.control_config['speed_control']['ki_scale']
        ki_limit = self.control_config['speed_control']['ki_saturation_limit']
        X_uu     = self.control_config['speed_control']['X_uu']

        K_p_u = self.vessel_model.M[0][0]*self.control_config['speed_control']['K_p']
        K_i_u = K_p_u / (abs(ki_scale) + e_u**2)

        self.qi_u += self.step_size*K_i_u*mu.saturate(e_u,-ki_limit,ki_limit)

        tau_X = X_uu*abs(self.nu_setpoint[0])*self.nu_setpoint[0] + K_p_u*e_u + self.qi_u

        # Opprett en Tau-melding for å sende de beregnede kreftene
        tau_message = Tau()
        tau_message.surge_x = tau_X
        tau_message.sway_y  = 0.0
        tau_message.heave_z = 0.0
        tau_message.roll_k  = 0.0
        tau_message.pitch_m = 0.0
        tau_message.yaw_n   = tau_N

        # Publiser kontrollkreftene på tau_propulsion-topic
        self.tau_pub.publish(tau_message)
        #self.get_logger().info("Kontrollkrefter publisert på tau_propulsion.")

# Hovedfunksjonen som starter ROS2-noden
def main(args=None):
    # Initialiserer ROS2-kommunikasjon
    rclpy.init(args=args)
    node = Kontroller()
    
    # Kjører noden til den stoppes
    rclpy.spin(node)
    
    # Når noden avsluttes, frigjør ressurser
    node.destroy_node()
    rclpy.shutdown()

# Starter hovedfunksjonen hvis denne filen kjøres som et skript
if __name__ == '__main__':
    main()
