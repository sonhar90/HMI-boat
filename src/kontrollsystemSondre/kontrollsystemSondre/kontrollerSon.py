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

class Kontroller(Node):

    def __init__(self):
        super().__init__('sondre_sin_kontroller')

        # Last inn YAML-filer for konfigurasjon
        self.declare_parameter('yaml_package_name', 'ngc_bringup')
        self.declare_parameter('vessel_config_file', 'config/vessel_config.yaml')
        self.declare_parameter('simulation_config_file', 'config/simulator_config.yaml')
        self.declare_parameter('control_config_file', 'config/control_config.yaml')


        # Last inn konfigurasjonsfilene
        yaml_package_name        = self.get_parameter('yaml_package_name').get_parameter_value().string_value
        yaml_package_path        = get_package_share_directory(yaml_package_name)
        vessel_config_path       = os.path.join(yaml_package_path, self.get_parameter('vessel_config_file').get_parameter_value().string_value)
        simulation_config_path   = os.path.join(yaml_package_path, self.get_parameter('simulation_config_file').get_parameter_value().string_value)
        self.control_config_path = os.path.join(yaml_package_path, self.get_parameter('control_config_file').get_parameter_value().string_value)

        #Laster YAML-konfigurasjonsfiler som inneholder infromasjon om fartøy og simulering
        self.vessel_config      = self.load_yaml_file(vessel_config_path)
        self.simulation_config  = self.load_yaml_file(simulation_config_path)
        self.control_config     = self.load_yaml_file(self.control_config_path)

        #initialiserer fartøymodellen basert på konfig 
        self.vessel_model   = VesselModel(self.vessel_config)
        self.vessel_mass    = self.vessel_config['vessel']['mass']

        #self.rho_water = self.simulation_config['physical_parameters']['rho_water']

        #Henter simuleringens tidssteg fra konfig
        self.step_size = self.simulation_config['simulation_settings']['step_size']


        # Sett opp abonnenter og publisher
        self.eta_hat_sub            = self.create_subscription(Eta, 'eta_hat', self.eta_callback, default_qos_profile)
        self.nu__hat_sub            = self.create_subscription(Nu, 'nu_hat', self.nu_callback, default_qos_profile)  # Abonnerer nå på nu_hat
        self.eta_setpoint_sub       = self.create_subscription(Eta, 'eta_setpoint', self.eta_setpoint_callback, default_qos_profile)
        self.nu_setpoint_sub        = self.create_subscription(Nu, 'nu_setpoint', self.nu_setpoint_callback, default_qos_profile)
        self.reload_configs_sub     = self.create_subscription(String,'reload_configs', self.reload_configs_callback, default_qos_profile )
        self.tau_max_sub            = self.create_subscription(Tau,'tau_max', self.tau_max_callback, default_qos_profile)


        #Setter opp publisher for å publisere kontrollsignal
        self.tau_control_pub        = self.create_publisher(Tau, 'tau_propulsion', default_qos_profile)

        # Initialiser variabler
        self.eta            = np.zeros(6)
        self.nu             = np.zeros(6)
        self.eta_setpoint   = np.zeros(6)
        self.nu_setpoint    = np.zeros(6)

        #Integral tilstander i PID
        self.qi_psi         = 0.0
        self.qi_u           = 0.0



        # Demping og thrust-konstanter fra YAML
        #self.added_mass = self.vessel_config['vessel']['hydrodynamic_coefficients']['added_mass']
        #self.nonlinear_damping = self.vessel_config['vessel']['hydrodynamic_coefficients']['nonlinear_damping']
        #self.X_uu = self.nonlinear_damping['X_uu']

        #self.get_logger().info(f"Fartøyets masse: {self.vessel_mass} kg")
        #self.get_logger().info(f"Vannets tetthet: {self.rho_water} kg/m^3")

        

        # Initialiser variabler
        self.eta = np.zeros(6)
        self.nu = np.zeros(6)
        self.eta_setpoint = np.zeros(6)
        self.nu_setpoint = np.zeros(6)
        self.qi_psi = 0.0
        self.qi_u = 0.0


        # Heading variabler fra config fila
        self.omega_heading                  = self.control_config['heading_control']['omega']
        self.zeta_heading                   = self.control_config['heading_control']['zeta']
        self.ki_scale_heading               = self.control_config['heading_control']['ki_scale']
        self.ki_saturation_limit_heading    = np.radians(self.control_config['heading_control']['ki_saturation_limit'])
        self.N_rr_heading                   = self.control_config['heading_control']['N_rr']
        self.linearization_point_heading    = self.control_config['heading_control']['linearization_point']
        self.heading_eps                    = self.control_config['heading_control']['eps'] 

        # Hastighets variabler fra config fila
        self.K_p_speed                      = self.control_config['speed_control']['K_p']
        self.ki_scale_speed                 = self.control_config['speed_control']['ki_scale']
        self.ki_saturation_limit_speed      = self.control_config['speed_control']['ki_saturation_limit']
        self.X_uu_speed                     = self.control_config['speed_control']['X_uu']
        self.speed_eps                      = self.control_config['speed_control']['eps'] 

         # Variabler
        self.fram_max_thrust    = 0
        self.bak_max_thrust     = 0
        self.total_max_moment   = 0



        # Start kontrollsløyfen
        self.timer = self.create_timer(self.step_size, self.step_control)
        self.get_logger().info("Kontroller node er startet!")

    def load_yaml_file(self, file_path):
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
        

    
    def tau_max_callback(self, msg: Tau):
        self.fram_max_thrust    = msg.surge_x
        self.bak_max_thrust     = msg.sway_y
        self.total_max_moment   = msg.yaw_n

    def reload_configs_callback(self, msg: String):

        self.control_config = self.load_yaml_file(self.control_config_path)
        self.get_logger().info("Kontroller-node har lastet in config!") 

    def eta_callback(self, msg: Eta):
        # Mottar estimerte posisjoner (fra estimator_node.py)
        self.eta = np.array([msg.lat, msg.lon, msg.z, msg.phi, msg.theta, msg.psi])

    def nu_callback(self, msg: Nu):
        # Mottar estimerte hastigheter (fra estimator_node.py)
        self.nu = np.array([msg.u, msg.v, msg.w, msg.p, msg.q, msg.r])

    def eta_setpoint_callback(self, msg: Eta):
        self.eta_setpoint = np.array([msg.lat, msg.lon, msg.z, msg.phi, msg.theta, msg.psi])

    def nu_setpoint_callback(self, msg: Nu):
        self.nu_setpoint = np.array([msg.u, msg.v, msg.w, msg.p, msg.q, msg.r])

    def step_control(self):
        
        
        ################## PID Heading #####################
        e_psi = mu.mapToPiPi(self.eta_setpoint[5] - self.eta[5])  # Heading-feil
        e_psi_dot = self.nu_setpoint[5] - self.nu[5]  # Yaw-rate feil

        # Bruk linear og nonlinear damping fra YAML
        d_stjerne = self.N_rr_heading * self.linearization_point_heading

        # Beregn PID-parameterne med verdiene fra YAML-filen som er hentet tidligere
        kp_psi = self.vessel_mass * (self.omega_heading ** 2)
        ki_psi = kp_psi / (self.ki_scale_heading + np.rad2deg(e_psi) ** 2)
        kd_psi = 2 * self.zeta_heading * self.omega_heading * self.vessel_mass - d_stjerne

        # Oppdater integral-delen med metning
        #self.qi_psi += self.step_size * mu.saturate(e_psi, -np.deg2rad(self.control_config['heading_control']['ki_saturation_limit']), np.deg2rad(self.control_config['heading_control']['ki_saturation_limit']))

        # Your integral windup handling for heading
        if e_psi > 1:
            self.sat_e_psi = 1
        elif e_psi < -1:
            self.sat_e_psi = -1
        else:
            self.sat_e_psi = e_psi

        if ((self.qi_psi > self.heading_eps * self.total_max_moment) and (e_psi > 0)) or ((self.qi_psi < -self.heading_eps * self.total_max_moment) and (e_psi < 0)):
            self.qi_psi += 0
        else:
            self.qi_psi += self.step_size * ki_psi * self.sat_e_psi

        # Beregn PID-komponentene
        P_ledd = float(kp_psi * e_psi)
        I_ledd = float(ki_psi * self.qi_psi)
        D_ledd = float(kd_psi * e_psi_dot)  # Yaw rate-feilen brukes her

        # Total kontroll handling (PID formel)
        tau_N = (P_ledd + I_ledd + D_ledd)  

        ################## PI Fart #####################
        e_u = self.nu_setpoint[0] - self.nu[0]  # Surge-hastighetsfeil

        # Beregn integral-forsterkningen og oppdater integral-delen
        kp_u = self.K_p_speed
        ki_u = kp_u / (self.ki_scale_speed + (e_u ** 2))
        #self.qi_u += self.step_size * mu.saturate(e_u, -self.control_config['speed_control']['ki_saturation_limit'], self.control_config['speed_control']['ki_saturation_limit'])

        # Your integral windup handling for speed
        if e_u > 1:
            self.sat_e_u = 1
        elif e_u < -1:
            self.sat_e_u = -1
        else:
            self.sat_e_u = e_u

        if ((self.qi_u > self.speed_eps * self.fram_max_thrust) and (e_u > 0)) or ((self.qi_u < -self.speed_eps * self.bak_max_thrust) and (e_u < 0)):
            self.qi_u += 0
        else:
            self.qi_u += self.step_size * ki_u * self.sat_e_u

        # Juster proporsjonal- og integral-leddene for thrust
        #P_u = self.control_config['speed_control']['K_p'] * e_u  # Proporsjonal del for surge
        #I_u = self.qi_u * ki_u  # Integral del for surge

        # Beregn total thrust (tau_X) med friksjon og regulator
        tau_X = self.X_uu_speed * abs(self.nu_setpoint[0]) * self.nu[0] + (kp_u * e_u) + (ki_u * self.qi_u)  # Begrens thrust til ±500

        # Logg thrust og hastighetsfeil for innsikt i hvordan thrust utvikler seg
        self.get_logger().info(f"Hastighetsfeil: {e_u}, Thrust: {tau_X}")

        # Opprett Tau-melding
        tau_message         = Tau()
        tau_message.surge_x = tau_X
        tau_message.sway_y  = 0.0
        tau_message.heave_z = 0.0
        tau_message.roll_k  = 0.0
        tau_message.pitch_m = 0.0

        # Log yaw før beregning
        self.get_logger().info(f"Before calculation: tau_N (type={type(tau_N)})")

        tau_message.yaw_n = tau_N


        # Publiser kontrollsignalene
        self.tau_control_pub.publish(tau_message)

        self.get_logger().info(f"Publiserer til /tau_propulsion: Thrust = {tau_message.surge_x}, Yaw = {tau_message.yaw_n}")
        self.get_logger().info(f"Thrust: {tau_X}, Yaw: {tau_N} (types: Thrust={type(tau_X)}, Yaw={type(tau_N)})")



def main(args=None):
    rclpy.init(args=args)
    node = Kontroller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
