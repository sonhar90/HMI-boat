import rclpy
from rclpy.node import Node
import numpy as np
import os
import yaml
from ngc_interfaces.msg import Eta, Nu
from ament_index_python.packages import get_package_share_directory
from ngc_utils.qos_profiles import default_qos_profile
from std_msgs.msg import String

class guide(Node):
    def __init__(self):
        super().__init__('guide')
        self.get_logger().info('Guide node is initialized.')

        # Deklarerer og laster konfigurasjonsparametere fra YAML-filer
        self.declare_parameter('yaml_package_name', 'ngc_bringup')
        self.declare_parameter('simulation_config_file', 'config/simulator_config.yaml')

        # Henter konfigurasjonsfilenes stier ved hjelp av pakkenavnet
        yaml_package_name = self.get_parameter('yaml_package_name').get_parameter_value().string_value
        yaml_package_path = get_package_share_directory(yaml_package_name)
        simulation_config_path = os.path.join(yaml_package_path, self.get_parameter('simulation_config_file').get_parameter_value().string_value)

        # Laster YAML-konfigurasjonsfiler som inneholder informasjon om fartøyet og simuleringen
        self.simulation_config = self.load_yaml_file(simulation_config_path)

        # Henter simulasjonens tidssteg fra konfigurasjonsfilen
        self.step_size = self.simulation_config['simulation_settings']['step_size']

        # Setter opp abonnementer
        self.eta_setpoint_HMI_sub = self.create_subscription(Eta, 'eta_setpoint_HMI', self.eta_setpoint_HMI_callback, default_qos_profile)
        self.nu_setpoint_HMI_sub = self.create_subscription(Nu, 'nu_setpoint_HMI', self.nu_setpoint_HMI_callback, default_qos_profile)

        # Setter opp en publisher
        self.eta_setpoint = self.create_publisher(Eta, "eta_setpoint", default_qos_profile) 
        self.nu_setpoint = self.create_publisher(Nu, "nu_setpoint", default_qos_profile)

        # Initialiserer variabler for å lagre data fra simulatoren og settpunkter
        self.eta = np.zeros(6)
        self.nu = np.zeros(6)
        self.eta_setpoint = np.zeros(6)
        self.nu_setpoint = np.zeros(6)

        # Starter kontroll-løkken som kjører med samme tidssteg som simulatoren
        self.timer = self.create_timer(self.step_size, self.step_control)

        self.get_logger().info("Guide node er initialisert.")

    def eta_setpoint_HMI_callback(self, msg):
        """Callback for eta_setpoint_HMI subscription."""
        self.eta_setpoint = np.array(msg.data)  # Tilpass dette basert på datatypen i Eta-meldingen
        self.get_logger().info(f"Received eta setpoint: {self.eta_setpoint}")

    def nu_setpoint_HMI_callback(self, msg):
        """Callback for nu_setpoint_HMI subscription."""
        self.nu_setpoint = np.array(msg.data)  # Tilpass dette basert på datatypen i Nu-meldingen
        self.get_logger().info(f"Received nu setpoint: {self.nu_setpoint}")

    def step_control(self):
        """Kontroll-løkke som kjører med samme tidssteg som simulatoren."""
        # Her kan du implementere kontrollalgoritmen din basert på setterpunktene.
        pass


    # Funksjon for å laste inn YAML-konfigurasjonsfiler
    def load_yaml_file(self, file_path):
        
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)



# Hovedfunksjonen som starter ROS2-noden
def main(args=None):
    # Initialiserer ROS2-kommunikasjon
    rclpy.init(args=args)
    node = guide()
    
    # Kjører noden til den stoppes
    rclpy.spin(node)
    
    # Når noden avsluttes, frigjør ressurser
    node.destroy_node()
    rclpy.shutdown()

# Starter hovedfunksjonen hvis denne filen kjøres som et skript
if __name__ == '__main__':
    main()