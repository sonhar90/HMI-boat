import rclpy
from rclpy.node import Node
from ngc_interfaces.msg import Tau, ThrusterSignals, Nu
from ngc_utils.qos_profiles import default_qos_profile
from ngc_utils.propulsion_model import PropulsionModel

class PropulsionSimulator(Node):
    def __init__(self):
        super().__init__("ngc_propulsion_sim")
        
        # Initialize PropulsionModel
        self.propulsion_model = PropulsionModel(self)
        
        self.get_logger().info("Configuration files loaded.")

        # Subscriptions
        self.nu_sub = self.create_subscription(Nu, "nu_sim", self.nu_callback, default_qos_profile)
        self.tau_pub = self.create_publisher(Tau, "tau_propulsion", default_qos_profile)

        self.thruster_subscribers = {}
        self.thruster_publishers = {}

        for thruster in self.propulsion_model.thrusters:
            topic_name_sp = f"thruster_{thruster.id}_setpoints"
            topic_name_fb = f"thruster_{thruster.id}_feedback"
            self.thruster_subscribers[thruster.id] = self.create_subscription(ThrusterSignals, topic_name_sp, self.create_thruster_callback(thruster), default_qos_profile)
            self.thruster_publishers[thruster.id] = self.create_publisher(ThrusterSignals, topic_name_fb, default_qos_profile)

        self.nu = np.array([0.0, 0.0, 0.0])
        self.timer = self.create_timer(self.propulsion_model.step_size, self.step_simulation)
        self.get_logger().info("Setup finished.")

    def step_simulation(self):
        tau = self.propulsion_model.step_simulation(self.nu)
        tau_message = Tau()
        tau_message.surge_x, tau_message.sway_y, tau_message.heave_z, tau_message.roll_k, tau_message.pitch_m, tau_message.yaw_n = tau
        self.tau_pub.publish(tau_message)

    def create_thruster_callback(self, thruster):
        def callback(msg: ThrusterSignals):
            thruster.setpoints = msg
        return callback

    def nu_callback(self, msg: Nu):
        self.nu = np.array([msg.u, msg.v, msg.r])

def main(args=None):
    rclpy.init(args=args)
    node = PropulsionSimulator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()