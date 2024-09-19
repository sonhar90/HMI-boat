import os
import signal
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QTextEdit, QPushButton, QLabel, QMessageBox
import yaml
import sys
from PyQt5.QtCore import QTimer
from ngc_utils.qos_profiles import default_qos_profile  # Assuming default_qos_profile is defined here


class HMI_YamlEditorNode(Node):
    def __init__(self):
        super().__init__('ngc_hmi_yaml_editor')

        # Declare parameters for the package and config file
        self.declare_parameter('yaml_package_name', 'ngc_bringup')
        self.declare_parameter('config_file', 'config/control_config.yaml')

        # Get parameters and find the config file dynamically
        yaml_package_name = self.get_parameter('yaml_package_name').get_parameter_value().string_value
        yaml_package_path = get_package_share_directory(yaml_package_name)
        config_file_path = self.get_parameter('config_file').get_parameter_value().string_value
        self.file_path = os.path.join(yaml_package_path, config_file_path)

        # Set up publisher for the reload signal with a QoS profile
        self.reload_publisher = self.create_publisher(String, 'reload_configs', default_qos_profile)

        # Initialize the PyQt5 UI
        self.init_ui()

    def init_ui(self):
        """Initialize the PyQt5 UI."""
        self.app = QApplication(sys.argv)
        self.window = QWidget()
        self.window.setWindowTitle("YAML Editor - control_config.yaml")

        # Set up layout
        layout = QVBoxLayout()

        # Label
        self.label = QLabel("Edit YAML Configuration:")
        layout.addWidget(self.label)

        # Text editor
        self.text_editor = QTextEdit()
        layout.addWidget(self.text_editor)

        # Load YAML content into the text editor
        self.load_yaml_content()

        # Update button
        self.update_button = QPushButton("Update")
        self.update_button.clicked.connect(self.update_button_pressed)
        layout.addWidget(self.update_button)

        # Set the layout and show the window
        self.window.setLayout(layout)
        self.window.show()

    def load_yaml_content(self):
        """Load the YAML file content into the text editor."""
        try:
            with open(self.file_path, 'r') as yaml_file:
                yaml_content = yaml_file.read()
                self.text_editor.setPlainText(yaml_content)
        except FileNotFoundError:
            QMessageBox.critical(self.window, "Error", "YAML file not found.")
        except Exception as e:
            QMessageBox.critical(self.window, "Error", f"Failed to load YAML: {e}")

    def update_button_pressed(self):
        """Save the YAML content and publish the reload signal only if saving is successful."""
        # Get the content from the text editor
        updated_content = self.text_editor.toPlainText()

        # Try to save the updated content back to the YAML file
        try:
            with open(self.file_path, 'w') as yaml_file:
                yaml_file.write(updated_content)
            QMessageBox.information(self.window, "Success", "YAML file updated successfully.")
        except Exception as e:
            QMessageBox.critical(self.window, "Error", f"Failed to save YAML file: {e}")
            return  # Exit the function if saving fails

        # If saving was successful, publish the reload signal
        self.publish_reload_signal()

    def publish_reload_signal(self):
        """Publish a signal to the topic to notify nodes to reload the configuration."""
        msg = String()
        msg.data = 'reload'  # Simple signal message
        self.reload_publisher.publish(msg)
        self.get_logger().info('Published reload signal.')


def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)

    # Create the node (HMI_YamlEditorNode)
    node = HMI_YamlEditorNode()

    # Use a QTimer to periodically call rclpy.spin_once, so the Qt event loop and ROS2 can work together
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.1))  # Spin ROS2 with a short timeout
    timer.start(100)  # Call every 100 milliseconds

    # Setup signal handling to shut down gracefully on Ctrl+C
    def signal_handler(sig, frame):
        print("SIGINT received, shutting down...")
        node.destroy_node()
        rclpy.shutdown()
        QApplication.quit()  # This will close the PyQt5 application

    signal.signal(signal.SIGINT, signal_handler)

    # Start the PyQt5 event loop
    sys.exit(node.app.exec_())


if __name__ == '__main__':
    main()
