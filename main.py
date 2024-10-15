import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QGridLayout, QWidget, QSizePolicy
from PyQt5.QtCore import QTimer
import PyQt5_stylesheets
from buttons import add_thruster_buttons, toggle_button_state, style_button
from map_graph import add_map, add_plotjuggler_graph_1, add_plotjuggler_graph_2, add_plotjuggler_graph_3
from heading_control import add_heading_control
from speed_control import add_speed_control
import math


class HMIWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # Initialize window properties
        self.setWindowTitle('Ship Control HMI')
        self.setGeometry(100, 100, 1200, 800)
        self.setFixedSize(1200, 800)

        # Create central widget and layout
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.grid_layout = QGridLayout(self.central_widget)

        # Add heading control
        heading_label, self.heading_dial, self.heading_display = add_heading_control(self)

        # Set fixed size for heading dial and display
        self.heading_dial.setFixedSize(200, 200)  # Adjust size as needed
        self.heading_display.setFixedSize(200, 80)  # Adjust size as needed

        self.grid_layout.addWidget(heading_label, 0, 0)
        self.grid_layout.addWidget(self.heading_dial, 1, 0)
        self.grid_layout.addWidget(self.heading_display, 2, 0)

        # Add speed control
        speed_label, self.speed_slider, self.speed_display = add_speed_control(self)
        self.speed_slider.setFixedSize(200, 200)  # Adjust size as needed
        self.speed_display.setFixedSize(200, 80)  # Adjust size as needed
        self.grid_layout.addWidget(speed_label, 3, 0)
        self.grid_layout.addWidget(self.speed_slider, 4, 0)
        self.grid_layout.addWidget(self.speed_display, 5, 0)

        # Add map widget
        map_widget = add_map(self)
        self.grid_layout.addWidget(map_widget, 2, 1, 4, 3)

        # Add three graph widgets
        graph_canvas_1 = add_plotjuggler_graph_1(self)
        graph_canvas_2 = add_plotjuggler_graph_2(self)
        graph_canvas_3 = add_plotjuggler_graph_3(self)

        # Place the three graphs in rows 0 to 2, spanning columns 1 to 3
        # Place the three graphs in row 0, in three different columns
        self.grid_layout.addWidget(graph_canvas_1, 0, 1, 2, 1)  # First graph in column 1
        self.grid_layout.addWidget(graph_canvas_2, 0, 2, 2, 1)  # Second graph in column 2
        self.grid_layout.addWidget(graph_canvas_3, 0, 3, 2, 1)  # Third graph in column 3

        # Add thruster buttons
        self.active_button = None
        thruster_layout = add_thruster_buttons(self)
        self.grid_layout.addLayout(thruster_layout, 7, 0, 1, 4)

        # Set size policy
        self.central_widget.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)

        # Set up ROS2 callbacks timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_ros_callbacks)
        self.timer.start(100)

    def update_heading(self, value):
        self.heading_display.display(value)
        radians_value = self.handle_heading_wraparound(value)
        print(f"Heading in radians: {radians_value}")

    def handle_heading_wraparound(self, value):
        radians_value = math.radians(value)
        return (radians_value + math.pi) % (2 * math.pi) - math.pi

    def remap_dial_value(self, value):
        return (value - 180) % 360

    def update_speed(self, value):
        self.speed_display.display(value)

    def update_ros_callbacks(self):
        pass

    def close_application(self):
        self.timer.stop()
        self.close()


def main():
    app = QApplication(sys.argv)

    # Load the stylesheet
    app.setStyleSheet(PyQt5_stylesheets.load_stylesheet_pyqt5(style="style_black"))

    # Initialize HMI window
    hmi_window = HMIWindow()
    hmi_window.show()

    # Start the application event loop
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
