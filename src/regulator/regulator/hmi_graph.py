from matplotlib.figure import Figure
import time
import numpy as np


class GraphManager:
    def __init__(self):
        self.start_time = time.time()
        self.window_size = 10  # Time window in seconds

        # Data storage
        self.eta_setpoints_HMI = []  # List of tuples (timestamp, value)
        self.eta_hat_values = []
        self.nu_setpoints_HMI = []
        self.nu_hat_values = []
        self.thruster_1_setpoints = []
        self.thruster_1_feedback = []
        self.thruster_2_setpoints = []
        self.thruster_2_feedback = []

        # Create figures and axes
        self.fig1, self.ax1 = self.create_figure(
            "Eta Setpoint vs Sim", "Heading (degrees)"
        )
        self.fig2, self.ax2 = self.create_figure("Nu Setpoint vs Sim", "Speed (m/s)")
        self.fig3, self.ax3 = self.create_figure(
            "Thruster 1 Setpoint vs Feedback", "Thruster Output"
        )
        self.fig4, self.ax4 = self.create_figure(
            "Thruster 2 Setpoint vs Feedback", "Thruster Output"
        )

    def create_figure(self, title, ylabel):
        fig = Figure(facecolor="#2E2E2E")
        ax = fig.add_subplot(111)
        ax.set_facecolor("#333333")
        ax.grid(True, color="gray")
        ax.set_title(title, color="white")
        ax.set_ylabel(ylabel, color="white")
        ax.tick_params(axis="x", colors="white")
        ax.tick_params(axis="y", colors="white")
        return fig, ax

    def update_graphs(self):
        current_time = time.time() - self.start_time
        min_time = current_time - self.window_size

        # Update Eta Graph
        self.update_graph(
            self.ax1,
            self.fig1,
            self.eta_setpoints_HMI,
            self.eta_hat_values,
            min_time,
            current_time,
            "Eta Setpoint vs Feedback",
        )

        # Update Nu Graph
        self.update_graph(
            self.ax2,
            self.fig2,
            self.nu_setpoints_HMI,
            self.nu_hat_values,
            min_time,
            current_time,
            "Nu Setpoint vs Feedback",
        )

        # Update Thruster 1 Graph
        self.update_graph(
            self.ax3,
            self.fig3,
            self.thruster_1_setpoints,
            self.thruster_1_feedback,
            min_time,
            current_time,
            "Thruster 1 Setpoint vs Feedback",
        )

        # Update Thruster 2 Graph
        self.update_graph(
            self.ax4,
            self.fig4,
            self.thruster_2_setpoints,
            self.thruster_2_feedback,
            min_time,
            current_time,
            "Thruster 2 Setpoint vs Feedback",
        )

    def update_graph(
        self, ax, fig, setpoint_data, feedback_data, min_time, current_time, title
    ):
        ax.clear()
        # Filter data within the time window
        setpoint_data_filtered = [(t, v) for t, v in setpoint_data if t >= min_time]
        feedback_data_filtered = [(t, v) for t, v in feedback_data if t >= min_time]

        if setpoint_data_filtered:
            times_setpoint, values_setpoint = zip(*setpoint_data_filtered)
            ax.plot(times_setpoint, values_setpoint, label="Setpoint", color="#39FF14")
        if feedback_data_filtered:
            times_feedback, values_feedback = zip(*feedback_data_filtered)
            ax.plot(times_feedback, values_feedback, label="Feedback", color="#FF073A")

        ax.legend(loc="upper right")
        ax.set_title(title, color="white")
        ax.set_xlabel("Time (s)", color="white")
        ax.tick_params(axis="x", colors="white")
        ax.tick_params(axis="y", colors="white")
        ax.set_xlim(min_time, current_time)  # Set x-axis limits to the time window
        fig.canvas.draw()
