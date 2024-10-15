from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtCore import QUrl
import folium
import os
import math


def add_plotjuggler_graph_1(self):
    # Create a figure and a canvas for plotting
    fig = Figure(facecolor='#2E2E2E')  # Set the figure background to a dark color
    self.canvas = FigureCanvas(fig)

    # Add a subplot
    ax = fig.add_subplot(111)

    # Customize the background and grid lines
    ax.set_facecolor('#333333')  # Darker background for the graph area
    ax.grid(True, color='gray')  # Set grid line color

    # Customize the lines (neon-like colors)
    x = range(100)
    y1 = [i ** 0.5 for i in x]
    y2 = [i ** 0.4 for i in x]
    ax.plot(x, y1, color='#39FF14', linewidth=2)  # Neon green line
    ax.plot(x, y2, color='#FF073A', linewidth=2)  # Neon red line

    # Set axis labels and title with white color for better contrast
    ax.set_title("Plot with Neon Colors", color='white')
    ax.set_xlabel("X-axis", color='white')
    ax.set_ylabel("Y-axis", color='white')

    # Set tick parameters to white for visibility
    ax.tick_params(axis='x', colors='white')
    ax.tick_params(axis='y', colors='white')

    return self.canvas


def add_plotjuggler_graph_2(self):
    # Create the second graph
    fig = Figure(facecolor='#2E2E2E')  # Set the figure background to a dark color
    canvas_2 = FigureCanvas(fig)

    # Plot another simple graph (e.g., linear)
    ax2 = fig.add_subplot(111)

    ax2.set_facecolor('#333333')  # Darker background for the graph area
    ax2.grid(True, color='gray')  # Set grid line color
    x = range(100)
    y = [i for i in x]
    ax2.plot(x, y)

    ax2.set_title("Graph 2", color='white')
    ax2.set_xlabel("X-axis", color='white')
    ax2.set_ylabel("Y-axis", color='white')

    # Set tick parameters to white for visibility
    ax2.tick_params(axis='x', colors='white')
    ax2.tick_params(axis='y', colors='white')

    return canvas_2


def add_plotjuggler_graph_3(self):
    # Create the third graph
    fig = Figure(facecolor='#2E2E2E')  # Set the figure background to a dark color
    canvas_3 = FigureCanvas(fig)

    # Plot another graph (e.g., logarithmic)
    ax3 = fig.add_subplot(111)

    ax3.set_facecolor('#333333')  # Darker background for the graph area
    ax3.grid(True, color='gray')  # Set grid line color

    x = range(1, 100)
    y = [math.log(i) for i in x]
    ax3.plot(x, y)

    ax3.set_title("Graph 2", color='white')
    ax3.set_xlabel("X-axis", color='white')
    ax3.set_ylabel("Y-axis", color='white')

    # Set tick parameters to white for visibility
    ax3.tick_params(axis='x', colors='white')
    ax3.tick_params(axis='y', colors='white')

    return canvas_3


def add_map(self):
    # Create Folium map and save it as an HTML file
    map_file = create_map()

    # Use QWebEngineView to display the map
    self.map_widget = QWebEngineView()
    self.map_widget.setUrl(QUrl.fromLocalFile(map_file))

    return self.map_widget


def create_map():
    map_obj = folium.Map(location=[60.3913, 5.3221], zoom_start=12)
    map_file = 'map.html'
    map_obj.save(map_file)
    return os.path.abspath(map_file)
