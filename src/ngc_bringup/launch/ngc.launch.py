import os
import launch
from launch import LaunchDescription
from launch.substitutions import TextSubstitution
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import RegisterEventHandler, LogInfo
from launch.event_handlers import OnExecutionComplete, OnProcessExit, OnProcessStart

#: https://roboticscasual.com/tutorial-ros2-launch-files-all-you-need-to-know/ 
def generate_launch_description():



    run_man_control_args = DeclareLaunchArgument(
        name = 'launch-prefix', default_value= TextSubstitution(text= "gnome-terminal --command")
    )
    
    plotjuggler_config = os.path.join(
        get_package_share_directory('ngc_bringup'),
        'config',
        'PlotJuggler_layout.xml'
    )

    
    sim_node = Node(
        package    = "ngc_hull_sim", 
        executable = "simulate",
        name       = 'ngc_hull_sim',
        #output    = 'screen'
    )

    propulsion_node = Node(
        package    = "ngc_propulsion_sim", 
        executable = "ngc_propulsion_sim",
        name       = 'ngc_propulsion_sim',
        output     = 'screen'
    )
    
    gnss_node = Node(
        package    = "ngc_sensor_sims", 
        executable = "gnss",
        name       = 'gnss'
    )
    
    compass_node = Node(
        package    = "ngc_sensor_sims", 
        executable = "compass",
        name       = 'compass'
    )
    
    anemometer_node = Node(
        package     = "ngc_sensor_sims", 
        executable  = "anemometer",
        name        = 'anemometer'
    )

    hmi_node = Node(
        package     = "ngc_hmi", 
        executable  = "ngc_hmi",
        name        = 'hmi',
        output     = 'screen'
    )

    hmi_node_autopilot = Node(
        package     = "ngc_hmi", 
        executable  = "ngc_hmi_autopilot",
        name        = 'hmi_ap',
        output     = 'screen'
    )

    plotjuggler_node = Node(
        package    = "plotjuggler", 
        executable = "plotjuggler",
        arguments  = ["--layout",  plotjuggler_config],
        additional_env={"LIBGL_ALWAYS_SOFTWARE": "1"}

        
    )

    hmi_node_yaml_editor = Node(
        package     = "ngc_hmi", 
        executable  = "ngc_hmi_yaml_editor",
        name        = 'hmi_editor',
        output     = 'screen'
    )

    sondre_sin_kontroller_node = Node(
    package="kontrollsystemSondre",  # Navnet på ROS-pakken din
    executable="kontrollerSon",      # Kjørbar fil (som du har angitt i setup.py)
    name="sondre_sin_kontroller",    # Navn på noden
    output="screen"                  # For å vise output i terminalen
)
    estimator_node = Node(
    package="kontrollsystemSondre",  # Husk å bruke riktig pakkenavn
    executable="estimator_node",      # Kjørbar fil (som du har angitt i setup.py)
    name="estimator_node",            # Navn på noden
    output="screen"                   # For å vise output i terminalen
)



    delayed_plotjuggler= TimerAction(period= 3.0, actions=[plotjuggler_node])

    ld = LaunchDescription() 
    
    ld.add_action(sim_node)
    ld.add_action(gnss_node)
    ld.add_action(compass_node)
    #ld.add_action(anemometer_node)
    #ld.add_action(propulsion_node)
    #ld.add_action(hmi_node)
    ld.add_action(hmi_node_yaml_editor)
    ld.add_action(hmi_node_autopilot)
    ld.add_action(delayed_plotjuggler)
    ld.add_action(sondre_sin_kontroller_node)
    ld.add_action(estimator_node)  # Legg til estimator_node i launch-beskrivelsen

    return ld
