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
        output    = 'screen'
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
        name       = 'compass',
        output     = 'screen'
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
        package="plotjuggler",
        executable="plotjuggler",
        arguments=["--layout", plotjuggler_config],
        additional_env={"LIBGL_ALWAYS_SOFTWARE": "1"}  # Setter programvarebasert rendering
    )

    hmi_node_yaml_editor = Node(
        package     = "ngc_hmi", 
        executable  = "ngc_hmi_yaml_editor",
        name        = 'hmi_editor',
        output     = 'screen'
    )

    regulator = Node(
        package     = "regulator",
        executable  = "kontroller",
        name        = 'kontroller',
        output      = 'screen'
    )

    estimator = Node(
        package     = "regulator",
        executable  = "estimator",
        name        = 'estimator',
        #output      = 'screen'
    )

    allokering = Node(
        package     = "regulator",
        executable  = "allokering",
        name        = 'allokering',
        output      = 'screen'
    )

    guide = Node(
        package     = "regulator",
        executable  = "guide",
        name        = 'guide',
        output      = 'screen'
    )

    delayed_plotjuggler= TimerAction(period= 6.0, actions=[plotjuggler_node])
    delayed_kontroller= TimerAction(period= 2.0, actions=[regulator])
    delayed_estimator= TimerAction(period= 1.0, actions=[estimator])
    delayed_allokering= TimerAction(period= 3.0, actions=[allokering])
    delayed_guide= TimerAction(period= 4.0, actions=[guide]) 

    ld = LaunchDescription() 
    
    ld.add_action(sim_node)
    ld.add_action(gnss_node)
    ld.add_action(compass_node)
    #ld.add_action(anemometer_node)
    ld.add_action(propulsion_node)
    #ld.add_action(hmi_node)
    ld.add_action(hmi_node_yaml_editor)
    ld.add_action(hmi_node_autopilot)
    ld.add_action(delayed_plotjuggler)
    ld.add_action(delayed_kontroller)
    ld.add_action(delayed_estimator)
    ld.add_action(delayed_allokering)
    ld.add_action(delayed_guide)

    return ld
