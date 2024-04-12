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
    sim_params = os.path.join(
        get_package_share_directory('ngc_bringup'),
        'config',
        'arbeidsbaat',
        'arbeidsbaat_params.yaml'
    ) 

    
    thruster_config = os.path.join(
        get_package_share_directory('ngc_bringup'),
        'config',
        'arbeidsbaat',
        'thrusters.yaml'
    ) 
    plotjuggler_config = os.path.join(
        get_package_share_directory('ngc_bringup'),
        'config',
        'PlotJuggler_layout.xml'
    )

    
    sim_node = Node(
        package = "ngc_sim", 
        executable= "simulate",
        name = 'ngc_sim',
        output = 'screen', 
        parameters= [sim_params]       
        
    )

    gnss_node = Node(
        package = "ngc_sensor_sims", 
        executable= "gnss",
        name = 'gnss'
        #parameters= [sim_config] 
    )
    
    compass_node = Node(
        package = "ngc_sensor_sims", 
        executable= "compass",
        name = 'compass'
        #parameters= [sim_config] 
    )

    plotjuggler_node = Node(
        package = "plotjuggler", 
        executable= "plotjuggler",
        arguments= ["--layout",  plotjuggler_config] 
        
    )
    delayed_plotjuggler= TimerAction(period= 3.0, actions=[plotjuggler_node])

    manual_control = Node(
        package= "ngc_teleop", 
        executable="manual_set_point_arbeidsbaat",
        #arguments= [f"thruster_config_file_name:= {thruster_config}"]
        )
    
    thruster_system = Node(
        package= "ngc_thruster_system", 
        executable="thruster_system",
        output = 'screen', 
        parameters= [{"thruster_config_file_name": thruster_config}]
        )

    delayed_thruster_system = TimerAction(period= 3.0, actions=[thruster_system])
    event_handler = RegisterEventHandler(
                event_handler=OnProcessStart(
                target_action=sim_node,
                on_start=[thruster_system],
            ))
    
    ld = LaunchDescription() 
    
    ld.add_action(sim_node)
    ld.add_action(gnss_node)
    ld.add_action(compass_node)
    ld.add_action(manual_control)
    ld.add_action(delayed_thruster_system)
    ld.add_action(delayed_plotjuggler)
    return ld
