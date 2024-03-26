import os
import launch
from launch import LaunchDescription
from launch.substitutions import TextSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():


    run_man_control_args = DeclareLaunchArgument(
        name = 'launch-prefix', default_value= TextSubstitution(text= "gnome-terminal --command")
    )
    sim_config = os.path.join(
        get_package_share_directory('ngc_bringup'),
        'config',
        'params.yaml'
    ) 

    ld = LaunchDescription()
    sim_node = Node(
        package = "ngc_sim", 
        executable= "simulate",
        name = 'ngc_sim',
        output = 'screen', 
        parameters= [sim_config]       
        
    )
    plotjuggler_node = Node(
        package = "plotjuggler", 
        executable= "plotjuggler" 
        
    )


    manual_control = Node(
        package= "ngc_sim", 
        executable="manual_control",
        #arguments= run_man_control_args
        )

    
    ld.add_action(sim_node)
    ld.add_action(manual_control)
    ld.add_action(plotjuggler_node)
    return ld