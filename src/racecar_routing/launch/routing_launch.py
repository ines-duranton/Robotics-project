from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare configurable launch arguments

    #parameters
    default_params_file = PathJoinSubstitution([
        FindPackageShare('racecar_routing'), 
        'config',
        'routing.yaml'

    ])

    param_file = LaunchConfiguration('param_file')

    declare_param = DeclareLaunchArgument(
        'param_file',
        default_value=default_params_file,
        description='parameters for routing_node',
    )

    return LaunchDescription([
        declare_param,
        Node(
            package = 'racecar_routing',
            executable = 'routing_node.py',
            name = 'routing_node',
            output = 'screen',
            parameters = [param_file]
        ),
        Node(
            package = 'racecar_routing',
            executable = 'map_vis_node.py',
            name = 'map_node',
            output = 'screen',
            parameters = []
        )
    ])