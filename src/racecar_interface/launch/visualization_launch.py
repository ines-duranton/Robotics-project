# truck_system.launch.py
from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
   

    racecar_interface_share = FindPackageShare('racecar_interface')
    racecar_routing_share   = FindPackageShare('racecar_routing')

    # ----- RViz2 -----
    default_rviz_config = PathJoinSubstitution([racecar_interface_share, 'rviz', 'simulation.rviz'])
    declare_rviz = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Path to rviz config file',
    )
    rviz_config = LaunchConfiguration('rviz_config')
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # ----- RQt (perspective) ----- dont got yet
    rqt_perspective = PathJoinSubstitution([racecar_interface_share, 'rviz', 'simulation.perspective'])
    rqt_process = ExecuteProcess(cmd=['rqt', '--perspective-file', rqt_perspective,], output='screen')

    
    #parameters
    default_params_file = PathJoinSubstitution([
        FindPackageShare('racecar_interface'), 
        'config',
        'interface_params.yaml'

    ])

    param_file = LaunchConfiguration('param_file')

    declare_param = DeclareLaunchArgument(
        'param_file',
        default_value=default_params_file,
        description='parameters for all descending Nodes',
    )

    # ----- racecar_interface visualization node -----
    viz_node = Node(
        package='racecar_interface',
        executable='visualization_node.py', 
        name='visualization_node',
        output='screen',
        parameters = [param_file]
    )

    routing_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([racecar_routing_share, 'launch', 'routing_launch.py'])
        ),
        launch_arguments={'param_file' : param_file}.items()
    )

    return LaunchDescription([
        declare_rviz,
        declare_param,
        viz_node,
        routing_launch,
        rviz_node,
        rqt_process

    ])
