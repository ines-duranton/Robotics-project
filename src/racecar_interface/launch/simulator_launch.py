# truck_system.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():


    racecar_interface_share = FindPackageShare('racecar_interface')

    #parameters
    default_params_file = PathJoinSubstitution([
        FindPackageShare('racecar_interface'),
        'config',
        'interface_params.yaml',
    ])

    #for parent argument
    param_file = LaunchConfiguration('param_file')
    rviz_config = LaunchConfiguration('rviz_config')

    declare_params = DeclareLaunchArgument(
        'param_file',
        default_value=default_params_file,
        description='YAML file for all preceding Nodes',
    )

    default_rviz_config = PathJoinSubstitution([racecar_interface_share, 'rviz', 'simulation.rviz'])
    declare_rviz = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Path to rviz config file',
    )

    

    simulator_node = Node(
        package='racecar_interface',
        executable='simulator_node.py', 
        name='simulator_node',
        output='screen',
        parameters = [param_file]
    )

    visualization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([racecar_interface_share, 'launch', 'visualization_launch.py'])
        ),
        launch_arguments={'param_file': param_file, 'rviz_config': rviz_config}.items()
    )

    return LaunchDescription([
        declare_params,
        declare_rviz,
        simulator_node,
        visualization_launch,

    ])
