from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    lab0_share = FindPackageShare('racecar_ece346')
    racecar_interface_share = FindPackageShare('racecar_interface')

    lab0_rviz = PathJoinSubstitution([racecar_interface_share, 'rviz', 'lab0.rviz'])

    #parameters
    default_params_file = PathJoinSubstitution([
        FindPackageShare('racecar_ece346'),
        'config',
        'lab0.yaml',
    ])

    #for parent argument
    param_file = LaunchConfiguration('param_file')

    declare_params = DeclareLaunchArgument(
        'param_file',
        default_value=default_params_file,
        description='YAML file for all preceding Nodes',
    )


    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([racecar_interface_share, 'launch', 'simulator_launch.py'])
        ),
        launch_arguments={'param_file': param_file, 'rviz_config': lab0_rviz}.items()
    )



    lab0_node = Node(
        package='racecar_ece346',
        executable='lab0_node.py',
        name='pure_pursuit_controller_node',
        output='screen',
        parameters=[param_file],
    )

    return LaunchDescription([
        declare_params,
        simulation_launch,
        lab0_node,
    ])
