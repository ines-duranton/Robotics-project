from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    ece_346_share = FindPackageShare('racecar_ece346')
    racecar_interface_share = FindPackageShare('racecar_interface')

    # parameters YAML
    default_params_file = PathJoinSubstitution([
        ece_346_share,
        'config',
        'lab3_launch.yaml',
    ])

    param_file = LaunchConfiguration('param_file')

    declare_params = DeclareLaunchArgument(
        'param_file',
        default_value=default_params_file,
        description='YAML file for all Lab 3 Nodes',
    )

    # Traffic simulation launch
    traffic_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([racecar_interface_share, 'launch', 'traffic_simulation_launch.py'])
        ),
        launch_arguments={'param_file': param_file}.items()
    )

    # Trajectory planner node
    trajectory_planner_node = Node(
        package='racecar_ece346',
        executable='Lab3_traj_planning_node.py',
        name='traj_planning_node',
        output='screen',
        parameters=[param_file],
    )

    # Dynamic Obstacle Node
    dynamic_obstacle_node = Node(
        package='racecar_ece346',
        executable='dyn_obs_node.py',
        name='dynamic_obstacle_node',
        output='screen',
        parameters=[param_file],
    )

    return LaunchDescription([
        declare_params,
        traffic_simulation_launch,
        trajectory_planner_node,
        dynamic_obstacle_node
    ])
