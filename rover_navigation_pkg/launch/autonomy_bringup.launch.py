import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_pkg_dir = get_package_share_directory('rover_navigation_pkg')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    nav2_params_file = os.path.join(nav_pkg_dir, 'config', 'nav2_params.yaml')

    # Launch arguments
    bt_exec_arg = DeclareLaunchArgument(
        'bt_executable',
        default_value='risk_autonomy_node',
        description='Behavior Tree executable to run (risk_autonomy_node, classic_autonomy_node)'
    )
    waypoint_arg = DeclareLaunchArgument(
        'waypoint_config',
        default_value='wp2',
        choices=['wp1', 'wp2', 'wp3'],
        description='Preset waypoint route matching Gazebo spawn configuration (wp1, wp2, wp3)'
    )

    bt_exec_name = LaunchConfiguration('bt_executable')
    waypoint_config = LaunchConfiguration('waypoint_config')

    # YOLOv8 perception node
    perception_node = Node(
        package='rover_perception_pkg',
        executable='perception_node.py',
        name='perception_node',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Behavior Tree decision-making node
    autonomy_node = Node(
        package='rover_autonomy_pkg',
        executable=bt_exec_name,
        name='bt_autonomy_node',
        parameters=[{
            'use_sim_time': True,
            'waypoint_config': waypoint_config
        }],
        output='screen'
    )

    # Nav2 bringup (planner and controller)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_file,
            'autostart': 'true'
        }.items()
    )

    # Static map -> odom transform
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', 'map', '--child-frame-id', 'odom'
        ]
    )

    return LaunchDescription([
        bt_exec_arg,
        waypoint_arg,
        perception_node,
        autonomy_node,
        nav2_launch,
        map_to_odom
    ])