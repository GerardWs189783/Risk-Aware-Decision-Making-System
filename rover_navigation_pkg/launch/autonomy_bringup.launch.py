import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # paths rover_navigation_pkg - package for nav2 parameters and lauch files;
    nav_pkg_dir = get_package_share_directory('rover_navigation_pkg')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # yaml file for nav2 config
    nav2_params_file = os.path.join(nav_pkg_dir, 'config', 'nav2_params.yaml')

    # Adding perception node to the launch file (YOLO perception) from rover_perception_pkg
    perception_node = Node(
        package='rover_perception_pkg',
        executable='perception_node.py',
        name='perception_node',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Adding Decision Making node (Behavior Tree) from rover_autonomy_pkg
    autonomy_node = Node(
        package='rover_autonomy_pkg',
        executable='autonomy_node',
        name='autonomy_node',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Adding Nav2 bringup nodes (planner and controller)
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
    # adding odometry node
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # Launch
    return LaunchDescription([
        perception_node,
        autonomy_node,
        nav2_launch,
        map_to_odom
    ])