"""Algae Twin — single entry point (TWIN MODE: real robot + Gazebo twin).

    ros2 launch algae_twin twin.launch.py

Nav2 localises and drives the REAL TurtleBot3 Burger — its standard
turtlebot3_bringup must be running on the same ROS_DOMAIN_ID — and the Gazebo
twin shadows it: pose_sync keeps the two glued, the obstacle mirror keeps the
digital world truthful, and the operator UI drives both. Everything runs on the
wall clock; the real robot is the time authority.

This is the twin-only build: it always runs against the real robot — there is no
`mode` argument and no sim-only / emulator code path.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('algae_twin')
    twin_params = os.path.join(pkg_share, 'config', 'twin.yaml')

    pass_through = {key: LaunchConfiguration(key)
                    for key in ('spawn_x', 'spawn_y', 'spawn_yaw')}

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false',
                              description='Gazebo server only (no 3D window)'),
        DeclareLaunchArgument('ui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'map', default_value=os.path.join(pkg_share, 'maps', 'map.yaml')),
        DeclareLaunchArgument(
            'world', default_value=os.path.join(pkg_share, 'worlds',
                                                'algae_world.sdf')),

        # the digital twin's Gazebo world (burger_twin + ros_gz bridge + the
        # twin's robot_state_publisher). Wall clock — the real robot is authority.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                pkg_share, 'launch', 'sim_gz.launch.py')),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'headless': LaunchConfiguration('headless'),
                'use_sim_time': 'false',
                **pass_through,
            }.items()),

        # the one navigation brain — localises/drives the REAL robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                pkg_share, 'launch', 'nav2.launch.py')),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                **pass_through,
            }.items()),

        # twin services
        Node(package='algae_twin', executable='twin_bridge', output='screen',
             parameters=[twin_params, {
                 'real_cmd_stamped': True,   # TB3 jazzy bringup default
                 'use_sim_time': False,
             }]),
        Node(package='algae_twin', executable='map_edit', output='screen',
             parameters=[twin_params, {'use_sim_time': False}]),
        Node(package='algae_twin', executable='mission', output='screen',
             parameters=[twin_params, {'use_sim_time': False}]),
        Node(package='algae_twin', executable='sim_battery', output='screen',
             parameters=[twin_params, {
                 'mirror_real': True,        # the twin reports the real pack
                 'use_sim_time': False,
             }]),

        # shadow controller + real->digital obstacle mirror (core twin behaviour)
        Node(package='algae_twin', executable='pose_sync', output='screen',
             parameters=[twin_params, {'use_sim_time': False}]),
        Node(package='algae_twin', executable='obstacle_mirror',
             output='screen', parameters=[twin_params, {'use_sim_time': False}]),

        # operator UI (browser dashboard, served from the Python stdlib)
        Node(package='algae_twin', executable='operator_ui', output='screen',
             condition=IfCondition(LaunchConfiguration('ui')),
             parameters=[{'use_sim_time': False}]),

        Node(package='rviz2', executable='rviz2', output='screen',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', os.path.join(pkg_share, 'rviz', 'twin.rviz')],
             parameters=[{'use_sim_time': False}]),
    ])
