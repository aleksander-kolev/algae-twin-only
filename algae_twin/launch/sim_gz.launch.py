"""Gazebo side of the twin: world, twin robot, ros_gz bridge, state publisher.

Included by twin.launch.py — the digital world runs in BOTH modes.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

WORLD_NAME = 'default'   # <world name=...> in worlds/algae_world.sdf


def generate_launch_description():
    pkg_share = get_package_share_directory('algae_twin')
    tb3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    tb3_description_share = get_package_share_directory('turtlebot3_description')

    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # the TB3 jazzy urdf is a xacro carrying ${namespace} placeholders; expand it
    # properly (empty namespace = un-prefixed link names, which frame_prefix
    # 'sim/' then turns into sim/base_footprint...). Fall back to a literal strip
    # if xacro is unavailable or the file shape differs from the jazzy version.
    urdf_path = os.path.join(tb3_description_share, 'urdf',
                             'turtlebot3_burger.urdf')
    try:
        import xacro
        burger_urdf = xacro.process_file(
            urdf_path, mappings={'namespace': ''}).toxml()
    except Exception:
        with open(urdf_path, encoding='utf-8') as fh:
            burger_urdf = fh.read().replace('${namespace}', '')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value=os.path.join(pkg_share, 'worlds',
                                                'algae_world.sdf')),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),

        # twin robot meshes live in turtlebot3_gazebo's model dir
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                                  os.path.join(tb3_gazebo_share, 'models')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                                  os.path.join(pkg_share, 'models')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
            condition=UnlessCondition(headless),
            launch_arguments={
                # NO on_exit_shutdown here: a gz GUI crash on a weak lab GPU must
                # NOT tear down Nav2 / the mux / the UI. Ctrl+C still stops all.
                'gz_args': ['-r -v 1 ', world],
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
            condition=IfCondition(headless),
            launch_arguments={
                # --headless-rendering gives the gpu_lidar an EGL context so
                # /sim/scan is still produced with no display (server-only).
                'gz_args': ['-r -s -v 1 --headless-rendering ', world],
                'on_exit_shutdown': 'true',
            }.items()),

        Node(
            package='ros_gz_sim', executable='create', output='screen',
            arguments=[
                '-name', 'burger_twin',
                '-file', os.path.join(pkg_share, 'models', 'burger_twin',
                                      'model.sdf'),
                '-x', LaunchConfiguration('spawn_x'),
                '-y', LaunchConfiguration('spawn_y'),
                '-z', '0.01',
                '-Y', LaunchConfiguration('spawn_yaw'),
            ]),

        # topics from config/bridge.yaml + entity services for the twin tools
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            output='screen',
            arguments=[
                f'/world/{WORLD_NAME}/create@ros_gz_interfaces/srv/SpawnEntity',
                f'/world/{WORLD_NAME}/remove@ros_gz_interfaces/srv/DeleteEntity',
                f'/world/{WORLD_NAME}/set_pose@ros_gz_interfaces/srv/SetEntityPose',
            ],
            parameters=[{
                'config_file': os.path.join(pkg_share, 'config', 'bridge.yaml'),
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }]),

        # TF for the twin's body (frames prefixed sim/)
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='sim_robot_state_publisher', output='screen',
            parameters=[{
                'robot_description': burger_urdf,
                'frame_prefix': 'sim/',
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }],
            remappings=[('joint_states', '/sim/joint_states'),
                        ('robot_description', '/sim/robot_description')]),
    ])
