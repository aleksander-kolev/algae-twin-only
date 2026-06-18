"""Twin-only build + an EMULATED real robot — hardware-free test on a laptop.

Runs the twin-only package's `twin.launch.py` completely UNCHANGED (the exact
thing that runs on the lab PC) and, alongside it, the wall-clock robot emulator
(fake_robot + its robot_state_publisher) that stands in for
`turtlebot3_bringup robot.launch.py`. On the lab PC you launch the real bringup
on the robot instead of this — nothing else changes.

    ros2 launch <this-file> twin_emulated.launch.py rviz:=true

What you see: the Gazebo window shows the digital twin (the real robot is never
in Gazebo in twin mode — on the lab PC it's the physical robot); RViz shows the
(emulated) real robot localising with its live scan, the costmaps and the plan;
the browser UI at :8088 shows BOTH robots, batteries and divergence.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

THIS_DIR = os.path.dirname(os.path.abspath(__file__))


def _burger_urdf():
    """Expand the TB3 burger xacro with an empty namespace (un-prefixed frames)
    — exactly the robot_state_publisher the real bringup runs on the Pi."""
    urdf = os.path.join(get_package_share_directory('turtlebot3_description'),
                        'urdf', 'turtlebot3_burger.urdf')
    try:
        import xacro
        return xacro.process_file(urdf, mappings={'namespace': ''}).toxml()
    except Exception:
        with open(urdf, encoding='utf-8') as fh:
            return fh.read().replace('${namespace}', '')


def generate_launch_description():
    pkg_share = get_package_share_directory('algae_twin')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('ui', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('start_x', default_value='0.0'),
        DeclareLaunchArgument('start_y', default_value='0.0'),
        DeclareLaunchArgument('start_yaw', default_value='0.0'),

        # the twin-only build, verbatim (same file the lab PC runs)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'twin.launch.py')),
            launch_arguments={
                'rviz': LaunchConfiguration('rviz'),
                'ui': LaunchConfiguration('ui'),
                'headless': LaunchConfiguration('headless'),
            }.items()),

        # --- the EMULATED real robot (replaces turtlebot3_bringup on the Pi) ---
        ExecuteProcess(
            cmd=['python3', os.path.join(THIS_DIR, 'fake_robot.py'),
                 '--ros-args',
                 '-p', ['start_x:=', LaunchConfiguration('start_x')],
                 '-p', ['start_y:=', LaunchConfiguration('start_y')],
                 '-p', ['start_yaw:=', LaunchConfiguration('start_yaw')]],
            output='screen'),
        # the real robot's own robot_state_publisher (base_footprint->base_scan,
        # wheels...). Un-prefixed frames, wall clock — like the bringup's RSP.
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             name='real_robot_state_publisher', output='screen',
             parameters=[{'robot_description': _burger_urdf(),
                          'use_sim_time': False}]),
    ])
