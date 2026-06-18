"""The ONE navigation brain — drives the REAL robot; the Gazebo twin shadows it.

Renders config/nav2_params.yaml and starts the Nav2 nodes. The per-mode
frame/topic/clock values are already baked into the params for the real-robot
twin (base_footprint/odom frames, /scan + /odom topics, wall clock); only the
map path and the robot's initial pose are rendered here. Velocity output is
remapped to /nav_cmd_vel — only the twin_bridge mux talks to actual robots.
"""
import atexit
import math
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

NAV_NODES = ['map_server', 'amcl', 'costmap_filter_info_server',
             'controller_server', 'planner_server', 'behavior_server',
             'bt_navigator']


def render_params(context):
    pkg_share = get_package_share_directory('algae_twin')
    tokens = {'@MAP_YAML@': context.launch_configurations['map']}
    # Coerce pose values to FINITE float strings so e.g. `spawn_x:=2` renders
    # `x: 2.0` (a bare `2` is a YAML int, but AMCL declares initial_pose.x as a
    # double and ROS 2 rejects an int override -> AMCL fails to configure).
    for token, arg in (('@INIT_X@', 'spawn_x'), ('@INIT_Y@', 'spawn_y'),
                       ('@INIT_YAW@', 'spawn_yaw')):
        value = float(context.launch_configurations[arg])
        if not math.isfinite(value):
            raise RuntimeError(f"{arg} must be a finite number, got "
                               f"{context.launch_configurations[arg]!r}")
        tokens[token] = repr(value)

    with open(os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
              encoding='utf-8') as fh:
        rendered = fh.read()
    for token, value in tokens.items():
        rendered = rendered.replace(token, value)

    out = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', prefix='algae_twin_nav2_', delete=False)
    out.write(rendered)
    out.close()

    def _cleanup(path=out.name):     # don't leak the rendered file on shutdown
        try:
            os.remove(path)
        except OSError:
            pass
    atexit.register(_cleanup)

    cmd_vel_remap = [('cmd_vel', '/nav_cmd_vel')]
    nodes = [
        Node(package='nav2_map_server', executable='map_server',
             name='map_server', output='screen', parameters=[out.name]),
        Node(package='nav2_amcl', executable='amcl',
             name='amcl', output='screen', parameters=[out.name]),
        Node(package='nav2_map_server', executable='costmap_filter_info_server',
             name='costmap_filter_info_server', output='screen',
             parameters=[out.name]),
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen',
             parameters=[out.name], remappings=cmd_vel_remap),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen', parameters=[out.name]),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen',
             parameters=[out.name], remappings=cmd_vel_remap),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen', parameters=[out.name]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'autostart': True,
                          'node_names': NAV_NODES,
                          'bond_timeout': 0.0,
                          'use_sim_time': False}]),
    ]
    return nodes


def generate_launch_description():
    pkg_share = get_package_share_directory('algae_twin')
    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value=os.path.join(pkg_share, 'maps', 'map.yaml')),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        OpaqueFunction(function=render_params),
    ])
