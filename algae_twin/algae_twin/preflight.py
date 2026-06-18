"""Demo-day go/no-go check: verifies every link of the twin in ~6 seconds.

    ros2 run algae_twin preflight

Listens to the live system (run it next to twin.launch.py), reports each link
with a hint when something is down, and exits non-zero on failure.
"""
import json
import sys
import time

import rclpy
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from .util import battery_voltage, latched_qos, sensor_qos

LISTEN_SEC = 6.0

HINTS = {
    'status': 'twin_bridge not running — is twin.launch.py up?',
    'gazebo': 'no ground truth from Gazebo — check gz sim / bridge',
    'twin lidar': 'no /sim/scan — render backend? try LIBGL_ALWAYS_SOFTWARE=1',
    'mask': 'no /keepout_mask — map_edit waits for /map (map_server up?)',
    'nav2': 'navigate_to_pose action missing — Nav2 lifecycle not active yet',
    'localization': 'no map->base TF — set the robot pose in the UI (AMCL)',
    'robot odom': 'no /odom — robot bringup/WiFi/ROS_DOMAIN_ID/RMW mismatch',
    'robot lidar': 'no /scan — lidar driver on the robot (LDS_MODEL set?)',
    'battery': 'no /battery_state — turtlebot3_node not running on robot',
    'cmd link': 'nobody subscribes /cmd_vel — robot bringup not connected',
    'cmd writer': 'expected exactly ONE /cmd_vel publisher (the mux); >1 means '
                  'Nav2 bypassed the safety bus, 0 means the mux is down',
}


class Preflight(Node):
    def __init__(self):
        super().__init__('preflight')
        self.counts = {}
        self.battery = None
        self.mode = None

        def count(name, extra=None):
            def cb(msg):
                self.counts[name] = self.counts.get(name, 0) + 1
                if extra:
                    extra(msg)
            return cb

        def on_status(msg):
            try:
                self.mode = json.loads(msg.data).get('mode')
            except (json.JSONDecodeError, TypeError):
                pass

        def on_battery(msg):
            self.battery = battery_voltage(msg)

        self.create_subscription(String, '/ui/status',
                                 count('status', on_status), 10)
        self.create_subscription(Odometry, '/sim/ground_truth',
                                 count('gazebo'), 10)
        self.create_subscription(LaserScan, '/sim/scan',
                                 count('twin lidar'), sensor_qos())
        self.create_subscription(OccupancyGrid, '/keepout_mask',
                                 count('mask'), latched_qos())
        self.create_subscription(Odometry, '/odom', count('robot odom'), 10)
        self.create_subscription(LaserScan, '/scan',
                                 count('robot lidar'), sensor_qos())
        self.create_subscription(BatteryState, '/battery_state',
                                 count('battery', on_battery), 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


def main(args=None):
    rclpy.init(args=args)
    node = Preflight()
    deadline = time.monotonic() + LISTEN_SEC
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

    mode = node.mode or 'unknown'
    sim_only = mode == 'sim'
    nav_ok = ActionClient(node, NavigateToPose,
                          'navigate_to_pose').wait_for_server(timeout_sec=1.0)
    base = 'sim/base_footprint' if sim_only else 'base_footprint'
    loc_ok = node.tf_buffer.can_transform('map', base, rclpy.time.Time())
    cmd_types = {s.topic_type
                 for s in node.get_subscriptions_info_by_topic('/cmd_vel')}
    cmd_pub_count = len(node.get_publishers_info_by_topic('/cmd_vel'))

    checks = [
        # name, ok, detail, required?
        ('status', node.counts.get('status', 0) > 2,
         f'mode={mode}', True),
        ('gazebo', node.counts.get('gazebo', 0) > 5, 'ground truth', True),
        ('twin lidar', node.counts.get('twin lidar', 0) > 2, '/sim/scan', True),
        ('mask', node.counts.get('mask', 0) >= 1, 'keepout pipeline', True),
        ('nav2', nav_ok, 'navigate_to_pose server', True),
        ('localization', loc_ok, f'map->{base}', True),
        ('robot odom', node.counts.get('robot odom', 0) > 5, '/odom',
         not sim_only),
        ('robot lidar', node.counts.get('robot lidar', 0) > 2, '/scan',
         not sim_only),
        ('battery', node.counts.get('battery', 0) > 2,
         f'{node.battery:.1f} V' if node.battery is not None else '—',
         not sim_only),
        ('cmd link', bool(cmd_types),
         ' / '.join(t.rsplit("/", 1)[-1] for t in sorted(cmd_types)) or '—',
         not sim_only),
        ('cmd writer', cmd_pub_count == 1, f'{cmd_pub_count} publisher(s)',
         not sim_only),
    ]

    failed = 0
    print(f'\nAlgae Twin preflight ({mode} mode)')
    print('-' * 58)
    for name, ok, detail, required in checks:
        if ok:
            mark = 'OK  '
        elif required:
            mark = 'FAIL'
            failed += 1
        else:
            mark = 'skip'
        line = f'  {mark}  {name:<13} {detail}'
        if not ok and required:
            line += f'\n        hint: {HINTS[name]}'
        print(line)
    print('-' * 58)
    print('GO — all links up\n' if failed == 0 else
          f'NO-GO — {failed} link(s) down\n')

    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(0 if failed == 0 else 1)


if __name__ == '__main__':
    main()
