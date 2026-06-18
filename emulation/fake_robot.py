#!/usr/bin/env python3
"""Wall-clock emulator of the physical TurtleBot3 Burger's ROS interface.

This stands in for `turtlebot3_bringup robot.launch.py` running on the robot's
Raspberry Pi, so the twin-only build can be exercised on a laptop with NO
hardware — and exactly the way it will run in the lab. It publishes precisely
what the real robot puts on the network, on the WALL clock (the real robot is
the time authority; the twin-only build runs use_sim_time:=false):

  * /odom            nav_msgs/Odometry        (diff-drive integration of /cmd_vel)
  * TF odom->base_footprint
  * /scan            sensor_msgs/LaserScan    (LDS-02: 360 beams, 0.16-8 m) — a
                     ray-cast of the SAME occupancy map Nav2 localises against,
                     so AMCL gets a real scan-to-map match (exactly like the lab)
  * /battery_state   sensor_msgs/BatteryState (OpenCR pack; slow discharge)
  * /joint_states    sensor_msgs/JointState   (wheel angles, for the robot model)
  * /sound           turtlebot3_msgs/srv/Sound (best-effort beep, like the OpenCR)

It subscribes /cmd_vel as TwistStamped — the TB3 jazzy bringup default — so the
twin_bridge mux auto-detects TwistStamped and the link is identical to the robot.

Run standalone (no algae_twin import needed beyond the installed map):
    python3 fake_robot.py --ros-args -p start_x:=0.0
"""
import math
import os

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PointStamped, TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, JointState, LaserScan
from std_msgs.msg import Empty
from tf2_ros import TransformBroadcaster

# burger geometry (matches turtlebot3_description / models/burger_twin)
LIDAR_X = -0.032        # base_scan offset from base_footprint (m, x)
WHEEL_SEP = 0.160
WHEEL_RADIUS = 0.033
ROBOT_RADIUS = 0.12     # for the soft wall-collision clamp
RANGE_MIN, RANGE_MAX = 0.16, 8.0
N_BEAMS = 360


def _load_map():
    """Load the package's occupancy map into a boolean 'occupied' grid."""
    from ament_index_python.packages import get_package_share_directory
    maps_dir = os.path.join(get_package_share_directory('algae_twin'), 'maps')
    with open(os.path.join(maps_dir, 'map.yaml'), encoding='utf-8') as fh:
        meta = yaml.safe_load(fh)
    img_path = meta['image']
    if not os.path.isabs(img_path):
        img_path = os.path.join(maps_dir, img_path)
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    negate = int(meta.get('negate', 0))
    occ_th = float(meta.get('occupied_thresh', 0.65))

    with open(img_path, 'rb') as fh:
        assert fh.readline().strip() == b'P5', 'expected a binary (P5) PGM map'
        line = fh.readline()
        while line.startswith(b'#'):
            line = fh.readline()
        width, height = (int(v) for v in line.split())
        int(fh.readline())                       # maxval
        pixels = np.frombuffer(fh.read(width * height),
                               dtype=np.uint8).reshape(height, width)
    prob = pixels / 255.0 if negate else (255.0 - pixels) / 255.0
    occupied = prob > occ_th                     # only solid walls stop a ray
    return occupied, res, ox, oy, width, height


class FakeRobot(Node):
    def __init__(self):
        super().__init__('fake_robot')
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('scan_rate', 5.0)
        self.declare_parameter('odom_rate', 30.0)
        self.declare_parameter('scan_noise', 0.01)
        self.declare_parameter('battery_start', 12.4)
        self.declare_parameter('battery_floor', 11.6)
        self.declare_parameter('obstacle_size', 0.30)   # emulated 'real' box (m)

        self.x = float(self.get_parameter('start_x').value)
        self.y = float(self.get_parameter('start_y').value)
        self.yaw = float(self.get_parameter('start_yaw').value)
        self.v = 0.0
        self.w = 0.0
        self.wheel_l = 0.0
        self.wheel_r = 0.0
        self.voltage = float(self.get_parameter('battery_start').value)
        self.batt_floor = float(self.get_parameter('battery_floor').value)
        self.scan_noise = float(self.get_parameter('scan_noise').value)

        self.occ_static, self.res, self.ox, self.oy, self.mw, self.mh = _load_map()
        self.occ = self.occ_static.copy()   # working grid: static map + any
        #                       emulated 'real' obstacles dropped at runtime
        self.beam_angles = np.linspace(0.0, 2.0 * math.pi, N_BEAMS,
                                       endpoint=False)
        self.steps = np.arange(RANGE_MIN, RANGE_MAX, self.res * 0.5)
        self.get_logger().info(
            f'fake_robot: map {self.mw}x{self.mh} @ {self.res} m, '
            f'start ({self.x:.2f}, {self.y:.2f}, {self.yaw:.2f})')

        self.tf = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan',
                                              qos_profile_sensor_data)
        self.batt_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
        # TB3 jazzy bringup subscribes TwistStamped on /cmd_vel
        self.create_subscription(TwistStamped, '/cmd_vel', self._on_cmd, 10)
        # drop / clear a REAL obstacle the static map can't explain, so
        # obstacle_mirror mirrors it into the twin (physical -> digital test)
        self.create_subscription(PointStamped, '/fake_world/add_box',
                                 self._on_add_box, 10)
        self.create_subscription(Empty, '/fake_world/clear',
                                 self._on_clear_boxes, 10)
        self._make_sound_service()

        odom_dt = 1.0 / float(self.get_parameter('odom_rate').value)
        self.odom_dt = odom_dt
        self.create_timer(odom_dt, self._step)
        self.create_timer(1.0 / float(self.get_parameter('scan_rate').value),
                          self._publish_scan)
        self.create_timer(1.0, self._publish_battery)

    def _make_sound_service(self):
        try:
            from turtlebot3_msgs.srv import Sound
            self.create_service(Sound, '/sound', self._on_sound)
        except Exception:
            self.get_logger().info('turtlebot3_msgs/Sound unavailable — no /sound')

    def _on_sound(self, request, response):
        self.get_logger().info(f'\U0001F50A beep (sound value={request.value})')
        return response

    def _on_cmd(self, msg):
        self.v = msg.twist.linear.x
        self.w = msg.twist.angular.z

    def _on_add_box(self, msg):
        """Drop a virtual real-world box into the lidar's view; the static map
        can't explain it, so obstacle_mirror mirrors it into the twin."""
        half = max(1, int(self.get_parameter('obstacle_size').value
                          / 2.0 / self.res))
        col = int((msg.point.x - self.ox) / self.res)
        row = self.mh - 1 - int((msg.point.y - self.oy) / self.res)
        self.occ[max(0, row - half):row + half + 1,
                 max(0, col - half):col + half + 1] = True
        self.get_logger().info(
            f'emulated REAL obstacle at ({msg.point.x:.2f}, {msg.point.y:.2f}) '
            '— the lidar now sees it')

    def _on_clear_boxes(self, _msg):
        self.occ = self.occ_static.copy()
        self.get_logger().info('cleared emulated real obstacles')

    # -- kinematics + odometry -------------------------------------------------
    def _step(self):
        dt = self.odom_dt
        nx = self.x + self.v * math.cos(self.yaw) * dt
        ny = self.y + self.v * math.sin(self.yaw) * dt
        # soft wall collision: don't drive the body centre into an occupied cell
        if self.v <= 0.0 or not self._blocked(nx, ny):
            self.x, self.y = nx, ny
        self.yaw = math.atan2(math.sin(self.yaw + self.w * dt),
                              math.cos(self.yaw + self.w * dt))
        self.wheel_l += (self.v - self.w * WHEEL_SEP / 2) / WHEEL_RADIUS * dt
        self.wheel_r += (self.v + self.w * WHEEL_SEP / 2) / WHEEL_RADIUS * dt

        now = self.get_clock().now().to_msg()
        qz, qw = math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0)

        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_footprint'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
        self.tf.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = qz, qw
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w
        self.odom_pub.publish(odom)

        js = JointState()
        js.header.stamp = now
        js.name = ['wheel_left_joint', 'wheel_right_joint']
        js.position = [self.wheel_l, self.wheel_r]
        self.js_pub.publish(js)

    def _blocked(self, wx, wy):
        """True if the robot body at (wx, wy) would overlap an occupied cell."""
        r = int(ROBOT_RADIUS / self.res)
        col = int((wx - self.ox) / self.res)
        row = self.mh - 1 - int((wy - self.oy) / self.res)
        r0, r1 = max(0, row - r), min(self.mh, row + r + 1)
        c0, c1 = max(0, col - r), min(self.mw, col + r + 1)
        return bool(self.occ[r0:r1, c0:c1].any())

    # -- ray-cast lidar (vectorised over all beams) ----------------------------
    def _publish_scan(self):
        sx = self.x + LIDAR_X * math.cos(self.yaw)
        sy = self.y + LIDAR_X * math.sin(self.yaw)
        ang = self.yaw + self.beam_angles                  # world beam headings
        xs = sx + self.steps[None, :] * np.cos(ang)[:, None]
        ys = sy + self.steps[None, :] * np.sin(ang)[:, None]
        cols = ((xs - self.ox) / self.res).astype(np.int32)
        rows = self.mh - 1 - ((ys - self.oy) / self.res).astype(np.int32)
        inb = (cols >= 0) & (cols < self.mw) & (rows >= 0) & (rows < self.mh)
        hit = np.zeros(xs.shape, dtype=bool)
        hit[inb] = self.occ[rows[inb], cols[inb]]
        first = np.argmax(hit, axis=1)
        has_hit = hit[np.arange(N_BEAMS), first]
        ranges = np.where(has_hit, self.steps[first], RANGE_MAX)
        if self.scan_noise > 0.0:
            ranges = ranges + np.random.normal(0.0, self.scan_noise, N_BEAMS)
        ranges = np.clip(ranges, RANGE_MIN, RANGE_MAX)

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_scan'
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi
        msg.angle_increment = 2.0 * math.pi / N_BEAMS
        msg.range_min = RANGE_MIN
        msg.range_max = RANGE_MAX
        msg.ranges = [float(r) for r in ranges]
        self.scan_pub.publish(msg)

    def _publish_battery(self):
        # a slow discharge so the gauges move; floored above the mission gate
        self.voltage = max(self.batt_floor,
                           self.voltage - (0.02 if abs(self.v) + abs(self.w) > 0.01
                                           else 0.004))
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = float(self.voltage)
        msg.present = True
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.design_capacity = 1.8
        self.batt_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
