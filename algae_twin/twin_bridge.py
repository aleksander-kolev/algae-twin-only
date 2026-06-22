"""Command mux + twin status aggregator.

The single point through which every velocity command flows:

    Nav2 (/nav_cmd_vel) ─┐
    cleaner (/clean_cmd_vel) ─┼─> mux ─┬─> /cmd_vel       (real robot)
    e-stop  (/estop)     ─┘           └─> /sim/cmd_vel   (digital twin,
                                            + shadow correction)

Also publishes /ui/status (JSON) — the one snapshot the operator UI renders.
"""
import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import Bool, Float32, String
from tf2_ros import Buffer, TransformListener

from .util import (battery_voltage, clamp, latched_qos, norm_angle,
                   quat_to_yaw, sensor_qos, twist_of)


class TwinBridge(Node):
    def __init__(self):
        super().__init__('twin_bridge')
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('real_cmd_stamped', True)
        self.declare_parameter('real_cmd_autodetect', True)
        self.declare_parameter('cmd_timeout', 0.6)
        self.declare_parameter('max_lin', 0.22)
        self.declare_parameter('max_ang', 2.84)
        self.declare_parameter('map_to_world', [0.0, 0.0, 0.0])
        # front-cone collision safety-stop (real robot, twin mode): cut forward
        # motion when the lidar sees an obstacle within stop_distance; latch the
        # cut until the front is clear past release_distance for release_hold s;
        # FAIL SAFE (block) when /scan is stale. Rotation/reverse stay allowed.
        self.declare_parameter('safety_enable', True)
        self.declare_parameter('safety_stop_distance', 0.25)
        self.declare_parameter('safety_release_distance', 0.35)
        self.declare_parameter('safety_release_hold', 0.3)
        self.declare_parameter('safety_front_angle', 0.5)   # half-cone (rad)
        self.declare_parameter('safety_scan_timeout', 0.5)
        self.mode = 'twin'   # twin-only build: real robot + Gazebo twin
        self.real_stamped = self.get_parameter('real_cmd_stamped').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.max_lin = self.get_parameter('max_lin').value
        self.max_ang = self.get_parameter('max_ang').value
        self.map_to_world = list(self.get_parameter('map_to_world').value)
        self.safety_enable = self.get_parameter('safety_enable').value
        self.stop_dist = self.get_parameter('safety_stop_distance').value
        self.release_dist = self.get_parameter('safety_release_distance').value
        self.release_hold = self.get_parameter('safety_release_hold').value
        self.front_half_angle = self.get_parameter('safety_front_angle').value
        self.scan_timeout = self.get_parameter('safety_scan_timeout').value

        self.estop = False
        self.last = {}      # name -> (stamp, msg)
        self.clean_active = False
        self.divergence = None
        self.last_scan = None
        self.last_scan_t = 0.0
        self.safety_blocked = False
        self.clear_since = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Twist, '/nav_cmd_vel',
                                 lambda m: self._keep('nav', m), 10)
        # stock Jazzy teleop_keyboard publishes TwistStamped. Two message types
        # on ONE topic name is unsupported DDS behaviour (it happens to work on
        # Fast DDS but fails at creation on CycloneDDS), so the stamped flavour
        # gets its OWN topic — teleop with `-r /cmd_vel:=/nav_cmd_vel_stamped`.
        self.create_subscription(TwistStamped, '/nav_cmd_vel_stamped',
                                 lambda m: self._keep('nav', twist_of(m)), 10)
        self.create_subscription(Twist, '/clean_cmd_vel',
                                 lambda m: self._keep('clean', m), 10)
        self.create_subscription(Twist, '/twin/correction',
                                 lambda m: self._keep('corr', m), 10)
        self.create_subscription(Bool, '/estop', self._on_estop, latched_qos())
        self.create_subscription(Bool, '/clean/active',
                                 self._on_clean_active, latched_qos())
        self.create_subscription(Float32, '/twin/divergence',
                                 self._on_divergence, 10)
        self.create_subscription(Odometry, '/odom',
                                 lambda m: self._keep('real_odom', m), 10)
        self.create_subscription(Odometry, '/sim/ground_truth',
                                 lambda m: self._keep('sim_gt', m), 10)
        self.create_subscription(BatteryState, '/battery_state',
                                 lambda m: self._keep('real_batt', m), 10)
        self.create_subscription(BatteryState, '/sim/battery_state',
                                 lambda m: self._keep('sim_batt', m), 10)

        self.sim_cmd_pub = self.create_publisher(Twist, '/sim/cmd_vel', 10)
        cmd_type = TwistStamped if self.real_stamped else Twist
        self.real_cmd_pub = self.create_publisher(cmd_type, '/cmd_vel', 10)
        if self.get_parameter('real_cmd_autodetect').value:
            self._detect_timer = self.create_timer(
                2.0, self._detect_real_cmd_type)
        else:
            self.get_logger().warn(
                'real_cmd_autodetect is OFF — if real_cmd_stamped '
                f'({self.real_stamped}) is wrong the robot ignores ALL '
                '/cmd_vel (TB3 jazzy default is TwistStamped)')
        if self.safety_enable:
            self.create_subscription(LaserScan, '/scan',
                                     self._on_scan, sensor_qos())
        self.status_pub = self.create_publisher(String, '/ui/status', 10)

        period = 1.0 / self.get_parameter('rate').value
        self.create_timer(period, self._mux_tick)
        self.create_timer(0.2, self._status_tick)
        self.get_logger().info(f'twin_bridge up (mode={self.mode})')

    # -- bookkeeping -----------------------------------------------------------
    def _keep(self, name, msg):
        self.last[name] = (time.monotonic(), msg)

    def _age(self, name):
        entry = self.last.get(name)
        return None if entry is None else time.monotonic() - entry[0]

    def _fresh(self, name, timeout=None):
        age = self._age(name)
        return age is not None and age < (timeout or self.cmd_timeout)

    def _detect_real_cmd_type(self):
        """Match the real robot's actual /cmd_vel flavour (Twist vs
        TwistStamped differs between TB3 bringup configs)."""
        types = {s.topic_type
                 for s in self.get_subscriptions_info_by_topic('/cmd_vel')}
        stamped = 'geometry_msgs/msg/TwistStamped' in types
        plain = 'geometry_msgs/msg/Twist' in types
        if stamped == plain:      # robot not up yet, or inconclusive: keep polling
            return
        if stamped != self.real_stamped:
            self.get_logger().warn(
                f"real robot expects {'TwistStamped' if stamped else 'Twist'} "
                'on /cmd_vel — switching publisher')
            self.destroy_publisher(self.real_cmd_pub)
            self.real_cmd_pub = self.create_publisher(
                TwistStamped if stamped else Twist, '/cmd_vel', 10)
            self.real_stamped = stamped
        else:
            self.get_logger().info(
                f"real robot /cmd_vel type confirmed "
                f"({'TwistStamped' if stamped else 'Twist'})")
        self.destroy_timer(self._detect_timer)

    def _on_estop(self, msg):
        if msg.data != self.estop:
            self.get_logger().warn(
                'E-STOP ENGAGED' if msg.data else 'e-stop released')
        self.estop = msg.data

    def _on_clean_active(self, msg):
        self.clean_active = msg.data

    def _on_divergence(self, msg):
        self.divergence = msg.data

    def _on_scan(self, msg):
        self.last_scan = msg
        self.last_scan_t = time.monotonic()

    # -- collision safety-stop (real robot, twin mode) ---------------------------
    def _front_min_range(self, scan):
        best = float('inf')
        angle = scan.angle_min
        for rng in scan.ranges:
            a, angle = angle, angle + scan.angle_increment
            na = math.atan2(math.sin(a), math.cos(a))   # wrap forward to 0
            if (abs(na) <= self.front_half_angle
                    and scan.range_min < rng < scan.range_max
                    and rng < best):
                best = rng
        return best

    def _safety_blocked(self):
        """True when forward motion must be cut. Latches at stop_distance, holds
        until clear past release_distance for release_hold s, fails SAFE on a
        stale/absent scan. safety_enable only."""
        if not self.safety_enable:
            return False
        now = time.monotonic()
        if self.last_scan is None or now - self.last_scan_t > self.scan_timeout:
            self.safety_blocked = True       # no fresh scan -> assume blocked
            self.clear_since = None
            return True
        front = self._front_min_range(self.last_scan)
        if self.safety_blocked:
            if front > self.release_dist:
                if self.clear_since is None:
                    self.clear_since = now
                elif now - self.clear_since >= self.release_hold:
                    self.safety_blocked = False
                    self.clear_since = None
            else:
                self.clear_since = None
        elif front < self.stop_dist:
            self.safety_blocked = True
            self.clear_since = None
        return self.safety_blocked

    # -- mux ---------------------------------------------------------------------
    def _mux_tick(self):
        base = Twist()
        if not self.estop:
            src = (self.last['clean'][1] if self._fresh('clean')
                   else self.last['nav'][1] if self._fresh('nav') else None)
            if src is not None:
                # clamp to the burger envelope for BOTH robots — the real robot
                # cannot exceed it, so keep the twin commanded to match.
                base.linear.x = clamp(src.linear.x, -self.max_lin, self.max_lin)
                base.angular.z = clamp(src.angular.z, -self.max_ang, self.max_ang)

        # cut forward motion (keep rotation + reverse) when the front cone is
        # blocked; applies to BOTH robots so the twin stops with the real one.
        blocked = self._safety_blocked()
        if blocked and base.linear.x > 0.0:
            base.linear.x = 0.0

        self.real_cmd_pub.publish(self._stamp(base) if self.real_stamped
                                  else base)

        sim_cmd = Twist()
        sim_cmd.linear.x, sim_cmd.angular.z = base.linear.x, base.angular.z
        if not self.estop and self._fresh('corr'):
            corr = self.last['corr'][1]
            sim_cmd.linear.x += corr.linear.x
            sim_cmd.angular.z += corr.angular.z
        sim_cmd.linear.x = clamp(sim_cmd.linear.x, -self.max_lin, self.max_lin)
        sim_cmd.angular.z = clamp(sim_cmd.angular.z, -self.max_ang, self.max_ang)
        if blocked and sim_cmd.linear.x > 0.0:
            sim_cmd.linear.x = 0.0
        self.sim_cmd_pub.publish(sim_cmd)

    def _stamp(self, twist):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist = twist
        return msg

    # -- status ------------------------------------------------------------------
    def _pose_from_tf(self, base_frame):
        try:
            tf = self.tf_buffer.lookup_transform('map', base_frame,
                                                 rclpy.time.Time())
        except Exception:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return [round(t.x, 3), round(t.y, 3),
                round(quat_to_yaw(q.x, q.y, q.z, q.w), 3)]

    def _sim_pose(self):
        entry = self.last.get('sim_gt')
        if entry is None:
            return None
        pose = entry[1].pose.pose
        yaw = quat_to_yaw(pose.orientation.x, pose.orientation.y,
                          pose.orientation.z, pose.orientation.w)
        ox, oy, oyaw = self.map_to_world
        # world -> map (twin assumption: world ≈ map, offset configurable)
        cos_o, sin_o = math.cos(oyaw), math.sin(oyaw)
        x, y = pose.position.x - ox, pose.position.y - oy
        return [round(cos_o * x + sin_o * y, 3),
                round(-sin_o * x + cos_o * y, 3),
                round(norm_angle(yaw - oyaw), 3)]

    def _battery_voltage(self, name):
        entry = self.last.get(name)
        if entry is None or self._age(name) > 5.0:
            return None
        volt = battery_voltage(entry[1])
        return round(volt, 2) if volt is not None else None

    def _status_tick(self):
        real_volt = self._battery_voltage('real_batt')
        sim_volt = self._battery_voltage('sim_batt')
        real_age = self._age('real_odom')
        sim_age = self._age('sim_gt')
        status = {
            'mode': self.mode,
            'estop': self.estop,
            'clean_active': self.clean_active,
            'real': {
                'pose': self._pose_from_tf('base_footprint'),
                'ok': real_age is not None and real_age < 2.0,
                'age': round(real_age, 2) if real_age is not None else None,
                'voltage': real_volt,
            },
            'sim': {
                'pose': self._sim_pose(),
                'ok': sim_age is not None and sim_age < 2.0,
                'age': round(sim_age, 2) if sim_age is not None else None,
                'voltage': sim_volt,
            },
            'divergence': self.divergence,
            'safety': self.safety_blocked,
        }
        self.status_pub.publish(String(data=json.dumps(status)))


def main(args=None):
    rclpy.init(args=args)
    node = TwinBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
