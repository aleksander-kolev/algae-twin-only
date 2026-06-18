"""Shadow controller (twin mode): keeps the digital robot glued to the real one.

Continuously measures real-vs-twin pose divergence. Small drift is corrected
smoothly by feeding a bounded corrective velocity into the command mux
(/twin/correction). Large, sustained divergence (kidnapped robot, operator
relocalisation) is fixed by teleporting the twin in Gazebo.
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, Float32
from tf2_ros import Buffer, TransformListener

from .gz_io import GzIo
from .util import (clamp, latched_qos, map_to_world_pose, norm_angle,
                   quat_to_yaw)


class PoseSync(Node):
    def __init__(self):
        super().__init__('pose_sync')
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('kp_lin', 0.6)
        self.declare_parameter('kp_ang', 1.2)
        self.declare_parameter('max_corr_lin', 0.06)
        self.declare_parameter('max_corr_ang', 0.5)
        self.declare_parameter('deadband_lin', 0.02)
        self.declare_parameter('deadband_ang', 0.05)
        self.declare_parameter('teleport_divergence', 1.0)
        self.declare_parameter('teleport_after', 2.0)
        self.declare_parameter('teleport_cooldown', 5.0)
        self.declare_parameter('model', 'burger_twin')
        self.declare_parameter('world', 'default')
        self.declare_parameter('map_to_world', [0.0, 0.0, 0.0])
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.kp_lin, self.kp_ang = p('kp_lin'), p('kp_ang')
        self.max_lin, self.max_ang = p('max_corr_lin'), p('max_corr_ang')
        self.dead_lin, self.dead_ang = p('deadband_lin'), p('deadband_ang')
        self.tp_div = p('teleport_divergence')
        self.tp_after = p('teleport_after')
        self.tp_cooldown = p('teleport_cooldown')
        self.model = p('model')
        self.map_to_world = list(p('map_to_world'))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.gz = GzIo(self, p('world'))
        self.sim_pose = None
        self.diverged_since = None
        self.last_teleport = 0.0
        self.estop = False
        self.clean_active = False
        self._resync_req = False     # operator pressed "Sync twin -> real"

        self.create_subscription(Odometry, '/sim/ground_truth',
                                 self._on_ground_truth, 10)
        self.create_subscription(Bool, '/estop', self._on_estop, latched_qos())
        self.create_subscription(Bool, '/clean/active', self._on_clean_active,
                                 latched_qos())
        # manual resync: snap the twin onto the real robot now (UI button / topic)
        self.create_subscription(Empty, '/twin/resync', self._on_resync, 10)
        self.corr_pub = self.create_publisher(Twist, '/twin/correction', 10)
        self.div_pub = self.create_publisher(Float32, '/twin/divergence', 10)
        self.create_timer(1.0 / p('rate'), self._tick)
        self.get_logger().info('pose_sync up — twin shadowing enabled')

    def _on_estop(self, msg):
        self.estop = msg.data

    def _on_resync(self, _msg):
        self._resync_req = True

    def _on_clean_active(self, msg):
        self.clean_active = msg.data

    def _on_ground_truth(self, msg):
        pose = msg.pose.pose
        yaw = quat_to_yaw(pose.orientation.x, pose.orientation.y,
                          pose.orientation.z, pose.orientation.w)
        ox, oy, oyaw = self.map_to_world
        cos_o, sin_o = math.cos(oyaw), math.sin(oyaw)
        x, y = pose.position.x - ox, pose.position.y - oy
        self.sim_pose = (cos_o * x + sin_o * y,
                         -sin_o * x + cos_o * y,
                         norm_angle(yaw - oyaw))

    def _real_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_footprint',
                                                 rclpy.time.Time())
        except Exception:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return (t.x, t.y, quat_to_yaw(q.x, q.y, q.z, q.w))

    def _tick(self):
        if self._resync_req:                  # operator pressed "Sync twin -> real"
            self._resync_req = False
            forced = self._real_pose()
            if forced is not None:
                self._force_teleport(*forced)
            else:
                self.get_logger().warning(
                    'RESYNC ignored — real pose unknown; localize the robot first '
                    '(2D Pose Estimate)')
        if self.estop:                        # E-STOP freezes the twin:
            self.corr_pub.publish(Twist())    # zero correction, no teleport
            return
        real = self._real_pose()
        if real is None or self.sim_pose is None:
            return
        rx, ry, ryaw = real
        sx, sy, syaw = self.sim_pose
        dx, dy = rx - sx, ry - sy
        dist = math.hypot(dx, dy)
        self.div_pub.publish(Float32(data=float(dist)))

        corr = Twist()
        if dist > self.dead_lin:
            forward_err = math.cos(syaw) * dx + math.sin(syaw) * dy
            corr.linear.x = clamp(self.kp_lin * forward_err,
                                  -self.max_lin, self.max_lin)
        heading_err = (norm_angle(math.atan2(dy, dx) - syaw) if dist > 0.10
                       else norm_angle(ryaw - syaw))
        if abs(heading_err) > self.dead_ang:
            corr.angular.z = clamp(self.kp_ang * heading_err,
                                   -self.max_ang, self.max_ang)
        self.corr_pub.publish(corr)
        self._maybe_teleport(dist, rx, ry, ryaw)

    def _maybe_teleport(self, dist, rx, ry, ryaw):
        # don't teleport the twin mid-clean: a discontinuous jump while the
        # operator watches the 3-spin dispersal breaks the "both move together"
        # beat. Severe divergence resolves right after cleaning ends.
        if self.clean_active:
            return
        now = time.monotonic()
        if dist < self.tp_div:
            self.diverged_since = None
            return
        if self.diverged_since is None:
            self.diverged_since = now
            return
        if (now - self.diverged_since > self.tp_after
                and now - self.last_teleport > self.tp_cooldown):
            wx, wy, wyaw = map_to_world_pose(rx, ry, ryaw, self.map_to_world)
            self.get_logger().warn(
                f'twin diverged {dist:.2f} m — teleporting to real pose')
            self.gz.set_pose(self.model, wx, wy, wyaw, z=0.01)
            self.last_teleport = now
            self.diverged_since = None

    def _force_teleport(self, rx, ry, ryaw):
        """Operator-triggered "Sync twin -> real": snap the twin onto the real
        robot NOW, bypassing the divergence threshold / cooldown that gate the
        automatic teleport. Needs a known real pose (AMCL localized)."""
        wx, wy, wyaw = map_to_world_pose(rx, ry, ryaw, self.map_to_world)
        self.get_logger().warn('manual RESYNC — teleporting twin onto the real robot')
        self.gz.set_pose(self.model, wx, wy, wyaw, z=0.01)
        self.last_teleport = time.monotonic()
        self.diverged_since = None


def main(args=None):
    rclpy.init(args=args)
    node = PoseSync()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
