"""Real world -> digital world obstacle mirroring (twin mode).

Watches the real lidar for persistent returns that the static map cannot
explain (someone put a box in the corridor). Stable detections become
'mirrored' world edits, so the obstacle appears in Gazebo, in the keepout
mask, and on the operator map — the digital twin stays truthful.
"""
import math
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformListener

from .grid import StaticMap
from .util import json_msg, latched_qos, parse_json, quat_to_yaw, sensor_qos


class ObstacleMirror(Node):
    def __init__(self):
        super().__init__('obstacle_mirror')
        self.declare_parameter('bin_size', 0.10)        # detection grid (m)
        self.declare_parameter('min_hits', 8)           # stability threshold
        self.declare_parameter('decay', 0.85)           # per-tick hit decay
        self.declare_parameter('map_clearance', 0.12)   # static-map tolerance
        self.declare_parameter('edit_clearance', 0.25)  # skip near known edits
        self.declare_parameter('self_clearance', 0.25)  # skip near the robot
        self.declare_parameter('box_size', 0.20)        # mirrored box size (m)
        self.declare_parameter('max_range', 8.0)        # LDS-02 spec (msg says 100)
        self.declare_parameter('max_divergence', 0.5)   # skip mirroring if lost
        self.declare_parameter('max_divergence_age', 2.0)  # stale sync == lost
        self.declare_parameter('min_scan_match', 0.5)   # require this fraction of
        #   returns to match the static map before mirroring; below it the AMCL
        #   pose is wrong (not seeded / kidnapped) and EVERY return reads as an
        #   "unmapped obstacle" -> phantom boxes. Gate it on the 2D Pose Estimate.
        self.declare_parameter('demote_enable', True)
        self.declare_parameter('demote_after', 8.0)     # retire unseen mirror (s)
        self.declare_parameter('demote_range', 3.0)     # judge presence within (m)

        self.static_map = None
        self.edit_centers = []
        self.mirrored = {}        # id -> (cx, cy) for source=='mirrored' edits
        self.mirror_seen = {}     # id -> last monotonic time the lidar confirmed it
        self.divergence = None
        self.div_t = 0.0          # monotonic stamp of the last /twin/divergence
        self.scan = None
        self.bins = {}
        self._last_warn = 0.0     # throttle the "not localized" warning

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(OccupancyGrid, '/map', self._on_map,
                                 latched_qos())
        self.create_subscription(String, '/edits/state', self._on_edits,
                                 latched_qos())
        self.create_subscription(LaserScan, '/scan', self._on_scan,
                                 sensor_qos())
        self.create_subscription(Float32, '/twin/divergence',
                                 self._on_divergence, 10)
        self.add_pub = self.create_publisher(String, '/edits/add', 10)
        self.remove_pub = self.create_publisher(String, '/edits/remove', 10)
        self.create_timer(0.5, self._tick)
        self.get_logger().info('obstacle_mirror up — watching the real lidar')

    def _on_map(self, msg):
        self.static_map = StaticMap(msg)

    def _on_edits(self, msg):
        payload = parse_json(msg, self.get_logger(), '/edits/state') or {}
        edits = payload.get('edits', [])
        self.edit_centers = [(e['cx'], e['cy']) for e in edits]
        self.mirrored = {e['id']: (e['cx'], e['cy'])
                         for e in edits if e.get('source') == 'mirrored'}
        self.mirror_seen = {k: v for k, v in self.mirror_seen.items()
                            if k in self.mirrored}

    def _on_divergence(self, msg):
        self.divergence = msg.data
        self.div_t = time.monotonic()

    def _on_scan(self, msg):
        self.scan = msg

    def _warn_throttled(self, msg, period=5.0):
        now = time.monotonic()
        if now - self._last_warn >= period:
            self.get_logger().warning(msg)
            self._last_warn = now

    def _robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_scan',
                                                 rclpy.time.Time())
        except Exception:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return t.x, t.y, quat_to_yaw(q.x, q.y, q.z, q.w)

    def _tick(self):
        if self.static_map is None or self.scan is None:
            return
        pose = self._robot_pose()
        if pose is None:
            return
        # a diverged / mislocalised pose projects returns into the WRONG cells,
        # which would both spawn phantoms and feed a positive-feedback loop into
        # the shared plan — so skip mirroring until localization is trustworthy
        # again (the "a diverged mirror is ignored" rule the lab demo relies on).
        # A STALE divergence (pose_sync dead / in E-STOP) is treated as lost too
        # — fail safe rather than trust a frozen "all good" reading.
        now = time.monotonic()
        diverged = (self.divergence is not None and self.divergence
                    > self.get_parameter('max_divergence').value)
        stale = (self.div_t > 0.0 and now - self.div_t
                 > self.get_parameter('max_divergence_age').value)
        if diverged or stale:
            self.scan = None
            return
        px, py, pyaw = pose
        scan, self.scan = self.scan, None
        bin_size = self.get_parameter('bin_size').value
        map_clear = self.get_parameter('map_clearance').value
        self_clear = self.get_parameter('self_clearance').value
        # the real LDS-02 driver reports range_max=100 (placeholder); clamp to
        # the sensor's real reach so far spurious returns can't mirror phantoms.
        max_range = min(scan.range_max, self.get_parameter('max_range').value)

        # tally this scan's unexplained returns per detection bin, and measure how
        # much of the scan the static map explains (a LOCALIZATION check).
        hits = {}
        total = explained = 0
        angle = scan.angle_min
        for i, rng in enumerate(scan.ranges):
            angle_i, angle = angle, angle + scan.angle_increment
            # skip every other beam, self-returns (< self_clear, which already
            # exceeds range_min), and over-range returns
            if i % 2 or not (self_clear < rng < max_range):
                continue
            total += 1
            wx = px + rng * math.cos(pyaw + angle_i)
            wy = py + rng * math.sin(pyaw + angle_i)
            if self.static_map.occupied_near(wx, wy, map_clear):
                explained += 1
                continue   # explained by the base map
            key = (round(wx / bin_size), round(wy / bin_size))
            hits[key] = hits.get(key, 0.0) + 1.0

        # LOCALIZATION GATE: if the static map explains too little of the scan, the
        # AMCL pose is wrong (not seeded / kidnapped). EVERY return then reads as an
        # "unmapped obstacle" and we'd spawn phantom boxes one-by-one all over the
        # map. Refuse to mirror until the scan matches the map (operator did the 2D
        # Pose Estimate). This is what stops phantom spawning before localization.
        min_match = self.get_parameter('min_scan_match').value
        if total > 0 and explained < min_match * total:
            self._warn_throttled(
                f'scan matches the map only {explained}/{total} '
                f'(< {min_match:.0%}) — pose not localized; skipping mirror '
                '(do the 2D Pose Estimate)')
            return

        self._demote(px, py, hits, bin_size)

        # Bins seen this tick ACCUMULATE; bins not seen decay and are pruned.
        # Accumulating (rather than decay-then-add) is what lets a persistent but
        # sparse return — e.g. a single beam/tick on a >2 m obstacle — eventually
        # cross min_hits, instead of asymptoting at 1/(1-decay) below it.
        decay = self.get_parameter('decay').value
        bins = {}
        for key, value in self.bins.items():
            if key in hits:
                bins[key] = value + hits[key]
            elif value * decay > 0.5:
                bins[key] = value * decay
        for key, count in hits.items():
            bins.setdefault(key, count)
        self.bins = bins

        self._promote(bin_size)

    def _promote(self, bin_size):
        min_hits = self.get_parameter('min_hits').value
        edit_clear = self.get_parameter('edit_clearance').value
        box = self.get_parameter('box_size').value
        for key, hits in list(self.bins.items()):
            if key not in self.bins or hits < min_hits:
                continue
            cx, cy = key[0] * bin_size, key[1] * bin_size
            near_known = any(math.hypot(cx - ex, cy - ey) < edit_clear
                             for ex, ey in self.edit_centers)
            if not near_known:
                self.get_logger().info(
                    f'unmapped obstacle at ({cx:.2f}, {cy:.2f}) — mirroring '
                    'into the digital world')
                self.add_pub.publish(json_msg({
                    'cx': cx, 'cy': cy, 'size_x': box, 'size_y': box,
                    'yaw': 0.0, 'source': 'mirrored'}))
            # drop the neighbourhood either way to avoid re-triggering
            for other in [k for k in self.bins
                          if abs(k[0] - key[0]) <= 2 and abs(k[1] - key[1]) <= 2]:
                del self.bins[other]

    def _demote(self, px, py, hits, bin_size):
        """Retire a mirrored obstacle once the robot is close enough to see its
        cell yet the lidar no longer returns anything there — so a transient real
        obstacle (a person who moved on) doesn't stay a permanent keepout box.
        The 360deg lidar still scans the cell from a distance even while the
        planner routes around the keepout, so absence is observable."""
        if not self.get_parameter('demote_enable').value or not self.mirrored:
            return
        now = time.monotonic()
        demote_after = self.get_parameter('demote_after').value
        demote_range = self.get_parameter('demote_range').value
        span = max(1, int(round(self.get_parameter('box_size').value / bin_size)))
        for eid, (cx, cy) in list(self.mirrored.items()):
            if math.hypot(cx - px, cy - py) > demote_range:
                continue                       # too far to judge presence
            ekey = (round(cx / bin_size), round(cy / bin_size))
            seen = any(abs(k[0] - ekey[0]) <= span and abs(k[1] - ekey[1]) <= span
                       for k in hits)
            if seen:
                self.mirror_seen[eid] = now
            elif now - self.mirror_seen.setdefault(eid, now) > demote_after:
                self.get_logger().info(
                    f'mirrored obstacle {eid} no longer detected — retiring')
                self.remove_pub.publish(json_msg({'id': eid}))
                self.mirror_seen.pop(eid, None)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleMirror()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
