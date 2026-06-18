"""Algae mission manager.

Lifecycle of an algae patch: placed in the UI -> green patch appears in Gazebo
-> robot pair navigates to it (one Nav2 brain, both robots move) -> cleaning:
3 full spins while the twin's dispersal motor runs (sprayer joint + chemical
particle emitter; /clean/active is the hook for real dispersal hardware, and
the real robot beeps if its /sound service is up) -> patch cleared everywhere.

Also handles manual nav goals, return-home, the battery gate and e-stop.
"""
import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Empty, Float64, String

from .grid import StaticMap
from .gz_io import GzIo
from .util import (battery_voltage, json_msg, latched_qos, map_to_world_pose,
                   yaw_to_quat)

ALGAE_SDF = """<?xml version="1.0"?>
<sdf version="1.8"><model name="{name}"><static>true</static>
<link name="link"><visual name="v">
  <pose>0 0 0.003 0 0 0</pose>
  <geometry><cylinder><radius>{radius}</radius><length>0.006</length></cylinder></geometry>
  <material><ambient>0.10 0.65 0.25 0.9</ambient><diffuse>0.15 0.85 0.35 0.9</diffuse>
            <emissive>0.02 0.25 0.08 1</emissive></material>
</visual></link></model></sdf>"""

FULL_TURN = 2.0 * math.pi


class Mission(Node):
    def __init__(self):
        super().__init__('mission')
        self.declare_parameter('world', 'default')
        self.declare_parameter('spin_turns', 3)
        self.declare_parameter('spin_time_allowance', 90.0)
        self.declare_parameter('fallback_spin_speed', 1.2)
        self.declare_parameter('battery_min_voltage', 11.3)
        self.declare_parameter('algae_radius', 0.15)
        self.declare_parameter('home_pose', [0.0, 0.0, 0.0])
        self.declare_parameter('sprayer_speed', 25.0)
        self.declare_parameter('emitter_topic', '/sim/sprayer_particles')
        self.declare_parameter('cleared_linger_sec', 20.0)
        self.declare_parameter('nav_timeout', 120.0)   # fail a stalled nav goal
        self.declare_parameter('placement_clearance', 0.10)  # algae must be this
        #                                              # clear of walls (m), >=0
        self.declare_parameter('map_to_world', [0.0, 0.0, 0.0])

        self.map_to_world = list(self.get_parameter('map_to_world').value)
        self.gz = GzIo(self, self.get_parameter('world').value)
        self.algae = {}            # id -> dict (insertion order = queue order)
        self.counter = 0
        self.state = 'idle'        # idle | nav | clean | clean_fallback
        self.current = None        # algae id being served (None for goto/home)
        self.estop = False
        self.note = None
        self.batteries = {}
        self.static_map = None     # latest /map, for placement validation
        self.nav_goal_handle = None
        self.nav_sent_at = None
        self.nav_accepted_at = None
        self.nav_gen = 0           # bumped per nav goal; stale callbacks ignored
        self.spin_goal_handle = None
        self.spin_gen = 0          # bumped per spin goal; stale callbacks ignored
        self.fallback_left = 0.0

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spin_client = ActionClient(self, Spin, 'spin')
        self.sound_client = self._make_sound_client()

        self.state_pub = self.create_publisher(String, '/algae/state',
                                               latched_qos())
        self.clean_pub = self.create_publisher(Bool, '/clean/active',
                                               latched_qos())
        self.spray_pub = self.create_publisher(Float64, '/sim/sprayer_cmd', 10)
        self.clean_cmd_pub = self.create_publisher(Twist, '/clean_cmd_vel', 10)

        self.create_subscription(PointStamped, '/algae/add', self._on_add, 10)
        self.create_subscription(String, '/algae/remove', self._on_remove, 10)
        self.create_subscription(PoseStamped, '/mission/goto', self._on_goto, 10)
        self.create_subscription(Empty, '/mission/home', self._on_home, 10)
        self.create_subscription(Bool, '/estop', self._on_estop, latched_qos())
        self.create_subscription(OccupancyGrid, '/map', self._on_map,
                                 latched_qos())
        self.create_subscription(BatteryState, '/battery_state',
                                 lambda m: self._on_battery('real', m), 10)
        self.create_subscription(BatteryState, '/sim/battery_state',
                                 lambda m: self._on_battery('sim', m), 10)

        self.clean_pub.publish(Bool(data=False))
        self.create_timer(0.5, self._scheduler)
        self.create_timer(0.1, self._fallback_spin_tick)
        self.create_timer(1.0, self._progress_tick)
        self._publish_state()
        self.get_logger().info('mission manager up')

    def _make_sound_client(self):
        try:
            from turtlebot3_msgs.srv import Sound
            return self.create_client(Sound, '/sound')
        except ImportError:
            return None

    # ---- inputs --------------------------------------------------------------
    def _on_add(self, msg):
        x, y = msg.point.x, msg.point.y
        if not (math.isfinite(x) and math.isfinite(y)):
            return
        # algae may only go on free, navigable map space — reject walls, unknown
        # / unmapped cells and off-map clicks so the robot can actually reach it.
        clearance = self.get_parameter('placement_clearance').value
        if (self.static_map is not None
                and not self.static_map.is_free(x, y, clearance)):
            self.note = 'cannot place algae there — not free space on the map'
            self.get_logger().warning(
                f'rejected algae at ({x:.2f}, {y:.2f}) — not free map space')
            self._publish_state()
            return
        self.counter += 1
        algae_id = f'a{self.counter}'
        self.algae[algae_id] = {
            'id': algae_id, 'x': round(msg.point.x, 3),
            'y': round(msg.point.y, 3), 'status': 'queued', 'progress': 0.0,
        }
        # algae position is stored/navigated in the map frame; the Gazebo patch
        # is spawned in the world frame (identity while world == map).
        wx, wy, _ = map_to_world_pose(msg.point.x, msg.point.y, 0.0,
                                      self.map_to_world)
        self.gz.spawn(f'algae_{algae_id}',
                      ALGAE_SDF.format(name=f'algae_{algae_id}',
                                       radius=self.get_parameter(
                                           'algae_radius').value),
                      wx, wy)
        self.get_logger().info(
            f'algae {algae_id} placed at ({msg.point.x:.2f}, {msg.point.y:.2f})')
        self._publish_state()

    def _on_remove(self, msg):
        algae = self.algae.pop(msg.data.strip(), None)
        if algae is None:
            return
        self.gz.remove(f"algae_{algae['id']}")
        if self.current == algae['id']:
            self._abort_motion()
            self._stop_clean_actuators()
            self.current, self.state = None, 'idle'
        self._publish_state()

    def _on_goto(self, msg):
        yaw = 2.0 * math.atan2(msg.pose.orientation.z, msg.pose.orientation.w)
        self._start_manual_goal(msg.pose.position.x, msg.pose.position.y, yaw,
                                'manual goal')

    def _on_home(self, _msg):
        x, y, yaw = self.get_parameter('home_pose').value
        self._start_manual_goal(x, y, yaw, 'returning home')

    def _start_manual_goal(self, x, y, yaw, label):
        if self.estop:
            return
        self._abort_motion()
        self._stop_clean_actuators()
        if self.current and self.current in self.algae:
            self.algae[self.current]['status'] = 'queued'
        self.current, self.note = None, label
        self.state = 'nav'
        self._send_nav_goal(x, y, yaw)
        self._publish_state()

    def _on_estop(self, msg):
        self.estop = msg.data
        if msg.data:
            self._abort_motion()
            self._stop_clean_actuators()
            if self.current and self.current in self.algae:
                self.algae[self.current]['status'] = 'queued'
            self.current, self.state = None, 'idle'
            self.note = 'E-STOP engaged'
        else:
            self.note = None
        self._publish_state()

    def _on_battery(self, key, msg):
        # voltage may be None (no pack reported) — stored, ignored in the gate.
        self.batteries[key] = (self.get_clock().now().nanoseconds * 1e-9,
                               battery_voltage(msg))

    def _on_map(self, msg):
        self.static_map = StaticMap(msg)

    # ---- scheduling -------------------------------------------------------------
    def _battery_ok(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        fresh = [volt for stamp, volt in self.batteries.values()
                 if now - stamp < 5.0 and volt is not None]
        return min(fresh) >= self.get_parameter(
            'battery_min_voltage').value if fresh else True

    def _scheduler(self):
        self._reap_cleared()
        self._check_nav_watchdog()
        if self.estop or self.state != 'idle':
            return
        queued = [a for a in self.algae.values() if a['status'] == 'queued']
        if not queued:
            return
        if not self._battery_ok():
            new_note = 'battery low — recharge before next mission'
            if self.note != new_note:
                self.note = new_note
                self._publish_state()
            return
        algae = queued[0]
        algae['status'] = 'active'
        self.current, self.state = algae['id'], 'nav'
        self.note = f"navigating to {algae['id']}"
        self._send_nav_goal(algae['x'], algae['y'], 0.0)
        self._publish_state()

    def _reap_cleared(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        linger = self.get_parameter('cleared_linger_sec').value
        for algae in list(self.algae.values()):
            if algae['status'] == 'cleared' and now - algae.get(
                    'cleared_at', now) > linger:
                del self.algae[algae['id']]
                self._publish_state()

    def _check_nav_watchdog(self):
        """Fail cleanly if Nav2 never answers the goal request (server down)."""
        if (self.state == 'nav' and self.nav_goal_handle is None
                and self.nav_sent_at is not None
                and self.get_clock().now().nanoseconds * 1e-9
                - self.nav_sent_at > 15.0):
            self.get_logger().error('navigation server unresponsive')
            self.nav_sent_at = None
            self._finish_nav(False)
        # post-acceptance stall: a goal was accepted but never returns a result
        # (controller wedged but not aborting). Cancel + fail so we don't hang.
        if (self.state == 'nav' and self.nav_goal_handle is not None
                and self.nav_accepted_at is not None
                and self.get_clock().now().nanoseconds * 1e-9
                - self.nav_accepted_at
                > self.get_parameter('nav_timeout').value):
            self.get_logger().error('navigation stalled — cancelling goal')
            handle, self.nav_goal_handle = self.nav_goal_handle, None
            self.nav_accepted_at = None
            self.nav_gen += 1        # invalidate the stalled goal's late callback
            handle.cancel_goal_async()
            self._finish_nav(False)

    # ---- navigation -----------------------------------------------------------
    def _send_nav_goal(self, x, y, yaw):
        if not self.nav_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().warning('navigate_to_pose server not ready yet')
        self.nav_gen += 1             # this attempt supersedes any earlier one
        gen = self.nav_gen
        self.nav_sent_at = self.get_clock().now().nanoseconds * 1e-9
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        (goal.pose.pose.orientation.x, goal.pose.pose.orientation.y,
         goal.pose.pose.orientation.z, goal.pose.pose.orientation.w) = \
            yaw_to_quat(yaw)
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self._nav_accepted(f, gen))

    def _nav_accepted(self, future, gen):
        handle = future.result()
        # ignore a goal that was superseded (new goal) or abandoned (estop /
        # remove / watchdog) between send and acceptance — and don't leak it.
        if gen != self.nav_gen or self.state != 'nav':
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
            return
        self.nav_sent_at = None
        if handle is None or not handle.accepted:
            self.get_logger().error('navigation goal rejected')
            self._finish_nav(False)
            return
        self.nav_goal_handle = handle
        self.nav_accepted_at = self.get_clock().now().nanoseconds * 1e-9
        handle.get_result_async().add_done_callback(
            lambda f: self._nav_done(f, gen))

    def _nav_done(self, future, gen):
        if gen != self.nav_gen or self.state != 'nav':   # superseded/abandoned
            return
        self.nav_goal_handle = None
        self.nav_accepted_at = None
        try:
            succeeded = future.result().status == GoalStatus.STATUS_SUCCEEDED
        except Exception as err:      # action server died mid-goal
            self.get_logger().error(f'navigation result error: {err}')
            succeeded = False
        self._finish_nav(succeeded)

    def _finish_nav(self, succeeded):
        if self.current is None:                      # manual goal / home
            self.state = 'idle'
            self.note = None if succeeded else 'manual goal failed'
            self._publish_state()
            return
        algae = self.algae.get(self.current)
        if algae is None:
            self.current, self.state = None, 'idle'
            return
        if not succeeded:
            algae['status'] = 'failed'
            self.note = f"could not reach {algae['id']}"
            self.current, self.state = None, 'idle'
            self._publish_state()
            return
        self._start_clean(algae)

    # ---- cleaning -----------------------------------------------------------------
    def _start_clean(self, algae):
        self.state = 'clean'
        algae['status'] = 'cleaning'
        algae['progress'] = 0.0
        self.note = f"cleaning {algae['id']} — dispersing"
        self._start_clean_actuators()

        turns = self.get_parameter('spin_turns').value
        self.spin_gen += 1
        gen = self.spin_gen
        goal = Spin.Goal()
        goal.target_yaw = float(turns) * FULL_TURN
        goal.time_allowance.sec = int(
            self.get_parameter('spin_time_allowance').value)
        if self.spin_client.wait_for_server(timeout_sec=2.0):
            future = self.spin_client.send_goal_async(
                goal, feedback_callback=self._spin_feedback)
            future.add_done_callback(lambda f: self._spin_accepted(f, gen))
        else:
            self.get_logger().warning(
                'spin action unavailable — using fallback rotation')
            self._begin_fallback_spin(goal.target_yaw)
        self._publish_state()

    def _spin_feedback(self, feedback):
        algae = self.algae.get(self.current)
        total = max(1, self.get_parameter('spin_turns').value) * FULL_TURN
        if algae:
            traveled = abs(feedback.feedback.angular_distance_traveled)
            algae['progress'] = min(1.0, traveled / total)

    def _spin_accepted(self, future, gen):
        handle = future.result()
        stale = gen != self.spin_gen or self.state != 'clean'
        if handle is None or not handle.accepted:
            if not stale:             # genuine rejection of the current spin
                self._begin_fallback_spin(
                    self.get_parameter('spin_turns').value * FULL_TURN)
            return
        if stale:                     # superseded/abandoned between send & accept
            handle.cancel_goal_async()
            return
        self.spin_goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f: self._spin_done(f, gen))

    def _spin_done(self, future, gen):
        if gen != self.spin_gen or self.state != 'clean':
            return
        self.spin_goal_handle = None
        try:
            succeeded = future.result().status == GoalStatus.STATUS_SUCCEEDED
        except Exception as err:      # action server died mid-goal
            self.get_logger().error(f'spin result error: {err}')
            succeeded = False
        if succeeded:
            self._finish_clean()
        else:
            algae = self.algae.get(self.current)
            done = algae['progress'] if algae else 0.0
            remaining = (1.0 - done) * \
                self.get_parameter('spin_turns').value * FULL_TURN
            self.get_logger().warning(
                f'spin behavior ended early — finishing {remaining:.1f} rad '
                'with fallback rotation')
            self._begin_fallback_spin(max(0.5, remaining))

    def _begin_fallback_spin(self, angle):
        self.state = 'clean_fallback'
        speed = self.get_parameter('fallback_spin_speed').value
        self.fallback_left = angle / speed

    def _fallback_spin_tick(self):
        if self.state != 'clean_fallback' or self.estop:
            return
        self.fallback_left -= 0.1
        if self.fallback_left <= 0.0:
            self._finish_clean()
            return
        cmd = Twist()
        cmd.angular.z = self.get_parameter('fallback_spin_speed').value
        self.clean_cmd_pub.publish(cmd)
        algae = self.algae.get(self.current)
        if algae:
            total = self.get_parameter('fallback_spin_speed').value * \
                self.fallback_left
            full = max(1, self.get_parameter('spin_turns').value) * FULL_TURN
            algae['progress'] = min(1.0, 1.0 - total / full)

    def _finish_clean(self):
        self._stop_clean_actuators()
        algae = self.algae.get(self.current)
        if algae:
            algae['status'] = 'cleared'
            algae['progress'] = 1.0
            algae['cleared_at'] = self.get_clock().now().nanoseconds * 1e-9
            self.gz.remove(f"algae_{algae['id']}")
            self.get_logger().info(f"algae {algae['id']} cleared")
        self._beep(times=2)
        self.current, self.state, self.note = None, 'idle', None
        self._publish_state()

    def _start_clean_actuators(self):
        self.clean_pub.publish(Bool(data=True))
        self.spray_pub.publish(
            Float64(data=float(self.get_parameter('sprayer_speed').value)))
        self.gz.set_emitter(self.get_parameter('emitter_topic').value, True)
        self._beep(times=1)

    def _stop_clean_actuators(self):
        self.clean_pub.publish(Bool(data=False))
        self.spray_pub.publish(Float64(data=0.0))
        self.gz.set_emitter(self.get_parameter('emitter_topic').value, False)
        # release the mux's clean slot at once: without a zeroing twist the last
        # fallback-spin command stays "fresh" for cmd_timeout (~0.6 s), over-
        # spinning both robots and hijacking a manual goal (clean outranks nav).
        self.clean_cmd_pub.publish(Twist())

    def _abort_motion(self):
        for handle_name in ('nav_goal_handle', 'spin_goal_handle'):
            handle = getattr(self, handle_name)
            if handle is not None:
                handle.cancel_goal_async()
                setattr(self, handle_name, None)
        self.nav_accepted_at = None
        self.fallback_left = 0.0
        if self.state == 'clean_fallback':
            self.state = 'idle'

    def _beep(self, times):
        """Physical feedback on the real robot, best-effort."""
        if self.sound_client is None or not self.sound_client.service_is_ready():
            return
        from turtlebot3_msgs.srv import Sound
        for _ in range(times):
            request = Sound.Request()
            request.value = 1   # 1 = ON (turtlebot3_msgs/srv/Sound value table)
            self.sound_client.call_async(request)

    def _progress_tick(self):
        """Keep the UI's progress fresh while a mission is running."""
        if self.state != 'idle':
            self._publish_state()

    # ---- state out ---------------------------------------------------------------
    def _publish_state(self):
        self.state_pub.publish(json_msg({
            'algae': [{k: v for k, v in a.items() if k != 'cleared_at'}
                      for a in self.algae.values()],
            'note': self.note,
        }))


def main(args=None):
    rclpy.init(args=args)
    node = Mission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
