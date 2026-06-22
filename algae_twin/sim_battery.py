"""Battery model for the digital twin.

Mirrors the real robot's battery (a twin reports the real pack's state). If the
real battery goes silent, it falls back to a simple discharge model from motion +
sprayer load so the twin gauge keeps moving; the operator can reset the simulated
charge via /sim/battery/set.
"""
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32

from .util import V_EMPTY, V_FULL, battery_voltage, clamp, latched_qos


class SimBattery(Node):
    def __init__(self):
        super().__init__('sim_battery')
        self.declare_parameter('mirror_real', False)
        self.declare_parameter('initial_percent', 100.0)
        self.declare_parameter('idle_drain', 0.01)       # %/s
        self.declare_parameter('drain_per_meter', 0.10)  # %/m driven
        self.declare_parameter('drain_per_rad', 0.02)    # %/rad turned
        self.declare_parameter('spray_drain', 0.08)      # %/s while dispersing
        self.mirror = self.get_parameter('mirror_real').value
        self.percent = float(self.get_parameter('initial_percent').value)

        self.speed = (0.0, 0.0)
        self.spraying = False
        self.real_msg = None
        self.real_stamp = 0.0
        self.last_tick = time.monotonic()

        self.create_subscription(Odometry, '/sim/ground_truth', self._on_odom, 10)
        self.create_subscription(Bool, '/clean/active', self._on_spray,
                                 latched_qos())
        self.create_subscription(BatteryState, '/battery_state', self._on_real, 10)
        self.create_subscription(Float32, '/sim/battery/set', self._on_set, 10)
        self.pub = self.create_publisher(BatteryState, '/sim/battery_state', 10)
        self.create_timer(1.0, self._tick)

    def _on_odom(self, msg):
        self.speed = (abs(msg.twist.twist.linear.x),
                      abs(msg.twist.twist.angular.z))

    def _on_spray(self, msg):
        self.spraying = msg.data

    def _on_real(self, msg):
        self.real_msg = msg
        self.real_stamp = time.monotonic()

    def _on_set(self, msg):
        self.percent = clamp(float(msg.data), 0.0, 100.0)
        self.get_logger().info(f'twin battery set to {self.percent:.0f} %')

    def _tick(self):
        now = time.monotonic()
        # clamp dt so a paused/resumed sim (or a long stall) can't apply one huge
        # drain step; steady-state drain is unaffected at the 1 s tick.
        dt, self.last_tick = min(now - self.last_tick, 5.0), now

        if self.mirror and self.real_msg and now - self.real_stamp < 3.0:
            msg = self.real_msg   # faithful mirror of the physical pack
            volt = battery_voltage(msg)
            if volt is not None:   # keep the internal model synced for fallback
                self.percent = clamp((volt - V_EMPTY) / (V_FULL - V_EMPTY)
                                     * 100.0, 0.0, 100.0)
            self.pub.publish(msg)
            return

        lin, ang = self.speed
        drain = (self.get_parameter('idle_drain').value
                 + lin * self.get_parameter('drain_per_meter').value
                 + ang * self.get_parameter('drain_per_rad').value
                 + (self.get_parameter('spray_drain').value
                    if self.spraying else 0.0))
        self.percent = clamp(self.percent - drain * dt, 0.0, 100.0)

        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = V_EMPTY + (V_FULL - V_EMPTY) * self.percent / 100.0
        msg.percentage = self.percent / 100.0
        msg.present = True
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.design_capacity = 1.8
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimBattery()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
