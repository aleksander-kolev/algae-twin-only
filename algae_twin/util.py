"""Small shared helpers: angles, quaternions, QoS profiles, JSON topics."""
import json
import math

from geometry_msgs.msg import Twist, TwistStamped
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String

# Burger pack voltage endpoints for the *usable* window. V_FULL = 4.2 V/cell (3S
# full); V_EMPTY = ~11.0 V, the OpenCR power-off cutoff (turtlebot3_diagnosis.cpp
# disables motor power below ~11.0 V) — i.e. the point the real robot stops. Used
# to render the simulated twin voltage and to scale the operator-UI battery bar.
V_FULL, V_EMPTY = 12.6, 11.0


def yaw_to_quat(yaw):
    """Yaw (rad) -> quaternion tuple (x, y, z, w)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quat_to_yaw(x, y, z, w):
    """Quaternion -> yaw (rad)."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def norm_angle(a):
    """Wrap angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def clamp(value, low, high):
    return max(low, min(high, value))


def battery_voltage(msg):
    """sensor_msgs/BatteryState -> pack voltage (V), or None if not reported.

    The twin works in volts: the real TurtleBot3 OpenCR reliably reports only
    voltage (its `percentage` field is frequently 0), so voltage is the single
    source of truth for the UI gauges and the mission battery gate. A reading of
    <= 1 V means 'no pack / not reported' rather than a real flat battery.
    """
    volt = msg.voltage
    return float(volt) if volt and volt > 1.0 else None


def map_to_world_pose(x, y, yaw, offset):
    """Transform a pose from the map frame into the Gazebo world frame.

    `offset` is the [ox, oy, oyaw] map->world transform (the inverse of the
    world->map applied to twin telemetry). With the default [0, 0, 0] — i.e.
    the world frame coincides with the map frame, which is the invariant the
    twin is built on — this is the identity.
    """
    ox, oy, oyaw = offset
    cos_o, sin_o = math.cos(oyaw), math.sin(oyaw)
    return (ox + cos_o * x - sin_o * y,
            oy + sin_o * x + cos_o * y,
            yaw + oyaw)


def latched_qos(depth=1):
    """QoS for state topics late joiners must still receive (maps, JSON state)."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def sensor_qos(depth=5):
    """Best-effort QoS matching sensor publishers (lidar, etc.)."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def json_msg(payload):
    """Build a std_msgs/String carrying a JSON object."""
    msg = String()
    msg.data = json.dumps(payload)
    return msg


def parse_json(msg, logger=None, context=''):
    """Parse a std_msgs/String JSON payload; None (and a log line) on garbage."""
    try:
        return json.loads(msg.data)
    except (json.JSONDecodeError, TypeError) as err:
        if logger is not None:
            logger.warning(f'Ignoring malformed JSON on {context}: {err}')
        return None


def stamp_twist(twist, clock, frame_id='base_link'):
    """Wrap a Twist into a TwistStamped."""
    msg = TwistStamped()
    msg.header.stamp = clock.now().to_msg()
    msg.header.frame_id = frame_id
    msg.twist = twist
    return msg


def twist_of(msg):
    """Accept Twist or TwistStamped, return the plain Twist."""
    return msg.twist if isinstance(msg, TwistStamped) else msg


def zero_twist():
    return Twist()
