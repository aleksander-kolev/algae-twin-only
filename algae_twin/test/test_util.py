"""Unit tests for the pure-Python helpers in algae_twin.util.

No ROS graph and no hardware — just the math the twin relies on (angles,
quaternions, the battery-voltage gate, the map<->world transform). A ROS 2
environment must be sourced because util imports geometry_msgs + rclpy.qos.

    python test/test_util.py          # standalone
    colcon test --packages-select algae_twin
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from algae_twin.util import (V_EMPTY, V_FULL, battery_voltage, clamp,  # noqa: E402
                             map_to_world_pose, norm_angle, quat_to_yaw,
                             yaw_to_quat)


def test_yaw_quat_roundtrip():
    for yaw in (-3.0, -1.2, 0.0, 0.7, 3.1):
        x, y, z, w = yaw_to_quat(yaw)
        assert abs(norm_angle(quat_to_yaw(x, y, z, w) - yaw)) < 1e-9


def test_norm_angle():
    assert abs(norm_angle(0.0)) < 1e-9
    assert abs(norm_angle(2 * math.pi)) < 1e-9
    assert abs(abs(norm_angle(3 * math.pi)) - math.pi) < 1e-9   # wraps to +/-pi


def test_clamp():
    assert clamp(5, 0, 1) == 1
    assert clamp(-5, 0, 1) == 0
    assert clamp(0.5, 0, 1) == 0.5


def test_battery_voltage_gate():
    class _Msg:                 # minimal stand-in for sensor_msgs/BatteryState
        voltage = 12.3
    assert battery_voltage(_Msg()) == 12.3
    _Msg.voltage = 0.0          # OpenCR frequently reports 0 -> "not present"
    assert battery_voltage(_Msg()) is None
    _Msg.voltage = 0.5          # <= 1 V means no pack reported, not a flat cell
    assert battery_voltage(_Msg()) is None


def test_map_to_world_identity():
    # the twin invariant: world == map -> identity transform
    assert map_to_world_pose(1.0, 2.0, 0.5, [0.0, 0.0, 0.0]) == (1.0, 2.0, 0.5)


def test_map_to_world_transform():
    x, y, _ = map_to_world_pose(1.0, 0.0, 0.0, [10.0, 0.0, 0.0])   # translate +x
    assert abs(x - 11.0) < 1e-9 and abs(y) < 1e-9
    x, y, _ = map_to_world_pose(1.0, 0.0, 0.0, [0.0, 0.0, math.pi / 2])  # +90 deg
    assert abs(x) < 1e-9 and abs(y - 1.0) < 1e-9


def test_voltage_window_sane():
    # the usable burger pack window straddles the OpenCR ~11.0 V motor cutoff
    assert V_EMPTY < V_FULL
    assert 10.5 < V_EMPTY < 11.5
    assert 12.0 < V_FULL < 13.0


if __name__ == '__main__':
    for _name, _fn in sorted(globals().items()):
        if _name.startswith('test_') and callable(_fn):
            _fn()
            print(f'PASS {_name}')
    print('UTIL TESTS PASS')
