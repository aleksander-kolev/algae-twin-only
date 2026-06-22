"""World edits: the bidirectional map-change channel.

Maintains the list of blocking boxes (operator-drawn or mirrored from the real
world) and applies every change to BOTH worlds at once:

  * Nav2  — rasterised into the keepout filter mask (/keepout_mask), which the
            shared planner consumes, re-routing the real robot AND the twin;
  * Gazebo — spawned/removed as physical box models, so the digital world and
            the twin's lidar see exactly what navigation sees.

Edits persist across restarts (~/.algae_twin/edits.json).
"""
import json
import math
import os

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import String

from .grid import EditGrid, GridGeometry
from .gz_io import GzIo
from .util import (clamp, json_msg, latched_qos, map_to_world_pose,
                   parse_json)

BOX_SDF = """<?xml version="1.0"?>
<sdf version="1.8"><model name="{name}"><static>true</static>
<link name="link">
  <collision name="c"><geometry><box><size>{sx} {sy} {h}</size></box></geometry></collision>
  <visual name="v"><geometry><box><size>{sx} {sy} {h}</size></box></geometry>
    <material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material>
  </visual>
</link></model></sdf>"""

COLORS = {'operator': '0.85 0.20 0.20 0.85', 'mirrored': '1.0 0.65 0.20 0.85'}


class MapEdit(Node):
    def __init__(self):
        super().__init__('map_edit')
        self.declare_parameter('world', 'default')
        self.declare_parameter('box_height', 0.25)
        self.declare_parameter('persist', True)
        self.declare_parameter(
            'persist_file', os.path.expanduser('~/.algae_twin/edits.json'))
        self.declare_parameter('map_to_world', [0.0, 0.0, 0.0])
        self.box_height = self.get_parameter('box_height').value
        self.persist_file = self.get_parameter('persist_file').value
        self.persist = self.get_parameter('persist').value
        self.map_to_world = list(self.get_parameter('map_to_world').value)

        self.gz = GzIo(self, self.get_parameter('world').value)
        self.geometry = None
        self.edits = {}     # id -> {id, cx, cy, size_x, size_y, yaw, source}
        self.counter = 0

        self.mask_pub = self.create_publisher(OccupancyGrid, '/keepout_mask',
                                              latched_qos())
        self.state_pub = self.create_publisher(String, '/edits/state',
                                               latched_qos())
        self.create_subscription(OccupancyGrid, '/map', self._on_map,
                                 latched_qos())
        self.create_subscription(String, '/edits/add', self._on_add, 10)
        self.create_subscription(String, '/edits/remove', self._on_remove, 10)
        self.create_subscription(String, '/edits/clear', self._on_clear, 10)

    # -- map arrival bootstraps everything ------------------------------------
    def _on_map(self, msg):
        first = self.geometry is None
        self.geometry = GridGeometry.from_msg(msg)
        if first:
            self._load()
            for edit in self.edits.values():
                self._spawn(edit)
            self.get_logger().info(
                f'map received ({self.geometry.width}x{self.geometry.height}), '
                f'{len(self.edits)} persisted edit(s) restored')
        self._apply()

    # -- operator / mirror commands --------------------------------------------
    def _on_add(self, msg):
        edit = parse_json(msg, self.get_logger(), '/edits/add')
        if edit is None or self.geometry is None:
            return
        try:
            edit = self._sanitise(edit)
        except (KeyError, TypeError, ValueError) as err:
            self.get_logger().warning(f'rejected edit: {err}')
            return
        if edit['source'] == 'mirrored' and self._near_existing(edit):
            return
        self.counter += 1
        edit['id'] = f"e{self.counter}"
        self.edits[edit['id']] = edit
        self._spawn(edit)
        self._apply()
        self.get_logger().info(
            f"edit {edit['id']} added ({edit['source']}) at "
            f"({edit['cx']:.2f}, {edit['cy']:.2f})")

    def _on_remove(self, msg):
        payload = parse_json(msg, self.get_logger(), '/edits/remove') or {}
        edit = self.edits.pop(payload.get('id'), None)
        if edit:
            self.gz.remove(f"edit_{edit['id']}")
            self._apply()

    def _on_clear(self, msg):
        payload = parse_json(msg, self.get_logger(), '/edits/clear') or {}
        source = payload.get('source', 'all')
        doomed = [e for e in self.edits.values()
                  if source == 'all' or e['source'] == source]
        for edit in doomed:
            del self.edits[edit['id']]
            self.gz.remove(f"edit_{edit['id']}")
        if doomed:
            self._apply()
            self.get_logger().info(f'cleared {len(doomed)} edit(s) [{source}]')

    # -- helpers -------------------------------------------------------------------
    def _sanitise(self, edit):
        clean = {
            'cx': float(edit['cx']), 'cy': float(edit['cy']),
            'size_x': clamp(float(edit['size_x']), 0.05, 3.0),
            'size_y': clamp(float(edit['size_y']), 0.05, 3.0),
            'yaw': float(edit.get('yaw', 0.0)),
            'source': edit.get('source', 'operator'),
        }
        if not all(math.isfinite(v) for v in
                   (clean['cx'], clean['cy'], clean['yaw'])):
            raise ValueError('non-finite coordinates')
        if clean['source'] not in COLORS:
            raise ValueError(f"unknown source '{clean['source']}'")
        return clean

    def _near_existing(self, edit):
        return any(math.hypot(e['cx'] - edit['cx'], e['cy'] - edit['cy']) < 0.15
                   for e in self.edits.values())

    def _spawn(self, edit):
        sdf = BOX_SDF.format(name=f"edit_{edit['id']}",
                             sx=edit['size_x'], sy=edit['size_y'],
                             h=self.box_height, rgba=COLORS[edit['source']])
        # edits live in the map frame (mask + UI); spawn the box in the world
        # frame (identity while world == map).
        wx, wy, wyaw = map_to_world_pose(edit['cx'], edit['cy'], edit['yaw'],
                                         self.map_to_world)
        self.gz.spawn(f"edit_{edit['id']}", sdf, wx, wy,
                      z=self.box_height / 2.0, yaw=wyaw)

    def _apply(self):
        """Rebuild + publish the keepout mask, state JSON and the persist file."""
        if self.geometry is None:
            return
        mask = EditGrid(self.geometry)
        for edit in self.edits.values():
            mask.mark_box(edit['cx'], edit['cy'],
                          edit['size_x'], edit['size_y'], edit['yaw'])
        self.mask_pub.publish(
            mask.to_msg(self.get_clock().now().to_msg()))
        self.state_pub.publish(json_msg({'edits': list(self.edits.values())}))
        self._save()

    def _load(self):
        if not (self.persist and os.path.isfile(self.persist_file)):
            return
        try:
            with open(self.persist_file, encoding='utf-8') as fh:
                data = json.load(fh)
            self.edits = {e['id']: e for e in data.get('edits', [])}
            # restore the id counter above every existing id so a new edit can
            # never collide, even for a legacy file without a 'counter' field
            self.counter = data.get('counter') or max(
                (int(i[1:]) for i in self.edits if i[1:].isdigit()), default=0)
        except (OSError, json.JSONDecodeError, KeyError) as err:
            self.get_logger().warning(f'could not load persisted edits: {err}')

    def _save(self):
        if not self.persist:
            return
        try:
            os.makedirs(os.path.dirname(self.persist_file), exist_ok=True)
            tmp = self.persist_file + '.tmp'
            with open(tmp, 'w', encoding='utf-8') as fh:
                json.dump({'edits': list(self.edits.values()),
                           'counter': self.counter}, fh, indent=1)
            os.replace(tmp, self.persist_file)   # atomic: no half-written file
        except OSError as err:
            self.get_logger().warning(f'could not persist edits: {err}')


def main(args=None):
    rclpy.init(args=args)
    node = MapEdit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
