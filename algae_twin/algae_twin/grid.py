"""Occupancy-grid helpers shared by map_edit and obstacle_mirror.

All grids follow nav_msgs/OccupancyGrid conventions: row-major from the map
origin (cell [0,0] is the bottom-left corner in world coordinates, +y up).
"""
import math

from nav_msgs.msg import OccupancyGrid

FREE = 0
OCCUPIED = 100
UNKNOWN = -1


class GridGeometry:
    """Geometry of an occupancy grid (no cell data)."""

    def __init__(self, width, height, resolution, origin_x, origin_y):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y

    @classmethod
    def from_msg(cls, msg):
        info = msg.info
        return cls(info.width, info.height, info.resolution,
                   info.origin.position.x, info.origin.position.y)

    def world_to_cell(self, x, y):
        """World (m) -> (col, row); may be out of bounds.

        Uses floor (not int truncation) so a point just below/left of the origin
        maps to -1 (out of bounds) rather than 0 — keeps `occupied_near`'s
        off-map test correct at the map's -x/-y edge.
        """
        col = math.floor((x - self.origin_x) / self.resolution)
        row = math.floor((y - self.origin_y) / self.resolution)
        return col, row

    def in_bounds(self, col, row):
        return 0 <= col < self.width and 0 <= row < self.height


class EditGrid:
    """A keepout mask: starts all-free, boxes get rasterised as occupied."""

    def __init__(self, geometry):
        self.geom = geometry
        self.data = [FREE] * (geometry.width * geometry.height)

    def clear(self):
        self.data = [FREE] * (self.geom.width * self.geom.height)

    def mark_box(self, cx, cy, size_x, size_y, yaw):
        """Rasterise a rotated box (centre, full sizes in m) as occupied."""
        g = self.geom
        half_diag = 0.5 * math.hypot(size_x, size_y)
        c_min, r_min = g.world_to_cell(cx - half_diag, cy - half_diag)
        c_max, r_max = g.world_to_cell(cx + half_diag, cy + half_diag)
        cos_y, sin_y = math.cos(-yaw), math.sin(-yaw)
        for row in range(max(0, r_min), min(g.height, r_max + 1)):
            wy = g.origin_y + (row + 0.5) * g.resolution
            for col in range(max(0, c_min), min(g.width, c_max + 1)):
                wx = g.origin_x + (col + 0.5) * g.resolution
                # transform cell centre into the box frame
                dx, dy = wx - cx, wy - cy
                bx = dx * cos_y - dy * sin_y
                by = dx * sin_y + dy * cos_y
                if abs(bx) <= size_x / 2.0 and abs(by) <= size_y / 2.0:
                    self.data[row * g.width + col] = OCCUPIED

    def to_msg(self, stamp, frame_id='map'):
        msg = OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.info.resolution = self.geom.resolution
        msg.info.width = self.geom.width
        msg.info.height = self.geom.height
        msg.info.origin.position.x = self.geom.origin_x
        msg.info.origin.position.y = self.geom.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = self.data
        return msg


class StaticMap:
    """Read-only view of the static map for 'is this point explained?' tests."""

    def __init__(self, msg):
        self.geom = GridGeometry.from_msg(msg)
        self.data = list(msg.data)

    def is_free(self, x, y, clearance=0.0):
        """True if (x, y) is in-bounds FREE space (and, with clearance > 0, the
        whole square of that radius is clear of occupied/unknown cells).

        Used to reject objects (e.g. algae) placed on a wall, in unknown space,
        or off the map — only genuinely navigable cells pass.
        """
        g = self.geom
        col0, row0 = g.world_to_cell(x, y)
        span = int(math.ceil(clearance / g.resolution)) if clearance > 0 else 0
        for row in range(row0 - span, row0 + span + 1):
            for col in range(col0 - span, col0 + span + 1):
                if not g.in_bounds(col, row):
                    return False              # off the map
                value = self.data[row * g.width + col]
                if value >= 50 or value == UNKNOWN:
                    return False              # wall / unknown
        return True

    def occupied_near(self, x, y, radius):
        """True if any cell within `radius` m is occupied or unknown."""
        g = self.geom
        col0, row0 = g.world_to_cell(x, y)
        span = max(1, int(math.ceil(radius / g.resolution)))
        for row in range(row0 - span, row0 + span + 1):
            for col in range(col0 - span, col0 + span + 1):
                if not g.in_bounds(col, row):
                    return True  # off-map counts as explained (map edge)
                value = self.data[row * g.width + col]
                if value >= 65 or value == UNKNOWN:
                    return True
        return False
