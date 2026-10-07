# Copyright 2026 stevej52
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""The occupied cells of an occupancy grid, as a point cloud - and the grid on the map.

nvblox publishes what the camera sees - steps, low rocks, table edges between
10 and 35 cm above the floor - as a nav_msgs/OccupancyGrid. Nav2's costmap
reads that directly, but the collision guard (nav2_collision_monitor) takes
laser scans and point clouds only. This turns every cell at or above the
threshold into one point at a fixed height, in the grid's own frame, each time
a grid arrives (40 Hz, a few hundred points), so the guard can stop the robot
for something the lidar's single plane cannot see.

It also redraws the grid onto the house map for Nav2's route planner. nvblox's grid
is in the odom frame and the planner's costmap in the map frame, and Nav2's
StaticLayer only moves a map between frames in a rolling costmap, so the grid is
turned through SLAM's map -> odom here and laid on /map's own cells
(``map_grid_topic``, the camera alone, for the recordings). Before this (2026-09-28)
the planner had only the lidar and the saved map: a box, a bag and a floor cushion
the lidar looks over made it plan straight through them twice on one drive, and
the controller, which does see the camera, refused and gave up.

What the planner reads is ``planner_map_topic``: /map with the camera's obstacles
stamped on, one grid (2026-09-29). It had read the two as separate StaticLayers,
and each resizes the whole costmap to its own map's size: /map in localization grows
and shrinks by a few cells as scans are added, the camera's copy followed a moment
later, and every mismatch (58 in an hour) wiped the other layer until its next
message - the planner at times saw only the camera's blobs and no walls. One grid
has one size. With no fresh camera grid it is /map as it is, so the planner never
loses the house.
"""

import array
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, PointField
from tf2_msgs.msg import TFMessage

try:
    import cv2
except ImportError:                   # no redrawing onto the map without OpenCV
    cv2 = None

# a TFMessage holding "map" as a frame id: the only /tf messages worth decoding here
# (CDR string: length 4 with the terminator, then the characters)
MAP_FRAME_CDR = b'\x04\x00\x00\x00map\x00'


class GridToPoints(Node):

    def __init__(self):
        super().__init__('grid_to_points')
        self.declare_parameter('grid_topic', '/nvblox_node/static_occupancy_grid')
        self.declare_parameter('points_topic', '/nvblox_node/obstacle_points')
        # Same threshold as Nav2's static layer reading this grid (nav2.yaml).
        self.declare_parameter('occupied_threshold', 65)
        # Where the points sit: the middle of nvblox's 0.10-0.35 m slice.
        self.declare_parameter('height', 0.20)
        # the grid on the house map, for the planner's camera_layer (nav2.yaml); '' = off
        self.declare_parameter('map_grid_topic', '/nvblox_node/map_grid')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('stale_s', 3.0)
        self.stale_s = float(self.get_parameter('stale_s').value)
        self.declare_parameter('map_grid_hz', 2.0)
        # /map plus the camera's obstacles, for the planner (navigation.launch.py nvblox:=true)
        self.declare_parameter('planner_map_topic', '/planner_map')
        # a camera grid older than this is not laid on the planner's map
        self.declare_parameter('grid_max_age_s', 2.0)
        # Places she must plan round although neither sensor sees them reliably, as map boxes
        # [x0, y0, x1, y1, ...], stamped on the planner's map. 2026-10-06: the robot vacuum and its
        # dock in the dining-room corner (Steve's turn 4; "where it has always been") - ~10 cm
        # tall, under the lidar's plane and the camera slice's 10 cm floor; the camera put it at
        # x 7.45-7.9, y -7.55..-7.9 (drive 46), she hit it nose-first at x ~7.2.
        self.declare_parameter('keep_out', [7.25, -8.0, 8.0, -7.35])
        k = [float(v) for v in self.get_parameter('keep_out').value]
        self.keep_out = [tuple(k[i:i + 4]) for i in range(0, len(k) - 3, 4)]
        # bumps (mission route_run): a spot she pushed at without moving, kept as a box this long
        self.declare_parameter('bump_half_m', 0.10)
        self.declare_parameter('bump_keep_s', 600.0)
        self.bumps = []                   # (x0, y0, x1, y1, until)

        self.threshold = int(self.get_parameter('occupied_threshold').value)
        self.height = float(self.get_parameter('height').value)
        self.pub = self.create_publisher(
            PointCloud2, self.get_parameter('points_topic').value,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(
            OccupancyGrid, self.get_parameter('grid_topic').value, self.on_grid,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))

        self.grid = None                  # nvblox's latest
        self.grid_t = 0.0                 # when it came (this node's clock)
        self.map_msg = None               # /map, all of it: the planner's map is built on it
        self.map_info = None
        self.last_planner = None          # (geometry, bytes) last published, to skip repeats
        self.max_age = float(self.get_parameter('grid_max_age_s').value)
        self.map_to_odom = None           # (x, y, yaw, when)
        topic = str(self.get_parameter('map_grid_topic').value)
        if topic and cv2 is not None:
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.map_pub = self.create_publisher(OccupancyGrid, topic, latched)
            self.planner_pub = self.create_publisher(
                OccupancyGrid, str(self.get_parameter('planner_map_topic').value), latched)
            self.create_subscription(OccupancyGrid, self.get_parameter('map_topic').value,
                                     self.on_map, latched)
            # /tf comes 140 times a second; only SLAM's map -> odom matters, so the
            # messages are looked at undecoded and only those naming "map" decoded
            self.create_subscription(TFMessage, '/tf', self.on_tf_raw,
                                     QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE),
                                     raw=True)
            self.create_timer(1.0 / float(self.get_parameter('map_grid_hz').value), self.publish_map_grid)
            self.create_subscription(PointStamped, 'bump', self.on_bump, 10)
        self.get_logger().info(
            f"{self.get_parameter('grid_topic').value} cells >= {self.threshold} -> "
            f"{self.get_parameter('points_topic').value} at z = {self.height:.2f}"
            + (f', and onto the map as {topic}' if topic and cv2 is not None else ''))

    def on_bump(self, msg: PointStamped) -> None:
        h = float(self.get_parameter('bump_half_m').value)
        until = self.get_clock().now().nanoseconds * 1e-9 + float(self.get_parameter('bump_keep_s').value)
        x, y = msg.point.x, msg.point.y
        self.bumps.append((x - h, y - h, x + h, y + h, until))
        self.get_logger().info(f'bump at ({x:+.2f}, {y:+.2f}): a box on the planner map for '
                               f'{self.get_parameter("bump_keep_s").value:.0f} s')
        self.publish_map_grid()

    def on_map(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg
        self.map_info = msg.info
        self.publish_map_grid()           # a new /map (maybe a new size): the planner's at once

    def on_tf_raw(self, raw: bytes) -> None:
        if MAP_FRAME_CDR not in raw:
            return
        for t in deserialize_message(raw, TFMessage).transforms:
            if t.header.frame_id.lstrip('/') == 'map' and t.child_frame_id.lstrip('/') == 'odom':
                q = t.transform.rotation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                self.map_to_odom = (t.transform.translation.x, t.transform.translation.y, yaw,
                                    self.get_clock().now().nanoseconds * 1e-9)

    def on_grid(self, grid: OccupancyGrid) -> None:
        self.grid = grid
        self.grid_t = self.get_clock().now().nanoseconds * 1e-9
        info = grid.info
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        width = info.width
        # Grids from nvblox are axis-aligned with their frame (no rotation in
        # the origin), so a cell's centre is a plain offset from the origin.
        # numpy, not a Python loop over every cell (2026-09-27: that loop was
        # ~4 % of a core at 9.5 grids a second).
        cells = np.flatnonzero(np.asarray(grid.data, dtype=np.int8) >= self.threshold)
        points = np.empty((len(cells), 3), dtype='<f4')
        points[:, 0] = ox + (cells % width + 0.5) * res
        points[:, 1] = oy + (cells // width + 0.5) * res
        points[:, 2] = self.height

        cloud = PointCloud2()
        cloud.header = grid.header
        # nvblox keeps publishing its last grid, with its last stamp, when nothing new is
        # integrated (2026-10-01 19:31: 73 s and counting). The guard refuses a stale source
        # and then holds EVERY command, the lidar's included, for as long as it lasts. A grid
        # older than stale_s goes out as an empty cloud stamped now: the camera's obstacles
        # are gone until it moves again, the lidar still guards, and guard_flow says so.
        age = (self.get_clock().now() - rclpy.time.Time.from_msg(grid.header.stamp)).nanoseconds * 1e-9
        if age > self.stale_s:
            points = points[:0]
            cells = cells[:0]
            cloud.header.stamp = self.get_clock().now().to_msg()
            self.get_logger().warning(f'nvblox grid {age:.0f} s stale: camera obstacles off until it moves again',
                                      throttle_duration_sec=30.0)
        cloud.height = 1
        cloud.width = len(cells)
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12 * len(cells)
        cloud.data = points.tobytes()
        cloud.is_dense = True
        self.pub.publish(cloud)

    def publish_map_grid(self) -> None:
        """nvblox's grid (odom frame) laid on /map's cells: occupied, free, or unknown (-1),
        on map_grid_topic; and /map with its occupied cells added, on planner_map_topic."""
        g, m, mo = self.grid, self.map_info, self.map_to_odom
        if m is None:
            return                                     # no SLAM: no map to plan on
        now = self.get_clock().now().nanoseconds * 1e-9
        out = None
        if g is not None and now - self.grid_t <= self.max_age and mo is not None and now - mo[3] <= 2.0:
            tx, ty, yaw, _ = mo
            c, s = math.cos(yaw), math.sin(yaw)
            res, ox, oy = g.info.resolution, g.info.origin.position.x, g.info.origin.position.y
            mr, mx, my = m.resolution, m.origin.position.x, m.origin.position.y
            # map cell (u, v) centre -> map metres -> odom metres (p_odom = R^T (p_map - t))
            # -> nvblox cell (i, j): one affine map, for warpAffine's inverse mapping
            k = mr / res
            ax, ay = mx + 0.5 * mr - tx, my + 0.5 * mr - ty
            warp = np.array([[c * k, s * k, (c * ax + s * ay - ox) / res - 0.5],
                             [-s * k, c * k, (-s * ax + c * ay - oy) / res - 0.5]])
            src = np.asarray(g.data, dtype=np.int8).reshape(g.info.height, g.info.width).astype(np.int16)
            out = cv2.warpAffine(src, warp, (m.width, m.height),
                                 flags=cv2.INTER_NEAREST | cv2.WARP_INVERSE_MAP,
                                 borderMode=cv2.BORDER_CONSTANT, borderValue=-1)
            msg = OccupancyGrid()
            msg.header.frame_id = 'map'
            msg.header.stamp = g.header.stamp
            msg.info = m
            msg.data = array.array('b', out.astype(np.int8).tobytes())
            self.map_pub.publish(msg)
        self.publish_planner_map(out)

    def publish_planner_map(self, camera) -> None:
        """/map, with the cells the camera has occupied (at the threshold) set to 100."""
        base = self.map_msg
        if base is None:
            return
        info = base.info
        grid = np.asarray(base.data, dtype=np.int8)
        if camera is not None and camera.shape == (info.height, info.width):
            grid = grid.copy()
            grid[camera.reshape(-1) >= self.threshold] = 100
        now = self.get_clock().now().nanoseconds * 1e-9
        self.bumps = [b for b in self.bumps if b[4] > now]
        boxes = list(self.keep_out) + [b[:4] for b in self.bumps]
        if boxes:
            grid = grid.copy().reshape(info.height, info.width)
            res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
            for x0, y0, x1, y1 in boxes:
                u0, u1 = max(0, round((min(x0, x1) - ox) / res)), min(info.width, round((max(x0, x1) - ox) / res))
                v0, v1 = max(0, round((min(y0, y1) - oy) / res)), min(info.height, round((max(y0, y1) - oy) / res))
                if u0 < u1 and v0 < v1:
                    grid[v0:v1, u0:u1] = 100
            grid = grid.reshape(-1)
        geometry = (info.width, info.height, info.resolution, info.origin.position.x, info.origin.position.y)
        data = grid.tobytes()
        if self.last_planner == (geometry, data):
            return                                     # nothing new: no re-inflating the whole house
        self.last_planner = (geometry, data)
        msg = OccupancyGrid()
        msg.header.frame_id = base.header.frame_id or 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info = info
        msg.data = array.array('b', data)
        self.planner_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GridToPoints()
    # rclpy's EventsExecutor: 40 grids and 140 /tf messages a second would have the
    # default executor rebuild its wait set in Python on every one (see web_teleop)
    try:
        from rclpy.experimental import EventsExecutor
        executor = EventsExecutor()
    except ImportError:
        from rclpy.executors import SingleThreadedExecutor
        executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except (Exception, KeyboardInterrupt):  # noqa: BLE001 - the context is gone, or a second SIGINT, after an external shutdown
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
