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

"""The occupied cells of an occupancy grid, as a point cloud.

nvblox publishes what the camera sees - steps, low rocks, table edges between
10 and 35 cm above the floor - as a nav_msgs/OccupancyGrid. Nav2's costmap
reads that directly, but the collision guard (nav2_collision_monitor) takes
laser scans and point clouds only. This turns every cell at or above the
threshold into one point at a fixed height, in the grid's own frame, each time
a grid arrives (9.5 Hz, a few hundred points), so the guard can stop the robot
for something the lidar's single plane cannot see.
"""

import struct

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


class GridToPoints(Node):

    def __init__(self):
        super().__init__('grid_to_points')
        self.declare_parameter('grid_topic', '/nvblox_node/static_occupancy_grid')
        self.declare_parameter('points_topic', '/nvblox_node/obstacle_points')
        # Same threshold as Nav2's static layer reading this grid (nav2.yaml).
        self.declare_parameter('occupied_threshold', 65)
        # Where the points sit: the middle of nvblox's 0.10-0.35 m slice.
        self.declare_parameter('height', 0.20)

        self.threshold = int(self.get_parameter('occupied_threshold').value)
        self.height = float(self.get_parameter('height').value)

        self.pub = self.create_publisher(
            PointCloud2, self.get_parameter('points_topic').value,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(
            OccupancyGrid, self.get_parameter('grid_topic').value, self.on_grid,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self._packer = struct.Struct('<fff')
        self.get_logger().info(
            f"{self.get_parameter('grid_topic').value} cells >= {self.threshold} -> "
            f"{self.get_parameter('points_topic').value} at z = {self.height:.2f}")

    def on_grid(self, grid: OccupancyGrid) -> None:
        info = grid.info
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        width = info.width
        # Grids from nvblox are axis-aligned with their frame (no rotation in
        # the origin), so a cell's centre is a plain offset from the origin.
        chunks = []
        for index, value in enumerate(grid.data):
            if value >= self.threshold:
                chunks.append(self._packer.pack(
                    ox + (index % width + 0.5) * res,
                    oy + (index // width + 0.5) * res,
                    self.height))

        cloud = PointCloud2()
        cloud.header = grid.header
        cloud.height = 1
        cloud.width = len(chunks)
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12 * len(chunks)
        cloud.data = b''.join(chunks)
        cloud.is_dense = True
        self.pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = GridToPoints()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
