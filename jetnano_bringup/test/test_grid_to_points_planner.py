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

"""grid_to_points' planner map: /map with the camera's obstacles on it, one grid, one size."""

import array

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('cv2')

from jetnano_bringup.grid_to_points import GridToPoints  # noqa: E402
from nav_msgs.msg import OccupancyGrid  # noqa: E402


def grid(width, height, res, ox, oy, frame, cells):
    g = OccupancyGrid()
    g.header.frame_id = frame
    g.info.width, g.info.height, g.info.resolution = width, height, res
    g.info.origin.position.x, g.info.origin.position.y = ox, oy
    g.info.origin.orientation.w = 1.0
    data = np.zeros(width * height, dtype=np.int8)
    for (i, j), v in cells.items():
        data[j * width + i] = v
    g.data = array.array('b', data.tobytes())
    return g


@pytest.fixture
def node():
    rclpy.init()
    n = GridToPoints()
    sent = []
    n.planner_pub.publish = lambda m: sent.append(np.asarray(m.data, dtype=np.int8).reshape(
        m.info.height, m.info.width).copy())
    n.map_pub.publish = lambda m: None
    n.sent = sent
    n.now = lambda: n.get_clock().now().nanoseconds * 1e-9
    yield n
    n.destroy_node()
    rclpy.shutdown()


def test_walls_and_the_camera_both(node):
    node.on_map(grid(20, 20, 0.05, 0.0, 0.0, 'map', {(0, 0): 100, (19, 19): 100}))
    assert len(node.sent) == 1 and node.sent[-1][0, 0] == 100          # the house alone at first
    node.map_to_odom = (0.0, 0.0, 0.0, node.now())
    node.on_grid(grid(20, 20, 0.05, 0.0, 0.0, 'odom', {(5, 7): 100, (6, 7): 0}))
    node.publish_map_grid()
    merged = node.sent[-1]
    assert merged[0, 0] == 100 and merged[19, 19] == 100               # walls kept
    assert merged[7, 5] == 100                                        # the camera's box added
    assert merged[7, 6] == 0                                          # its free cells change nothing


def test_camera_free_never_clears_a_wall(node):
    node.on_map(grid(10, 10, 0.05, 0.0, 0.0, 'map', {(3, 3): 100}))
    node.map_to_odom = (0.0, 0.0, 0.0, node.now())
    node.on_grid(grid(10, 10, 0.05, 0.0, 0.0, 'odom', {(3, 3): 0}))
    node.publish_map_grid()
    assert node.sent[-1][3, 3] == 100


def test_follows_a_new_map_size_at_once(node):
    node.on_map(grid(10, 10, 0.05, 0.0, 0.0, 'map', {}))
    node.on_map(grid(12, 10, 0.05, 0.0, 0.0, 'map', {(11, 0): 100}))
    assert node.sent[-1].shape == (10, 12) and node.sent[-1][0, 11] == 100


def test_stale_camera_is_left_out(node):
    node.on_map(grid(10, 10, 0.05, 0.0, 0.0, 'map', {}))
    node.map_to_odom = (0.0, 0.0, 0.0, node.now())
    node.on_grid(grid(10, 10, 0.05, 0.0, 0.0, 'odom', {(2, 2): 100}))
    node.grid_t -= 10.0                                               # ten seconds old
    node.publish_map_grid()
    assert node.sent[-1][2, 2] == 0


def test_unchanged_is_not_sent_again(node):
    node.on_map(grid(10, 10, 0.05, 0.0, 0.0, 'map', {(1, 1): 100}))
    n = len(node.sent)
    node.publish_map_grid()
    node.publish_map_grid()
    assert len(node.sent) == n


def test_camera_turned_through_map_to_odom(node):
    # odom sits 0.25 m (5 cells) along map x: a camera cell at odom (0.025, 0.025) is map cell (5, 0)
    node.on_map(grid(20, 10, 0.05, 0.0, 0.0, 'map', {}))
    node.map_to_odom = (0.25, 0.0, 0.0, node.now())
    node.on_grid(grid(10, 10, 0.05, 0.0, 0.0, 'odom', {(0, 0): 100}))
    node.publish_map_grid()
    assert node.sent[-1][0, 5] == 100 and node.sent[-1][0, 0] == 0
