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

"""nav_goal's goal check on corridors inflated as Nav2 does (nav2.yaml: padded half-width
0.178, inflation 0.31, scaling 6.0), and the costmap service's 0-255 costs translated."""

import math
import types

import numpy as np
import pytest

pytest.importorskip('rclpy')

from jetnano_navigation import nav_goal  # noqa: E402

RES = 0.05


def corridor(width_m, length_m=3.0):
    """Two walls `width_m` apart (clear floor between them), inflated -> Grid, walls along x."""
    h = int(round((width_m + 1.0) / RES))
    w = int(round(length_m / RES))
    a = np.zeros((h, w), np.int16)
    wall = int(round(0.5 / RES))
    gap = int(round(width_m / RES))
    rows = np.arange(h)[:, None]
    d = np.minimum(np.abs(rows - (wall - 1)), np.abs(rows - (wall + gap))) * RES   # to the nearer wall row
    d = np.broadcast_to(d, a.shape)
    inflated = (d > 0) & (d <= 0.31)
    a[inflated] = np.round(1 + 97 * ((252 * np.exp(-6.0 * (d[inflated] - 0.178))).clip(1, 252) - 1) / 251)
    a[(d > 0) & (d <= 0.178)] = 99
    a[:wall] = 100
    a[wall + gap:] = 100
    return nav_goal.Grid(RES, w, h, 0.0, 0.0, a.reshape(-1)), (wall + gap / 2.0) * RES


@pytest.mark.parametrize('width, fits', [(0.5, False), (0.6, False), (0.7, True), (1.0, True), (1.1, True)])
def test_corridor_widths(width, fits):
    """She is 0.30 m wide and 0.44 long; turned tyres reach 0.32 m from her centre. A goal in a
    0.7 m corridor is somewhere she fits; the old check wanted about 1.0 m."""
    grid, mid = corridor(width)
    found = nav_goal.clear_goal(grid, 1.5, mid)
    assert (found is not None) == fits


def test_goal_by_a_wall_is_moved_to_the_middle():
    grid, mid = corridor(1.1)
    found = nav_goal.clear_goal(grid, 1.5, mid - 0.45)      # 10 cm off the lower wall
    assert found is not None and 0.1 < found[2] < 0.5
    assert abs(found[1] - mid) < 0.25


@pytest.mark.parametrize('level, home, allowed', [('ok', False, True), ('soon', False, True), ('low', False, False),
                                                  ('flat', False, False), ('low', True, True), ('flat', True, True),
                                                  ('none', False, True)])
def test_battery_verdict(level, home, allowed):
    import json
    ok, note = nav_goal.battery_verdict(json.dumps({'level': level, 'why': 'x'}), home)
    assert ok == allowed
    assert bool(note) == (level in ('low', 'flat', 'soon'))


def test_battery_verdict_without_a_reading_allows():
    assert nav_goal.battery_verdict(None, False) == (True, '')
    assert nav_goal.battery_verdict('garbage', False) == (True, '')


def test_costmap_service_costs_translate_as_nav2_publishes_them():
    md = types.SimpleNamespace(resolution=0.05, size_x=8, size_y=1,
                               origin=types.SimpleNamespace(position=types.SimpleNamespace(x=1.0, y=2.0)))
    cm = types.SimpleNamespace(metadata=md, data=[0, 1, 100, 128, 252, 253, 254, 255])
    g = nav_goal.grid_from_costmap(cm)
    assert list(g.data) == [0, 1, 39, 50, 98, 99, 100, -1]
    assert (g.info.width, g.info.height, g.info.resolution) == (8, 1, 0.05)
    assert math.isclose(g.info.origin.position.x, 1.0) and math.isclose(g.info.origin.position.y, 2.0)
