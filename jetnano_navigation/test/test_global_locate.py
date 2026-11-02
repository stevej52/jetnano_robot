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

"""global_locate on synthetic rooms: a scan ray-cast from a known pose must lead back
to it; a room that looks the same both ways round must come back 'unsure'."""

import math

from jetnano_navigation.global_locate import judge, locate, MapModel
import numpy as np
import pytest

RES = 0.05


def room(shape: str):
    """Occupied/free grids (row 0 = lowest y) of a small test room, origin (0, 0)."""
    h, w = 120, 160                                  # 6 x 8 m
    occ = np.zeros((h, w), dtype=bool)
    occ[0, :] = occ[-1, :] = occ[:, 0] = occ[:, -1] = True
    if shape == 'L':
        occ[60:, 100:] = True                        # the missing corner of the L
        occ[20:26, 30:50] = True                     # a sofa
        occ[80:84, 20:24] = True                     # a table leg
    free = ~occ
    if shape == 'L':
        free[60:, 100:] = False                      # inside the missing corner is not floor
    return occ, free


def cast(occ, x, y, yaw, n=360, rmax=8.0):
    """A perfect lidar scan from (x, y, yaw): ray-march each beam to the first wall."""
    pts = []
    for a in np.linspace(-math.pi, math.pi, n, endpoint=False):
        for r in np.arange(0.05, rmax, 0.01):
            gx, gy = x + r * math.cos(yaw + a), y + r * math.sin(yaw + a)
            ci, ri = int(gx / RES), int(gy / RES)
            if not (0 <= ri < occ.shape[0] and 0 <= ci < occ.shape[1]):
                break
            if occ[ri, ci]:
                pts.append((r * math.cos(a), r * math.sin(a)))
                break
    return np.array(pts)


def test_finds_the_pose_it_was_cast_from():
    occ, free = room('L')
    model = MapModel(occ, free, RES, 0.0, 0.0)
    x, y, yaw = 2.37, 1.81, math.radians(37)
    cands = locate(model, cast(occ, x, y, yaw))
    best = cands[0]
    assert math.hypot(best[1] - x, best[2] - y) < 0.06
    assert abs(math.degrees(math.remainder(best[3] - yaw, 2 * math.pi))) < 3.0
    assert best[0] > 0.9
    assert judge(cands, 0.7, 1.15)[0] == 'placed'


def test_a_symmetric_room_is_unsure():
    occ, free = room('box')                          # a bare rectangle: 180 degrees alike
    model = MapModel(occ, free, RES, 0.0, 0.0)
    cands = locate(model, cast(occ, 2.0, 2.0, 0.0))
    assert len(cands) >= 2
    verdict, why = judge(cands, 0.7, 1.15)
    assert verdict == 'unsure', why


def test_nothing_to_go_on():
    occ, free = room('L')
    model = MapModel(occ, free, RES, 0.0, 0.0)
    assert judge(locate(model, np.zeros((5, 2))), 0.7, 1.15)[0] == 'not_on_map'


def test_near_searches_only_round_the_spot():
    occ, free = room('L')
    model = MapModel(occ, free, RES, 0.0, 0.0)
    x, y, yaw = 2.37, 1.81, math.radians(37)
    pts = cast(occ, x, y, yaw)
    there = locate(model, pts, near=(2.0, 2.0), radius=1.0)[0]
    assert math.hypot(there[1] - x, there[2] - y) < 0.06
    far = locate(model, pts, near=(6.5, 1.0), radius=1.0)
    assert far and all(math.hypot(c[1] - 6.5, c[2] - 1.0) <= 1.0 + 0.13 for c in far)
    assert far[0][0] < there[0]


def test_a_drawn_heading_settles_a_symmetric_room():
    occ, free = room('box')                          # the middle of a bare rectangle: 0 and 180 alike
    model = MapModel(occ, free, RES, 0.0, 0.0)
    pts = cast(occ, 4.0, 3.0, math.radians(10))
    assert judge(locate(model, pts, near=(4.0, 3.0), radius=0.75), 0.5, 1.15)[0] == 'unsure'
    cands = locate(model, pts, near=(4.1, 2.9), radius=0.75, heading=math.radians(40))
    assert judge(cands, 0.5, 1.15)[0] == 'placed'
    assert abs(math.degrees(math.remainder(cands[0][3] - math.radians(10), 2 * math.pi))) < 3.0
    assert math.hypot(cands[0][1] - 4.0, cands[0][2] - 3.0) < 0.06


@pytest.mark.parametrize('pose, then, now, expect', [
    # facing +y on the map, 1 m forward on the odometry: 1 m further along +y
    ((5.0, 5.0, math.pi / 2), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (5.0, 6.0, math.pi / 2)),
    # the odometry's own frame turned 90 degrees: still 1 m forward for her
    ((5.0, 5.0, 0.0), (0.0, 0.0, math.pi / 2), (0.0, 1.0, math.pi / 2), (6.0, 5.0, 0.0)),
    # a turn in place turns her on the map by the same amount
    ((1.0, 2.0, 0.5), (3.0, 3.0, 1.0), (3.0, 3.0, 1.3), (1.0, 2.0, 0.8)),
])
def test_propagate(pose, then, now, expect):
    from jetnano_navigation.where_am_i import propagate
    got = propagate(pose, then, now)
    assert got == pytest.approx(expect, abs=1e-9)


@pytest.mark.parametrize('best_at, verdict', [((0.5, 0.0), 'placed'), ((0.65, 0.0), 'unsure')])
def test_judge_wants_the_best_inside_the_circle(best_at, verdict):
    # searched 0.75 m round (0, 0): a best on the edge was pulled there from outside
    cands = [(0.8, best_at[0], best_at[1], 0.0), (0.4, -0.3, 0.2, 2.0)]
    assert judge(cands, 0.5, 1.15, (0.0, 0.0), 0.75)[0] == verdict


@pytest.mark.parametrize('score, second, verdict', [(0.66, 0.64, 'not_on_map'),   # the bench
                                                     (0.85, 0.83, 'unsure'),
                                                     (0.85, 0.60, 'placed')])
def test_judge(score, second, verdict):
    cands = [(score, 0, 0, 0), (second, 5, 5, 1)]
    assert judge(cands, 0.7, 1.15)[0] == verdict
