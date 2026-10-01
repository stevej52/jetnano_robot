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

"""nav_route's route parsing and pose building."""

import math

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('tf2_ros')

from jetnano_navigation import nav_route  # noqa: E402


def test_a_route_is_triplets():
    r = nav_route.parse_route(['2.8', '-3.5', '-90', '1.2', '-4.4', '180'])
    assert r == [(2.8, -3.5, math.radians(-90)), (1.2, -4.4, math.radians(180))]


@pytest.mark.parametrize('args', [[], ['1', '2'], ['1', '2', '3', '4']])
def test_a_broken_route_is_refused(args):
    with pytest.raises(SystemExit):
        nav_route.parse_route(args)


def test_drive_12s_lap_closed_on_itself():
    """The hall entry (last) lay 0.86 m from the hall exit (first), and under 0.5 m from the
    first leg's line as she drove it: refused."""
    lap = nav_route.parse_route('2.8 -3.5 -90  1.2 -4.4 180  -0.45 -5.6 -90  1.5 -7.3 0  4.5 -7.5 0  '
                                '6.5 -7.7 20  7.05 -6.5 90  5.5 -5.5 180  3.2 -4.3 110'.split())
    d_wp, d_leg = nav_route.closes_on_itself(lap)
    assert 0.8 < d_wp < 0.9 and nav_route.route_is_closed(lap)
    assert nav_route.route_is_closed(lap[:-1]) == ''                    # ending at the table's north side is fine


def test_a_short_or_open_route_is_fine():
    assert nav_route.closes_on_itself(nav_route.parse_route(['0', '0', '0', '1', '0', '0'])) is None
    assert nav_route.route_is_closed(nav_route.parse_route(['0', '0', '0', '1', '0', '0'])) == ''
    assert nav_route.route_is_closed(nav_route.parse_route(['0', '0', '0', '3', '0', '0', '3', '2', '0'])) == ''
    # a route that comes back along its own first leg
    assert nav_route.route_is_closed(nav_route.parse_route(['0', '0', '0', '3', '0', '0', '3', '2', '0', '1.5', '0.3', '0']))


LAP = '2.8 -3.5 -90  1.2 -4.4 180  -0.45 -5.6 -90  1.5 -7.3 0  4.5 -7.5 0  6.5 -7.7 20  7.05 -6.5 90  5.5 -5.5 180  3.2 -4.3 110'


def test_drive_12s_lap_is_two_stretches():
    lap = nav_route.parse_route(LAP.split())
    st = nav_route.chunks(lap, (0.0, 0.0))
    assert [len(c) for c in st] == [2, 8]                      # cut after the hall exit: the rest is clear of its end
    assert st[0] == lap[:2] and st[1] == lap[1:]               # the second starts with the first's last
    assert nav_route.chunks(lap[:-1], (0.0, 0.0)) == [lap[:-1]]


def test_two_laps_are_two_stretches_and_a_figure_eight_is_one():
    lap = nav_route.parse_route('3 0 0  3 3 90  0 3 180  0 1.5 -90'.split())
    assert nav_route.chunks(lap, (0.0, 0.0)) == [lap]
    # lap 2 starts where lap 1 began and ends where lap 1 ended: cut at each, swapped on the move
    assert [len(c) for c in nav_route.chunks(lap + lap, (0.0, 0.0))] == [2, 4, 4]
    eight = nav_route.parse_route('2 1 45  3 2 0  2 3 135  1 2 180  2 1 -45  3 0 0  4 1 90'.split())
    assert len(nav_route.chunks(eight, (1.0, 0.0))) == 1


def test_the_start_leg_counts():
    # out along the hall and back to a point on the way out: the second stretch is just the return
    r = nav_route.parse_route('3 -3 -90  1 -5 180  2 -2 135'.split())
    assert [len(c) for c in nav_route.chunks(r, (0.0, 0.0))] == [2, 2]


def test_pose_msg_carries_the_heading():
    p = nav_route.pose_msg(1.0, -2.0, math.radians(90))
    assert p.header.frame_id == 'map'
    assert (p.pose.position.x, p.pose.position.y) == (1.0, -2.0)
    assert math.isclose(p.pose.orientation.z, math.sin(math.radians(45)))
    assert math.isclose(p.pose.orientation.w, math.cos(math.radians(45)))
