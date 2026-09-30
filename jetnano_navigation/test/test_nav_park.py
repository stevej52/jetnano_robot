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

"""nav_park's straightening, on a car that steers (a bicycle model at full lock)."""

import math

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('tf2_ros')

from jetnano_navigation import nav_park  # noqa: E402


def drive(heading, throttle_sign, steer_sign, dist):
    """A car's heading after `dist` m at full lock: turns by v * steer / R."""
    return heading + throttle_sign * steer_sign * dist / nav_park.TURN_RADIUS_M


def straighten(err0_deg, tol_deg=6.0):
    target, heading = math.radians(err0_deg), 0.0
    for cycle in range(nav_park.MAX_CYCLES + 1):
        err = nav_park.wrap(target - heading)
        if abs(err) <= math.radians(tol_deg):
            return cycle, err
        s = 1.0 if err > 0 else -1.0
        d = nav_park.leg_length(err)
        before = abs(err)
        heading = drive(heading, +1, s, d)             # forward, steering s
        heading = drive(heading, -1, -s, d)            # back, steering the other way
        assert abs(nav_park.wrap(target - heading)) < before + 1e-9   # never the wrong way
    return None, nav_park.wrap(target - heading)


@pytest.mark.parametrize('err_deg', [-179, -120, -65, -30, -8, 8, 30, 43, 65, 120, 179])
def test_straightens_within_the_cycles(err_deg):
    cycles, err = straighten(err_deg)
    assert cycles is not None, f'still {math.degrees(err):+.0f} deg off'
    assert abs(math.degrees(err)) <= 6.0


def test_already_straight_does_nothing():
    assert straighten(3)[0] == 0


def test_leg_lengths_bounded():
    assert nav_park.leg_length(math.radians(1)) == nav_park.LEG_MIN_M
    assert nav_park.leg_length(math.pi) == nav_park.LEG_MAX_M
    # a small error: one pair of legs turns her about that much
    e = math.radians(20)
    assert abs(2 * nav_park.leg_length(e) / nav_park.TURN_RADIUS_M - e) < 1e-9
