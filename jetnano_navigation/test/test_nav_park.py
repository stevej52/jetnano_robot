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

"""nav_park's straightening, on a car that steers (a bicycle model) - one that turns harder
than the geometry says and coasts on after the throttle drops, as Rosie did on the floor
(2026-09-29: each shuffle pair turned her about twice what the geometry predicted)."""

import math

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('tf2_ros')

from jetnano_navigation import nav_park  # noqa: E402


def car_leg(heading, throttle_sign, steer, stop_at, radius_full, coast_m, leg_max=nav_park.LEG_MAX_M):
    """Drive until the heading has turned `stop_at` (what nav_park watches), then coast on.
    -> (new heading, how far it turned in all)."""
    radius = radius_full * nav_park.STEER / abs(steer)      # half lock: twice the radius
    rate = throttle_sign * (1 if steer > 0 else -1) / radius
    start, dist = heading, 0.0
    while abs(heading - start) < stop_at and dist < leg_max:
        heading += rate * 0.005
        dist += 0.005
    heading += rate * coast_m
    return heading, abs(heading - start)


def straighten(err0_deg, radius_full=0.25, coast_m=0.02, tol_deg=6.0, leg_max=nav_park.LEG_MAX_M):
    """-> (cycles, error left, legs driven)."""
    target, heading = math.radians(err0_deg), 0.0
    plan = nav_park.Shuffles(leg_max)
    legs = 0
    for cycle in range(nav_park.MAX_CYCLES + 1):
        err = nav_park.wrap(target - heading)
        if abs(err) <= math.radians(tol_deg):
            return cycle, err, legs
        for sign, steer, stop_at in plan.next_legs(err):
            heading, turned = car_leg(heading, sign, steer, stop_at, radius_full, coast_m, leg_max)
            plan.observe(stop_at, turned)
            legs += 1
    return None, nav_park.wrap(target - heading), legs


@pytest.mark.parametrize('leg_max', [nav_park.LEG_MAX_M, nav_park.LEG_MAX_OPEN_M])
@pytest.mark.parametrize('err_deg', [-179, -120, -65, -30, -23, -14, -8, 8, 14, 23, 30, 43, 65, 120, 179])
@pytest.mark.parametrize('radius, coast', [(0.37, 0.0), (0.25, 0.02), (0.20, 0.03)])
def test_straightens_within_the_cycles(err_deg, radius, coast, leg_max):
    cycles, err, _ = straighten(err_deg, radius, coast, leg_max=leg_max)
    assert cycles is not None, f'still {math.degrees(err):+.0f} deg off'
    assert abs(math.degrees(err)) <= 6.0


def test_a_big_turn_in_the_open_is_a_three_point_turn():
    """Drive 7: arriving the way she was going, ~125 deg from the spot's heading."""
    legs = nav_park.Shuffles(nav_park.LEG_MAX_OPEN_M).next_legs(math.radians(125))
    assert [sign for sign, _, _ in legs] == [1.0, -1.0, 1.0]            # forward, back, forward
    assert all(steer > 0 for sign, steer, _ in legs if sign > 0)         # all three turn her left
    assert all(steer < 0 for sign, steer, _ in legs if sign < 0)
    _, err, n = straighten(125, 0.37, 0.02, leg_max=nav_park.LEG_MAX_OPEN_M)
    assert n <= 5 and abs(math.degrees(err)) <= 6.0


def test_open_legs_need_fewer_moves_than_short_ones():
    for err in (65, 120, 179):
        short = straighten(err, 0.37, 0.02)[2]
        open_ = straighten(err, 0.37, 0.02, leg_max=nav_park.LEG_MAX_OPEN_M)[2]
        assert open_ < short, f'{err} deg: {open_} legs in the open vs {short} short'


def test_already_straight_does_nothing():
    assert straighten(3)[0] == 0 and straighten(3)[2] == 0


def test_small_corrections_use_half_lock_and_one_leg():
    assert nav_park.steer_for(math.radians(10)) == 0.5 * nav_park.STEER
    assert nav_park.steer_for(math.radians(40)) == nav_park.STEER
    assert len(nav_park.Shuffles().next_legs(math.radians(10))) == 1
    assert len(nav_park.Shuffles().next_legs(math.radians(40))) == 2


def test_learns_the_coast():
    plan = nav_park.Shuffles()
    plan.observe(math.radians(5), math.radians(15))
    assert math.radians(4) < plan.coast < math.radians(6)
