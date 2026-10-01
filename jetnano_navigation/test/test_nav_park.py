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
    n = math.ceil(math.radians(125) / (0.9 * nav_park.LEG_MAX_OPEN_M / nav_park.TURN_RADIUS_M))
    assert [sign for sign, _, _ in legs] == [1.0 if k % 2 == 0 else -1.0 for k in range(n)]   # forward, back, ...
    assert all(steer > 0 for sign, steer, _ in legs if sign > 0)         # all three turn her left
    assert all(steer < 0 for sign, steer, _ in legs if sign < 0)
    _, err, n = straighten(125, 0.43, 0.02, leg_max=nav_park.LEG_MAX_OPEN_M)
    assert n <= 6 and abs(math.degrees(err)) <= 6.0


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
    # a 12 cm leg at STEER turns her at most ~14 deg: 40 deg is three legs on the spot,
    # a pair in the open (30 cm legs)
    assert len(nav_park.Shuffles().next_legs(math.radians(40))) == 3
    assert len(nav_park.Shuffles(nav_park.LEG_MAX_OPEN_M).next_legs(math.radians(40))) == 2


def test_learns_the_coast():
    plan = nav_park.Shuffles()
    plan.observe(math.radians(5), math.radians(15))
    assert math.radians(4) < plan.coast < math.radians(6)


# --- turning early (2026-10-01): one arc into the point in front of the spot, then a straight
# reverse in. Drive 8 arrived in front of the spot facing -131 deg and three-point-turned there.

def drive_arc(x, y, heading, delta, radius):
    """A car at full lock from (x, y, heading), turning `delta` (+ left) -> where it ends."""
    s = 1.0 if delta > 0 else -1.0
    n = 2000
    for _ in range(n):
        step = abs(delta) / n * radius
        x += step * math.cos(heading)
        y += step * math.sin(heading)
        heading += s * step / radius
    return x, y, heading


@pytest.mark.parametrize('heading_in_deg', [0, 45, 123, -131, 180, -60])
@pytest.mark.parametrize('delta_deg', [30, 90, 131, 179, -30, -90, -131, -179])
def test_the_arc_from_its_start_ends_in_front_of_the_spot(heading_in_deg, delta_deg):
    px, py, a, d = 1.2, 0.0, math.radians(heading_in_deg), math.radians(delta_deg)
    qx, qy = nav_park.arc_start(px, py, a, d)
    r = nav_park.ARC_RADIUS_LEFT_M if d > 0 else nav_park.ARC_RADIUS_RIGHT_M
    ex, ey, eh = drive_arc(qx, qy, a, d, r)
    assert math.hypot(ex - px, ey - py) < 0.005
    assert abs(nav_park.wrap(eh - (a + d))) < 1e-6


def test_drive_8_turns_early_instead_of_three_point_turning():
    """From the dining room (4.49, -5.08) to the spot (0, 0, 0 deg): she comes in at ~123 deg
    and must leave the point in front facing 0: a right arc of ~-123 deg, started short of
    and to the left of the point in front, so the arc ends there facing the spot's heading."""
    here = (4.49, -5.08, math.radians(-131))
    qx, qy, approach, delta = nav_park.plan_approach(here, 1.2, 0.0, 0.0)
    assert delta != 0.0
    assert -math.radians(135) < delta < -math.radians(110)          # a right turn of ~123 deg
    assert math.isclose(approach, math.atan2(qy - here[1], qx - here[0]), abs_tol=1e-9)   # faces the way she goes
    r = nav_park.ARC_RADIUS_RIGHT_M
    ex, ey, eh = drive_arc(qx, qy, approach, delta, r)
    assert math.hypot(ex - 1.2, ey) < 0.01 and abs(nav_park.wrap(eh)) < 1e-6
    assert 0.3 < math.hypot(qx - 1.2, qy) < 1.0                       # started short, within the clear 1.35 m


def test_a_small_turn_or_a_short_hop_keeps_the_old_approach():
    qx, qy, approach, delta = nav_park.plan_approach((3.0, 0.0, 0.0), 1.2, 0.0, math.pi)   # straight at it: 0 to turn
    assert (qx, qy, delta) == (1.2, 0.0, 0.0)
    qx, qy, approach, delta = nav_park.plan_approach((1.5, 0.1, 0.0), 1.2, 0.0, 0.0)       # already there
    assert (qx, qy, delta) == (1.2, 0.0, 0.0)
    assert nav_park.plan_approach(None, 1.2, 0.0, 0.0)[3] == 0.0


def test_reverse_steering_turns_her_the_right_way():
    # she should turn left (heading up): reversing, that is right-hand steering
    assert nav_park.reverse_steer(math.radians(10)) < 0
    assert nav_park.reverse_steer(math.radians(-10)) > 0
    assert abs(nav_park.reverse_steer(math.radians(90))) == nav_park.REVERSE_STEER_MAX


@pytest.mark.parametrize('x0, y0, h0_deg', [(1.26, 0.09, -7), (1.2, 0.0, 0), (1.1, -0.2, 12), (1.3, 0.15, -20), (1.2, 0.1, 25)])
def test_reversing_in_lands_on_the_spot_straight(x0, y0, h0_deg):
    """Drive 8's reverse in started at (1.26, 0.09) facing -7 deg and ended +31 deg off under
    Nav2. Under the aimed reverse it ends on the spot within a few degrees."""
    x, y, h = x0, y0, math.radians(h0_deg)
    radius = 0.37
    for _ in range(4000):
        steer, along, dist = nav_park.reverse_aim(x, y, h, 0.0, 0.0, 0.0)
        if along <= nav_park.REVERSE_DONE_M or dist <= nav_park.REVERSE_DONE_M:
            break
        step = 0.001
        x -= step * math.cos(h)                        # reversing
        y -= step * math.sin(h)
        h += -1 * (steer / nav_park.STEER) / radius * step   # reversing flips the turn
    assert math.hypot(x, y) <= 0.08, f'ended {math.hypot(x, y) * 100:.0f} cm from the spot'
    assert abs(math.degrees(nav_park.wrap(h))) < 6.0, f'ended {math.degrees(nav_park.wrap(h)):+.0f} deg off'


def test_the_reverse_aim_steers_back_to_the_line():
    # she is left of the line (y > 0), facing along it. Reversing, her tail goes where her nose
    # does not point: the nose must swing left (heading up) for the tail to come right, back
    # to the line - and heading up while reversing is right-hand steering, negative (the bag
    # of drive 8: Nav2 steered negative all the way and her heading rose)
    steer, along, dist = nav_park.reverse_aim(1.0, 0.1, 0.0, 0.0, 0.0, 0.0)
    assert steer < 0 and math.isclose(along, 1.0) and math.isclose(dist, math.hypot(1.0, 0.1))
    assert nav_park.reverse_aim(1.0, -0.1, 0.0, 0.0, 0.0, 0.0)[0] > 0
    assert nav_park.reverse_aim(1.0, 0.0, 0.0, 0.0, 0.0, 0.0)[0] == 0.0
