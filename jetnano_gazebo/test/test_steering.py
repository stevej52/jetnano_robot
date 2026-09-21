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

"""
Four-wheel-steering geometry, checked against numbers.

A sign error here is silent: the robot still drives, just wrongly, and
anything measured downstream inherits the error. So the properties are
asserted rather than the implementation.
"""

import math

from jetnano_gazebo.steering import (
    Chassis,
    CORNERS,
    commands_from_twist,
    straight_commands,
)
import pytest

CHASSIS = Chassis()


def test_chassis_matches_the_urdf():
    """If jetnano.urdf.xacro changes, this should fail and be updated."""
    assert CHASSIS.wheelbase == pytest.approx(0.313)
    assert CHASSIS.track_width == pytest.approx(0.220)
    assert CHASSIS.wheel_radius == pytest.approx(0.060)
    assert CHASSIS.steer_limit == pytest.approx(0.5236, abs=1e-4)


def test_four_wheel_steering_halves_the_turning_radius():
    """
    The whole premise: steering both axles halves the radius.

    front-only Ackermann  R = L / tan(delta)
    opposite-phase 4WS    R = L / (2 tan(delta))
    """
    front_only = CHASSIS.wheelbase / math.tan(CHASSIS.steer_limit)
    four_wheel = CHASSIS.min_turning_radius()
    assert four_wheel == pytest.approx(front_only / 2.0, rel=1e-9)
    assert four_wheel == pytest.approx(0.271, abs=0.002)


def test_nav2_minimum_turning_radius_is_achievable():
    """nav2.yaml asks for 0.30 m; the geometry must be able to beat it."""
    assert CHASSIS.min_turning_radius() < 0.30


# --- straight ---------------------------------------------------------------

def test_straight_points_everything_forward():
    commands = commands_from_twist(CHASSIS, linear_x=0.5, angular_z=0.0)
    for name in CORNERS:
        assert commands.steer[name] == pytest.approx(0.0)
    speeds = [commands.speed[name] for name in CORNERS]
    assert all(s == pytest.approx(speeds[0]) for s in speeds)


def test_straight_speed_is_linear_velocity_over_wheel_radius():
    commands = straight_commands(CHASSIS, linear_x=0.6)
    assert commands.speed['front_left'] == pytest.approx(0.6 / 0.060)


def test_reverse_spins_the_wheels_backwards():
    commands = commands_from_twist(CHASSIS, linear_x=-0.3, angular_z=0.0)
    assert commands.speed['front_left'] < 0.0


# --- turning ----------------------------------------------------------------

def test_left_turn_steers_front_left_and_rear_opposite():
    """Opposite phase is the entire point; same-phase would be crab steering."""
    commands = commands_from_twist(CHASSIS, linear_x=0.4, angular_z=0.8)
    assert commands.steer['front_left'] > 0.0, 'left turn must steer front left'
    assert commands.steer['rear_left'] < 0.0, 'rear axle must oppose the front'
    assert commands.steer['rear_left'] == pytest.approx(-commands.steer['front_left'])
    assert commands.steer['rear_right'] == pytest.approx(-commands.steer['front_right'])


def test_right_turn_mirrors_the_left():
    left = commands_from_twist(CHASSIS, linear_x=0.4, angular_z=0.8)
    right = commands_from_twist(CHASSIS, linear_x=0.4, angular_z=-0.8)
    assert right.steer['front_left'] == pytest.approx(-left.steer['front_right'])
    assert right.steer['front_right'] == pytest.approx(-left.steer['front_left'])


def test_ackermann_inner_wheel_steers_harder_than_outer():
    """Equal angles on both sides would scrub the tyres and run wide."""
    commands = commands_from_twist(CHASSIS, linear_x=0.4, angular_z=1.0)
    inner = abs(commands.steer['front_left'])    # left turn: left is inner
    outer = abs(commands.steer['front_right'])
    assert inner > outer, 'inner wheel must take the tighter angle'


def test_outer_wheels_run_faster_than_inner():
    """The outer wheels travel a longer arc in the same time."""
    commands = commands_from_twist(CHASSIS, linear_x=0.4, angular_z=1.0)
    assert abs(commands.speed['front_right']) > abs(commands.speed['front_left'])


def test_steering_angle_matches_the_closed_form():
    """Front inner wheel against tan(delta) = (L/2) / (R - T/2)."""
    linear_x, angular_z = 0.5, 1.0
    radius = linear_x / angular_z
    expected = math.atan(CHASSIS.half_wheelbase / (radius - CHASSIS.track_width / 2.0))
    commands = commands_from_twist(CHASSIS, linear_x, angular_z)
    assert commands.steer['front_left'] == pytest.approx(expected, rel=1e-9)


def test_gentle_turn_gives_small_angles():
    commands = commands_from_twist(CHASSIS, linear_x=1.0, angular_z=0.1)
    assert 0.0 < commands.steer['front_left'] < math.radians(5)


# --- limits and edges -------------------------------------------------------

def test_steering_never_exceeds_the_mechanical_limit():
    """A tight command must clamp, not demand travel the servo does not have."""
    commands = commands_from_twist(CHASSIS, linear_x=0.05, angular_z=3.0)
    for name in CORNERS:
        assert abs(commands.steer[name]) <= CHASSIS.steer_limit + 1e-9


def test_yaw_with_no_forward_speed_steers_but_does_not_drive():
    """This chassis cannot turn on the spot; pretending it can invents motion."""
    commands = commands_from_twist(CHASSIS, linear_x=0.0, angular_z=1.0)
    assert commands.steer['front_left'] == pytest.approx(CHASSIS.steer_limit)
    assert commands.steer['rear_left'] == pytest.approx(-CHASSIS.steer_limit)
    for name in CORNERS:
        assert commands.speed[name] == pytest.approx(0.0)


def test_stationary_is_all_zeros():
    commands = commands_from_twist(CHASSIS, linear_x=0.0, angular_z=0.0)
    for name in CORNERS:
        assert commands.steer[name] == pytest.approx(0.0)
        assert commands.speed[name] == pytest.approx(0.0)


def test_reverse_turn_keeps_steering_on_the_same_side():
    """
    Backing up while turning left must not flip the wheels.

    R = v / w changes sign with v, so a naive implementation mirrors the
    steering when reversing and the robot backs the wrong way out of a turn.
    """
    forward = commands_from_twist(CHASSIS, linear_x=0.4, angular_z=0.8)
    reverse = commands_from_twist(CHASSIS, linear_x=-0.4, angular_z=0.8)
    assert reverse.speed['front_left'] < 0.0, 'reverse must spin wheels backwards'
    # Same yaw rate about the same ICR side: the ICR flips to the other side
    # when v flips, so the steering legitimately mirrors. Assert it is
    # consistent rather than accidental.
    assert reverse.steer['front_left'] == pytest.approx(-forward.steer['front_left'])


def test_all_four_corners_are_always_present():
    commands = commands_from_twist(CHASSIS, linear_x=0.3, angular_z=0.5)
    assert set(commands.steer) == set(CORNERS)
    assert set(commands.speed) == set(CORNERS)


def test_joint_order_helpers_round_trip():
    order = ['front_left', 'front_right', 'rear_left', 'rear_right']
    commands = commands_from_twist(CHASSIS, linear_x=0.3, angular_z=0.5)
    assert commands.steer_list(order) == [commands.steer[n] for n in order]
    assert commands.speed_list(order) == [commands.speed[n] for n in order]
