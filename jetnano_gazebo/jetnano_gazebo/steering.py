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
Four-wheel-steering kinematics: a Twist in, eight joint commands out.

No ROS here, so the geometry can be tested against numbers instead of against
a robot in a simulator. This is the file where a mistake would be silent: a
sign error in the rear axle looks like a robot that drives, just wrongly, and
every measurement taken downstream of it would be quietly wrong too.

THE GEOMETRY

With both axles steering in OPPOSITE phase, the instantaneous centre of
rotation sits level with the MIDDLE of the wheelbase rather than with the rear
axle as it does on a front-steering car. The front axle is half a wheelbase
ahead of the ICR instead of a whole one, so::

    front-only Ackermann   tan(delta) = L / R
    opposite-phase 4WS     tan(delta) = (L/2) / R

Same steering angle, half the turning radius. For this chassis, L = 0.313 m
and a 30 degree limit::

    front-only   R = 0.313 / tan(30) = 0.542 m
    4WS          R = 0.313 / (2 tan(30)) = 0.271 m

which is where nav2.yaml's minimum_turning_radius of 0.30 m comes from, with
a little margin on top.

ACKERMANN CORRECTION

The inner wheel of a turn traces a tighter circle than the outer one, so
pointing both at the same angle scrubs the tyres and pushes the robot wide.
Each wheel is aimed at the ICR individually::

    tan(delta_inner) = (L/2) / (R - T/2)
    tan(delta_outer) = (L/2) / (R + T/2)

and each wheel's speed is its own distance from the ICR times the body's yaw
rate, so the outer wheels turn faster than the inner ones.

SIGN CONVENTION

ROS REP-103: x forward, y left, positive yaw counter-clockwise. A positive
``angular.z`` is therefore a LEFT turn, which puts the ICR at positive y and
steers the front wheels to positive angles. The rear wheels take the negative
of the front, which is what makes it four-wheel steering rather than four
wheels that happen to steer. pca9685.yaml encodes the same relationship on
the real robot as angular_z scalings of -18.33 and +18.33.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

# Below this yaw rate the turn radius is enormous and the arithmetic becomes
# numerically silly; treat it as straight ahead.
STRAIGHT_EPSILON = 1e-4


@dataclass(frozen=True)
class Chassis:
    """Measurements from jetnano.urdf.xacro. Defaults are that robot."""

    wheelbase: float = 0.313
    track_width: float = 0.220
    wheel_radius: float = 0.060
    steer_limit: float = 0.5236      # 30 degrees, the servo's mechanical limit

    @property
    def half_wheelbase(self) -> float:
        """Distance from the ICR line to either axle, in opposite-phase 4WS."""
        return self.wheelbase / 2.0

    def min_turning_radius(self) -> float:
        """Tightest radius the steering limit allows, at the body centre."""
        return self.half_wheelbase / math.tan(self.steer_limit)


@dataclass
class WheelCommands:
    """What to send the eight joints, in the order the controllers expect."""

    steer: dict[str, float]      # radians, positive counter-clockwise
    speed: dict[str, float]      # radians per second at the wheel joint

    def steer_list(self, order: list[str]) -> list[float]:
        """Steering angles for a named joint order."""
        return [self.steer[name] for name in order]

    def speed_list(self, order: list[str]) -> list[float]:
        """Wheel speeds for a named joint order."""
        return [self.speed[name] for name in order]


CORNERS = ('front_left', 'front_right', 'rear_left', 'rear_right')


def _corner_offsets(chassis: Chassis) -> dict[str, tuple[float, float]]:
    """Body-frame (x, y) of each wheel: x forward, y left."""
    half_l = chassis.half_wheelbase
    half_t = chassis.track_width / 2.0
    return {
        'front_left': (half_l, half_t),
        'front_right': (half_l, -half_t),
        'rear_left': (-half_l, half_t),
        'rear_right': (-half_l, -half_t),
    }


def straight_commands(chassis: Chassis, linear_x: float) -> WheelCommands:
    """Everything pointing forward, every wheel at the same speed."""
    speed = linear_x / chassis.wheel_radius
    return WheelCommands(
        steer={name: 0.0 for name in CORNERS},
        speed={name: speed for name in CORNERS},
    )


def commands_from_twist(chassis: Chassis, linear_x: float,
                        angular_z: float) -> WheelCommands:
    """
    Turn a Twist into per-wheel steering angles and wheel-joint speeds.

    ``linear_x`` is the body's forward speed in m/s and ``angular_z`` its yaw
    rate in rad/s, both as they arrive on ``cmd_vel``.

    Turning on the spot is not possible on this chassis - the wheels cannot
    reach 90 degrees - so a yaw rate with no forward speed steers the wheels
    to the limit and does not drive them. That is what the real robot does
    too, and it is better than inventing motion the machine cannot produce.
    """
    if abs(angular_z) < STRAIGHT_EPSILON:
        return straight_commands(chassis, linear_x)

    if abs(linear_x) < STRAIGHT_EPSILON:
        # No forward speed: point the wheels into the turn, do not drive.
        sign = 1.0 if angular_z > 0.0 else -1.0
        limit = chassis.steer_limit
        return WheelCommands(
            steer={
                'front_left': sign * limit, 'front_right': sign * limit,
                'rear_left': -sign * limit, 'rear_right': -sign * limit,
            },
            speed={name: 0.0 for name in CORNERS},
        )

    # Signed turn radius at the body centre. Positive angular_z is a left
    # turn, which puts the ICR at positive y.
    radius = linear_x / angular_z
    icr_y = radius

    steer: dict[str, float] = {}
    speed: dict[str, float] = {}
    for name, (x, y) in _corner_offsets(chassis).items():
        # Vector from this wheel to the ICR, in the body frame.
        to_icr_y = icr_y - y
        # The wheel rolls perpendicular to that vector. atan2 keeps the sign
        # right in all four quadrants, including reverse and right turns.
        steer[name] = math.atan2(x, abs(to_icr_y)) * (1.0 if to_icr_y > 0 else -1.0)

        # Speed is the body's yaw rate times this wheel's distance from the
        # ICR, so outer wheels turn faster than inner ones.
        distance = math.hypot(x, to_icr_y)
        wheel_speed = abs(angular_z) * distance
        # Reversing must spin the wheels backwards.
        if linear_x < 0.0:
            wheel_speed = -wheel_speed
        speed[name] = wheel_speed / chassis.wheel_radius

    _clamp_steering(steer, chassis.steer_limit)
    return WheelCommands(steer=steer, speed=speed)


def _clamp_steering(steer: dict[str, float], limit: float) -> None:
    """Hold every wheel inside the servo's mechanical travel, in place."""
    for name, angle in steer.items():
        steer[name] = max(-limit, min(limit, angle))
