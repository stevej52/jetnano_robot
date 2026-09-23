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
Decide when the robot is tilting far enough to back out of it.

This is the whole decision, with no ROS in it, so the behaviour can be tested
against a list of angles and timestamps instead of against a robot on a slope.

The shape of the problem is not "is the angle too big right now". Three things
make it a state machine:

* A jolt over a rock spikes the IMU for a few tens of milliseconds. Acting on
  that would have the robot reversing away from every stone it drives over, so
  the angle has to stay past the trigger for ``debounce`` before anything
  happens.
* Triggering and releasing on the same angle oscillates forever on the
  threshold. Release is therefore a separate, lower angle.
* Releasing the moment the angle drops means stopping while still on the thing
  that caused it. Recovery runs for at least ``min_recovery`` regardless.

Roll gets a tighter trigger than pitch, and that is not arbitrary: this
chassis is longer than it is wide (0.330 m wheelbase against 0.230 m track,
tape-measured 2026-09-22), so it tips sideways sooner than it tips end over
end. A crawler is also meant to climb steep pitch, so a pitch limit as tight
as the roll limit would fight the robot's whole purpose.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math


class State(Enum):
    """Where the guard is in its cycle."""

    SAFE = 'safe'
    ARMED = 'armed'          # past the trigger, waiting out the debounce
    RECOVERING = 'recovering'
    LOCKED_OUT = 'locked_out'  # gave up; will not retry until it reads level


def roll_pitch_from_quaternion(x: float, y: float, z: float, w: float) -> tuple[float, float]:
    """
    Return (roll, pitch) in radians from a quaternion, ZYX convention.

    ``asin`` is clamped because a quaternion that is fractionally off unit
    length - which any real IMU will hand you - can otherwise push the
    argument past 1.0 and raise straight out of a sensor callback.
    """
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    return roll, pitch


Quaternion = tuple[float, float, float, float]   # (x, y, z, w), as ROS orders them


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Build a quaternion the way URDF reads an ``rpy``: Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def quaternion_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    """Hamilton product a * b: apply b's rotation first, then a's."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quaternion_conjugate(q: Quaternion) -> Quaternion:
    """The inverse rotation of a unit quaternion."""
    x, y, z, w = q
    return (-x, -y, -z, w)


def orientation_of_base(imu_orientation: Quaternion, mount: Quaternion) -> Quaternion:
    """
    Turn the IMU's reported orientation into the robot's.

    ``imu_orientation`` is what the driver publishes: the IMU frame's rotation
    in the world. ``mount`` is how the IMU is bolted on - the rotation that
    takes imu_link vectors into base_link, which is the URDF ``imu_joint``
    rotation and what ``lookup_transform('base_link', 'imu_link')`` returns.
    With R_world<-imu = R_world<-base * R_base<-imu, the robot's orientation is
    R_world<-imu * R_base<-imu^-1. The IMU's own yaw does not touch the roll
    and pitch this yields, which is all the guard reads.
    """
    return quaternion_multiply(imu_orientation, quaternion_conjugate(mount))


@dataclass
class TiltLimits:
    """
    The angles, in degrees, and the times, in seconds.

    Defaults come from this chassis's geometry. With the measured 0.230 m
    track and 0.330 m wheelbase and a guessed 0.10 m centre-of-mass height,
    the static tipping angles are about 49 degrees in roll and 59 in pitch;
    the true centre of mass (battery, Jetson, a lidar at 0.160 m) is probably
    higher, which lowers both, and dynamic rollover happens well below
    static in any case. These triggers sit near half the static figure,
    which is a starting point to tune down from, not a measurement.
    """

    roll_trigger_deg: float = 25.0
    pitch_trigger_deg: float = 35.0
    roll_release_deg: float = 15.0
    pitch_release_deg: float = 20.0
    debounce_s: float = 0.2
    min_recovery_s: float = 1.5
    max_recovery_s: float = 5.0

    def validate(self) -> None:
        """Raise if the numbers describe a guard that cannot work."""
        if self.roll_release_deg >= self.roll_trigger_deg:
            raise ValueError('roll_release_deg must be below roll_trigger_deg, or the '
                             'guard oscillates on the threshold')
        if self.pitch_release_deg >= self.pitch_trigger_deg:
            raise ValueError('pitch_release_deg must be below pitch_trigger_deg, or the '
                             'guard oscillates on the threshold')
        if self.max_recovery_s <= self.min_recovery_s:
            raise ValueError('max_recovery_s must exceed min_recovery_s')
        if self.debounce_s < 0.0:
            raise ValueError('debounce_s cannot be negative')


class TiltGuard:
    """
    Track roll and pitch over time and say when to reverse.

    Feed it :meth:`update` with angles in radians and a monotonic timestamp in
    seconds. Ask :meth:`reversing` whether a recovery command should go out.
    """

    def __init__(self, limits: TiltLimits | None = None) -> None:
        """Start in SAFE with no history."""
        self.limits = limits or TiltLimits()
        self.limits.validate()
        self.state = State.SAFE
        self._armed_at: float | None = None
        self._recovery_at: float | None = None
        self.last_reason = ''
        self.gave_up = False

    def _past_trigger(self, roll_deg: float, pitch_deg: float) -> str:
        """Return which axis is past its trigger, or '' for neither."""
        if abs(roll_deg) >= self.limits.roll_trigger_deg:
            return 'roll'
        if abs(pitch_deg) >= self.limits.pitch_trigger_deg:
            return 'pitch'
        return ''

    def _past_release(self, roll_deg: float, pitch_deg: float) -> bool:
        """True while either axis is still above its release angle."""
        return (abs(roll_deg) >= self.limits.roll_release_deg
                or abs(pitch_deg) >= self.limits.pitch_release_deg)

    def update(self, roll: float, pitch: float, now: float) -> State:
        """
        Advance the state machine one sample and return the new state.

        ``now`` is compared against stored timestamps with explicit ``is not
        None`` checks rather than ``or``: a timestamp of exactly 0.0 is falsy
        in Python, and simulated clocks start at 0.0, so ``self._armed_at or
        now`` would silently make every elapsed time zero and the guard would
        never leave ARMED. That is a bug you would only meet in Gazebo.
        """
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        axis = self._past_trigger(roll_deg, pitch_deg)

        if self.state is State.SAFE:
            if axis:
                self.state = State.ARMED
                self._armed_at = now
                self.last_reason = f'{axis} {roll_deg if axis == "roll" else pitch_deg:.1f} deg'

        elif self.state is State.ARMED:
            if not axis:
                # A jolt, not a slope. Forget it.
                self.state = State.SAFE
                self._armed_at = None
            elif self._armed_at is not None and now - self._armed_at >= self.limits.debounce_s:
                self.state = State.RECOVERING
                self._recovery_at = now
                self.gave_up = False

        elif self.state is State.RECOVERING:
            elapsed = 0.0 if self._recovery_at is None else now - self._recovery_at
            if elapsed >= self.limits.max_recovery_s:
                # Reversing has not helped. Hand control back - and stay out of
                # the way. Returning straight to SAFE would re-arm on the very
                # next sample, because the robot is still tilted, and the guard
                # would retry forever instead of letting the operator drive out.
                self.state = State.LOCKED_OUT
                self.gave_up = True
                self._recovery_at = None
                self._armed_at = None
            elif elapsed >= self.limits.min_recovery_s and not self._past_release(roll_deg,
                                                                                  pitch_deg):
                self.state = State.SAFE
                self._recovery_at = None
                self._armed_at = None

        elif self.state is State.LOCKED_OUT:
            # Only a genuinely level reading clears a lockout.
            if not self._past_release(roll_deg, pitch_deg):
                self.state = State.SAFE
                self.gave_up = False

        return self.state

    def reversing(self) -> bool:
        """True when a recovery command should be published this cycle."""
        return self.state is State.RECOVERING
