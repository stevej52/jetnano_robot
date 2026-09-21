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

"""The tilt decision, driven by a list of angles instead of a robot on a slope."""

import math

from jetnano_bringup.tilt import (
    roll_pitch_from_quaternion,
    State,
    TiltGuard,
    TiltLimits,
)
import pytest

LEVEL = 0.0


def quaternion_from_roll_pitch(roll: float, pitch: float) -> tuple:
    """Build (x, y, z, w) for a roll/pitch with zero yaw, for the conversion tests."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    return (sr * cp, cr * sp, -sr * sp, cr * cp)


def drive(guard: TiltGuard, roll_deg: float, pitch_deg: float,
          start: float, duration: float, step: float = 0.02) -> float:
    """Hold an attitude for ``duration`` seconds; return the time reached."""
    now = start
    end = start + duration
    while now <= end:
        guard.update(math.radians(roll_deg), math.radians(pitch_deg), now)
        now += step
    return now


# --- the quaternion conversion ---------------------------------------------

def test_identity_quaternion_is_level():
    roll, pitch = roll_pitch_from_quaternion(0.0, 0.0, 0.0, 1.0)
    assert roll == pytest.approx(0.0, abs=1e-9)
    assert pitch == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize('roll_deg,pitch_deg', [
    (30.0, 0.0), (-30.0, 0.0), (0.0, 25.0), (0.0, -25.0), (15.0, -20.0),
])
def test_conversion_round_trips(roll_deg, pitch_deg):
    x, y, z, w = quaternion_from_roll_pitch(math.radians(roll_deg), math.radians(pitch_deg))
    roll, pitch = roll_pitch_from_quaternion(x, y, z, w)
    assert math.degrees(roll) == pytest.approx(roll_deg, abs=1e-6)
    assert math.degrees(pitch) == pytest.approx(pitch_deg, abs=1e-6)


def test_non_unit_quaternion_does_not_raise():
    """A real IMU sends slightly off-unit quaternions; asin must not blow up."""
    roll, pitch = roll_pitch_from_quaternion(0.0, 0.7072, 0.0, 0.7072)
    assert math.isfinite(roll) and math.isfinite(pitch)


# --- the limits themselves --------------------------------------------------

def test_release_at_or_above_trigger_is_rejected():
    """Equal trigger and release oscillates forever; refuse it at construction."""
    with pytest.raises(ValueError, match='roll_release_deg'):
        TiltGuard(TiltLimits(roll_trigger_deg=25.0, roll_release_deg=25.0))
    with pytest.raises(ValueError, match='pitch_release_deg'):
        TiltGuard(TiltLimits(pitch_trigger_deg=35.0, pitch_release_deg=40.0))


def test_max_recovery_must_exceed_min():
    with pytest.raises(ValueError, match='max_recovery_s'):
        TiltGuard(TiltLimits(min_recovery_s=3.0, max_recovery_s=1.0))


def test_roll_is_tighter_than_pitch_by_default():
    """The chassis is longer than it is wide, so it tips sideways sooner."""
    limits = TiltLimits()
    assert limits.roll_trigger_deg < limits.pitch_trigger_deg


# --- the state machine ------------------------------------------------------

def test_level_robot_stays_safe():
    guard = TiltGuard()
    drive(guard, LEVEL, LEVEL, start=0.0, duration=5.0)
    assert guard.state is State.SAFE
    assert not guard.reversing()


def test_a_jolt_shorter_than_the_debounce_does_not_trigger():
    """Driving over a stone spikes the IMU. That must not start a recovery."""
    guard = TiltGuard(TiltLimits(debounce_s=0.2))
    now = drive(guard, 40.0, LEVEL, start=0.0, duration=0.1)   # past trigger, briefly
    assert guard.state is State.ARMED, 'should be counting down, not recovering'
    drive(guard, LEVEL, LEVEL, start=now, duration=0.5)
    assert guard.state is State.SAFE
    assert not guard.reversing()


def test_sustained_roll_triggers_after_the_debounce():
    guard = TiltGuard(TiltLimits(debounce_s=0.2))
    drive(guard, 30.0, LEVEL, start=0.0, duration=0.5)
    assert guard.state is State.RECOVERING
    assert guard.reversing()
    assert 'roll' in guard.last_reason


def test_sustained_pitch_triggers_too():
    guard = TiltGuard(TiltLimits(debounce_s=0.2))
    drive(guard, LEVEL, 40.0, start=0.0, duration=0.5)
    assert guard.state is State.RECOVERING
    assert 'pitch' in guard.last_reason


def test_pitch_below_its_trigger_does_not_fire_even_past_the_roll_trigger():
    """30 degrees of pitch is a climb, not an emergency."""
    guard = TiltGuard()
    drive(guard, LEVEL, 30.0, start=0.0, duration=2.0)
    assert guard.state is State.SAFE


def test_recovery_holds_for_min_recovery_even_once_level():
    """Releasing the instant the angle drops stops the robot still on the obstacle."""
    guard = TiltGuard(TiltLimits(debounce_s=0.2, min_recovery_s=1.5))
    now = drive(guard, 30.0, LEVEL, start=0.0, duration=0.5)
    assert guard.state is State.RECOVERING
    now = drive(guard, LEVEL, LEVEL, start=now, duration=0.5)   # level again, early
    assert guard.state is State.RECOVERING, 'released before min_recovery_s'
    drive(guard, LEVEL, LEVEL, start=now, duration=1.5)
    assert guard.state is State.SAFE


def test_hysteresis_keeps_recovering_between_release_and_trigger():
    """Past min_recovery but still at 20 deg: above release, so keep going."""
    guard = TiltGuard(TiltLimits(debounce_s=0.2, min_recovery_s=1.0,
                                 roll_trigger_deg=25.0, roll_release_deg=15.0))
    now = drive(guard, 30.0, LEVEL, start=0.0, duration=0.5)
    now = drive(guard, 20.0, LEVEL, start=now, duration=2.0)
    assert guard.state is State.RECOVERING, 'released while still above roll_release_deg'
    drive(guard, 10.0, LEVEL, start=now, duration=0.5)
    assert guard.state is State.SAFE


def test_gives_up_after_max_recovery():
    """Reversing is not working. Stop driving blind and hand control back."""
    guard = TiltGuard(TiltLimits(debounce_s=0.2, min_recovery_s=1.0, max_recovery_s=3.0))
    drive(guard, 40.0, LEVEL, start=0.0, duration=6.0)
    assert guard.state is State.SAFE
    assert guard.gave_up
    assert not guard.reversing()


def test_negative_roll_triggers_the_same_as_positive():
    guard = TiltGuard(TiltLimits(debounce_s=0.2))
    drive(guard, -30.0, LEVEL, start=0.0, duration=0.5)
    assert guard.state is State.RECOVERING


def test_a_second_tilt_after_recovery_triggers_again():
    guard = TiltGuard(TiltLimits(debounce_s=0.2, min_recovery_s=0.5))
    now = drive(guard, 30.0, LEVEL, start=0.0, duration=0.5)
    now = drive(guard, LEVEL, LEVEL, start=now, duration=2.0)
    assert guard.state is State.SAFE
    drive(guard, 30.0, LEVEL, start=now, duration=0.5)
    assert guard.state is State.RECOVERING, 'guard did not re-arm after recovering'
