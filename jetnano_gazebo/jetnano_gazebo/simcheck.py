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
Refuse to measure the simulator unless the measurement can be trusted.

This exists because of three wasted runs. Every test on this machine uses the
same ROS_DOMAIN_ID, so a ``ros2 launch`` or ``ros2 topic pub`` left over from
an earlier test keeps driving the robot during the next one. Two Gazebo
servers published ground truth at once; a stale cmd_vel publisher moved the
robot before the "before" reading was taken. The robot appeared to be at
yaw 146 degrees before anything was commanded, and a right turn appeared to
go left.

The dangerous part was not that the numbers were wrong. It was that they
looked *plausible* - a turn radius, a heading, all in believable ranges - so
they were nearly written down as findings. A measurement that cannot fail
loudly is worse than no measurement.

The rule these functions enforce: assert the preconditions of a measurement
BEFORE taking it, and make a violated precondition raise rather than warn.

    wait_for_clean_slate()          nothing from a previous run is alive
    assert_single_publisher(topic)  exactly one thing is publishing it
    assert_no_publisher(topic)      nothing is commanding the robot

Waiting for a test command to return is NOT the same as waiting for its
processes to exit: a launch with a 150 s timeout outlives a 50 s test.
"""

from __future__ import annotations

import subprocess
import time

# Anything that can drive the robot or publish its state.
SIM_PROCESS_NAMES = ('gz', 'ruby', 'parameter_br', 'robot_state', 'spawner',
                     'sim_drive', 'rgbd_odometry', 'ekf_node', 'async_slam')


class DirtyStateError(RuntimeError):
    """Raised when the simulator is not in a state worth measuring."""


def _running_sim_processes() -> list[str]:
    """Names of processes that could interfere with a measurement."""
    out = subprocess.run(['ps', '-eo', 'comm'], capture_output=True, text=True).stdout
    found = []
    for line in out.splitlines()[1:]:
        name = line.strip()
        if any(name.startswith(prefix) for prefix in SIM_PROCESS_NAMES):
            found.append(name)
    return found


def wait_for_clean_slate(timeout: float = 120.0) -> None:
    """
    Block until nothing from a previous run is alive.

    Raises rather than continuing, because continuing is exactly the mistake
    this module exists to prevent.
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        leftovers = _running_sim_processes()
        if not leftovers:
            return
        time.sleep(2.0)
    raise DirtyStateError(
        f'still running after {timeout:g}s: {sorted(set(_running_sim_processes()))}. '
        'A previous run is still alive and would corrupt this measurement.')


def publisher_count(topic: str) -> int:
    """How many publishers a topic currently has."""
    out = subprocess.run(['ros2', 'topic', 'info', topic],
                         capture_output=True, text=True, timeout=20).stdout
    for line in out.splitlines():
        if 'Publisher count:' in line:
            return int(line.split(':')[1].strip())
    return 0


def assert_single_publisher(topic: str) -> None:
    """
    Require exactly one publisher on a topic before trusting it.

    Two Gazebo servers on one domain both publish ground truth, and the
    readings interleave into a pose that never existed. This is the specific
    check that would have caught all three bad runs.
    """
    count = publisher_count(topic)
    if count != 1:
        raise DirtyStateError(
            f'{topic} has {count} publishers, expected exactly 1. '
            + ('Nothing is publishing it.' if count == 0
               else 'More than one run is alive; the readings would interleave.'))


def assert_no_publisher(topic: str) -> None:
    """Require that nothing is commanding the robot before a baseline."""
    count = publisher_count(topic)
    if count != 0:
        raise DirtyStateError(
            f'{topic} has {count} publishers; something is still commanding '
            'the robot and a baseline taken now would not be a baseline.')


def preflight(measured_topics: list[str], command_topics: list[str] | None = None) -> None:
    """
    Run every check that must hold before a measurement is worth taking.

    Call this immediately before sampling, not at the start of a script: the
    point is to catch a leftover that appeared while the simulator was
    starting up.
    """
    for topic in measured_topics:
        assert_single_publisher(topic)
    for topic in command_topics or []:
        assert_no_publisher(topic)
