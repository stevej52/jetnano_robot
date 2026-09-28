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

"""The lidar odometry relay's outlier gates (LoGate): what passes, what is dropped, and
the episode memory that drops the samples after a failure too."""

import pytest

pytest.importorskip('rclpy')
from jetnano_bringup.lidar_odom_relay import LoGate  # noqa: E402

DT = 0.128


def run(gate, samples, t0=0.0):
    out = []
    for i, (vx, vy) in enumerate(samples):
        out.append(gate.check(t0 + i * DT, vx, vy, DT))
    return out


def test_smooth_driving_passes():
    gate = LoGate()
    speeds = [0.1 * i for i in range(8)] + [0.7] * 10          # speeding up at 0.8 m/s^2
    assert all(ok for ok, _ in run(gate, [(v, 0.02) for v in speeds]))


def test_sideways_motion_is_dropped_with_the_next_four():
    gate = LoGate()
    samples = [(0.5, 0.0)] * 3 + [(0.5, 0.4)] + [(0.5, 0.0)] * 6
    oks = [ok for ok, _ in run(gate, samples)]
    # the sideways sample fails, and so does the snap back (0.4 m/s sideways to none in
    # 0.128 s is 3.1 m/s^2); then four clean ones are still dropped before it is trusted
    assert oks == [True] * 3 + [False] * 6 + [True]


def test_an_impossible_speed_change_is_dropped():
    gate = LoGate()
    samples = [(0.3, 0.0)] * 3 + [(0.9, 0.0)] + [(0.9, 0.0)] * 6   # +0.6 m/s in 0.128 s = 4.7 m/s^2
    oks = [ok for ok, _ in run(gate, samples)]
    assert oks[3] is False and oks[-1] is True


def test_the_end_of_an_episode_is_reported():
    gate = LoGate()
    reports = [r for _, r in run(gate, [(0.5, 0.0)] * 2 + [(0.5, 0.6)] * 10 + [(0.5, 0.0)] * 6)]
    ended = [r for r in reports if r is not None]
    assert len(ended) == 1
    seconds, samples = ended[0]
    # ten sideways failures, the snap back, and the four to recover
    assert samples == 10 + 1 + 4
    assert seconds == pytest.approx(15 * DT)


def test_a_gap_forgets_the_previous_speed():
    gate = LoGate()
    run(gate, [(0.2, 0.0)] * 3)
    gate.reset()
    # after a gap the first sample has nothing to be compared with, so no speed change
    assert gate.check(10.0, 0.8, 0.0, DT)[0]


def test_an_impossible_jump_counts_as_part_of_an_episode():
    gate = LoGate()
    run(gate, [(0.2, 0.0)] * 3)
    gate.fail(1.0)
    oks = [ok for ok, _ in run(gate, [(0.2, 0.0)] * 5, t0=1.1)]
    assert oks == [False] * 4 + [True]
