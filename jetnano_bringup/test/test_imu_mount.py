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

"""imu/base: the BNO055's reading turned into base_link, which the EKF's heading rests on."""

import math
import os
import re
from types import SimpleNamespace

from jetnano_bringup.imu_mount import BASE_MOUNT_RPY, Mount, q_from_rpy, qmul
import pytest

URDF = os.path.join(os.path.dirname(__file__), '..', '..', 'jetnano_description', 'urdf', 'jetnano.urdf.xacro')


def rpy(q):
    w, x, y, z = q
    return (math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
            math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))),
            math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def angle(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


@pytest.mark.skipif(not os.path.exists(URDF), reason='no source tree here')
def test_mount_is_the_urdf_one():
    text = open(URDF).read()
    value = re.search(r'name="imu_rpy"\s+value="([^"]+)"', text).group(1)
    assert tuple(float(v) for v in value.split()) == BASE_MOUNT_RPY


@pytest.mark.parametrize('heading_deg', [-170, -90, -30, 0, 45, 90, 135, 180])
def test_level_robot_reads_level_with_its_heading(heading_deg):
    mount = Mount()
    m = q_from_rpy(*BASE_MOUNT_RPY)
    robot = q_from_rpy(0.0, 0.0, math.radians(heading_deg))
    chip = qmul(robot, m)                       # what the upside-down chip reports
    r, p, y = rpy(mount.orientation(chip))
    assert abs(r) < 1e-9 and abs(p) < 1e-9
    assert angle(y, math.radians(heading_deg)) < 1e-9


def test_tilted_robot_keeps_its_heading():
    mount = Mount()
    robot = q_from_rpy(math.radians(4), math.radians(-6), math.radians(70))
    r, p, y = rpy(mount.orientation(qmul(robot, q_from_rpy(*BASE_MOUNT_RPY))))
    assert angle(r, math.radians(4)) < 1e-9 and angle(p, math.radians(-6)) < 1e-9
    assert angle(y, math.radians(70)) < 1e-9


def test_a_pure_turn_is_a_pure_yaw_rate():
    mount = Mount()
    # the chip sees the robot's yaw rate on its own (upside-down) axes
    rot = mount.rot
    w = 1.3                                    # rad/s about base z
    chip = tuple(rot[0][i] * 0 + rot[1][i] * 0 + rot[2][i] * w for i in range(3))   # R^T (0, 0, w)
    x, y, z = mount.vector(chip)
    assert abs(x) < 1e-12 and abs(y) < 1e-12 and abs(z - w) < 1e-12
    # upside down: the chip's own z axis reads it with the opposite sign (as recorded)
    assert chip[2] < 0


def test_to_base_fills_a_message():
    mount = Mount()
    v3 = lambda x, y, z: SimpleNamespace(x=x, y=y, z=z)   # noqa: E731
    chip_q = qmul(q_from_rpy(0, 0, 0.5), q_from_rpy(*BASE_MOUNT_RPY))
    src = SimpleNamespace(
        header=SimpleNamespace(stamp='t', frame_id='imu_link'),
        orientation=SimpleNamespace(w=chip_q[0], x=chip_q[1], y=chip_q[2], z=chip_q[3]),
        orientation_covariance=[0.0159] * 9, angular_velocity=v3(0.0, 0.0, -1.0),
        angular_velocity_covariance=[0.04] * 9, linear_acceleration=v3(0.0, 0.0, 0.0),
        linear_acceleration_covariance=[0.017] * 9)
    dst = SimpleNamespace(header=SimpleNamespace(), orientation=SimpleNamespace(),
                          angular_velocity=SimpleNamespace(), linear_acceleration=SimpleNamespace())
    out = mount.to_base(src, dst, 'base_link')
    assert out.header.frame_id == 'base_link' and out.header.stamp == 't'
    q = (out.orientation.w, out.orientation.x, out.orientation.y, out.orientation.z)
    assert angle(rpy(q)[2], 0.5) < 1e-9
    assert out.angular_velocity.z > 0.95          # the chip's -z is the robot's +z (~10 deg tilt)
