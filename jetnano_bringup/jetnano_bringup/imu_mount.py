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

"""The IMU's reading turned into the robot's own frame (imu/base), for the EKF.

The BNO055's fused heading (NDOF: gyro, accelerometer and magnetometer) is the best
heading on the robot: on the 2026-09-29 hour of floor driving it ended 12.4 deg off
SLAM's with its scale right (0.998), while the raw gyro rate the EKF fused read 3.6 %
short on every turn and the camera's 5.7 %, so the EKF lost 63 deg over 3700 deg of
turning (tools/drive_analysis/heading_drift.py). The chip is mounted upside down and
turned, so its orientation has to be taken out of the mount; doing it here, with the
URDF's own angles (test_imu_mount checks they are equal), leaves the EKF no mount
convention to get wrong. The check against SLAM used exactly this arithmetic
(tools/drive_analysis/imu_mount_check.py: level to 1-3 deg, heading slope +0.998).

Used by bno055_lean on the robot and by the imu_base_relay node in the simulator.
"""

import math

BASE_MOUNT_RPY = (2.9698, 0.0552, 1.5680)       # jetnano.urdf.xacro imu_rpy: base_link -> imu_link


def q_from_rpy(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)          # w x y z


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)


def rotation(q):
    """3x3 rotation of unit quaternion q (w x y z), as nested tuples."""
    w, x, y, z = q
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
            (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
            (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)))


class Mount:
    """The fixed turn from the IMU's frame into base_link."""

    def __init__(self, rpy=BASE_MOUNT_RPY):
        m = q_from_rpy(*rpy)                        # imu_link in base_link
        self.inverse = (m[0], -m[1], -m[2], -m[3])
        self.rot = rotation(m)                      # imu_link vectors -> base_link

    def orientation(self, q_imu):
        """base_link's orientation in the world (w x y z) from the chip's (imu_link's)."""
        return qmul(q_imu, self.inverse)            # world<-imu * imu<-base

    def vector(self, v):
        r = self.rot
        return tuple(r[i][0] * v[0] + r[i][1] * v[1] + r[i][2] * v[2] for i in range(3))

    def to_base(self, src, dst, frame):
        """Fill Imu message dst from Imu message src (frame imu_link), in base frame `frame`."""
        dst.header.stamp = src.header.stamp
        dst.header.frame_id = frame
        o = src.orientation
        dst.orientation.w, dst.orientation.x, dst.orientation.y, dst.orientation.z = \
            self.orientation((o.w, o.x, o.y, o.z))
        dst.orientation_covariance = src.orientation_covariance
        g, a = src.angular_velocity, src.linear_acceleration
        dst.angular_velocity.x, dst.angular_velocity.y, dst.angular_velocity.z = self.vector((g.x, g.y, g.z))
        dst.angular_velocity_covariance = src.angular_velocity_covariance
        dst.linear_acceleration.x, dst.linear_acceleration.y, dst.linear_acceleration.z = \
            self.vector((a.x, a.y, a.z))
        dst.linear_acceleration_covariance = src.linear_acceleration_covariance
        return dst
