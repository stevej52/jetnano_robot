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

"""MOLA's lidar odometry, turned into speeds the EKF can fuse next to the camera's.

    ros2 run jetnano_bringup lidar_odom_relay      (lidar_odometry.launch.py starts it)

MOLA publishes where it thinks the robot is, in its own frame (lidar_odom), on
lidar_odometry/pose. The EKF must not take that as a position: MOLA starts from
zero whenever it restarts, and slam_toolbox, not the odometry, owns the map. The
camera's odometry is fused the same way for the same reason (ekf.yaml odom0).

So this node differentiates consecutive poses into forward, sideways and turning
speed in the robot's own frame, stamps each at the middle of its interval, and
publishes them on lidar_odom with a covariance chosen here (MOLA's own is not
meant for this). The EKF reads the twist (ekf.yaml odom2). The message also
carries MOLA's pose unchanged, in lidar_odom, which the safety monitor uses to
see how far she has moved.

It refuses what cannot be real: a restarted or lost MOLA can jump metres in one
scan, and anything faster than max_speed or max_yaw_rate is dropped. After a gap
longer than max_gap_s, or time running backwards, it starts afresh instead of
spreading the jump over the gap. MOLA also re-localizes itself to any pose set
with RViz's "2D Pose Estimate" (/initialpose, hard-wired in MOLA); that jump is
not motion either, so the next two poses after one only start afresh.
"""

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time

BIG = 1.0e6     # variance for what this message does not measure (z, roll, pitch)


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class LidarOdomRelay(Node):

    def __init__(self):
        super().__init__('lidar_odom_relay')
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        self.max_speed = float(self.declare_parameter('max_speed', 3.0).value)
        self.max_yaw_rate = float(self.declare_parameter('max_yaw_rate', 6.0).value)
        self.max_gap = float(self.declare_parameter('max_gap_s', 0.5).value)
        self.var_v = float(self.declare_parameter('velocity_sd', 0.05).value) ** 2
        self.var_w = float(self.declare_parameter('yaw_rate_sd', 0.05).value) ** 2
        self.prev = None        # (t, x, y, yaw) of the last pose
        self.skip = 0           # poses to take as a fresh start (after /initialpose)
        self.published = 0
        self.dropped = 0
        self.pub = self.create_publisher(Odometry, 'lidar_odom', 10)
        self.create_subscription(Odometry, 'lidar_odometry/pose', self.on_pose, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_initialpose, 10)
        self.get_logger().info(
            f'lidar_odometry/pose -> lidar_odom (speeds for the EKF); dropping jumps over '
            f'{self.max_speed:.1f} m/s or {self.max_yaw_rate:.1f} rad/s')

    def on_initialpose(self, _msg):
        self.skip = 2
        self.get_logger().info('2D Pose Estimate: MOLA re-localizes; not counting that jump as motion')

    def on_pose(self, m):
        p = m.pose.pose
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        cur = (t, p.position.x, p.position.y, _yaw(p.orientation))
        prev, self.prev = self.prev, cur
        if self.skip > 0:
            self.skip -= 1
            return
        if prev is None:
            return
        dt = cur[0] - prev[0]
        if dt <= 0.0 or dt > self.max_gap:
            return      # a restart, a replayed bag, or a gap: start afresh from this pose
        dyaw = _wrap(cur[3] - prev[3])
        heading = prev[3] + dyaw / 2.0      # the robot's heading half-way through the interval
        dx, dy = cur[1] - prev[1], cur[2] - prev[2]
        c, s = math.cos(heading), math.sin(heading)
        vx = (c * dx + s * dy) / dt
        vy = (-s * dx + c * dy) / dt
        wz = dyaw / dt
        if math.hypot(vx, vy) > self.max_speed or abs(wz) > self.max_yaw_rate:
            self.dropped += 1
            self.get_logger().warn(
                f'lidar odometry jumped {math.hypot(dx, dy):.2f} m / {math.degrees(dyaw):.0f} deg '
                f'in {dt:.2f} s - not real, dropped ({self.dropped} so far)',
                throttle_duration_sec=5.0)
            return
        out = Odometry()
        out.header.stamp = Time(nanoseconds=int((prev[0] + dt / 2.0) * 1e9)).to_msg()
        out.header.frame_id = m.header.frame_id or 'lidar_odom'
        out.child_frame_id = self.base_frame
        # MOLA's pose of base_link; base_footprint is straight below it, so in 2D the same
        out.pose = m.pose
        out.twist.twist.linear.x = vx
        out.twist.twist.linear.y = vy
        out.twist.twist.angular.z = wz
        cov = [0.0] * 36
        cov[0] = cov[7] = self.var_v
        cov[14] = cov[21] = cov[28] = BIG
        cov[35] = self.var_w
        out.twist.covariance = cov
        self.pub.publish(out)
        self.published += 1
        if self.published == 1:
            self.get_logger().info('lidar odometry is flowing')


def main(args=None):
    rclpy.init(args=args)
    node = LidarOdomRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
