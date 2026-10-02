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

Outlier gates (LoGate), tuned on the five recorded drives of 2026-09-27 against
the camera's odometry: while she moved, 25-32 % of the samples were more than
0.2 m/s off (MOLA claimed up to 2.8 m/s and 6.6 rad/s; her top speed is 0.87),
and the bad ones came in episodes - 615 of 968 in runs of four or more, the
longest 99 samples, MOLA having lost track for 12 s. So a sample that moves her
sideways (a car cannot: |vy| > 0.08 + 0.25 |vx|) or changes speed faster than
2.5 m/s^2 is dropped, and so is everything after it until four in a row pass.
On the recordings: 86 % of the bad samples caught, 6 % of the good lost, the
worst 1 % of accepted errors while moving 1.53 -> 0.36 m/s. (Checking the yaw
rate against the IMU caught no more once these were in.) Timing was not the
problem: shifting the stamps +-0.3 s changed nothing.

The covariance follows what got through: speed +-0.03 m/s standing, +-0.10
moving (the p68 of the accepted samples was 0.08-0.11 at every speed), yaw rate
+-0.01 rad/s holding a heading, +-0.08 turning.

The pose it passes on is MOLA's, re-anchored after every dropped stretch, gap or
restart so that it carries on from where the last published one left off: the
safety monitor measures how far she moved from it, and a jump across a lost
episode would look like sudden motion ("camera blind").
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


class LoGate:
    """The outlier gates with their episode memory; no ROS, so it can be tested."""

    def __init__(self, side_base=0.08, side_per_forward=0.25, max_accel=2.5, recover=4):
        self.side_base, self.side_per_forward = side_base, side_per_forward
        self.max_accel, self.recover = max_accel, recover
        self.passed = recover          # trusted from the start
        self.prev_v = None
        self.lost_since = None         # time of the first dropped sample of this episode
        self.lost_count = 0

    def reset(self):
        """A gap or restart: the next sample has no predecessor to compare with."""
        self.prev_v = None

    def fail(self, t):
        """A sample refused elsewhere (an impossible jump): part of the episode."""
        self.passed = 0
        self.prev_v = None
        if self.lost_since is None:
            self.lost_since = t
        self.lost_count += 1

    def check(self, t, vx, vy, dt):
        """(publish?, episode_report): episode_report is (seconds, samples) when an
        episode of dropped samples has just ended, else None."""
        accel = 0.0 if self.prev_v is None else \
            math.hypot(vx - self.prev_v[0], vy - self.prev_v[1]) / max(dt, 1e-3)
        self.prev_v = (vx, vy)
        fails = abs(vy) > self.side_base + self.side_per_forward * abs(vx) or accel > self.max_accel
        if fails:
            self.passed = 0
        else:
            self.passed += 1
        if fails or self.passed <= self.recover:
            if self.lost_since is None:
                self.lost_since = t
            self.lost_count += 1
            return False, None
        report = None
        if self.lost_since is not None:
            report = (t - self.lost_since, self.lost_count)
            self.lost_since, self.lost_count = None, 0
        return True, report


class LidarOdomRelay(Node):

    def __init__(self):
        super().__init__('lidar_odom_relay')
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        self.max_speed = float(self.declare_parameter('max_speed', 3.0).value)
        self.max_yaw_rate = float(self.declare_parameter('max_yaw_rate', 6.0).value)
        self.max_gap = float(self.declare_parameter('max_gap_s', 0.5).value)
        # standard deviations reported to the EKF, still -> moving (see the docstring)
        self.sd_v = (float(self.declare_parameter('velocity_sd_still', 0.03).value),
                     float(self.declare_parameter('velocity_sd_moving', 0.10).value))
        self.sd_w = (float(self.declare_parameter('yaw_rate_sd_still', 0.01).value),
                     float(self.declare_parameter('yaw_rate_sd_turning', 0.08).value))
        self.gate = LoGate(
            float(self.declare_parameter('max_sideways', 0.08).value),
            float(self.declare_parameter('max_sideways_per_forward', 0.25).value),
            float(self.declare_parameter('max_accel', 2.5).value),
            int(self.declare_parameter('recover_samples', 4).value))
        self.prev = None        # (t, x, y, yaw) of the last pose
        self.anchor = None      # (ox, oy, oyaw): MOLA pose -> published pose
        self.last_out = None    # (x, y, yaw) of the last published pose
        self.reanchor = True
        self.skip = 0           # poses to take as a fresh start (after /initialpose)
        self.published = 0
        self.dropped = 0
        self.pub = self.create_publisher(Odometry, 'lidar_odom', 10)
        self.create_subscription(Odometry, 'lidar_odometry/pose', self.on_pose, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_initialpose, 10)
        self.get_logger().info(
            f'lidar_odometry/pose -> lidar_odom (speeds for the EKF); dropping jumps over '
            f'{self.max_speed:.1f} m/s or {self.max_yaw_rate:.1f} rad/s, sideways motion, '
            f'speed changes over {self.gate.max_accel:.1f} m/s2, and {self.gate.recover} samples after each')

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
            self.gate.reset()
            self.reanchor = True
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
            self.gate.fail(cur[0])           # part of an episode like any other failure
            self.reanchor = True
            self.get_logger().warn(
                f'lidar odometry jumped {math.hypot(dx, dy):.2f} m / {math.degrees(dyaw):.0f} deg '
                f'in {dt:.2f} s - not real, dropped ({self.dropped} so far)',
                throttle_duration_sec=5.0)
            return
        ok, report = self.gate.check(cur[0], vx, vy, dt)
        if not ok:
            self.dropped += 1
            self.reanchor = True
            return
        if report is not None and report[0] >= 2.0:
            self.get_logger().info(
                f'lidar odometry back after {report[0]:.1f} s off track ({report[1]} samples dropped)')
        out = Odometry()
        out.header.stamp = Time(nanoseconds=int((prev[0] + dt / 2.0) * 1e9)).to_msg()
        out.header.frame_id = m.header.frame_id or 'lidar_odom'
        out.child_frame_id = self.base_frame
        # MOLA's pose of base_link (base_footprint is straight below it, so in 2D the
        # same), re-anchored so it carries on from the last published pose
        if self.reanchor or self.anchor is None:
            if self.last_out is None:
                self.anchor = (0.0, 0.0, 0.0)
            else:
                ay = _wrap(self.last_out[2] - cur[3])
                ca, sa = math.cos(ay), math.sin(ay)
                self.anchor = (self.last_out[0] - (ca * cur[1] - sa * cur[2]),
                               self.last_out[1] - (sa * cur[1] + ca * cur[2]), ay)
            self.reanchor = False
        ox, oy, oyaw = self.anchor
        co, so = math.cos(oyaw), math.sin(oyaw)
        px, py = ox + co * cur[1] - so * cur[2], oy + so * cur[1] + co * cur[2]
        pyaw = _wrap(oyaw + cur[3])
        self.last_out = (px, py, pyaw)
        out.pose.covariance = m.pose.covariance
        out.pose.pose.position.x, out.pose.pose.position.y = px, py
        out.pose.pose.position.z = p.position.z
        out.pose.pose.orientation.z, out.pose.pose.orientation.w = math.sin(pyaw / 2), math.cos(pyaw / 2)
        out.twist.twist.linear.x = vx
        out.twist.twist.linear.y = vy
        out.twist.twist.angular.z = wz
        moving = min(1.0, math.hypot(vx, vy) / 0.1)
        turning = min(1.0, abs(wz) / 0.3)
        sd_v = self.sd_v[0] + (self.sd_v[1] - self.sd_v[0]) * moving
        sd_w = self.sd_w[0] + (self.sd_w[1] - self.sd_w[0]) * turning
        cov = [0.0] * 36
        cov[0] = cov[7] = sd_v * sd_v
        cov[14] = cov[21] = cov[28] = BIG
        cov[35] = sd_w * sd_w
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
        try:
            node.destroy_node()
        except (Exception, KeyboardInterrupt):  # noqa: BLE001 - the context is gone, or a second SIGINT, after an external shutdown
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
