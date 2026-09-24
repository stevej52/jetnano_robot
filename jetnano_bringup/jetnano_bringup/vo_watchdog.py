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

"""Two things that must happen when the visual odometry goes quiet.

On 2026-09-24 the camera hit a USB streaming glitch mid-drive; its driver
restarted the streams, but the splitter that feeds cuVSLAM never resumed, so
/vo simply stopped - no process died, so nothing restarted it. The EKF, with
no more pose measurements, coasted at its last velocity: 21 m in 10 s with the
robot standing still, and everything built on the pose (SLAM, nvblox, the
collision guard) went with it. This node closes both holes:

1. ``hold``: while /vo has been silent for longer than ``hold_after_s``, it
   publishes a zero-velocity odometry on ``odom_hold`` (vx, vy only) that
   ekf.yaml fuses as a second source. The filter's velocity decays to zero
   and the pose stays put instead of running away. The gyro still owns the
   yaw rate, so a robot that IS turning is not fought. As soon as /vo is back
   the hold stops.

2. ``restart``: after ``restart_after_s`` of silence (and a grace period
   after start-up) it stops the launch inside the isaac_vo container, whose
   wrapper (cuvslam_vo.sh) then exits and is respawned by odometry.launch.py
   - camera reset, splitter, cuVSLAM, nvblox all fresh. At most once per
   ``restart_min_interval_s``.
"""

import subprocess
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class VoWatchdog(Node):

    def __init__(self):
        super().__init__('vo_watchdog')
        self.declare_parameter('vo_topic', 'vo')
        self.declare_parameter('hold_topic', 'odom_hold')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('hold_after_s', 0.5)
        self.declare_parameter('restart_after_s', 5.0)
        self.declare_parameter('startup_grace_s', 90.0)
        self.declare_parameter('restart_min_interval_s', 60.0)
        self.declare_parameter('container', 'isaac_vo')
        # Matches both container launch files.
        self.declare_parameter('launch_pattern', 'cuvslam_.*\\.launch\\.py')

        self.hold_after = float(self.get_parameter('hold_after_s').value)
        self.restart_after = float(self.get_parameter('restart_after_s').value)
        self.grace = float(self.get_parameter('startup_grace_s').value)
        self.min_interval = float(self.get_parameter('restart_min_interval_s').value)
        self.container = str(self.get_parameter('container').value)
        self.pattern = str(self.get_parameter('launch_pattern').value)
        self.base_frame = str(self.get_parameter('base_frame').value)

        self.started = time.monotonic()
        self.last_vo = None
        self.last_restart = 0.0
        self.holding = False

        self.hold_pub = self.create_publisher(Odometry, self.get_parameter('hold_topic').value, 10)
        self.create_subscription(Odometry, self.get_parameter('vo_topic').value, self._on_vo, 10)
        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f'holding the EKF after {self.hold_after:.1f} s without /vo, restarting the '
            f'container launch after {self.restart_after:.0f} s (grace {self.grace:.0f} s)')

    def _on_vo(self, _msg):
        self.last_vo = time.monotonic()
        if self.holding:
            self.holding = False
            self.get_logger().info('/vo is back; releasing the hold')

    def _tick(self):
        now = time.monotonic()
        since_start = now - self.started
        silent = (now - self.last_vo) if self.last_vo is not None else since_start

        if silent > self.hold_after and (self.last_vo is not None or since_start > self.grace):
            if not self.holding:
                self.holding = True
                self.get_logger().warning(f'/vo silent for {silent:.1f} s: holding the EKF still')
            msg = Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'odom'
            msg.child_frame_id = self.base_frame
            msg.twist.covariance[0] = 1e-3     # vx
            msg.twist.covariance[7] = 1e-3     # vy
            self.hold_pub.publish(msg)

        if (silent > self.restart_after and since_start > self.grace
                and now - self.last_restart > self.min_interval):
            self.last_restart = now
            self.get_logger().error(
                f'/vo silent for {silent:.0f} s: stopping the launch in {self.container} so it is respawned')
            try:
                subprocess.run(['docker', 'exec', self.container, 'pkill', '-INT', '-f', self.pattern],
                               timeout=10, check=False)
            except Exception as exc:  # noqa: B902 - docker missing or hung: say so and keep holding
                self.get_logger().error(f'could not reach the container: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = VoWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
