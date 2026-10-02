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

"""/ground_truth/odom -> /vo, for the simulator's test drives.

The simulated camera odometry (rgbd_odometry) is the weak link of the nightly drive on
H2-Host (2026-09-30: it ran the position 19 m off in one run and inverted her motion in
another). full_stack.launch.py ground_truth_odometry:=true feeds the EKF the simulator's
exact odometry on the camera's topic instead, so the drive tests navigation, not RTAB-Map.
Only the velocities are used (ekf.yaml odom0: vx, vy), so the child frame is set to the
robot's base frame, where the EKF wants them. Never on the real robot: it has no such topic.
"""

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class GroundTruthVo(Node):

    def __init__(self):
        super().__init__('ground_truth_vo')
        self.child = self.declare_parameter('base_frame', 'base_footprint').value
        self.pub = self.create_publisher(Odometry, 'vo', 10)
        self.create_subscription(Odometry, 'ground_truth/odom', self.on_odom, 10)
        self.get_logger().info('the simulator\'s exact odometry goes out as /vo (velocities for the EKF)')

    def on_odom(self, msg):
        msg.header.frame_id = 'vo'
        msg.child_frame_id = self.child
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthVo()
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
