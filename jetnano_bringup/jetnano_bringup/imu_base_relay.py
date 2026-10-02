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

"""imu/data -> imu/base, for the simulator.

On the robot bno055_lean publishes imu/base itself; Gazebo's IMU sits on the same URDF
mount (upside down, turned) and publishes only imu/data, so this does the same turn
(jetnano_bringup.imu_mount) for the EKF there.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from jetnano_bringup.imu_mount import BASE_MOUNT_RPY, Mount


class ImuBaseRelay(Node):

    def __init__(self):
        super().__init__('imu_base_relay')
        self.frame = self.declare_parameter('base_frame', 'base_link').value
        self.mount = Mount(tuple(float(a) for a in
                                 self.declare_parameter('base_mount_rpy', list(BASE_MOUNT_RPY)).value))
        self.pub = self.create_publisher(Imu, 'imu/base', 10)
        self.create_subscription(Imu, 'imu/data', self.on_imu, qos_profile_sensor_data)

    def on_imu(self, msg):
        self.pub.publish(self.mount.to_base(msg, Imu(), self.frame))


def main(args=None):
    rclpy.init(args=args)
    node = ImuBaseRelay()
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
