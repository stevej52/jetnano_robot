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

"""A pretend robot in a pretend room, so the stack can be tested with no hardware.

This is NOT a physics simulator. It exists so that SLAM, Nav2, the mapping
modes and the TF tree can be exercised on a desk while the real robot is in
storage. It publishes the three things the rest of the stack needs:

  /scan                sensor_msgs/LaserScan   a rectangular room with a pillar
  /odometry/filtered   nav_msgs/Odometry       where the robot thinks it is
  odom -> base_footprint  TF                    the transform the EKF normally owns

It drives itself in a slow circle by default so that slam_toolbox sees motion
and actually accumulates scans - it ignores scans until the robot has moved
minimum_travel_distance. Set drive:=false to park it.

Run the EKF and this at the same time and they will fight over odom ->
base_footprint. sim.launch.py starts the right combination; do not add this to
robot.launch.py.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class FakeSensors(Node):

    def __init__(self):
        super().__init__('fake_sensors')

        self.declare_parameter('room_width', 6.0)
        self.declare_parameter('room_height', 4.0)
        self.declare_parameter('scan_rate', 10.0)
        self.declare_parameter('scan_points', 360)
        self.declare_parameter('range_max', 10.0)
        self.declare_parameter('drive', True)
        self.declare_parameter('speed', 0.25)
        self.declare_parameter('turn_radius', 1.2)

        self.width = float(self.get_parameter('room_width').value)
        self.height = float(self.get_parameter('room_height').value)
        self.points = int(self.get_parameter('scan_points').value)
        self.range_max = float(self.get_parameter('range_max').value)
        self.drive = bool(self.get_parameter('drive').value)
        self.speed = float(self.get_parameter('speed').value)
        self.radius = float(self.get_parameter('turn_radius').value)

        rate = float(self.get_parameter('scan_rate').value)

        # Pose in the odom frame.
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', best_effort)
        self.odom_pub = self.create_publisher(Odometry, 'odometry/filtered', 10)
        self.tf = TransformBroadcaster(self)

        self.dt = 1.0 / rate
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f'fake room {self.width:g} x {self.height:g} m, '
            f'{"driving" if self.drive else "parked"}')

    # A rectangular room centred on the origin, with a square pillar in it so
    # the map has an interior feature to close loops against.
    def _range_at(self, world_angle: float) -> float:
        best = self.range_max
        cos_a = math.cos(world_angle)
        sin_a = math.sin(world_angle)

        half_w = self.width / 2.0
        half_h = self.height / 2.0

        # Distance to each of the four walls along this ray.
        for bound, comp, origin in ((half_w, cos_a, self.x), (-half_w, cos_a, self.x),
                                    (half_h, sin_a, self.y), (-half_h, sin_a, self.y)):
            if abs(comp) < 1e-9:
                continue
            t = (bound - origin) / comp
            if t <= 0.0:
                continue
            # Check the hit is actually on the wall segment.
            hx = self.x + t * cos_a
            hy = self.y + t * sin_a
            if -half_w - 1e-6 <= hx <= half_w + 1e-6 and -half_h - 1e-6 <= hy <= half_h + 1e-6:
                best = min(best, t)

        # A 0.4 m square pillar at (1.5, 0.8).
        best = min(best, self._box_hit(cos_a, sin_a, 1.5, 0.8, 0.2, best))
        return best

    def _box_hit(self, cos_a, sin_a, cx, cy, half, current) -> float:
        # Slab method, axis aligned.
        lo, hi = 0.0, current
        for comp, origin, centre in ((cos_a, self.x, cx), (sin_a, self.y, cy)):
            if abs(comp) < 1e-9:
                if not (centre - half <= origin <= centre + half):
                    return current
                continue
            t1 = (centre - half - origin) / comp
            t2 = (centre + half - origin) / comp
            if t1 > t2:
                t1, t2 = t2, t1
            lo = max(lo, t1)
            hi = min(hi, t2)
            if lo > hi:
                return current
        return lo if lo > 0.0 else current

    def _tick(self) -> None:
        now = self.get_clock().now()

        if self.drive:
            # A gentle arc: enough motion for slam_toolbox to take scans.
            self.yaw += (self.speed / self.radius) * self.dt
            self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
            self.x += self.speed * math.cos(self.yaw) * self.dt
            self.y += self.speed * math.sin(self.yaw) * self.dt

        stamp = now.to_msg()

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation = yaw_to_quaternion(self.yaw)
        self.tf.sendTransform(transform)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = yaw_to_quaternion(self.yaw)
        odom.twist.twist.linear.x = self.speed if self.drive else 0.0
        odom.twist.twist.angular.z = (self.speed / self.radius) if self.drive else 0.0
        self.odom_pub.publish(odom)

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = 'lidar_link'      # matches the URDF
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = (2.0 * math.pi) / self.points
        scan.time_increment = 0.0
        scan.scan_time = self.dt
        scan.range_min = 0.15
        scan.range_max = self.range_max
        scan.ranges = [
            self._range_at(self.yaw + scan.angle_min + i * scan.angle_increment)
            for i in range(self.points)
        ]
        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = FakeSensors()
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
