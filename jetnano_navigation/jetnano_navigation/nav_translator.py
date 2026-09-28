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

"""Nav2 speaks metres per second and radians per second; Rosie's driver speaks throttle and
steering. This turns one into the other (navigation.launch.py runs it).

    cmd_vel_nav_mps (Nav2, after its collision monitor)  ->  cmd_vel_nav (twist_mux)

Speed: linear.x in m/s becomes a throttle fraction through the speeds measured on the
hard floor on 2026-09-27 (page slider and Steve's drive; see SPEED_FWD / SPEED_REV), plus
a slow correction from the EKF's measured speed, because the same throttle is slower on a
tired battery. Anything slower than she can crawl (~0.18 m/s) is sent as her crawl.

Steering: the path's curvature is angular.z / linear.x; Steve's steering circles the same
evening gave curvature = GAIN x steering command, 1.05 turning left and 1.30 turning right
(tightest radius ~0.33-0.39 m at full lock, 2.4). Sent as that steering command, so her
wheels follow the curve Nav2 planned whatever the speed (the driver's angular.z is a
steering position, not a turn rate).

Silence in, silence out: when Nav2 stops publishing, so does this, and twist_mux's 0.5 s
timeout hands the wheels back.
"""

import bisect
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

# (m/s, throttle) measured on the hard floor, 2026-09-27
SPEED_FWD = [(0.18, 0.08), (0.23, 0.12), (0.29, 0.15), (0.39, 0.20), (0.51, 0.25), (0.65, 0.375), (0.87, 0.50)]
SPEED_REV = [(0.21, 0.125), (0.33, 0.25), (0.45, 0.375), (0.52, 0.50)]


def throttle_for(speed, table):
    """Interpolated throttle for |speed|; below the table's first row, its first throttle (her crawl)."""
    v = abs(speed)
    xs = [s for s, _ in table]
    if v <= xs[0]:
        return table[0][1]
    if v >= xs[-1]:
        return table[-1][1]
    i = bisect.bisect_left(xs, v)
    (v0, t0), (v1, t1) = table[i - 1], table[i]
    return t0 + (t1 - t0) * (v - v0) / (v1 - v0)


class NavTranslator(Node):

    def __init__(self):
        super().__init__('nav_translator')
        self.gain_left = float(self.declare_parameter('curvature_per_steer_left', 1.05).value)
        self.gain_right = float(self.declare_parameter('curvature_per_steer_right', 1.30).value)
        self.max_steer = float(self.declare_parameter('max_steer', 2.4).value)
        self.max_throttle = float(self.declare_parameter('max_throttle', 0.30).value)
        self.stop_below = float(self.declare_parameter('stop_below_mps', 0.03).value)
        self.ki = float(self.declare_parameter('speed_correction_gain', 0.15).value)   # throttle per (m/s * s)
        self.trim_limit = float(self.declare_parameter('speed_correction_limit', 0.08).value)
        # the simulator's drive model takes m/s and rad/s as they are
        self.passthrough = bool(self.declare_parameter('passthrough', False).value)
        # steering units per second: near a goal Nav2's tiny corrections become sharp curves at
        # crawl speed, and without this the wheels flicked to full lock in the last 10 cm
        self.max_steer_rate = float(self.declare_parameter('max_steer_rate', 4.0).value)
        self.steer = 0.0
        self.trim = 0.0
        self.measured = 0.0
        self.last_t = None
        self.pub = self.create_publisher(Twist, 'cmd_vel_nav', 10)
        self.create_subscription(Twist, 'cmd_vel_nav_mps', self.on_cmd, 10)
        self.create_subscription(Odometry, 'odometry/filtered',
                                 lambda m: setattr(self, 'measured', m.twist.twist.linear.x), 10)
        self.get_logger().info(
            f'Nav2 m/s -> throttle (max {self.max_throttle:.2f}), curvature -> steering '
            f'(left x{self.gain_left:.2f}, right x{self.gain_right:.2f}, lock {self.max_steer:.1f})')

    def on_cmd(self, m):
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = min(0.2, now - self.last_t) if self.last_t is not None else 0.0
        self.last_t = now
        if self.passthrough:
            self.pub.publish(m)
            return
        v, w = m.linear.x, m.angular.z
        out = Twist()
        if abs(v) < self.stop_below:
            self.trim = 0.0
            self.steer = 0.0
            self.pub.publish(out)          # Nav2 wants her stopped: all zeros
            return
        # speed: the measured table, then a slow nudge toward the speed Nav2 asked for
        base = throttle_for(v, SPEED_FWD if v > 0 else SPEED_REV)
        err = abs(v) - abs(self.measured) if (self.measured > 0) == (v > 0) else abs(v)
        self.trim = max(-self.trim_limit, min(self.trim_limit, self.trim + self.ki * err * dt))
        throttle = max(0.0, min(self.max_throttle, base + self.trim))
        out.linear.x = math.copysign(throttle, v)
        # steering: the planned curve, whatever the speed
        curvature = w / v
        steer = curvature / (self.gain_left if curvature > 0 else self.gain_right)
        steer = max(-self.max_steer, min(self.max_steer, steer))
        step = self.max_steer_rate * (dt if dt > 0 else 0.05)
        self.steer = max(self.steer - step, min(self.steer + step, steer))
        out.angular.z = self.steer
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = NavTranslator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
