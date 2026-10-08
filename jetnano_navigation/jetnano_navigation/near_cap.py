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

"""Slow down BEFORE the collision monitor's fast stop zone would stop her (navigation.launch.py runs it).

    cmd_vel_nav_smoothed (velocity smoother)  ->  cmd_vel_nav_capped (collision monitor)

The monitor's DirectionalStop picks its zone from the speed Nav2 asks for: below 0.25 m/s the
zone reaches 0.372 m ahead of her centre, from 0.25 m/s 0.502 m. Something 0.38-0.50 m ahead
is fine at a crawl but stops her dead at cruise - and Nav2 keeps asking for cruise, so she sits
there: drive 51 (2.9 s, curtain at the corridor's north end), drive 43's turn 3 (30 s), the
real-time sim's dining-north stops (17-28 s, a chair leg 0.44 m ahead).

So when the lidar sees min_points or more returns inside the fast zone (plus a margin) on the
side she is driving towards, the asked speed is capped below the fast zone's threshold. The
turn rate is scaled by the same factor, so the curve she drives does not change. The zones
themselves are untouched: anything inside the slow zone still stops her.

Silence in, silence out (the monitor then stops her); no scan for scan_timeout s = no cap.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def cap_twist(v, w, near_fwd, near_rev, cap):
    """(v, w) with |v| held to cap when something is near on the side she drives towards; w scaled alike."""
    near = near_fwd if v > 0 else near_rev if v < 0 else False
    if not near or abs(v) <= cap:
        return v, w
    k = cap / abs(v)
    return math.copysign(cap, v), w * k


def zone_counts(px, py, x_in, x_out, half_w):
    """Returns ahead / behind between x_in and x_out (base frame), within half_w of her centreline."""
    side = np.abs(py) <= half_w
    fwd = int(np.count_nonzero(side & (px >= x_in) & (px <= x_out)))
    rev = int(np.count_nonzero(side & (px <= -x_in) & (px >= -x_out)))
    return fwd, rev


class NearCap(Node):
    def __init__(self):
        super().__init__('near_cap')
        p = self.declare_parameter
        self.base = p('base_frame', 'base_footprint').value
        self.cap = float(p('cap', 0.24).value)               # under the fast zone's linear_min 0.25
        self.x_in = float(p('x_in', 0.12).value)             # DirectionalStop zones start 0.12 m from centre
        self.x_out = float(p('x_out', 0.55).value)           # fast zone 0.502 + ~5 cm of warning
        self.half_w = float(p('half_width', 0.19).value)     # zones 0.17 + 2 cm
        self.min_points = int(p('min_points', 4).value)      # as the monitor
        self.scan_timeout = float(p('scan_timeout', 0.5).value)
        self.near = (False, False)
        self.scan_t = None
        self.laser = None                                    # (x, y, yaw) of the scan frame in base
        self.capped = False
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.pub = self.create_publisher(Twist, 'cmd_vel_nav_capped', 10)
        self.create_subscription(Twist, 'cmd_vel_nav_smoothed', self.on_cmd, 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, m):
        if self.laser is None:
            try:                                  # static mount; lidar_link is turned 180 deg
                t = self.tf.lookup_transform(self.base, m.header.frame_id, rclpy.time.Time())
            except Exception:
                return
            q = t.transform.rotation
            self.laser = (t.transform.translation.x, t.transform.translation.y,
                          math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        lx, ly, lth = self.laser
        r = np.asarray(m.ranges, dtype=float)
        a = m.angle_min + np.arange(len(r)) * m.angle_increment + lth
        ok = np.isfinite(r) & (r > m.range_min) & (r < self.x_out + 0.5)
        px, py = lx + r[ok] * np.cos(a[ok]), ly + r[ok] * np.sin(a[ok])
        fwd, rev = zone_counts(px, py, self.x_in, self.x_out, self.half_w)
        self.near = (fwd >= self.min_points, rev >= self.min_points)
        self.scan_t = self.get_clock().now()

    def on_cmd(self, m):
        fresh = (self.scan_t is not None and
                 (self.get_clock().now() - self.scan_t).nanoseconds * 1e-9 < self.scan_timeout)
        nf, nr = self.near if fresh else (False, False)
        v, w = cap_twist(m.linear.x, m.angular.z, nf, nr, self.cap)
        capped = v != m.linear.x
        if capped != self.capped:
            self.capped = capped
            self.get_logger().info('something near ahead: speed held to %.2f m/s' % self.cap if capped
                                   else 'clear: full speed allowed')
        out = Twist()
        out.linear.x, out.angular.z = v, w
        self.pub.publish(out)


def main():
    rclpy.init()
    node = NearCap()
    try:                                        # as nav_translator: the stock executor rebuilds its wait set per wake-up
        from rclpy.experimental import EventsExecutor
        executor = EventsExecutor()
    except ImportError:
        from rclpy.executors import SingleThreadedExecutor
        executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as e:                      # a SIGTERM mid-spin invalidates the context; not a fault
        if rclpy.ok():
            raise
        del e
    finally:
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
