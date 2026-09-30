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

"""Park Rosie straight: at a spot AND facing a heading.

    ros2 run jetnano_navigation nav_park                 (the parking spot: 0, 0, 0 deg)
    ros2 run jetnano_navigation nav_park X Y HEADING_DEG [--pre 1.2] [--tol 6]

Nav2 no longer insists on a final heading (nav2.yaml yaw_goal_tolerance, 2026-09-29): a
steering robot that reaches its spot pointing the wrong way cannot turn on the spot and
shuffled there for 100 s. Parking needs the heading, so:

1. nav_goal to a point `pre` metres out in front of the spot, any heading;
2. straighten there: short forward-and-back shuffles at full lock - forward steering one
   way, back steering the other, both turn her the same way - until her heading is within
   `tol` degrees (each pair turns about 2 d / R: 10 cm legs at R 0.37 m, ~30 deg);
3. nav_goal to the spot: a straight reverse in.

Step 2 drives on cmd_vel_nav, Nav2's own input to twist_mux, while Nav2 is idle - so the
joystick outranks it and the e-stop lock, collision guard and motion check all apply. It
stops at the first motion lock.
"""

import math
import subprocess
import sys
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from std_msgs.msg import Bool
import tf2_ros

TURN_RADIUS_M = 0.37      # full lock, four-wheel steering (measured 0.33 right / 0.39 left, 2026-09-27)
STEER = 2.2               # steering units, just short of the translator's 2.4 lock
THROTTLE_FWD = 0.18       # at full lock: the translator's crawl plus its lock boost
THROTTLE_REV = 0.20       # reverse is the weaker side
LEG_MIN_M, LEG_MAX_M = 0.04, 0.12
LEG_TIMEOUT_S = 2.5       # a leg that has not got there by then is stuck: the motion check is 1.5 s
MAX_CYCLES = 6


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def leg_length(err_rad):
    """How far each of the pair of legs goes to turn her err_rad (both legs turn her)."""
    return max(LEG_MIN_M, min(LEG_MAX_M, abs(err_rad) * TURN_RADIUS_M / 2.0))


def nav_goal(x, y, h_deg, timeout=90):
    cmd = [sys.executable, '-m', 'jetnano_navigation.nav_goal', f'{x:.3f}', f'{y:.3f}', f'{h_deg:.1f}', str(timeout)]
    print(f'-- nav_goal {x:+.2f} {y:+.2f} {h_deg:+.0f}', flush=True)
    out = subprocess.run(cmd, capture_output=True, text=True)
    lines = [ln for ln in out.stdout.splitlines() if ln.startswith(('result', 'goal', 'start'))]
    print('\n'.join('   ' + ln for ln in lines), flush=True)
    return any(ln.startswith('result SUCCEEDED') for ln in lines)


class Straighten:
    """Shuffle at full lock until she faces `heading` (map frame)."""

    def __init__(self, node, heading, tol):
        self.n, self.heading, self.tol = node, heading, tol
        self.pub = node.create_publisher(Twist, 'cmd_vel_nav', 10)
        self.odom = None
        self.lock = False
        node.create_subscription(Odometry, 'odometry/filtered', self._on_odom, 10)
        node.create_subscription(Bool, 'e_stop_motion', lambda m: setattr(self, 'lock', m.data), 10)
        self.buf = tf2_ros.Buffer()
        self.tf = tf2_ros.TransformListener(self.buf, node)

    def _on_odom(self, m):
        self.odom = (m.pose.pose.position.x, m.pose.pose.position.y)

    def spin(self, s):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(self.n, timeout_sec=0.02)

    def map_heading(self):
        for _ in range(50):
            if self.buf.can_transform('map', 'base_footprint', rclpy.time.Time()):
                q = self.buf.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform.rotation
                return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            self.spin(0.1)
        return None

    def send(self, throttle, steer, seconds):
        m = Twist()
        m.linear.x, m.angular.z = throttle, steer
        end = time.time() + seconds
        while time.time() < end and not self.lock:
            self.pub.publish(m)
            self.spin(0.05)

    def leg(self, throttle, steer, dist):
        """Steer first, standing, then drive until `dist` metres by odometry."""
        self.send(0.0, steer, 0.4)                    # the wheels to lock before moving
        self.spin(0.1)
        if self.odom is None or self.lock:
            return False
        x0, y0 = self.odom
        m = Twist()
        m.linear.x, m.angular.z = throttle, steer
        t0 = time.time()
        while not self.lock and time.time() - t0 < LEG_TIMEOUT_S:
            if math.hypot(self.odom[0] - x0, self.odom[1] - y0) >= dist:
                break
            self.pub.publish(m)
            self.spin(0.05)
        self.send(0.0, steer, 0.3)                    # stop, wheels still turned
        return not self.lock

    def run(self):
        for cycle in range(MAX_CYCLES + 1):
            h = self.map_heading()
            if h is None:
                print('no map -> base_footprint: not straightening', flush=True)
                return False
            err = wrap(self.heading - h)
            print(f'   heading {math.degrees(h):+.0f} deg, {math.degrees(err):+.0f} to go', flush=True)
            if abs(err) <= self.tol:
                return True
            if cycle == MAX_CYCLES:
                break
            s = 1.0 if err > 0 else -1.0              # + turns her left (counter-clockwise)
            d = leg_length(err)
            if not (self.leg(THROTTLE_FWD, s * STEER, d) and self.leg(-THROTTLE_REV, -s * STEER, d)):
                print('   the motion lock stopped her: not straightening further', flush=True)
                return False
        print(f'   still off after {MAX_CYCLES} shuffles', flush=True)
        return False


def main():
    argv = sys.argv[1:]

    def opt(name, default):
        if name in argv:
            i = argv.index(name)
            v = float(argv[i + 1])
            del argv[i:i + 2]
            return v
        return default

    pre = opt('--pre', 1.2)
    tol = math.radians(opt('--tol', 6.0))
    x, y, h_deg = (float(a) for a in argv[:3]) if len(argv) >= 3 else (0.0, 0.0, 0.0)
    h = math.radians(h_deg)
    px, py = x + pre * math.cos(h), y + pre * math.sin(h)   # in front of the spot: she reverses in

    print(f'parking at ({x:+.2f}, {y:+.2f}) facing {h_deg:+.0f} deg, from ({px:+.2f}, {py:+.2f})', flush=True)
    if not nav_goal(px, py, h_deg):
        raise SystemExit('could not reach the point in front of the spot')
    rclpy.init()
    node = rclpy.create_node('nav_park')
    try:
        print('-- straightening', flush=True)
        Straighten(node, h, tol).run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    ok = nav_goal(x, y, h_deg)
    print('parked' if ok else 'the last leg did not succeed', flush=True)


if __name__ == '__main__':
    main()
