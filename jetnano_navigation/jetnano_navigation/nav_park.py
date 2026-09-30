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

1. nav_goal to a point `pre` metres out in front of the spot, facing the way she is already
   going (the bearing from where she starts). Drive 7, 2026-09-29: asked to arrive facing the
   spot's heading, the planner put a 128 deg turn in the last half metre and she dithered
   there - five forward/reverse switches in 3 s - and arrived 59 deg off anyway;
2. turn there: forward and back at full lock - forward steering one way, back steering the
   other, both turn her the same way - until her heading is within `tol` degrees. The point
   is in the open, so the legs may be 30 cm: a big turn is a three-point turn;
3. nav_goal to the spot: a straight reverse in;
4. straighten again there, with short legs (the wall is close), if the reverse in curved.

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
# At the point in front of the spot there is room for a proper three-point turn: the house map
# is clear for 1.35 m all round (1.2, 0). On the spot itself the wall is 0.58 m away: short legs.
LEG_MAX_OPEN_M = 0.30
LEAD_RAD = math.radians(1.5)   # she coasts on a little after the throttle drops
SMALL_RAD = math.radians(20)   # below this much to go: half lock, finer
LEG_TIMEOUT_S = 2.5       # a leg that has not got there by then is stuck: the motion check is 1.5 s
MAX_CYCLES = 6


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def leg_length(err_rad):
    """How far each of the pair of legs goes to turn her err_rad (both legs turn her)."""
    return max(LEG_MIN_M, min(LEG_MAX_M, abs(err_rad) * TURN_RADIUS_M / 2.0))


def steer_for(err_rad):
    """Full lock for a big correction, half lock for a small one (finer, less overshoot)."""
    return STEER if abs(err_rad) > SMALL_RAD else 0.5 * STEER


class Shuffles:
    """What to do next, learning how far she coasts past where the throttle was cut.

    2026-09-29 on the floor, full lock and legs by distance: +23 -> -9 -> +14 -> -10 -> +14 ->
    -15 -> +12 deg - a pair meant to turn her 12 deg turned her 23. Now each leg is stopped by
    heading, the stop comes early by the coast measured on the legs before, and a small
    correction is one leg, not a pair (forward and back in turn, so she stays on the spot)."""

    SINGLE_RAD = math.radians(14)

    def __init__(self, leg_max_m=LEG_MAX_M):
        self.coast = LEAD_RAD
        self.last_single = -1.0
        # the most one full-lock leg can turn her before its length runs out (10 % in hand)
        self.max_turn = 0.9 * leg_max_m / TURN_RADIUS_M

    def next_legs(self, err):
        """-> [(throttle sign, steering, turn to stop at)] for this correction: one leg for a
        small one, else as few alternating legs as the leg length allows, forward first - with
        room (LEG_MAX_OPEN_M), 125 deg is a three-point turn: forward, back, forward."""
        s = 1.0 if err > 0 else -1.0                 # + turns her left (counter-clockwise)
        steer = steer_for(err)
        if abs(err) < self.SINGLE_RAD:
            self.last_single = -self.last_single
            sign = self.last_single                   # forward and back in turn
            return [(sign, sign * s * steer, max(0.0, abs(err) - self.coast))]
        n = max(2, math.ceil(abs(err) / self.max_turn))
        each = max(0.0, abs(err) / n - self.coast)
        return [((1.0 if k % 2 == 0 else -1.0), (1.0 if k % 2 == 0 else -1.0) * s * steer, each) for k in range(n)]

    def observe(self, stop_at, turned):
        """How far she turned on a leg that was stopped at `stop_at`: learn the coast."""
        over = max(0.0, abs(turned) - stop_at)
        self.coast = min(math.radians(12), 0.5 * self.coast + 0.5 * over)


def nav_goal(x, y, h_deg, timeout=90, exact=False):
    cmd = [sys.executable, '-m', 'jetnano_navigation.nav_goal', f'{x:.3f}', f'{y:.3f}', f'{h_deg:.1f}', str(timeout)]
    if exact:
        cmd.append('--exact')
    print(f'-- nav_goal {x:+.2f} {y:+.2f} {h_deg:+.0f}', flush=True)
    out = subprocess.run(cmd, capture_output=True, text=True)
    lines = [ln for ln in out.stdout.splitlines() if ln.startswith(('result', 'goal', 'start'))]
    print('\n'.join('   ' + ln for ln in lines), flush=True)
    return any(ln.startswith('result SUCCEEDED') for ln in lines)


class Straighten:
    """Shuffle at full lock until she faces `heading` (map frame)."""

    def __init__(self, node, heading, tol, leg_max_m=LEG_MAX_M):
        self.n, self.heading, self.tol = node, heading, tol
        self.leg_max = leg_max_m
        self.plan = Shuffles(leg_max_m)
        self.pub = node.create_publisher(Twist, 'cmd_vel_nav', 10)
        self.odom = None
        self.lock = False
        node.create_subscription(Odometry, 'odometry/filtered', self._on_odom, 10)
        node.create_subscription(Bool, 'e_stop_motion', lambda m: setattr(self, 'lock', m.data), 10)
        self.buf = tf2_ros.Buffer()
        self.tf = tf2_ros.TransformListener(self.buf, node)

    def _on_odom(self, m):
        q = m.pose.pose.orientation
        self.odom = (m.pose.pose.position.x, m.pose.pose.position.y,
                     math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def spin(self, s):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(self.n, timeout_sec=0.02)

    def map_pose(self):
        """(x, y, heading) on the map, or None."""
        for _ in range(50):
            if self.buf.can_transform('map', 'base_footprint', rclpy.time.Time()):
                tr = self.buf.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform
                q = tr.rotation
                return (tr.translation.x, tr.translation.y,
                        math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
            self.spin(0.1)
        return None

    def map_heading(self):
        pose = self.map_pose()
        return None if pose is None else pose[2]

    def send(self, throttle, steer, seconds):
        m = Twist()
        m.linear.x, m.angular.z = throttle, steer
        end = time.time() + seconds
        while time.time() < end and not self.lock:
            self.pub.publish(m)
            self.spin(0.05)

    def leg(self, throttle, steer, turn):
        """Steer first, standing, then drive until her heading has turned `turn` radians (by
        odometry, whose heading is the IMU's since 2026-09-29), or LEG_MAX_M metres.
        -> how far she turned in all, coasting included, or None if the motion lock stopped her."""
        self.send(0.0, steer, 0.4)                    # the wheels over before moving
        self.spin(0.1)
        if self.odom is None or self.lock:
            return None
        x0, y0, h0 = self.odom
        m = Twist()
        m.linear.x, m.angular.z = throttle, steer
        t0 = time.time()
        timeout = max(LEG_TIMEOUT_S, self.leg_max / 0.1)      # a long leg at a crawl (0.1 m/s)
        while not self.lock and time.time() - t0 < timeout:
            if abs(wrap(self.odom[2] - h0)) >= turn or math.hypot(self.odom[0] - x0, self.odom[1] - y0) >= self.leg_max:
                break
            self.pub.publish(m)
            self.spin(0.02)
        self.send(0.0, steer, 0.5)                    # stop, wheels still turned; let her settle
        return None if self.lock else abs(wrap(self.odom[2] - h0))

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
            for sign, steer, stop_at in self.plan.next_legs(err):
                turned = self.leg(THROTTLE_FWD if sign > 0 else -THROTTLE_REV, steer, stop_at)
                if turned is None:
                    print('   the motion lock stopped her: not straightening further', flush=True)
                    return False
                self.plan.observe(stop_at, turned)
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
    rclpy.init()
    node = rclpy.create_node('nav_park')
    ok = False
    try:
        straighten = Straighten(node, h, tol, LEG_MAX_OPEN_M)
        here = straighten.map_pose()
        approach = h_deg
        if here is not None and math.hypot(px - here[0], py - here[1]) > 0.6:
            approach = math.degrees(math.atan2(py - here[1], px - here[0]))   # the way she is going
        if not nav_goal(px, py, approach):
            raise SystemExit('could not reach the point in front of the spot')
        print('-- turning to face the spot\'s heading', flush=True)
        straighten.run()
        ok = nav_goal(x, y, h_deg, exact=True)   # her spot, even if the costmap calls it tight
        if ok:
            # 2026-09-29 drive 6: straight at the point in front (+4 deg), then the reverse in
            # curved - the shuffles had moved her sideways off the line - and ended -21 deg.
            # Short legs here, forward and back in turn: the wall is 0.58 m behind the spot.
            print('-- straightening on the spot', flush=True)
            coast = straighten.plan.coast                  # what it learned out there still holds
            straighten.leg_max = LEG_MAX_M
            straighten.plan = Shuffles(LEG_MAX_M)
            straighten.plan.coast = coast
            straighten.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print('parked' if ok else 'the last leg did not succeed', flush=True)


if __name__ == '__main__':
    main()
