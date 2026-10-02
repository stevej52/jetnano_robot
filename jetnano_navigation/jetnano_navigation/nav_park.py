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

1. turn early. She reverses in, so at the point `pre` metres in front of the spot she must
   face AWAY from it - and she arrives facing it, the way she came. Drive 8 (2026-09-30):
   a 131 deg three-point turn in front of the spot, too close to the wall every time. Steve:
   "if you turned a lot sooner you'd end up directly in front of the spot and back right in".
   So nav_goal goes to the point where ONE gentle forward arc (2/3 lock), started facing the way she
   is going, ends at the point in front of the spot facing the spot's heading; then
2. that arc, stopped by the IMU heading; a shuffle or two there if it fell short;
3. a straight reverse in under this script's control, the tail aimed at the spot. Drive 8:
   given as a Nav2 goal, the reverse curved 37 deg - Nav2 chases the point, not the heading;
4. straighten again there, with short legs (the wall is close), if it still needs it.

Steps 2-4 drive on cmd_vel_nav, Nav2's own input to twist_mux, while Nav2 is idle - so the
joystick outranks it and the e-stop lock, collision guard and motion check all apply. They
stop at the first motion lock.
"""

import fcntl
import math
import subprocess
import sys
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from std_msgs.msg import Bool
import tf2_ros

# Steering: the translator's lock is 2.4 and the measured full-lock circles 0.33 m right /
# 0.39 m left (2026-09-27). Full lock binds the linkage a little (Steve, 2026-10-01), so the
# shuffles use 2.0 and the arc in the open, where there is room for a bigger circle, 1.6.
# Radii scale with the steering: curvature = 1.05 x steer left, 1.30 x steer right.
STEER = 2.0               # the shuffles' steering, 5/6 lock
TURN_RADIUS_M = 0.43      # her circle at STEER (0.48 left / 0.38 right)
ARC_STEER = 1.6           # the early arc: 2/3 lock
ARC_RADIUS_LEFT_M, ARC_RADIUS_RIGHT_M = 0.60, 0.48   # at ARC_STEER
ARC_MIN_RAD = math.radians(25)   # less than this to turn at the point in front: no arc, shuffles
THROTTLE_FWD = 0.18       # at full lock: the translator's crawl plus its lock boost
THROTTLE_REV = 0.20       # reverse is the weaker side
LEG_MIN_M, LEG_MAX_M = 0.04, 0.12
# At the point in front of the spot there is room for a proper three-point turn: the house map
# is clear for 1.35 m all round (1.2, 0). On the spot itself the wall is 0.58 m away: short legs.
LEG_MAX_OPEN_M = 0.30
LOCK_FILE = '/tmp/nav_park.lock'
LEAD_RAD = math.radians(1.5)   # she coasts on a little after the throttle drops
SMALL_RAD = math.radians(20)   # below this much to go: half lock, finer
LEG_TIMEOUT_S = 2.5       # a leg that has not got there by then is stuck: the motion check is 1.5 s
MAX_CYCLES = 6
# the reverse in: steering per radian the tail is off the line to the spot, at most half lock
REVERSE_GAIN, REVERSE_STEER_MAX = 4.0, 0.5 * STEER
REVERSE_DONE_M = 0.05     # close enough to the spot
REVERSE_SPEED_MPS = 0.1   # a crawl, for the timeout
REVERSE_LOOKAHEAD_M = 0.3 # the tail aims at the point this far down the spot's line from her


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def leg_length(err_rad):
    """How far each of the pair of legs goes to turn her err_rad (both legs turn her)."""
    return max(LEG_MIN_M, min(LEG_MAX_M, abs(err_rad) * TURN_RADIUS_M / 2.0))


def steer_for(err_rad):
    """Full lock for a big correction, half lock for a small one (finer, less overshoot)."""
    return STEER if abs(err_rad) > SMALL_RAD else 0.5 * STEER


def arc_start(px, py, heading_in, delta):
    """Where a forward arc at ARC_STEER must START, facing `heading_in`, to END at (px, py) having
    turned `delta` radians (+ left). Circle geometry: the centre is one radius to the side."""
    r = ARC_RADIUS_LEFT_M if delta > 0 else ARC_RADIUS_RIGHT_M
    s = 1.0 if delta > 0 else -1.0
    a, b = heading_in, heading_in + delta
    # end - start, for a circle of radius r turned delta (left: centre at +90 deg)
    dx = s * r * (math.sin(b) - math.sin(a))
    dy = s * r * (math.cos(a) - math.cos(b))
    return px - dx, py - dy


def plan_approach(here, px, py, heading):
    """-> (qx, qy, approach, delta): the point to drive to facing `approach` (the way she will be
    going) so that an arc of `delta` ends at (px, py) facing `heading`; or (px, py, heading, 0)
    when the turn there would be small or she is already close."""
    if here is None or math.hypot(px - here[0], py - here[1]) <= 0.6:
        return px, py, heading, 0.0
    qx, qy = px, py
    approach = delta = 0.0
    for _ in range(4):                    # the bearing depends on the point and the point on the bearing
        approach = math.atan2(qy - here[1], qx - here[0])
        delta = wrap(heading - approach)
        if abs(delta) < ARC_MIN_RAD:
            return px, py, approach, 0.0
        qx, qy = arc_start(px, py, approach, delta)
    return qx, qy, approach, delta


AT_ARC_START_M = 0.35


def at_arc_start(here, px, py, heading):
    """Is she already standing at the start of an arc that ends at (px, py) facing `heading`?
    The lap ends there since 2026-10-01 (drive.sh: its last waypoint is the arc start for a
    lap coming down the hall), so the parking carries straight on instead of standing 15 s
    while a nav_goal is planned to a point she is on. -> (yes, qx, qy, delta)."""
    if here is None:
        return False, px, py, 0.0
    delta = wrap(heading - here[2])
    if abs(delta) < ARC_MIN_RAD:
        return False, px, py, 0.0
    qx, qy = arc_start(px, py, here[2], delta)
    return math.hypot(qx - here[0], qy - here[1]) <= AT_ARC_START_M, qx, qy, delta


def reverse_steer(err_rad):
    """Steering while reversing, for the heading error `err_rad` (+ = she should turn left).
    Reversing, left-turn steering turns her right: the sign flips (see Shuffles.next_legs)."""
    return max(-REVERSE_STEER_MAX, min(REVERSE_STEER_MAX, -REVERSE_GAIN * err_rad))


def reverse_aim(px, py, h, x, y, heading):
    """She is at (px, py) facing h; the spot is (x, y) facing `heading`, and she backs into it
    along the spot's line. -> (steering, metres still to go along the line, metres from the spot).
    The tail aims at the point REVERSE_LOOKAHEAD_M down the line from where she is, which goes
    on past the spot: aiming at the spot itself swings wildly in the last centimetres."""
    ux, uy = math.cos(heading), math.sin(heading)
    along = (px - x) * ux + (py - y) * uy             # how far in front of the spot she is
    tx, ty = x + (along - REVERSE_LOOKAHEAD_M) * ux, y + (along - REVERSE_LOOKAHEAD_M) * uy
    away = math.atan2(py - ty, px - tx)               # from the aim point to her: the way to face
    return reverse_steer(wrap(away - h)), along, math.hypot(px - x, py - y)


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
    cmd = [sys.executable, '-m', 'jetnano_navigation.nav_goal', f'{x:.3f}', f'{y:.3f}', f'{h_deg:.1f}', str(timeout),
           '--home']                             # parking is the way home: allowed on a low battery
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

    def map_pose(self, fresh_s=0.5):
        """(x, y, heading) on the map, or None. Spins `fresh_s` first: the listener only hears
        transforms while this node is spun, and it is not spun while nav_goal runs - drive 8
        (2026-09-30) read the heading from BEFORE the reverse in, -5 deg, when she stood at +31,
        and the straightening on the spot did nothing."""
        self.spin(fresh_s)
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

    def leg(self, throttle, steer, turn, leg_max=None):
        """Steer first, standing, then drive until her heading has turned `turn` radians (by
        odometry, whose heading is the IMU's since 2026-09-29), or `leg_max` metres.
        -> how far she turned in all, coasting included, or None if the motion lock stopped her."""
        leg_max = self.leg_max if leg_max is None else leg_max
        self.send(0.0, steer, 0.4)                    # the wheels over before moving
        self.spin(0.1)
        if self.odom is None or self.lock:
            return None
        x0, y0, h0 = self.odom
        m = Twist()
        m.linear.x, m.angular.z = throttle, steer
        t0 = time.time()
        timeout = max(LEG_TIMEOUT_S, leg_max / 0.1)      # a long leg at a crawl (0.1 m/s)
        while not self.lock and time.time() - t0 < timeout:
            if abs(wrap(self.odom[2] - h0)) >= turn or math.hypot(self.odom[0] - x0, self.odom[1] - y0) >= leg_max:
                break
            self.pub.publish(m)
            self.spin(0.02)
        self.send(0.0, steer, 0.5)                    # stop, wheels still turned; let her settle
        return None if self.lock else abs(wrap(self.odom[2] - h0))

    def arc(self):
        """One forward arc at ARC_STEER to the heading, stopped by the IMU (step 1 of the plan).
        -> True if she drove it; False if the motion lock stopped her (the caller shuffles)."""
        h = self.map_heading()
        if h is None:
            return False
        err = wrap(self.heading - h)
        print(f'   heading {math.degrees(h):+.0f} deg, one arc {math.degrees(err):+.0f}', flush=True)
        if abs(err) <= self.tol:
            return True
        r = ARC_RADIUS_LEFT_M if err > 0 else ARC_RADIUS_RIGHT_M
        stop_at = max(0.0, abs(err) - self.plan.coast)
        turned = self.leg(THROTTLE_FWD, ARC_STEER if err > 0 else -ARC_STEER, stop_at, leg_max=r * abs(err) + 0.3)
        if turned is None:
            print('   the motion lock stopped the arc', flush=True)
            return False
        self.plan.observe(stop_at, turned)
        return True

    def reverse_in(self, x, y):
        """Reverse into (x, y) along the spot's line, the tail aimed down the line (step 3 of
        the plan). -> (metres from the spot, heading error in rad) at the end, or None at a
        motion lock."""
        pose = self.map_pose()
        if pose is None:
            return None
        _, along0, _ = reverse_aim(pose[0], pose[1], pose[2], x, y, self.heading)
        timeout = abs(along0) / REVERSE_SPEED_MPS + 10.0
        t0 = time.time()
        least = along0
        m = Twist()
        self.send(0.0, 0.0, 0.4)                      # wheels straight before moving
        while not self.lock and time.time() - t0 < timeout:
            self.spin(0.05)
            if not self.buf.can_transform('map', 'base_footprint', rclpy.time.Time()):
                continue
            tr = self.buf.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform
            q = tr.rotation
            px, py = tr.translation.x, tr.translation.y
            h = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            steer, along, dist = reverse_aim(px, py, h, x, y, self.heading)
            if along <= REVERSE_DONE_M or dist <= REVERSE_DONE_M:
                break
            least = min(least, along)
            if along > least + 0.15:                  # going away from it: something is wrong
                print(f'   drifting away from the spot ({dist:.2f} m): stopping the reverse', flush=True)
                break
            m.linear.x, m.angular.z = -THROTTLE_REV, steer
            self.pub.publish(m)
        self.send(0.0, 0.0, 0.5)                      # stop, wheels straight; let her settle
        if self.lock:
            return None
        pose = self.map_pose(0.3)
        if pose is None:
            return None
        return math.hypot(pose[0] - x, pose[1] - y), wrap(self.heading - pose[2])

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

    # one parking at a time: battery_home may start one while a drive script is about to
    lock = open(LOCK_FILE, 'w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        raise SystemExit('another nav_park is running (battery_home, or a drive script): not starting a second')
    print(f'parking at ({x:+.2f}, {y:+.2f}) facing {h_deg:+.0f} deg, from ({px:+.2f}, {py:+.2f})', flush=True)
    rclpy.init()
    node = rclpy.create_node('nav_park')
    ok = False
    try:
        straighten = Straighten(node, h, tol, LEG_MAX_OPEN_M)
        here = straighten.map_pose()
        there, qx, qy, delta = at_arc_start(here, px, py, h)
        if there:
            print(f'-- already at the start of an arc of {math.degrees(delta):+.0f} deg that ends in front of '
                  f'the spot facing {h_deg:+.0f} ({math.hypot(qx - here[0], qy - here[1]) * 100:.0f} cm off it)', flush=True)
        else:
            qx, qy, approach, delta = plan_approach(here, px, py, h)
            if delta:
                print(f'-- turning early: an arc of {math.degrees(delta):+.0f} deg from ({qx:+.2f}, {qy:+.2f}) '
                      f'ends in front of the spot facing {h_deg:+.0f}', flush=True)
            if not nav_goal(qx, qy, math.degrees(approach)):
                raise SystemExit('could not reach the point in front of the spot')
        print('-- turning to face the spot\'s heading', flush=True)
        if delta:
            straighten.arc()
        straighten.run()                               # the shuffles, if the arc left any
        print('-- reversing in', flush=True)
        end = straighten.reverse_in(x, y)
        if end is None:
            raise SystemExit('the motion lock stopped the reverse in')
        print(f'   ended {end[0] * 100:.0f} cm from the spot, heading off by {math.degrees(-end[1]):+.0f} deg', flush=True)
        ok = end[0] <= 0.25
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
        try:
            node.destroy_node()
        except (Exception, KeyboardInterrupt):  # noqa: BLE001 - the context is gone, or a second SIGINT, after an external shutdown
            pass
        rclpy.shutdown()
    print('parked' if ok else 'the last leg did not succeed', flush=True)


if __name__ == '__main__':
    main()
