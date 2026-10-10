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
from rclpy.serialization import deserialize_message

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


def scrub_factor(lock, loss):
    """Her speed at this much of full lock (0..1) over her speed going straight, same throttle.
    Four-wheel steering with no differential scrubs all four tyres in a turn (Steve heard it,
    2026-10-09); drives 50-52, held steering: 0.89 at 1/4-1/2 lock, 0.77 at 0.6, 0.72 at 0.8,
    0.56-0.74 at 0.9-1.0 -> about 1 - 0.35 x lock."""
    return max(0.4, 1.0 - loss * max(0.0, min(1.0, lock)))


class NavTranslator(Node):

    def __init__(self):
        super().__init__('nav_translator')
        self.gain_left = float(self.declare_parameter('curvature_per_steer_left', 1.05).value)
        self.gain_right = float(self.declare_parameter('curvature_per_steer_right', 1.30).value)
        self.max_steer = float(self.declare_parameter('max_steer', 2.4).value)
        # 0.30 until 2026-10-04: drive 35 (cruise 0.40 m/s) peaked at exactly 0.30 on the straights;
        # 0.34 for the 0.42 cruise, a little headroom, still well short of the ESC's range
        self.max_throttle = float(self.declare_parameter('max_throttle', 0.34).value)
        self.stop_below = float(self.declare_parameter('stop_below_mps', 0.03).value)
        self.ki = float(self.declare_parameter('speed_correction_gain', 0.15).value)   # throttle per (m/s * s)
        self.trim_limit = float(self.declare_parameter('speed_correction_limit', 0.08).value)
        # the simulator's drive model takes m/s and rad/s as they are
        self.passthrough = bool(self.declare_parameter('passthrough', False).value)
        # steering units per second: near a goal Nav2's tiny corrections become sharp curves at
        # crawl speed, and without this the wheels flicked to full lock in the last 10 cm
        # 8/s: a full-lock swap in 0.6 s. At 4/s it took 0.36 m of travel, too slow for the
        # direction changes of a turn-around (2026-09-27 tests)
        self.max_steer_rate = float(self.declare_parameter('max_steer_rate', 8.0).value)
        # throttle added near full lock, none below half lock: four-wheel steering at full lock
        # scrubs all four tyres, and the speed table was measured going straight. 2026-09-28,
        # a goal's last metre: Nav2 asked 0.08 m/s on a 10 cm radius, full lock at throttle 0.08,
        # and she slowed from 0.22 m/s to a standstill until the motion check stopped her.
        # 0.04 -> 0.08 2026-10-09: with lock_speed_loss she held the asked speed up to 0.7 lock
        # (drive 54) but made only 0.63-0.74 of it at 0.7-0.95 lock and crawl speeds (0.18-0.22
        # m/s), where the table is flattest and a fixed add-on counts most.
        self.lock_boost = float(self.declare_parameter('lock_throttle_boost', 0.08).value)
        # ... and the speed table asked for more as the wheels turn: 1/scrub_factor (2026-10-09)
        self.lock_loss = float(self.declare_parameter('lock_speed_loss', 0.35).value)
        # steering added to every command: where she actually goes straight. Drives 52-54
        # (2026-10-09, three laps, steering held): sent -0.20 -> straight (+0.01 1/m), -0.41 ->
        # -0.09, +0.19 -> +0.18 - so at 0 she pulled LEFT and the wheels barely answer within
        # ~0.3 of centre (play). Pure pursuit only steered that -0.2 to -0.4 once she was 10-15 cm
        # left of the path: by the curtain going out past Steve's chair, by the chair coming back,
        # while the plan ran down the middle.
        self.steer_trim = float(self.declare_parameter('steer_trim', -0.20).value)
        self.steer = 0.0
        self.trim = 0.0
        self.measured = 0.0
        self._odom_raw = None                   # the EKF's latest, still serialized
        self.last_t = None
        self.pub = self.create_publisher(Twist, 'cmd_vel_nav', 10)
        self.create_subscription(Twist, 'cmd_vel_nav_mps', self.on_cmd, 10)
        # The EKF publishes 100 times a second and only its speed is needed, and only
        # when Nav2 asks for a speed (20 times a second, driving): keep the latest message
        # as it came and decode it then. Decoding every one in Python was most of this
        # node's 13 % of a core on the 2026-09-28 drive.
        self.create_subscription(Odometry, 'odometry/filtered',
                                 lambda raw: setattr(self, '_odom_raw', raw), 10, raw=True)
        self.get_logger().info(
            f'Nav2 m/s -> throttle (max {self.max_throttle:.2f}), curvature -> steering '
            f'(left x{self.gain_left:.2f}, right x{self.gain_right:.2f}, lock {self.max_steer:.1f}, trim {self.steer_trim:+.2f})')

    def steered(self):
        """The steering to send: the planned curve's plus the straight-ahead trim, within lock."""
        return max(-self.max_steer, min(self.max_steer, self.steer + self.steer_trim))

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
            # stopped, often only for a direction change: keep the wheels where they are rather
            # than centring them and swinging back (twist_mux / the driver centre them anyway
            # once Nav2 goes quiet)
            self.trim = 0.0
            out.angular.z = self.steered()
            self.pub.publish(out)
            return
        if self._odom_raw is not None:
            self.measured = deserialize_message(self._odom_raw, Odometry).twist.twist.linear.x
        # steering: the planned curve, whatever the speed
        curvature = w / v
        steer = curvature / (self.gain_left if curvature > 0 else self.gain_right)
        steer = max(-self.max_steer, min(self.max_steer, steer))
        step = self.max_steer_rate * (dt if dt > 0 else 0.05)
        self.steer = max(self.steer - step, min(self.steer + step, steer))
        out.angular.z = self.steered()
        lock_frac = abs(self.steer) / self.max_steer
        # speed: the measured table (asked for what the turn's scrub will take off), then a slow
        # nudge toward the speed Nav2 asked for
        base = throttle_for(v / scrub_factor(lock_frac, self.lock_loss), SPEED_FWD if v > 0 else SPEED_REV)
        err = abs(v) - abs(self.measured) if (self.measured > 0) == (v > 0) else abs(v)
        self.trim = max(-self.trim_limit, min(self.trim_limit, self.trim + self.ki * err * dt))
        throttle = max(0.0, min(self.max_throttle, base + self.trim))
        lock = max(0.0, min(1.0, (lock_frac - 0.5) / 0.5))
        throttle = min(self.max_throttle, throttle + self.lock_boost * lock)
        out.linear.x = math.copysign(throttle, v)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = NavTranslator()
    # rclpy's EventsExecutor: the default SingleThreadedExecutor rebuilds its wait set in
    # Python on every wake-up, 100 of them a second here (see web_teleop, 2026-09-28)
    try:
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
    finally:
        try:
            node.destroy_node()
        except (Exception, KeyboardInterrupt):  # noqa: BLE001 - the context is gone, or a second SIGINT, after an external shutdown
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
