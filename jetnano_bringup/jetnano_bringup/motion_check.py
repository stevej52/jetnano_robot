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

"""Told to drive, but the odometry says she is not moving: stop her.

    ros2 run jetnano_bringup motion_check

On 2026-09-26 the D435's new firmware left the projector on in every frame;
cuVSLAM saw a dot pattern that moved with the camera, reported "tracking, all
fine" and a robot standing still, and she drove three metres into the
curtains - twice. Every watchdog checked that the odometry was *publishing*;
none checked that it *saw her move*. This one does, from what actually
reaches the driver:

* the command on ``cmd_vel`` (after twist_mux and the collision guard, so a
  guard stop or an e-stop is not "being told to drive") has been at least
  ``min_cmd`` for ``window_s``, and
* in that time ``vo`` has moved less than ``min_move_m`` - or has not
  published at all,

then it raises its own twist_mux lock, ``e_stop_motion`` (priority 255, one
lock per source so nobody else's "false" can clear it), says "nope" and logs
why. The lock lifts once every driver has let go - nothing non-zero on
``cmd_vel_web``, ``cmd_vel_teleop``, ``cmd_vel_nav`` or ``cmd_vel_tilt`` for
``release_after_s`` - so a second try gets one more short window, never a
free run.

It also stops her for a stalled motor or wheels spinning in the air, which
is the right answer too; on the bench with the wheels up, set ``enabled``
false. ``throttle_calibration`` creeps below the motor's start on purpose and
pauses this check by publishing ``motion_check/pause`` (true, repeated; the
pause lapses 2 s after the last one, so a crashed tool cannot leave it off).
"""

import math
import time
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String

INPUTS = ('cmd_vel_web', 'cmd_vel_teleop', 'cmd_vel_nav', 'cmd_vel_tilt')


class MotionCheck(Node):

    def __init__(self):
        super().__init__('motion_check')
        self.declare_parameter('enabled', True)
        self.declare_parameter('cmd_topic', 'cmd_vel')
        self.declare_parameter('vo_topic', 'vo')
        self.declare_parameter('lock_topic', 'e_stop_motion')
        # 2026-09-27 floor calibration: command 0.125 (the page at 25 %) = 0.28 m/s;
        # 0.08 is well above the motor's start, so it always means "moving".
        self.declare_parameter('min_cmd', 0.08)
        self.declare_parameter('window_s', 1.5)
        self.declare_parameter('min_move_m', 0.04)
        self.declare_parameter('release_after_s', 1.0)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.min_cmd, self.window = float(p('min_cmd')), float(p('window_s'))
        self.min_move, self.release_after = float(p('min_move_m')), float(p('release_after_s'))

        self.vo = deque(maxlen=400)           # (time, x, y)
        self.cmd_at = 0.0                     # last command on cmd_vel
        self.cmd_since = None                 # when an unbroken run of real commands began
        self.input_at = 0.0                   # last non-zero command from any driver
        self.paused_until = 0.0
        self.locked = False
        self.locked_at = 0.0
        self.lock_sent = 0.0
        self.events = 0

        self.lock_pub = self.create_publisher(Bool, str(p('lock_topic')), 10)
        self.say_pub = self.create_publisher(String, 'say', 10)
        self.state_pub = self.create_publisher(String, 'motion_check/state', 10)
        self.create_subscription(Twist, str(p('cmd_topic')), self._on_cmd, 10)
        self.create_subscription(Odometry, str(p('vo_topic')), self._on_vo, 50)
        self.create_subscription(Bool, 'motion_check/pause', self._on_pause, 10)
        for topic in INPUTS:
            self.create_subscription(Twist, topic, self._on_input, 10)
        self.create_timer(0.1, self._tick)
        self._send(False)
        self.get_logger().info(
            f'stops her if told to drive (>= {self.min_cmd:g}) for {self.window:g} s while the odometry '
            f'moves < {self.min_move * 100:.0f} cm; lock {p("lock_topic")}')

    # ---------------------------------------------------------------- input --
    def _on_cmd(self, msg: Twist) -> None:
        now = time.monotonic()
        self.cmd_at = now
        if abs(msg.linear.x) >= self.min_cmd:
            if self.cmd_since is None:
                self.cmd_since = now
        else:
            self.cmd_since = None

    def _on_vo(self, msg: Odometry) -> None:
        q = msg.pose.pose.position
        self.vo.append((time.monotonic(), q.x, q.y))

    def _on_input(self, msg: Twist) -> None:
        if abs(msg.linear.x) > 0.02 or abs(msg.angular.z) > 0.02:
            self.input_at = time.monotonic()

    def _on_pause(self, msg: Bool) -> None:
        if msg.data:
            self.paused_until = time.monotonic() + 2.0

    # ---------------------------------------------------------------- logic --
    def _tick(self) -> None:
        now = time.monotonic()
        if self.locked:
            if now - self.input_at >= self.release_after and now - self.locked_at >= 2.0:
                self.locked = False
                self._send(False)
                self.get_logger().info('released: nobody is driving now')
                self._state('ok')
            elif now - self.lock_sent >= 1.0:
                self._send(True)
            return
        if not self.get_parameter('enabled').value or now < self.paused_until:
            return
        if self.cmd_since is None or now - self.cmd_at > 0.3 or now - self.cmd_since < self.window:
            return
        recent = [(t, x, y) for t, x, y in self.vo if now - t <= self.window]
        if len(recent) < 5:
            reason = f'no odometry for {self.window:g} s'
        else:
            moved = max(math.hypot(x - recent[0][1], y - recent[0][2]) for _, x, y in recent)
            if moved >= self.min_move:
                return
            reason = f'the odometry moved {moved * 100:.1f} cm in {self.window:g} s'
        self.locked = True
        self.locked_at = now
        self.events += 1
        self.cmd_since = None
        self._send(True)
        self._say('nope')
        self._state(f'stopped: told to drive, {reason}')
        self.get_logger().warning(
            f'STOPPED her: told to drive for {self.window:g} s but {reason} - odometry blind, wheels stuck '
            f'or in the air (event {self.events}); released when nobody is driving')

    # --------------------------------------------------------------- output --
    def _send(self, value: bool) -> None:
        m = Bool()
        m.data = value
        self.lock_pub.publish(m)
        self.lock_sent = time.monotonic()

    def _say(self, mood: str) -> None:
        m = String()
        m.data = mood
        self.say_pub.publish(m)

    def _state(self, text: str) -> None:
        m = String()
        m.data = text
        self.state_pub.publish(m)


def main(args=None):
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=args)
    node = MotionCheck()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except (Exception, KeyboardInterrupt):  # noqa: BLE001 - the context is gone, or a second SIGINT, after an external shutdown
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
