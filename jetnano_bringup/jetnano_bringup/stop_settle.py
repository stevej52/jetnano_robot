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

"""Quiet the steering after every stop, without the microphone (drive.launch.py runs it).

Which side the wheels come back to centre from decides the buzz. Measured 2026-10-09 with her
mic (buzz_source, tyres on the floor): after 15 deg one way and back to centre the 1.6 kHz
buzz was 17-34 dB every time, after 15 deg the other way 4-8 dB every time. Going past centre
on the quiet side for 0.4 s and back: 6 deg 4/4 quiet, 10 deg 5/5, 3 deg 4/5, waiting 0/5.
Releasing the servos and re-centring did not help (the servo strains again when it comes back).

So once she has stopped and every driver has gone quiet, this goes approach_deg past centre on
the quiet side and back, once, through twist_mux's lowest input (cmd_vel_settle): anyone
driving, and every lock, wins over it. A driver speaking up mid-way cuts it short.

The quiet side is the front wheels at home + 6 deg (rear home - 6): cmd_vel angular.z
approach_deg / deg_per_unit, pca9685.yaml steering angular_z -18.33 -> -0.327.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool


class StopWatch:
    """Has she driven, then stopped long enough, with every driver quiet? Times in seconds."""

    def __init__(self, still_s, quiet_s):
        self.still_s, self.quiet_s = still_s, quiet_s
        self.moved = False
        self.last_move = self.last_msg = -1e9

    def heard(self, t, moving):
        self.last_msg = t
        if moving:
            self.moved = True
            self.last_move = t

    def due(self, t, locked):
        return (self.moved and not locked and t - self.last_move >= self.still_s
                and t - self.last_msg >= self.quiet_s)


def plan(z, hold_s, centre_s, tick_s):
    """The steering commands, one per tick: past centre for hold_s, then centre for centre_s."""
    return [z] * max(1, round(hold_s / tick_s)) + [0.0] * max(1, round(centre_s / tick_s))


class StopSettle(Node):
    def __init__(self):
        super().__init__('stop_settle')
        p = self.declare_parameter
        self.enabled = bool(p('enabled', True).value)
        drivers = list(p('driver_topics', ['cmd_vel_nav', 'cmd_vel_web', 'cmd_vel_teleop',
                                           'cmd_vel_park', 'cmd_vel_tilt']).value)
        locks = list(p('lock_topics', ['e_stop', 'e_stop_web', 'e_stop_joy', 'e_stop_motion',
                                       'e_stop_gate']).value)
        self.watch = StopWatch(float(p('still_s', 0.6).value),      # stopped this long
                               float(p('quiet_s', 0.3).value))      # and no driver talking
        self.tick_s = float(p('tick_s', 0.05).value)
        deg = float(p('approach_deg', 6.0).value)
        per_unit = float(p('deg_per_unit', -18.33).value)          # pca9685.yaml steering angular_z
        self.steps = plan(deg / per_unit, float(p('hold_s', 0.4).value),
                          float(p('centre_s', 0.3).value), self.tick_s)
        self.todo = []
        self.locked = {t: False for t in locks}
        self.pub = self.create_publisher(Twist, 'cmd_vel_settle', 10)
        for topic in drivers:
            self.create_subscription(Twist, topic, self.on_driver, 10)
        for topic in locks:
            self.create_subscription(Bool, topic, lambda m, k=topic: self.locked.__setitem__(k, m.data), 10)
        self.create_timer(self.tick_s, self.on_tick)
        self.get_logger().info(f'after each stop: {deg:g} deg past centre on the quiet side '
                               f'(angular.z {deg / per_unit:+.3f}) and back'
                               + ('' if self.enabled else ' - DISABLED'))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_driver(self, m):
        moving = abs(m.linear.x) > 0.01 or abs(m.angular.z) > 0.05
        self.watch.heard(self.now(), moving)
        if self.todo and moving:
            self.todo = []                                   # someone is driving: theirs

    def on_tick(self):
        if not self.enabled:
            return
        if not self.todo:
            if self.watch.due(self.now(), any(self.locked.values())):
                self.watch.moved = False
                self.todo = list(self.steps)
            else:
                return
        out = Twist()
        out.angular.z = self.todo.pop(0)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = StopSettle()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001 - the context is gone after an external shutdown
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
