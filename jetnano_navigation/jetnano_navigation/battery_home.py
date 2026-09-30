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

"""When the battery goes low, she goes home and parks (Steve, 2026-09-30: "go home when the
countdown plays").

Runs beside Nav2 (navigation.launch.py). Watches battery_monitor's verdict, battery/level.
When it turns 'low' - a whole minute under 10.5 V with no recovery, the five-note countdown -
and she is neither on the bench (where_am_i) nor already home (/pose within near_home_m of
the spot), it cancels whatever Nav2 is doing and runs nav_park, once per low episode. nav_park
takes /tmp/nav_park.lock, so a drive script's own parking cannot start on top of it, and
nav_goal refuses every other goal while the battery is low, so the script's remaining legs
simply return at once. At 'flat' the motors are locked already: nothing to do but log.
"""

import json
import math
import os
import subprocess
import sys
import threading
import time

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def should_go_home(level: str, where_state, dist_home_m, already: bool):
    """(go?, why) - the decision, with no ROS in it."""
    if level != 'low':
        return False, f'battery {level}'
    if already:
        return False, 'already heading home for this low battery'
    if where_state == 'bench':
        return False, 'on the bench'
    if dist_home_m is not None and dist_home_m <= 0.6:
        return False, f'already home ({dist_home_m:.2f} m from the spot)'
    return True, 'battery low: heading home'


class BatteryHome(Node):

    def __init__(self):
        super().__init__('battery_home')
        p = self.declare_parameter
        p('enabled', True)
        p('home_x', 0.0)
        p('home_y', 0.0)
        p('home_heading_deg', 0.0)
        p('near_home_m', 0.6)
        p('log_dir', os.path.expanduser('~/audit'))
        self.level = None
        self.where = None
        self.pose = None                     # (x, y) on the map, from slam_toolbox's /pose
        self.going = False                   # this low episode is being handled
        self.create_subscription(String, 'battery/level', self._on_level, LATCHED)
        self.create_subscription(String, 'where_am_i/state', self._on_where, LATCHED)
        self.create_subscription(PoseWithCovarianceStamped, '/pose', self._on_pose, 10)
        self.speak = self.create_publisher(String, 'speak', 10)
        self.cancel = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.get_logger().info('watching battery/level: on "low" she cancels her goal and parks')

    def _on_where(self, m):
        try:
            self.where = json.loads(m.data).get('state')
        except ValueError:
            pass

    def _on_pose(self, m):
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y)

    def _on_level(self, m):
        try:
            level = json.loads(m.data).get('level', 'ok')
        except ValueError:
            return
        if level != self.level:
            self.get_logger().info(f'battery level {level}')
        self.level = level
        if level in ('ok', 'none'):
            self.going = False                # a fresh pack or a recovery: armed again
            return
        if not bool(self.get_parameter('enabled').value):
            return
        dist = None
        if self.pose is not None:
            dist = math.hypot(self.pose[0] - float(self.get_parameter('home_x').value),
                              self.pose[1] - float(self.get_parameter('home_y').value))
        go, why = should_go_home(level, self.where, dist, self.going)
        if level == 'low' and not go and not self.going:
            self.get_logger().info(f'not heading home: {why}')
        if not go:
            return
        self.going = True
        self.get_logger().warning(why)
        threading.Thread(target=self._go_home, daemon=True).start()

    def _cancel_nav(self):
        if not self.cancel.wait_for_service(timeout_sec=3.0):
            return 'Nav2 not running: nothing to cancel'
        fut = self.cancel.call_async(CancelGoal.Request())     # an empty goal id: everything
        end = time.monotonic() + 5.0
        while not fut.done() and time.monotonic() < end:
            time.sleep(0.05)
        if not fut.done():
            return 'cancel: no answer in 5 s'
        return f'cancelled {len(fut.result().goals_canceling)} goal(s)'

    def _go_home(self):
        self.get_logger().warning(self._cancel_nav())
        words = String()
        words.data = 'My battery is low. I am heading home.'
        self.speak.publish(words)
        os.makedirs(str(self.get_parameter('log_dir').value), exist_ok=True)
        log = os.path.join(str(self.get_parameter('log_dir').value), time.strftime('battery_home_%Y%m%d-%H%M%S.log'))
        cmd = [sys.executable, '-m', 'jetnano_navigation.nav_park',
               f'{float(self.get_parameter("home_x").value):.3f}', f'{float(self.get_parameter("home_y").value):.3f}',
               f'{float(self.get_parameter("home_heading_deg").value):.1f}']
        with open(log, 'w') as f:
            rc = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT).returncode
        with open(log) as f:
            tail = [ln.strip() for ln in f if ln.startswith(('parked', 'the last leg', 'another nav_park', 'result'))]
        self.get_logger().warning(f'home: nav_park exit {rc}: ' + ('; '.join(tail[-2:]) or 'see ' + log))


def main(args=None):
    rclpy.init(args=args)
    node = BatteryHome()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
