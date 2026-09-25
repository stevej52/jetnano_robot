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

"""Throttle calibration: where she really starts moving, and how fast each
step of the page's speed slider really is. Drives the robot - on the floor,
motors powered, with someone at the web page's EMERGENCY STOP.

    ros2 run jetnano_bringup throttle_calibration --dry-run   # checks only, no motion
    ros2 run jetnano_bringup throttle_calibration             # the test, about two minutes
    ros2 run jetnano_bringup throttle_calibration --apply     # and keep the new start points

Why: the start points in pca9685.yaml (``throttle.esc.reverse_start`` 0.12 for
forward, ``forward_start`` 0.09 for reverse) were found on the bench with the
wheels in the air. On the floor she needs more before she moves, so the page's
40 % did nothing and 30 % in the guard's slow zone was a stop (Steve,
2026-09-25).

1. Start points. Both are set to 0 for the moment, so a command becomes a
   plain pulse; from a standstill the throttle creeps up 0.01 every 0.7 s
   until her odometry shows real motion (0.05 m/s, or 3 cm), forward, then
   backward. That output is the floor start point.
2. Speeds. With the new start points in, she drives the page's full-stick
   command at slider 25, 50, 75 and 100 % (linear 0.125 ... 0.5), forward then
   back each time, so she ends where she began; each run stops after 0.3 m
   or 2.5 s, and the speed is the median of its last part.

She moves only while there is at least ``clear_m`` of open floor in the
direction of travel (lidar), and stops at once on the e-stop, a collision
guard stop, stale odometry, an obstacle inside ``abort_m``, or Ctrl-C. The
start points are put back as they were unless --apply. Commands go out on
cmd_vel_web, like the page's, through twist_mux and the collision guard.
"""

import math
import os
import statistics
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

try:
    from nav2_msgs.msg import CollisionMonitorState
except ImportError:  # pragma: no cover
    CollisionMonitorState = None

FWD_START = 'throttle.esc.reverse_start'     # the driver names sides by pulse: robot forward = its "reverse"
BACK_START = 'throttle.esc.forward_start'
DEADBAND = 0.02                              # throttle.esc.deadband in pca9685.yaml
SLIDER = (25, 50, 75, 100)
MAX_LINEAR = 0.5                             # web_teleop max_linear: the slider scales up to this


class Abort(Exception):
    pass


class Calibrate(Node):

    def __init__(self, dry_run: bool, apply: bool):
        super().__init__('throttle_calibration')
        self.dry_run, self.apply = dry_run, apply
        self.clear_m, self.abort_m = 1.6, 0.6
        self.pub = self.create_publisher(Twist, 'cmd_vel_web', 10)
        self.v = None
        self.x = self.y = 0.0
        self.odom_at = 0.0
        self.scan = None
        self.e_stop = False
        self.guard_stop = False
        self.cmd = 0.0
        self.create_subscription(Odometry, 'odometry/filtered', self._on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Bool, 'e_stop', lambda m: setattr(self, 'e_stop', m.data), 10)
        if CollisionMonitorState is not None:
            self.create_subscription(CollisionMonitorState, 'collision_guard/state', self._on_guard, 10)
        self.get_client = self.create_client(GetParameters, '/pca9685/get_parameters')
        self.set_client = self.create_client(SetParameters, '/pca9685/set_parameters')
        self.create_timer(0.05, self._publish)         # 20 Hz, like a hand on the stick

    # ------------------------------------------------------------- inputs --

    def _on_odom(self, msg):
        self.v = msg.twist.twist.linear.x
        self.x, self.y = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.odom_at = time.monotonic()

    def _on_scan(self, msg):
        self.scan = msg

    def _on_guard(self, msg):
        self.guard_stop = msg.action_type == CollisionMonitorState.STOP

    def _publish(self):
        t = Twist()
        t.linear.x = self.cmd
        self.pub.publish(t)

    def clearance(self, direction: int) -> float:
        """Nearest lidar return within +-15 deg of the direction of travel,
        in base_link (the lidar is mounted backwards: yaw pi)."""
        s = self.scan
        if s is None:
            return 0.0
        best = math.inf
        for i, r in enumerate(s.ranges):
            if not (s.range_min < r < s.range_max):
                continue
            a = s.angle_min + i * s.angle_increment + math.pi        # into the robot's frame
            a = math.atan2(math.sin(a), math.cos(a))
            ahead = a if direction > 0 else math.atan2(math.sin(a - math.pi), math.cos(a - math.pi))
            if abs(ahead) <= math.radians(15):
                best = min(best, r)
        return best

    # ---------------------------------------------------------- parameters --

    def _call(self, client, req, what):
        if not client.wait_for_service(timeout_sec=5.0):
            raise Abort(f'the motor driver does not answer ({what})')
        fut = client.call_async(req)
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < 5.0:
            time.sleep(0.02)
        if not fut.done():
            raise Abort(f'the motor driver did not answer ({what})')
        return fut.result()

    def get_starts(self):
        res = self._call(self.get_client, GetParameters.Request(names=[FWD_START, BACK_START]), 'get')
        return res.values[0].double_value, res.values[1].double_value

    def set_starts(self, fwd: float, back: float):
        req = SetParameters.Request(parameters=[
            Parameter(name=n, value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(v)))
            for n, v in ((FWD_START, fwd), (BACK_START, back))])
        res = self._call(self.set_client, req, 'set')
        if not all(r.successful for r in res.results):
            raise Abort('the motor driver refused: ' + '; '.join(r.reason for r in res.results))

    # -------------------------------------------------------------- motion --

    def check(self, direction: int, moving: bool):
        now = time.monotonic()
        if self.e_stop:
            raise Abort('EMERGENCY STOP pressed')
        if now - self.odom_at > 0.5:
            raise Abort('odometry went quiet')
        if moving and self.guard_stop:
            raise Abort('the collision guard stopped her')
        c = self.clearance(direction)
        if moving and c < self.abort_m:
            raise Abort(f'something {c:.2f} m in the way')
        if self.v is not None and abs(self.v) > 2.5:
            raise Abort(f'too fast: {self.v:.2f} m/s')

    def stop_and_settle(self, direction: int):
        self.cmd = 0.0
        t0 = time.monotonic()
        while time.monotonic() - t0 < 4.0:
            self.check(direction, moving=False)
            if self.v is not None and abs(self.v) < 0.02 and time.monotonic() - t0 > 1.0:
                break
            time.sleep(0.05)
        time.sleep(0.8)

    def ready(self, direction: int):
        c = self.clearance(direction)
        if c < self.clear_m:
            raise Abort(f'not enough room {"ahead" if direction > 0 else "behind"}: {c:.2f} m, '
                        f'needs {self.clear_m} m')

    def find_start(self, direction: int) -> float:
        """Creep up from standstill until she moves; returns the output fraction."""
        self.ready(direction)
        x0, y0 = self.x, self.y
        cmd = 0.04
        while cmd <= 0.60:
            self.cmd = direction * cmd
            t0, moving_since = time.monotonic(), None
            while time.monotonic() - t0 < 0.7:
                self.check(direction, moving=True)
                fast = self.v is not None and direction * self.v > 0.05
                moving_since = (moving_since or time.monotonic()) if fast else None
                moved = math.hypot(self.x - x0, self.y - y0)
                if (moving_since and time.monotonic() - moving_since > 0.25) or moved > 0.03:
                    self.stop_and_settle(direction)
                    return (cmd - DEADBAND) / (1.0 - DEADBAND)      # starts are 0: output = shaped command
                time.sleep(0.02)
            cmd = round(cmd + 0.01, 3)
        self.stop_and_settle(direction)
        raise Abort(f'no motion {"forward" if direction > 0 else "backward"} even at 0.60: '
                    'is the motor rail on and the ESC armed?')

    def speed(self, direction: int, command: float) -> float:
        self.ready(direction)
        x0, y0 = self.x, self.y
        self.cmd = direction * command
        t0, samples = time.monotonic(), []
        while time.monotonic() - t0 < 2.5 and math.hypot(self.x - x0, self.y - y0) < 0.30:
            self.check(direction, moving=True)
            if self.v is not None:
                samples.append((time.monotonic() - t0, abs(self.v)))
            time.sleep(0.02)
        self.stop_and_settle(direction)
        late = [v for t, v in samples if t > 0.6 * (samples[-1][0] if samples else 0)]
        return statistics.median(late) if late else 0.0

    # ---------------------------------------------------------------- run --

    def run(self):
        t0 = time.monotonic()
        while (self.scan is None or time.monotonic() - self.odom_at > 0.5) and time.monotonic() - t0 < 10:
            time.sleep(0.1)
        if self.scan is None:
            raise Abort('no lidar')
        self.check(1, moving=False)
        old = self.get_starts()
        print(f'start points now: forward {old[0]:.3f}, backward {old[1]:.3f}')
        print(f'room: {self.clearance(1):.2f} m ahead, {self.clearance(-1):.2f} m behind (needs {self.clear_m})')
        if self.dry_run:
            print('dry run: everything answers; no motion.')
            return
        new = old
        try:
            self.set_starts(0.0, 0.0)
            fwd = self.find_start(+1)
            print(f'forward: moves at output {fwd:.3f}')
            back = self.find_start(-1)
            print(f'backward: moves at output {back:.3f}')
            new = (round(max(0.0, fwd - 0.01), 3), round(max(0.0, back - 0.01), 3))
            self.set_starts(*new)
            print(f'new start points: forward {new[0]}, backward {new[1]}')
            rows = []
            for pct in SLIDER:
                command = MAX_LINEAR * pct / 100.0
                f = self.speed(+1, command)
                b = self.speed(-1, command)
                rows.append((pct, command, f, b))
                print(f'  slider {pct:3d} %  (linear {command:.3f}):  forward {f:.2f} m/s, backward {b:.2f} m/s')
        finally:
            self.cmd = 0.0
            time.sleep(0.3)
            keep = new if self.apply else old
            try:
                self.set_starts(*keep)
                print(f'start points {"kept at" if self.apply and keep != old else "put back to"} '
                      f'forward {keep[0]}, backward {keep[1]}')
            except Abort as exc:
                print(f'COULD NOT RESTORE the start points ({exc}): set {FWD_START}={keep[0]} '
                      f'and {BACK_START}={keep[1]} by hand')
        out = os.path.expanduser(f'~/calibration/throttle-{time.strftime("%Y%m%d-%H%M")}.yaml')
        os.makedirs(os.path.dirname(out), exist_ok=True)
        with open(out, 'w') as fh:
            fh.write(f'# throttle_calibration {time.strftime("%Y-%m-%d %H:%M")}, on the floor\n')
            fh.write(f'forward_moves_at: {fwd:.3f}\nbackward_moves_at: {back:.3f}\n')
            fh.write(f'# pca9685.yaml:\n#   reverse_start: {new[0]}   # robot forward\n'
                     f'#   forward_start: {new[1]}   # robot backward\nspeeds:\n')
            for pct, command, f, b in rows:
                fh.write(f'  - {{slider: {pct}, linear: {command:.3f}, forward_mps: {f:.2f}, backward_mps: {b:.2f}}}\n')
        print(f'results: {out}')


def main(args=None):
    argv = sys.argv[1:]
    rclpy.init(args=args)
    node = Calibrate('--dry-run' in argv, '--apply' in argv)
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    code = 0
    try:
        node.run()
    except Abort as exc:
        print(f'STOPPED: {exc}')
        code = 1
    except KeyboardInterrupt:
        print('STOPPED by hand')
        code = 1
    finally:
        node.cmd = 0.0
        time.sleep(0.3)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
