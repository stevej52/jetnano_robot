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

"""Something moved: turn and look at it.

    ros2 run jetnano_bringup motion_watch

While the robot stands still, the lidar sees the room as a fixed ring of
ranges. This node learns that ring slowly (so a chair that gets moved is
absorbed within a minute) and looks for a run of adjacent beams that are
suddenly a good deal *closer* than the ring - legs, a dog, a person - seen on
two scans running. That is a mover, with a bearing and a range.

It then points the pan-tilt camera at it (``/pca9685/pan/angle`` and
``/pca9685/tilt/angle``, degrees, the servo channels ros2_pca9685 exposes),
tilting to face height for the distance, says "curious" the first time and
"hm" as it follows, publishes the mover on ``motion/mover`` for RViz, and
returns the camera to centre after ``return_after_s`` with nothing seen.
When the robot drives, everything moves, so it stops watching and centres.

Bearings are in the robot's frame (0 = nose, left positive). The lidar is
mounted backwards (jetnano.urdf.xacro), hence ``scan_yaw_offset``. Pan range
and centre are the mount's; set them when it exists. Without the pan-tilt
the angle topics simply have no subscriber and the chirps still happen.

Three modes (Steve, 2026-09-27), parameter ``mode`` or a String on
``motion/mode`` (listen sends it: "Rosie, keep watch" / "Rosie, stand down"):

* ``approach`` (the default): quiet about people moving around the room; she
  reacts only to someone coming right at her - a mover that has closed in by
  ``approach_by_m`` within ``approach_window_s`` on a steady bearing and is now
  inside ``close_m``. Then she says "huh" and follows them like watch mode
  until nobody has been seen for ``return_after_s``.
* ``watch``: watchdog mode - "huh" and a look at every mover, as before.
* ``off``: nothing.

The mode in force is latched on ``motion/mode_state``. It is not remembered
across a restart: she always comes up in the ``mode`` parameter's mode.

``aim`` and the pan/tilt centre, limit and sign parameters are live, so the
mount can be set up without a restart::

    ros2 param set /motion_watch aim true             # starts false: hands off the servos
    ros2 param set /motion_watch pan_center_deg 92.0
"""

import math
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Twist
from rcl_interfaces.msg import SetParametersResult

from jetnano_bringup.quiet_node import QuietNode
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, String


class MotionWatch(QuietNode):

    def __init__(self):
        super().__init__('motion_watch')
        self.declare_parameter('scan_yaw_offset', math.pi)      # lidar 0 deg is the tail
        self.declare_parameter('still_after_s', 2.0)
        self.declare_parameter('learn_scans', 15)               # scans to build the first background
        self.declare_parameter('learn_rate', 0.02)              # background follows the room this fast
        self.declare_parameter('closer_by_m', 0.25)
        self.declare_parameter('min_beams', 3)
        self.declare_parameter('max_span_deg', 30.0)
        self.declare_parameter('max_range_m', 5.0)
        self.declare_parameter('persist_scans', 2)
        self.declare_parameter('return_after_s', 8.0)
        self.declare_parameter('pan_center_deg', 90.0)          # servo degrees that look straight ahead
        self.declare_parameter('pan_limit_deg', 80.0)           # +- from centre the mount allows
        self.declare_parameter('pan_sign', 1.0)                 # -1 if the servo turns the wrong way
        self.declare_parameter('tilt_center_deg', 90.0)
        self.declare_parameter('tilt_limit_deg', 40.0)
        self.declare_parameter('tilt_sign', 1.0)
        self.declare_parameter('camera_height_m', 0.30)
        self.declare_parameter('look_at_height_m', 1.2)        # a face, roughly; a dog is 0.4
        self.declare_parameter('say', True)
        # Off until the pan-tilt is centred and its reach measured: on 2026-09-25 it
        # aimed 2 s after a reboot and drove an uncalibrated mount into its stops.
        self.declare_parameter('aim', False)                    # False: chirp, leave the servos alone
        self.declare_parameter('mode', 'approach')              # approach | watch | off (see the top)
        self.declare_parameter('close_m', 0.9)                  # approach: this close to the lidar (bumper ~0.25 m)
        self.declare_parameter('approach_by_m', 0.8)            # ... having come this much closer
        self.declare_parameter('approach_window_s', 4.0)        # ... within this long
        self.declare_parameter('approach_bearing_deg', 25.0)    # ... on a steady bearing (walking past sweeps it)

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.yaw_off = float(p('scan_yaw_offset'))
        self.still_after = float(p('still_after_s'))
        self.learn_scans = int(p('learn_scans'))
        self.learn_rate = float(p('learn_rate'))
        self.closer_by = float(p('closer_by_m'))
        self.min_beams = int(p('min_beams'))
        self.max_span = math.radians(float(p('max_span_deg')))
        self.max_range = float(p('max_range_m'))
        self.persist = int(p('persist_scans'))
        self.return_after = float(p('return_after_s'))
        self.pan_c, self.pan_lim, self.pan_sign = float(p('pan_center_deg')), float(p('pan_limit_deg')), float(p('pan_sign'))
        self.tilt_c, self.tilt_lim, self.tilt_sign = float(p('tilt_center_deg')), float(p('tilt_limit_deg')), float(p('tilt_sign'))
        self.cam_h, self.look_h = float(p('camera_height_m')), float(p('look_at_height_m'))
        self.chatty = bool(p('say'))
        self.aim = bool(p('aim'))
        self.mode = str(p('mode'))
        self.close_m, self.approach_by = float(p('close_m')), float(p('approach_by_m'))
        self.approach_window = float(p('approach_window_s'))
        self.approach_bearing = math.radians(float(p('approach_bearing_deg')))
        self.add_on_set_parameters_callback(self._on_params)

        self.pan_pub = self.create_publisher(Float64, '/pca9685/pan/angle', 10)
        self.tilt_pub = self.create_publisher(Float64, '/pca9685/tilt/angle', 10)
        self.mover_pub = self.create_publisher(PointStamped, 'motion/mover', 10)
        self.say_pub = self.create_publisher(String, 'say', 10)
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd, 10)
        self.create_subscription(String, 'motion/mode', self._on_mode, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mode_pub = self.create_publisher(String, 'motion/mode_state', latched)

        self.last_cmd = 0.0                # monotonic time of the last non-zero command
        self.seen = deque()                # (time, bearing, range) of movers, for approach mode
        self.background = None             # per-beam range, learned while still
        self.learned = 0
        self.hits = 0                      # consecutive scans with a mover
        self.last_seen = 0.0
        self.tracking = False
        self.centred = True
        self.create_timer(0.5, self._housekeeping)
        self._publish_mode()
        self.get_logger().info(f'watching for movement while the robot stands still; mode {self.mode}')

    MODES = ('approach', 'watch', 'off')
    _LIVE = {'aim': 'aim', 'say': 'chatty', 'close_m': 'close_m', 'approach_by_m': 'approach_by',
             'approach_window_s': 'approach_window',
             'pan_center_deg': 'pan_c', 'pan_limit_deg': 'pan_lim', 'pan_sign': 'pan_sign',
             'tilt_center_deg': 'tilt_c', 'tilt_limit_deg': 'tilt_lim', 'tilt_sign': 'tilt_sign'}

    def _on_params(self, params) -> SetParametersResult:
        for q in params:
            if q.name == 'mode':
                if q.value not in self.MODES:
                    return SetParametersResult(successful=False, reason=f'mode must be one of {self.MODES}')
                self._set_mode(q.value)
                continue
            attr = self._LIVE.get(q.name)
            if attr:
                setattr(self, attr, bool(q.value) if attr in ('aim', 'chatty') else float(q.value))
                self.get_logger().info(f'{q.name} = {q.value}')
        return SetParametersResult(successful=True)

    # ---------------------------------------------------------------- input --

    def _on_cmd(self, msg: Twist) -> None:
        # Only throttle moves her. Steering alone (the settle node's wiggle after
        # a stop) turns the wheels in place; counting it as motion made her forget
        # the room and greet the same people with a fresh "huh" (2026-09-27).
        if msg.linear.x != 0.0 or msg.linear.y != 0.0:
            self.last_cmd = time.monotonic()

    def _on_mode(self, msg: String) -> None:
        mode = msg.data.strip().lower()
        if mode in self.MODES:
            self._set_mode(mode)
        else:
            self.get_logger().warning(f'unknown mode {msg.data!r}; one of {self.MODES}')

    def _set_mode(self, mode: str) -> None:
        if mode != self.mode:
            self.get_logger().info(f'mode {self.mode} -> {mode}')
        self.mode = mode
        self.seen.clear()
        if mode == 'off':
            self._centre()
        self._publish_mode()

    def _publish_mode(self) -> None:
        m = String(); m.data = self.mode
        self.mode_pub.publish(m)

    def _still(self) -> bool:
        return time.monotonic() - self.last_cmd > self.still_after

    def _on_scan(self, scan: LaserScan) -> None:
        if not self._still():
            self.background, self.learned, self.hits = None, 0, 0
            self.seen.clear()
            self._centre()
            return
        r = np.array(scan.ranges, dtype=np.float32)
        valid = np.isfinite(r) & (r >= scan.range_min) & (r <= min(scan.range_max, self.max_range))
        r = np.where(valid, r, np.nan)
        # For the background a beam that sees nothing is "far": a person stepping
        # into an open doorway is then closer than the room, as they should be.
        r_bg = np.where(valid, r, self.max_range)

        if self.background is None or len(self.background) != len(r):
            self.background, self.learned = r_bg.copy(), 1
            return
        if self.learned < self.learn_scans:
            # keep the larger of the two so a mover during learning does not stick
            self.background = np.maximum(self.background, r_bg)
            self.learned += 1
            return

        closer = (self.background - r) > self.closer_by       # NaNs compare false
        mover = self._cluster(closer, r, scan)
        # learn slowly from beams that are NOT moving, so the room can change
        quiet = ~closer
        self.background[quiet] += self.learn_rate * (r_bg[quiet] - self.background[quiet])

        if mover is None:
            self.hits = 0
            return
        self.hits += 1
        if self.hits < self.persist:
            return
        bearing, rng = mover
        if self.mode == 'watch':
            self._look(bearing, rng)
        elif self.mode == 'approach':
            self._approach(bearing, rng)

    def _approach(self, bearing: float, rng: float) -> None:
        """Approach mode: follow only someone who came right at her."""
        now = time.monotonic()
        self.seen.append((now, bearing, rng))
        while self.seen and now - self.seen[0][0] > self.approach_window:
            self.seen.popleft()
        if self.tracking and now - self.last_seen <= self.return_after:
            self._look(bearing, rng)                    # already noticed: keep following
            return
        if rng > self.close_m:
            return
        steady = [(t, r) for t, b, r in self.seen
                  if abs((b - bearing + math.pi) % (2 * math.pi) - math.pi) <= self.approach_bearing]
        t_far, r_far = max(steady, key=lambda tr: tr[1])
        if r_far - rng >= self.approach_by:
            self.get_logger().info(f'coming right at her: {r_far:.1f} m -> {rng:.1f} m in {now - t_far:.1f} s')
            self._look(bearing, rng)

    def _cluster(self, closer, r, scan):
        """The nearest run of adjacent 'closer' beams that looks like a body, as (bearing, range)."""
        n = len(closer)
        idx = np.flatnonzero(closer)
        if len(idx) < self.min_beams:
            return None
        # split into runs of consecutive indices (the scan wraps at 0/n)
        runs, run = [], [idx[0]]
        for a, b in zip(idx, idx[1:]):
            if b == a + 1:
                run.append(b)
            else:
                runs.append(run); run = [b]
        runs.append(run)
        if len(runs) > 1 and runs[0][0] == 0 and runs[-1][-1] == n - 1:
            runs[0] = runs[-1] + runs[0]; runs.pop()
        best = None
        for run in runs:
            if len(run) < self.min_beams or len(run) * scan.angle_increment > self.max_span:
                continue
            rng = float(np.nanmedian(r[run]))
            mid = run[len(run) // 2]
            bearing = scan.angle_min + mid * scan.angle_increment + self.yaw_off
            bearing = (bearing + math.pi) % (2 * math.pi) - math.pi
            if best is None or rng < best[1]:
                best = (bearing, rng)
        return best

    # --------------------------------------------------------------- output --

    def _look(self, bearing: float, rng: float) -> None:
        now = time.monotonic()
        new = not self.tracking or now - self.last_seen > self.return_after
        self.last_seen = now
        self.tracking = True
        self.centred = False
        pan = max(-self.pan_lim, min(self.pan_lim, math.degrees(bearing)))
        tilt = math.degrees(math.atan2(self.look_h - self.cam_h, max(rng, 0.3)))
        tilt = max(-self.tilt_lim, min(self.tilt_lim, tilt))
        self._servos(pan, tilt)
        m = PointStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.point.x, m.point.y, m.point.z = rng * math.cos(bearing), rng * math.sin(bearing), self.look_h
        self.mover_pub.publish(m)
        if new:
            self.get_logger().info(f'movement at {math.degrees(bearing):+.0f} deg, {rng:.1f} m: looking')
            self._say('huh')

    def _servos(self, pan_deg: float, tilt_deg: float) -> None:
        if not self.aim:
            return
        a = Float64(); a.data = self.pan_c + self.pan_sign * pan_deg
        b = Float64(); b.data = self.tilt_c + self.tilt_sign * tilt_deg
        self.pan_pub.publish(a)
        self.tilt_pub.publish(b)

    def _centre(self) -> None:
        if not self.centred:
            self._servos(0.0, 0.0)
            self.centred = True
            self.tracking = False

    def _say(self, mood: str) -> None:
        if self.chatty:
            m = String(); m.data = mood
            self.say_pub.publish(m)

    def _housekeeping(self) -> None:
        if self.tracking and time.monotonic() - self.last_seen > self.return_after:
            self.get_logger().info('nothing moving; back to centre')
            self._centre()


def main(args=None):
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=args)
    node = MotionWatch()
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
