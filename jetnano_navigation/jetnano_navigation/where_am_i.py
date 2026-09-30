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

"""Where am I? At boot, and whenever asked, Rosie finds herself on the saved map.

robot.launch.py runs this with where_am_i:=true. Once the stack has settled and she
is standing still, one lidar scan is matched against the whole saved map
(global_locate). Then:

    placed      a clear winner: localization is started there - jetnano-localize.service
                (slam_toolbox, start_pose = the answer) - or, if a slam_toolbox already
                runs in localization, it is told with /initialpose
    unsure      several places fit alike: nothing is started; searched again the moment
                she moves (25 cm or 20 degrees - driven, carried or put down), and again
                and again while she keeps moving, until one search places her
    not_on_map  nothing fits well (a room the map lacks): the same
    bench       the scan looks like the one remembered from the bench (~/remember_bench,
                places.py; Steve 2026-09-30): nothing is started, and nav_goal refuses to
                drive her; searched again the moment she moves, as above

A search takes ~7 s and she may be moving: the answer is her pose when the scan was
taken, so it is carried forward with the odometry before localization gets it.

The verdict, the best candidates and their scores go out on where_am_i/state (JSON,
latched) for the dashboard, which also calls ~/locate (std_srvs/Trigger) - its
WHERE AM I? button. accept_score and margin are provisional until measured on the
floor (2026-09-28: to be tested).

Stillness and "has she moved" come from the lidar odometry (8 Hz, cheap; set
odom_topic to odometry/filtered if the lidar odometry is off), followed only while
she is not placed. The lidar itself is subscribed only for the moment of a search:
reading every scan while waiting cost 2.5 % of a core for nothing (2026-09-28).
"""

import json
import math
import os
import subprocess
import threading
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
import tf2_ros

from jetnano_navigation import places
from jetnano_navigation.global_locate import judge, locate, MapModel

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def propagate(pose, odom_then, odom_now):
    """Her map pose now: ``pose`` (x, y, yaw) was her map pose when the odometry read
    ``odom_then`` (x, y, yaw); add the way she has moved since, as the odometry saw it."""
    x0, y0, a0 = odom_then
    x1, y1, a1 = odom_now
    c, s = math.cos(a0), math.sin(a0)
    lx, ly = c * (x1 - x0) + s * (y1 - y0), -s * (x1 - x0) + c * (y1 - y0)   # in her frame then
    mx, my, ma = pose
    cm, sm = math.cos(ma), math.sin(ma)
    return (mx + cm * lx - sm * ly, my + sm * lx + cm * ly,
            math.remainder(ma + a1 - a0, 2 * math.pi))


class WhereAmI(Node):

    def __init__(self):
        super().__init__('where_am_i')
        p = self.declare_parameter
        self.map_base = os.path.expanduser(p('map', '~/maps/home').value)
        self.accept = float(p('accept_score', 0.70).value)
        self.margin = float(p('margin', 1.15).value)
        self.auto = bool(p('auto', True).value)
        self.settle_s = float(p('settle_s', 25.0).value)       # after start, before the first try
        self.still_s = float(p('still_s', 2.0).value)
        # not placed yet: search again as soon as she has moved this much since the last
        # scan (Steve: "reevaluate the moment it starts moving")
        self.retry_m = float(p('retry_move_m', 0.25).value)
        self.retry_deg = float(p('retry_turn_deg', 20.0).value)
        self.follow_up_s = float(p('follow_up_s', 8.0).value)   # slam_toolbox up -> /initialpose
        self.start_localizer = bool(p('start_localizer', True).value)
        self.localize_unit = p('localize_unit', 'jetnano-localize').value
        self.mapping_unit = p('mapping_unit', 'jetnano-slam').value
        self.pose_file = os.path.expanduser(p('start_pose_file', '~/.ros/where_am_i.env').value)
        self.base_frame = p('base_frame', 'base_footprint').value
        odom_topic = p('odom_topic', 'lidar_odom').value
        # places that are not on the map, known by their lidar scan (places.py): the bench.
        # ~/remember_bench saves the scan she sees right now under "bench".
        self.places_file = os.path.expanduser(p('places_file', '~/maps/places.json').value)
        self.place_accept = float(p('place_accept', 0.6).value)

        self.model, self.model_mtime = None, None
        self.scan = None
        self.odom = None                 # (t, x, y, yaw)
        self.recent = deque(maxlen=64)   # the same, the last few seconds
        self.attempt_odom = None         # where she was at the last unplaced try
        self.done = False                # placed: nothing more to do unless asked
        self.busy = threading.Lock()
        self.t_start = time.monotonic()
        self.state = {'state': 'waiting', 'why': 'settling after start'}

        # only the static transforms (the lidar's mount): /tf itself runs at well over
        # 100 Hz and would cost this node more than everything else it does
        self.buf = tf2_ros.Buffer()
        self.create_subscription(TFMessage, '/tf_static', self._on_tf_static,
                                 QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pub_state = self.create_publisher(String, 'where_am_i/state', LATCHED)
        self.pub_initial = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.odom_topic, self.odom_sub = odom_topic, None
        self._follow_odom(True)
        self.create_service(Trigger, '~/locate', self._on_locate)
        self.create_service(Trigger, '~/remember_bench', self._on_remember_bench)
        self.create_timer(1.0, self._tick)
        self._publish()
        self.get_logger().info(
            f'map {self.map_base}; placed when a fit reaches {self.accept:.2f} and beats the '
            f'next place x{self.margin:.2f}; first try {self.settle_s:.0f} s after start')

    # ----------------------------------------------------------------- inputs --
    def _on_tf_static(self, msg) -> None:
        for t in msg.transforms:
            self.buf.set_transform_static(t, 'tf_static')

    def _follow_odom(self, on: bool) -> None:
        """Follow the odometry only while it matters (not placed yet)."""
        if on and self.odom_sub is None:
            self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self._on_odom, 10)
        elif not on and self.odom_sub is not None:
            self.destroy_subscription(self.odom_sub)
            self.odom_sub, self.odom = None, None
            self.recent.clear()

    def _fresh_scan(self, timeout: float = 3.0):
        """One lidar scan taken now: subscribed for the moment, then dropped."""
        got = threading.Event()

        def keep(m):
            self.scan = m
            got.set()
        self.scan = None
        sub = self.create_subscription(LaserScan, 'scan', keep, qos_profile_sensor_data)
        try:
            got.wait(timeout)
        finally:
            self.destroy_subscription(sub)
        return self.scan

    def _on_odom(self, m) -> None:
        p = m.pose.pose
        self.odom = (time.monotonic(), p.position.x, p.position.y, _yaw(p.orientation))
        self.recent.append(self.odom)

    def _still(self) -> bool:
        """Still = over the last still_s her pose moved < 3 cm and turned < 2 degrees.

        Not the odometry's speed: the lidar odometry's, differentiated from 8 Hz poses,
        reads 1-4 cm/s with her parked (2026-09-28), so a speed threshold never held.
        """
        if not self.recent or time.monotonic() - self.recent[-1][0] > 1.0:
            return False
        _, x, y, yaw = self.recent[-1]
        window = [r for r in self.recent if self.recent[-1][0] - r[0] <= self.still_s]
        if self.recent[-1][0] - window[0][0] < 0.8 * self.still_s:
            return False                 # not enough history yet
        return all(math.hypot(r[1] - x, r[2] - y) < 0.03
                   and abs(math.remainder(r[3] - yaw, 2 * math.pi)) < math.radians(2) for r in window)

    def _on_locate(self, request, response):
        if self.busy.locked():
            response.success, response.message = False, 'already searching'
            return response
        threading.Thread(target=self._run, args=('asked',), daemon=True).start()
        response.success = True
        response.message = 'searching' + ('' if self._still() or self.odom is None
                                         else ' (she is moving - the answer may be off)')
        return response

    def _on_remember_bench(self, request, response):
        """Save the scan she sees now as "bench" (she must be standing on it)."""
        if self.busy.locked():
            response.success, response.message = False, 'busy searching: try again in a moment'
            return response
        m = self._fresh_scan()
        if m is None:
            response.success, response.message = False, 'no lidar scan'
            return response
        old = places.load_places(self.places_file).get('bench')
        same = places.place_match(m.ranges, [np.nan if v is None else v for v in old['ranges']]) if old else None
        places.save_place(self.places_file, 'bench', m.ranges, 'remembered by ~/remember_bench')
        response.success = True
        response.message = (f'bench remembered from {len(m.ranges)} beams in {self.places_file}'
                            + (f' (matched the old bench scan {same:.2f})' if same is not None else ''))
        self.get_logger().info(response.message)
        return response

    def _known_place(self, scan):
        """(name, score) if the scan looks like a remembered place, else (None, score)."""
        known = places.load_places(self.places_file)
        if not known or scan is None:
            return None, 0.0
        name, score = places.best_place(scan.ranges, known)
        return (name if score >= self.place_accept else None), score

    # ------------------------------------------------------------------ logic --
    def _tick(self) -> None:
        if not self._load_map() or self.busy.locked() or self.done or not self.auto:
            return
        if time.monotonic() - self.t_start < self.settle_s:
            return
        if self.attempt_odom is None:
            # the first try: once she stands still (she boots parked)
            if self.odom is not None and not self._still():
                return
            why = 'boot'
        else:
            # not placed: the moment she moves, search again - and keep searching while
            # she moves (each try starts from where the last one's scan was taken)
            if self.odom is None:
                return
            _, x0, y0, yaw0 = self.attempt_odom
            moved = math.hypot(self.odom[1] - x0, self.odom[2] - y0)
            turned = abs(math.degrees(math.remainder(self.odom[3] - yaw0, 2 * math.pi)))
            if moved < self.retry_m and turned < self.retry_deg:
                return
            why = 'moved' if self._still() else 'moving'
        threading.Thread(target=self._run, args=(why,), daemon=True).start()

    def _load_map(self) -> bool:
        try:
            mtime = os.path.getmtime(self.map_base + '.yaml')
        except OSError:
            if self.state.get('state') != 'no_map':
                self._set({'state': 'no_map', 'why': f'no saved map at {self.map_base}.yaml'})
            return False
        if mtime != self.model_mtime:
            try:
                self.model, self.model_mtime = MapModel.load(self.map_base), mtime
            except (OSError, ValueError, KeyError, IndexError) as exc:
                self._set({'state': 'no_map', 'why': f'cannot read the map: {exc}'})
                return False
        return True

    def _scan_points(self):
        m = self._fresh_scan()
        if m is None or not self.buf.can_transform(self.base_frame, m.header.frame_id,
                                                   rclpy.time.Time()):
            return None
        t = self.buf.lookup_transform(self.base_frame, m.header.frame_id, rclpy.time.Time()).transform
        q = t.rotation
        lyaw = _yaw(q)
        r = np.asarray(m.ranges, dtype=np.float32)
        a = m.angle_min + np.arange(len(r)) * m.angle_increment
        ok = np.isfinite(r) & (r > m.range_min) & (r < min(m.range_max, 10.0))
        return np.stack([t.translation.x + r[ok] * np.cos(a[ok] + lyaw),
                         t.translation.y + r[ok] * np.sin(a[ok] + lyaw)], axis=1)

    def _run(self, why: str) -> None:
        if not self.busy.acquire(blocking=False):
            return
        try:
            if not self._load_map():
                return
            pts = self._scan_points()
            then = self.odom                 # her odometry pose when the scan was taken
            if pts is None:
                self._set({'state': 'waiting', 'why': 'no lidar scan or lidar transform yet'})
                return
            st = self.scan.header.stamp
            scan_time = st.sec + st.nanosec * 1e-9
            self._set({'state': 'searching', 'why': why})
            t0 = time.monotonic()
            # a remembered place first (the bench): cheap, and the map would only say "not
            # on the map" there
            place, place_score = self._known_place(self.scan)
            if place is not None:
                verdict, reason, cands = place, f'the lidar sees the {place} (scan match {place_score:.2f})', []
            else:
                cands = locate(self.model, pts)
                verdict, reason = judge(cands, self.accept, self.margin)
            took = time.monotonic() - t0
            result = {'state': verdict, 'why': reason, 'trigger': why, 'took_s': round(took, 1),
                      'scan_time': scan_time, 'place_score': round(place_score, 3),
                      'points': int(len(pts)), 'accept': self.accept, 'margin': self.margin,
                      'candidates': [{'score': round(c[0], 3), 'x': round(c[1], 2),
                                      'y': round(c[2], 2), 'yaw': round(c[3], 3)} for c in cands]}
            self.get_logger().info(
                f'{verdict} ({reason}) in {took:.1f} s from {len(pts)} points; candidates: '
                + '; '.join(f'{c[0]:.2f} at ({c[1]:+.2f}, {c[2]:+.2f}, '
                            f'{math.degrees(c[3]):.0f} deg)' for c in cands))
            if verdict == 'placed':
                self.done, self.attempt_odom = True, None
                result['action'] = self._act(cands[0], then)
            else:
                self.done = False
                self._follow_odom(True)
                if then is not None:
                    self.attempt_odom = then[:4]
                elif self.odom is not None:
                    self.attempt_odom = self.odom[:4]
                else:
                    self.attempt_odom = (0.0, 0.0, 0.0, 0.0)
                    self.auto = False            # nothing to tell a move by: wait to be asked
            self._set(result)
        except Exception as exc:  # noqa: B902 - a failed search must not kill the node
            self.get_logger().error(f'search failed: {exc!r}')
            self._set({'state': 'error', 'why': repr(exc)})
        finally:
            self.busy.release()

    def _propagate(self, pose, then):
        if then is None or self.odom is None:
            return pose
        return propagate(pose, then[1:4], self.odom[1:4])

    def _act(self, best, then) -> str:
        """Start localization at the answer (carried forward to now), or correct the one
        that runs. The odometry is followed until localization has been told."""
        found = tuple(best[1:4])
        if self._unit_active(self.mapping_unit):
            self._follow_odom(False)
            return f'mapping runs ({self.mapping_unit}): left alone'
        if self._unit_active(self.localize_unit) or self._slam_running():
            self._initial_pose(found, then)
            self._follow_odom(False)
            return 'told the running localization (/initialpose)'
        if not self.start_localizer:
            self._follow_odom(False)
            return 'found; start_localizer is off'
        x, y, yaw = self._propagate(found, then)
        os.makedirs(os.path.dirname(self.pose_file), exist_ok=True)
        with open(self.pose_file, 'w') as f:
            f.write(f'START_POSE={x:.3f},{y:.3f},{yaw:.4f}\n')
        try:
            r = subprocess.run(['sudo', '-n', 'systemctl', 'start', '--no-block', self.localize_unit],
                               capture_output=True, text=True, timeout=15)
        except (OSError, subprocess.TimeoutExpired) as exc:
            return f'could not start {self.localize_unit}: {exc}'
        if r.returncode != 0:
            self._follow_odom(False)
            return f'could not start {self.localize_unit}: {r.stderr.strip()[:120]}'
        threading.Thread(target=self._follow_up, args=(found, then, (x, y, yaw)), daemon=True).start()
        return f'started {self.localize_unit} there'

    def _follow_up(self, found, then, started_at) -> None:
        """slam_toolbox starts some seconds after the pose it was given was worked out;
        if she has moved meanwhile, tell it where she is now."""
        end = time.monotonic() + 60.0
        while time.monotonic() < end and not self._slam_running():
            time.sleep(1.0)
        time.sleep(self.follow_up_s)     # lifecycle configure/activate and a first scan
        now = self._propagate(found, then)
        if (math.hypot(now[0] - started_at[0], now[1] - started_at[1]) > 0.05
                or abs(math.remainder(now[2] - started_at[2], 2 * math.pi)) > math.radians(2)):
            self.get_logger().info('she moved while localization started: /initialpose '
                                   f'({now[0]:+.2f}, {now[1]:+.2f}, {math.degrees(now[2]):.0f} deg)')
            self._initial_pose(found, then)
        self._follow_odom(False)

    def _unit_active(self, unit: str) -> bool:
        try:
            return subprocess.run(['systemctl', 'is-active', '--quiet', unit],
                                  timeout=5).returncode == 0
        except (OSError, subprocess.TimeoutExpired):
            return False

    def _slam_running(self) -> bool:
        return any(name == 'slam_toolbox' for name, _ in self.get_node_names_and_namespaces())

    def _initial_pose(self, found, then) -> None:
        """/initialpose three times (slam_toolbox has missed a single one), each carried
        forward to the moment it is sent."""
        for _ in range(3):
            x, y, yaw = self._propagate(found, then)
            m = PoseWithCovarianceStamped()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.pose.pose.position.x, m.pose.pose.position.y = x, y
            m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            m.pose.covariance[0] = m.pose.covariance[7] = 0.05 ** 2
            m.pose.covariance[35] = math.radians(3) ** 2
            self.pub_initial.publish(m)
            time.sleep(1.0)

    def _set(self, state: dict) -> None:
        state['time'] = time.time()
        self.state = state
        self._publish()

    def _publish(self) -> None:
        self.pub_state.publish(String(data=json.dumps(self.state)))


def main(args=None):
    rclpy.init(args=args)
    node = WhereAmI()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
