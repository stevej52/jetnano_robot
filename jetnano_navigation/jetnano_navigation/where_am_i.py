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
    unsure      several places fit alike: nothing is started; tried again once she has
                driven a metre or turned 60 degrees and stopped
    not_on_map  nothing fits well (the bench, a room the map lacks): the same

The verdict, the best candidates and their scores go out on where_am_i/state (JSON,
latched) for the dashboard, which also calls ~/locate (std_srvs/Trigger) - its
WHERE AM I? button. accept_score and margin are provisional until measured on the
floor (2026-09-28: to be tested).

Stillness and "has she moved" come from the lidar odometry (8 Hz, cheap); set
odom_topic to odometry/filtered if the lidar odometry is off.
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

from jetnano_navigation.global_locate import judge, locate, MapModel

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


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
        self.retry_m = float(p('retry_move_m', 1.0).value)
        self.retry_deg = float(p('retry_turn_deg', 60.0).value)
        self.start_localizer = bool(p('start_localizer', True).value)
        self.localize_unit = p('localize_unit', 'jetnano-localize').value
        self.mapping_unit = p('mapping_unit', 'jetnano-slam').value
        self.pose_file = os.path.expanduser(p('start_pose_file', '~/.ros/where_am_i.env').value)
        self.base_frame = p('base_frame', 'base_footprint').value
        odom_topic = p('odom_topic', 'lidar_odom').value

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
        self.create_subscription(LaserScan, 'scan', lambda m: setattr(self, 'scan', m),
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_service(Trigger, '~/locate', self._on_locate)
        self.create_timer(1.0, self._tick)
        self._publish()
        self.get_logger().info(
            f'map {self.map_base}; placed when a fit reaches {self.accept:.2f} and beats the '
            f'next place x{self.margin:.2f}; first try {self.settle_s:.0f} s after start')

    # ----------------------------------------------------------------- inputs --
    def _on_tf_static(self, msg) -> None:
        for t in msg.transforms:
            self.buf.set_transform_static(t, 'tf_static')

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

    # ------------------------------------------------------------------ logic --
    def _tick(self) -> None:
        if not self._load_map() or self.busy.locked() or self.done or not self.auto:
            return
        if time.monotonic() - self.t_start < self.settle_s or self.scan is None:
            return
        if self.odom is not None and not self._still():
            return
        if self.attempt_odom is not None:
            if self.odom is None:
                return
            _, x0, y0, yaw0 = self.attempt_odom
            moved = math.hypot(self.odom[1] - x0, self.odom[2] - y0)
            turned = abs(math.degrees(math.remainder(self.odom[3] - yaw0, 2 * math.pi)))
            if moved < self.retry_m and turned < self.retry_deg:
                return
        threading.Thread(target=self._run, args=('boot' if self.attempt_odom is None else 'moved',),
                         daemon=True).start()

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
        m = self.scan
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
            if pts is None:
                self._set({'state': 'waiting', 'why': 'no lidar scan or lidar transform yet'})
                return
            self._set({'state': 'searching', 'why': why})
            t0 = time.monotonic()
            cands = locate(self.model, pts)
            verdict, reason = judge(cands, self.accept, self.margin)
            took = time.monotonic() - t0
            result = {'state': verdict, 'why': reason, 'trigger': why, 'took_s': round(took, 1),
                      'points': int(len(pts)), 'accept': self.accept, 'margin': self.margin,
                      'candidates': [{'score': round(c[0], 3), 'x': round(c[1], 2),
                                      'y': round(c[2], 2), 'yaw': round(c[3], 3)} for c in cands]}
            self.get_logger().info(
                f'{verdict} ({reason}) in {took:.1f} s from {len(pts)} points; candidates: '
                + '; '.join(f'{c[0]:.2f} at ({c[1]:+.2f}, {c[2]:+.2f}, '
                            f'{math.degrees(c[3]):.0f} deg)' for c in cands))
            if verdict == 'placed':
                result['action'] = self._act(cands[0])
                self.done, self.attempt_odom = True, None
            else:
                self.done = False
                if self.odom is not None:
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

    def _act(self, best) -> str:
        """Start localization at the answer, or correct the one that runs."""
        _, x, y, yaw = best
        if self._unit_active(self.mapping_unit):
            return f'mapping runs ({self.mapping_unit}): left alone'
        if self._unit_active(self.localize_unit) or self._slam_running():
            self._initial_pose(x, y, yaw)
            return 'told the running localization (/initialpose)'
        if not self.start_localizer:
            return 'found; start_localizer is off'
        os.makedirs(os.path.dirname(self.pose_file), exist_ok=True)
        with open(self.pose_file, 'w') as f:
            f.write(f'START_POSE={x:.3f},{y:.3f},{yaw:.4f}\n')
        try:
            r = subprocess.run(['sudo', '-n', 'systemctl', 'start', '--no-block', self.localize_unit],
                               capture_output=True, text=True, timeout=15)
        except (OSError, subprocess.TimeoutExpired) as exc:
            return f'could not start {self.localize_unit}: {exc}'
        if r.returncode != 0:
            return f'could not start {self.localize_unit}: {r.stderr.strip()[:120]}'
        return f'started {self.localize_unit} there'

    def _unit_active(self, unit: str) -> bool:
        try:
            return subprocess.run(['systemctl', 'is-active', '--quiet', unit],
                                  timeout=5).returncode == 0
        except (OSError, subprocess.TimeoutExpired):
            return False

    def _slam_running(self) -> bool:
        return any(name == 'slam_toolbox' for name, _ in self.get_node_names_and_namespaces())

    def _initial_pose(self, x: float, y: float, yaw: float) -> None:
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        m.pose.covariance[0] = m.pose.covariance[7] = 0.05 ** 2
        m.pose.covariance[35] = math.radians(3) ** 2
        for _ in range(3):              # slam_toolbox has missed a single one before
            m.header.stamp = self.get_clock().now().to_msg()
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
