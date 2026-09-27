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

"""Everything Rosie knows, gathered for the iPad dashboard (web_teleop, /dash).

Steve, 2026-09-27: an iPad page with every camera (the rear one as a rear-view
mirror), every sensor and all she knows - no driving on it, but a STOP. This
collects it on the web_teleop node (so the STOP is the page's own e_stop_web
lock, shared with the phone) and hands it out as one JSON document plus the
SLAM map as a PNG:

* the watchdog's status (every stream's rate, the machine), the battery,
  the collision guard, every stop lock, motion_check, the motion_watch mode;
* the lidar (one range per degree, in her own frame) and nvblox's obstacle
  points around her, for the radar; her speed, heading and command;
* what she heard, who said it, what she said and played, the room's level,
  the steering buzz; a rolling log of all of that;
* CPU, GPU, temperatures, memory; the map and where she is on it.

Nothing here is needed for driving: a failure only blanks a tile.
"""

from __future__ import annotations

import json
import math
import os
import struct
import threading
import time
import zlib
from collections import deque

import numpy as np
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Bool, Float32, String

try:
    from sensor_msgs_py import point_cloud2 as pc2
except ImportError:  # pragma: no cover
    pc2 = None
try:
    import tf2_ros
except ImportError:  # pragma: no cover
    tf2_ros = None

LOCKS = ('e_stop', 'e_stop_web', 'e_stop_joy', 'e_stop_motion')
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
GPU_LOAD = '/sys/devices/platform/bus@0/17000000.gpu/load'


def _yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def png_indexed(width: int, height: int, rows: bytes, palette: bytes) -> bytes:
    """A palette PNG from raw 8-bit index rows (top row first). Standard library only."""
    def chunk(kind: bytes, data: bytes) -> bytes:
        return struct.pack('>I', len(data)) + kind + data + struct.pack('>I', zlib.crc32(kind + data) & 0xffffffff)
    raw = b''.join(b'\x00' + rows[y * width:(y + 1) * width] for y in range(height))
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 3, 0, 0, 0))
            + chunk(b'PLTE', palette) + chunk(b'tRNS', b'\x00') + chunk(b'IDAT', zlib.compress(raw, 6))
            + chunk(b'IEND', b''))


# unknown (transparent), free, occupied - drawn over the page's dark background
MAP_PALETTE = bytes([0, 0, 0, 40, 58, 84, 110, 255, 240])


class Dashboard:
    """Subscriptions on the web_teleop node; snapshot() is called from HTTP threads."""

    def __init__(self, node):
        self.node = node
        node.declare_parameter('dash_scan_yaw_offset', math.pi)    # the lidar is mounted backwards
        self.lock = threading.Lock()
        now = time.monotonic()
        self.t0 = now
        self.watchdog, self.watchdog_t = None, 0.0
        self.locks = {k: False for k in LOCKS}
        self.motion_check = 'ok'
        self.mode = None
        self.mover, self.mover_t = None, 0.0
        self.odom, self.odom_t = None, 0.0
        self.cmd, self.cmd_t = (0.0, 0.0), 0.0
        self.scan, self.scan_t = None, 0.0
        self.obstacles, self.obstacles_t = [], 0.0
        self.heard, self.speaker, self.said, self.sound = None, None, None, None
        self.speech_state = None
        self.level, self.buzz = None, None
        self.log = deque(maxlen=40)
        self.map_png, self.map_info, self.map_version = None, None, 0
        self._cpu_prev = None
        self.sys = {}
        self._pc_busy = False

        sub = node.create_subscription
        sub(String, 'watchdog/status', self._on_watchdog, LATCHED)
        for k in LOCKS:
            sub(Bool, k, (lambda name: lambda m: self._on_lock(name, m.data))(k), 10)
        sub(String, 'motion_check/state', self._on_motion_check, 10)
        sub(String, 'motion/mode_state', lambda m: setattr(self, 'mode', m.data), LATCHED)
        sub(PointStamped, 'motion/mover', self._on_mover, 10)
        sub(Odometry, 'odometry/filtered', self._on_odom, 10)
        sub(Twist, 'cmd_vel', self._on_cmd, 10)
        sub(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        sub(PointCloud2, '/nvblox_node/obstacle_points', self._on_points, 2)
        sub(String, 'speech/text', lambda m: self._event('heard', m.data, 'heard'), 10)
        sub(String, 'speech/speaker', self._on_speaker, 10)
        sub(String, 'speech/state', lambda m: setattr(self, 'speech_state', m.data), 10)
        sub(String, 'speak', lambda m: self._event('said', m.data.replace('\n', ' '), 'said'), 10)
        sub(String, 'say', self._on_sound, 10)
        sub(Float32, 'sound/level', lambda m: setattr(self, 'level', round(m.data, 1)), 10)
        sub(Float32, 'steering/buzz', lambda m: setattr(self, 'buzz', round(m.data, 1)), 10)
        sub(OccupancyGrid, 'map', self._on_map, LATCHED)
        self.tf = None
        if tf2_ros is not None:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf = tf2_ros.TransformListener(self.tf_buffer, node)
        node.create_timer(2.0, self._system)

    # ------------------------------------------------------------ inputs --
    def _event(self, kind: str, text: str, attr: str | None = None) -> None:
        with self.lock:
            if attr:
                setattr(self, attr, text)
            self.log.appendleft({'t': time.time(), 'kind': kind, 'text': text[:240]})

    def _on_watchdog(self, msg) -> None:
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        old = set((self.watchdog or {}).get('problems', []))
        for p in data.get('problems', []):
            if p not in old:
                self._event('problem', p)
        self.watchdog, self.watchdog_t = data, time.monotonic()

    def _on_lock(self, name: str, value: bool) -> None:
        if value != self.locks.get(name):
            self._event('stop' if value else 'go', f'{name} {"raised" if value else "cleared"}')
        self.locks[name] = value

    def _on_motion_check(self, msg) -> None:
        self.motion_check = msg.data
        if msg.data != 'ok':
            self._event('stop', 'motion check: ' + msg.data)

    def _on_mover(self, msg) -> None:
        self.mover = (round(msg.point.x, 2), round(msg.point.y, 2))
        if time.monotonic() - self.mover_t > 8.0:
            b = math.degrees(math.atan2(msg.point.y, msg.point.x))
            self._event('mover', f'someone moving at {b:+.0f} deg, {math.hypot(*self.mover):.1f} m')
        self.mover_t = time.monotonic()

    def _on_odom(self, msg) -> None:
        self.odom, self.odom_t = msg, time.monotonic()

    def _on_cmd(self, msg) -> None:
        self.cmd, self.cmd_t = (round(msg.linear.x, 3), round(msg.angular.z, 3)), time.monotonic()

    def _on_scan(self, msg) -> None:
        # one range per degree of her own frame (0 = nose, left positive); the lidar is mounted backwards
        off = float(self.node.get_parameter('dash_scan_yaw_offset').value)
        bins = [None] * 360
        for i, r in enumerate(msg.ranges):
            if not (msg.range_min < r < msg.range_max):
                continue
            a = math.degrees(msg.angle_min + i * msg.angle_increment + off)
            k = int(round(a)) % 360
            if bins[k] is None or r < bins[k]:
                bins[k] = round(r, 2)
        self.scan, self.scan_t = bins, time.monotonic()

    def _on_points(self, msg) -> None:
        if pc2 is None or self.odom is None or self._pc_busy:
            return
        self._pc_busy = True
        try:
            p = self.odom.pose.pose
            yaw = _yaw(p.orientation)
            c, s = math.cos(-yaw), math.sin(-yaw)
            out = []
            for x, y, _z in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
                dx, dy = x - p.position.x, y - p.position.y
                if dx * dx + dy * dy < 25.0:
                    out.append((round(c * dx - s * dy, 2), round(s * dx + c * dy, 2)))
            self.obstacles, self.obstacles_t = out[::max(1, len(out) // 500)], time.monotonic()
        finally:
            self._pc_busy = False

    def _on_speaker(self, msg) -> None:
        try:
            self.speaker = json.loads(msg.data)
        except ValueError:
            pass

    def _on_sound(self, msg) -> None:
        name = os.path.basename(msg.data).rsplit('.', 1)[0] if msg.data.endswith('.wav') else msg.data
        self._event('sound', name, 'sound')

    def _on_map(self, msg) -> None:
        w, h = msg.info.width, msg.info.height
        grid = np.asarray(msg.data, dtype=np.int8).reshape(h, w)[::-1]     # the grid's origin is bottom-left
        idx = np.where(grid < 0, 0, np.where(grid >= 65, 2, 1)).astype(np.uint8)
        png = png_indexed(w, h, idx.tobytes(), MAP_PALETTE)
        o = msg.info.origin.position
        with self.lock:
            self.map_png = png
            self.map_version += 1
            self.map_info = {'w': w, 'h': h, 'res': msg.info.resolution, 'x0': o.x, 'y0': o.y,
                             'yaw0': _yaw(msg.info.origin.orientation), 'v': self.map_version}

    # ------------------------------------------------------------ system --
    def _system(self) -> None:
        s = {}
        try:
            with open('/proc/stat') as f:
                v = [int(x) for x in f.readline().split()[1:]]
            idle, total = v[3] + v[4], sum(v)
            if self._cpu_prev:
                di, dt = idle - self._cpu_prev[0], total - self._cpu_prev[1]
                s['cpu'] = round(100.0 * (1 - di / dt)) if dt else None
            self._cpu_prev = (idle, total)
        except (OSError, ValueError):
            pass
        try:
            with open(GPU_LOAD) as f:
                s['gpu'] = round(int(f.read().strip()) / 10)
        except (OSError, ValueError):
            pass
        try:
            for zone in os.listdir('/sys/class/thermal'):
                if not zone.startswith('thermal_zone'):
                    continue
                with open(f'/sys/class/thermal/{zone}/type') as f:
                    kind = f.read().strip().replace('-thermal', '')
                with open(f'/sys/class/thermal/{zone}/temp') as f:
                    t = int(f.read().strip()) / 1000.0
                if -40 < t < 150:
                    s.setdefault('temps', {})[kind] = round(t, 1)
            with open('/proc/meminfo') as f:
                mem = {ln.split(':')[0]: int(ln.split()[1]) for ln in f}
            s['mem_used_pct'] = round(100 * (1 - mem['MemAvailable'] / mem['MemTotal']))
            with open('/proc/uptime') as f:
                s['uptime_s'] = int(float(f.read().split()[0]))
        except (OSError, ValueError, KeyError):
            pass
        self.sys = s

    # ------------------------------------------------------------ output --
    def _pose_on_map(self):
        if self.tf is None or self.map_info is None:
            return None
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy_time_zero())
        except Exception:  # noqa: B902 - no map yet, or no transform
            return None
        tr = t.transform
        return {'x': round(tr.translation.x, 2), 'y': round(tr.translation.y, 2), 'yaw': round(_yaw(tr.rotation), 3)}

    def snapshot(self, page_status: dict) -> dict:
        now = time.monotonic()
        o = self.odom if now - self.odom_t < 2.0 else None
        motion = None
        if o is not None:
            tw = o.twist.twist
            motion = {'speed': round(math.hypot(tw.linear.x, tw.linear.y), 2), 'turn': round(tw.angular.z, 2),
                      'heading': round(math.degrees(_yaw(o.pose.pose.orientation))),
                      'x': round(o.pose.pose.position.x, 2), 'y': round(o.pose.pose.position.y, 2)}
        wd = self.watchdog if now - self.watchdog_t < 15.0 else None
        with self.lock:
            log = list(self.log)[:25]
            heard, said, sound = self.heard, self.said, self.sound
            map_info = dict(self.map_info) if self.map_info else None
        return {
            'page': page_status,
            'watchdog': wd,
            'locks': dict(self.locks),
            'motion_check': self.motion_check,
            'mode': self.mode,
            'mover': {'x': self.mover[0], 'y': self.mover[1], 'age': round(now - self.mover_t, 1)} if self.mover else None,
            'motion': motion,
            'cmd': self.cmd if now - self.cmd_t < 1.0 else (0.0, 0.0),
            'scan': self.scan if now - self.scan_t < 2.0 else None,
            'obstacles': self.obstacles if now - self.obstacles_t < 3.0 else [],
            'speech': {'heard': heard, 'said': said, 'sound': sound, 'state': self.speech_state,
                       'speaker': self.speaker},
            'level': self.level, 'buzz': self.buzz,
            'system': {**(wd or {}).get('system', {}), **self.sys},
            'map': map_info, 'pose': self._pose_on_map(),
            'log': log,
            'time': time.time(),
        }

    def map_image(self):
        with self.lock:
            return self.map_png


def rclpy_time_zero():
    from rclpy.time import Time
    return Time()
