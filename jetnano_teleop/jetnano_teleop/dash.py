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
* CPU, GPU, temperatures, memory; the map and where she is on it. The map is
  slam_toolbox's live /map while one comes in, else the last saved map
  (``dash_map``, map_saver's .yaml + .pgm, reloaded when it is saved again).

Nothing here is needed for driving: a failure only blanks a tile.

Cost: standing Python subscriptions to fast streams are expensive (a first
version cost 64 % of a core, 2026-09-27), so the lidar, the camera's cloud, the
odometry are subscribed only while a dashboard has asked for data in the last
``LIVE_S`` seconds, read with numpy, and the two fastest (the EKF at 100 Hz,
nvblox's points at 40 Hz) are taken raw and decoded only as often as the page
can use them. The map pose comes from slam_toolbox's own ``/pose`` (published
when it matches a scan) carried forward with the EKF, not from /tf (well over
100 Hz - too dear to read in Python for a picture).
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
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger

try:
    from sensor_msgs_py import point_cloud2 as pc2
except ImportError:  # pragma: no cover
    pc2 = None
from geometry_msgs.msg import PoseWithCovarianceStamped

LOCKS = ('e_stop', 'e_stop_web', 'e_stop_joy', 'e_stop_motion')
LIVE_S = 6.0            # keep the fast subscriptions this long after the last dashboard poll
LIVE_MAP_S = 60.0       # a live SLAM map this recent wins over the saved one
# The page polls 4 times a second; the EKF publishes at 100 Hz and nvblox's obstacle
# points at 40 Hz. Received raw, decoded no oftener than this (2026-09-27: decoding
# every message kept the web server at ~50 % of a core with the page open).
ODOM_EVERY_S = 0.05
POINTS_EVERY_S = 0.2
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


def read_map_yaml(path: str) -> dict:
    """map_saver's .yaml: flat 'key: value' lines, origin as [x, y, yaw]."""
    meta = {}
    with open(path) as f:
        for line in f:
            key, _, value = line.partition(':')
            key, value = key.strip(), value.split('#', 1)[0].strip()
            if not key or not value:
                continue
            if value.startswith('['):
                meta[key] = [float(x) for x in value.strip('[]').split(',')]
            else:
                try:
                    meta[key] = float(value)
                except ValueError:
                    meta[key] = value
    return meta


def read_pgm(path: str) -> np.ndarray:
    """An 8-bit binary (P5) PGM as a height x width array, top row first."""
    with open(path, 'rb') as f:
        data = f.read()
    tokens, i = [], 0
    while len(tokens) < 4:
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            i = data.index(b'\n', i) + 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        tokens.append(data[i:j])
        i = j
    if tokens[0] != b'P5' or int(tokens[3]) > 255:
        raise ValueError('not an 8-bit binary PGM')
    w, h = int(tokens[1]), int(tokens[2])
    return np.frombuffer(data, dtype=np.uint8, count=w * h, offset=i + 1).reshape(h, w)


def saved_map_indices(pixels: np.ndarray, meta: dict) -> np.ndarray:
    """Palette indices (0 unknown, 1 free, 2 occupied) as map_server reads the image."""
    p = pixels.astype(np.float32) / 255.0
    occupied = p if int(meta.get('negate', 0)) else 1.0 - p
    return np.where(occupied > float(meta.get('occupied_thresh', 0.65)), 2,
                    np.where(occupied < float(meta.get('free_thresh', 0.196)), 1, 0)).astype(np.uint8)


class Dashboard:
    """Subscriptions on the web_teleop node; snapshot() is called from HTTP threads."""

    def __init__(self, node):
        self.node = node
        node.declare_parameter('dash_scan_yaw_offset', math.pi)    # the lidar is mounted backwards
        # the saved map (map_saver's <base>.yaml + image), shown whenever no SLAM runs
        node.declare_parameter('dash_map', '~/maps/home')
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
        self.partial, self.partial_until = '', 0.0      # what he has said so far, while she waits for the rest
        self.level, self.buzz = None, None
        self.log = deque(maxlen=40)
        self.map_png, self.map_info, self.map_version, self.map_t = None, None, 0, 0.0
        self.saved_png, self.saved_info, self.saved_mtime = None, None, None
        self._odom_decoded = self._points_decoded = 0.0
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
        sub(Twist, 'cmd_vel', self._on_cmd, 10)
        sub(String, 'speech/text', lambda m: self._event('heard', m.data, 'heard'), 10)
        sub(String, 'speech/speaker', self._on_speaker, 10)
        sub(String, 'speech/partial', self._on_partial, 10)
        sub(String, 'speech/state', lambda m: setattr(self, 'speech_state', m.data), 10)
        sub(String, 'speak', lambda m: self._event('said', m.data.replace('\n', ' '), 'said'), 10)
        sub(String, 'say', self._on_sound, 10)
        sub(Float32, 'sound/level', lambda m: setattr(self, 'level', round(m.data, 1)), 10)
        sub(Float32, 'steering/buzz', lambda m: setattr(self, 'buzz', round(m.data, 1)), 10)
        sub(OccupancyGrid, 'map', self._on_map, LATCHED)
        # where_am_i's verdict on where she is (JSON), and its WHERE AM I? service
        self.wami = None
        sub(String, 'where_am_i/state', self._on_wami, LATCHED)
        self._locate = node.create_client(Trigger, 'where_am_i/locate')
        self.map_odom = None                 # map -> odom (x, y, yaw), from slam_toolbox's /pose
        self.pose_t = 0.0                    # when the last /pose came
        self.pending_pose = None             # a /pose that came while nobody watched
        self.odom_hist = deque(maxlen=120)   # (stamp, x, y, yaw) of the EKF, ~6 s at 20 Hz
        # slam_toolbox publishes /pose only after she has moved, so listen all the time
        # (a few messages a minute) - a page opened while she is parked still has a fix
        sub(PoseWithCovarianceStamped, '/pose', self._on_pose, 5)
        self._load_saved_map()
        self.polled = 0.0                    # last dashboard request
        self.live = []                       # the fast subscriptions, while watched
        node.create_timer(2.0, self._system)
        node.create_timer(1.0, self._manage)

    def _manage(self) -> None:
        """Subscribe to the fast streams only while someone is looking."""
        watched = time.monotonic() - self.polled < LIVE_S
        if watched and not self.live:
            sub = self.node.create_subscription
            self.live = [sub(Odometry, 'odometry/filtered', self._on_odom_raw, 5, raw=True),
                         sub(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data),
                         sub(PointCloud2, '/nvblox_node/obstacle_points', self._on_points_raw, 1,
                             raw=True)]
            self.node.get_logger().info('dashboard: someone is watching, following the sensors')
        elif not watched and self.live:
            for handle in self.live:
                self.node.destroy_subscription(handle)
            self.live = []
            self.odom_hist.clear()       # stale once nobody follows the odometry
            self.node.get_logger().info('dashboard: nobody watching, sensors dropped')

    def _on_pose(self, msg) -> None:
        """slam_toolbox's pose on the map, paired with the EKF's odom pose of the
        same moment: that is map -> odom until the next one."""
        self.pose_t = time.monotonic()
        if not self.odom_hist:
            # nobody watching, so no odometry to pair it with yet: the first odometry
            # after the page opens will do (she has not moved since, or slam_toolbox
            # would have sent a newer pose)
            self.pending_pose = msg
            return
        self.pending_pose = None
        self._pair(msg)

    def _pair(self, msg) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        _, ox, oy, oyaw = min(self.odom_hist, key=lambda h: abs(h[0] - t))
        p = msg.pose.pose
        yaw = _yaw(p.orientation) - oyaw
        c, s = math.cos(yaw), math.sin(yaw)
        self.map_odom = (p.position.x - (c * ox - s * oy), p.position.y - (s * ox + c * oy), yaw)

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

    def _on_odom_raw(self, data: bytes) -> None:
        now = time.monotonic()
        if now - self._odom_decoded < ODOM_EVERY_S:
            return
        self._odom_decoded = now
        self._on_odom(deserialize_message(data, Odometry))

    def _on_odom(self, msg) -> None:
        self.odom, self.odom_t = msg, time.monotonic()
        p = msg.pose.pose
        self.odom_hist.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                               p.position.x, p.position.y, _yaw(p.orientation)))
        if self.pending_pose is not None:
            self._pair(self.pending_pose)
            self.pending_pose = None

    def _on_cmd(self, msg) -> None:
        self.cmd, self.cmd_t = (round(msg.linear.x, 3), round(msg.angular.z, 3)), time.monotonic()

    def _on_scan(self, msg) -> None:
        # one range per degree of her own frame (0 = nose, left positive); the lidar is mounted backwards
        off = float(self.node.get_parameter('dash_scan_yaw_offset').value)
        r = np.asarray(msg.ranges, dtype=np.float32)
        a = np.degrees(msg.angle_min + np.arange(len(r)) * msg.angle_increment + off)
        ok = (r > msg.range_min) & (r < msg.range_max)
        k = np.round(a[ok]).astype(int) % 360
        near = np.full(360, np.inf, dtype=np.float32)
        np.minimum.at(near, k, r[ok])
        self.scan = [None if not np.isfinite(v) else round(float(v), 2) for v in near]
        self.scan_t = time.monotonic()

    def _on_points_raw(self, data: bytes) -> None:
        now = time.monotonic()
        if now - self._points_decoded < POINTS_EVERY_S:
            return
        self._points_decoded = now
        self._on_points(deserialize_message(data, PointCloud2))

    def _on_points(self, msg) -> None:
        if pc2 is None or self.odom is None or self._pc_busy:
            return
        self._pc_busy = True
        try:
            p = self.odom.pose.pose
            yaw = _yaw(p.orientation)
            c, s = math.cos(-yaw), math.sin(-yaw)
            xy = pc2.read_points_numpy(msg, field_names=('x', 'y'), skip_nans=True).astype(np.float32)
            dx, dy = xy[:, 0] - p.position.x, xy[:, 1] - p.position.y
            near = dx * dx + dy * dy < 25.0
            rx, ry = c * dx[near] - s * dy[near], s * dx[near] + c * dy[near]
            step = max(1, len(rx) // 500)
            # float64 before rounding: float32's 3.74 prints as 3.740000009536743, and that
            # was 36 of the 43 KB the page fetched 4 times a second (measured 2026-09-28)
            xy = np.stack([rx[::step], ry[::step]], axis=1).astype(np.float64)
            self.obstacles = np.round(xy, 2).tolist()
            self.obstacles_t = time.monotonic()
        finally:
            self._pc_busy = False

    def _on_partial(self, msg) -> None:
        """listen is waiting for the rest of a sentence (JSON text, hold_s): 'Listening' on the page."""
        try:
            d = json.loads(msg.data)
            self.partial = str(d.get('text', ''))
            self.partial_until = time.monotonic() + float(d.get('hold_s', 0.0)) + 1.0 if self.partial else 0.0
        except (ValueError, TypeError):
            pass

    def _on_speaker(self, msg) -> None:
        try:
            self.speaker = json.loads(msg.data)
        except ValueError:
            pass

    def _on_sound(self, msg) -> None:
        name = os.path.basename(msg.data).rsplit('.', 1)[0] if msg.data.endswith('.wav') else msg.data
        self._event('sound', name, 'sound')

    def _on_wami(self, msg) -> None:
        try:
            self.wami = json.loads(msg.data)
        except ValueError:
            self.wami = None
            return
        # a clear match is a fix of its own: slam_toolbox, started there, says nothing
        # on /pose until she moves
        c = self.wami.get('candidates') or []
        if self.wami.get('state') == 'placed' and c and self.wami.get('time'):
            fix = PoseWithCovarianceStamped()
            t = float(self.wami.get('scan_time') or self.wami['time'])   # her pose at the scan
            fix.header.stamp.sec, fix.header.stamp.nanosec = int(t), int((t % 1) * 1e9)
            fix.header.frame_id = 'map'
            fix.pose.pose.position.x, fix.pose.pose.position.y = c[0]['x'], c[0]['y']
            fix.pose.pose.orientation.z = math.sin(c[0]['yaw'] / 2)
            fix.pose.pose.orientation.w = math.cos(c[0]['yaw'] / 2)
            self._on_pose(fix)

    def locate(self):
        """Ask where_am_i to search now; (sent, why). The answer comes on where_am_i/state."""
        if not self._locate.service_is_ready():
            return False, 'where_am_i is not running (robot.launch.py where_am_i:=true)'
        self._locate.call_async(Trigger.Request())
        return True, 'searching'

    def _on_map(self, msg) -> None:
        w, h = msg.info.width, msg.info.height
        grid = np.asarray(msg.data, dtype=np.int8).reshape(h, w)[::-1]     # the grid's origin is bottom-left
        idx = np.where(grid < 0, 0, np.where(grid >= 65, 2, 1)).astype(np.uint8)
        png = png_indexed(w, h, idx.tobytes(), MAP_PALETTE)
        o = msg.info.origin.position
        with self.lock:
            self.map_png = png
            self.map_version += 1
            self.map_t = time.monotonic()
            self.map_info = {'w': w, 'h': h, 'res': msg.info.resolution, 'x0': o.x, 'y0': o.y,
                             'yaw0': _yaw(msg.info.origin.orientation),
                             'v': f'L{self.map_version}', 'src': 'live'}

    def _load_saved_map(self) -> None:
        """The last saved map, reloaded whenever its image changes (a new save)."""
        base = os.path.expanduser(str(self.node.get_parameter('dash_map').value))
        try:
            meta = read_map_yaml(base + '.yaml')
            image = os.path.join(os.path.dirname(base), str(meta['image']))
            mtime = os.path.getmtime(image)
        except (OSError, KeyError, ValueError):
            return                           # no saved map (yet): the page says so
        if mtime == self.saved_mtime:
            return
        try:
            pixels = read_pgm(image)
            origin = meta['origin']
            h, w = pixels.shape
            png = png_indexed(w, h, saved_map_indices(pixels, meta).tobytes(), MAP_PALETTE)
            info = {'w': w, 'h': h, 'res': float(meta['resolution']), 'x0': origin[0],
                    'y0': origin[1], 'yaw0': origin[2], 'v': f'S{mtime:.0f}', 'src': 'saved',
                    'saved': time.strftime('%b %d %H:%M', time.localtime(mtime))}
        except (OSError, KeyError, ValueError, IndexError) as exc:
            self.node.get_logger().warning(f'dashboard: cannot read the saved map {base}: {exc}',
                                           throttle_duration_sec=300.0)
            return
        with self.lock:
            self.saved_png, self.saved_info, self.saved_mtime = png, info, mtime
        self.node.get_logger().info(f'dashboard: saved map {image} ({w} x {h}, {info["saved"]})')

    def _current_map(self):
        """(png, info): the live SLAM map while one is coming in, else the saved one."""
        with self.lock:
            if self.map_png is not None and time.monotonic() - self.map_t < LIVE_MAP_S:
                return self.map_png, self.map_info
            return self.saved_png, self.saved_info

    # ------------------------------------------------------------ system --
    def _system(self) -> None:
        self._load_saved_map()               # a stat() unless the map was saved again
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
        for zone in sorted(os.listdir('/sys/class/thermal')):
            if not zone.startswith('thermal_zone'):
                continue
            try:
                with open(f'/sys/class/thermal/{zone}/type') as f:
                    kind = f.read().strip().replace('-thermal', '')
                with open(f'/sys/class/thermal/{zone}/temp') as f:
                    t = int(f.read().strip()) / 1000.0
            except (OSError, ValueError):
                continue                           # some zones refuse to be read
            if -40 < t < 150:
                s.setdefault('temps', {})[kind] = round(t, 1)
        try:
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
        """map -> base: slam_toolbox's map -> odom composed with the EKF's odom -> base.

        ``fix`` is how long ago slam_toolbox last placed her (it only does while she
        moves); after SLAM stops, the last fix carries on with the odometry alone.
        """
        if self.map_odom is None or self.odom is None:
            return None
        mx, my, myaw = self.map_odom
        p = self.odom.pose.pose
        c, s = math.cos(myaw), math.sin(myaw)
        return {'x': round(mx + c * p.position.x - s * p.position.y, 2),
                'y': round(my + s * p.position.x + c * p.position.y, 2),
                'yaw': round(myaw + _yaw(p.orientation), 3),
                'fix': round(time.monotonic() - self.pose_t)}

    def snapshot(self, page_status: dict) -> dict:
        now = time.monotonic()
        self.polled = now
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
        _, current = self._current_map()
        map_info = dict(current) if current else None
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
                       'speaker': self.speaker, 'partial': self.partial if now < self.partial_until else ''},
            'level': self.level, 'buzz': self.buzz,
            'system': {**(wd or {}).get('system', {}), **self.sys},
            'map': map_info, 'pose': self._pose_on_map(), 'wami': self.wami,
            'log': log,
            'time': time.time(),
        }

    def map_image(self):
        return self._current_map()[0]

