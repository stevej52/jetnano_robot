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

"""Drive the robot from a web page: the camera feed on top, a virtual joystick underneath.

    ros2 run jetnano_teleop web_teleop
    http://<robot>:8081/

For a phone or a laptop on the robot's Wi-Fi, no ROS needed on it. The page
shows the camera's MJPEG stream (the D435, relayed by jetnano_bringup csi_cameras
on port 8082) and a round pad with a knob: up and down is throttle, left and right is
steering, further from the centre is more of it, and any direction in between
is the mix - so the upper right corner is forward and turning right. The knob
springs back to the centre when let go. The page posts the command ten times a
second for as long as the knob (or an arrow key) is held, and this node
publishes on ``cmd_vel_web`` only while those commands keep arriving. When
they stop - finger lifted, page closed, phone asleep, Wi-Fi gone - it publishes
zero for a moment and then goes quiet, so twist_mux lets Nav2 drive again
(``cmd_vel_web`` sits between the joystick and Nav2 in priority) and the
driver's own 0.5 s timeout is the last line of defence.

STOP raises the ``e_stop_web`` lock, which blocks everything including Nav2,
and stays raised until GO is pressed on the page. The joystick has its own
lock (``e_stop_joy``), so neither can cancel the other (review, 2026-09-26). Speeds are fractions of full throttle, like the joystick's.

The page also shows what the collision guard (drive.launch.py) did with the
last command and has a switch for it: off sets the guard's zones' ``enabled``
parameters false, so commands pass through it untouched, until the switch is
put back or the guard restarts (it is on at every boot).

The whole server is Python's http.server: one small JSON API, one HTML page,
nothing to install. It listens on every interface and has no login - it is for
the robot's own network only.
"""

from __future__ import annotations

import collections
import json
import os
import threading
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float64, String

QUIET_FLAG = os.path.expanduser('~/voice/quiet')      # jetnano_bringup quiet.py / sounds.py
DIANE_FLAG = os.path.expanduser('~/voice/diane')      # jetnano_bringup listen.py
VOICE_OFF_FLAG = os.path.expanduser('~/voice/voice_off')   # listen's "over and out"
ROBOT_FLAG = os.path.expanduser('~/voice/robot_only')   # listen: her own sounds only, no English
# What she says on TALK, one after another, round and round (Steve, 2026-10-01); the
# counter is a file, so she carries on from where she was after a restart.
HELLOS = ["Hi! Have you seen any cute Chihuahuas around here?",
          "There's like, a lot of animals in here.",
          "Is that a cat? So cute.",
          "Whatcha doin'?"]
HELLO_COUNT = os.path.expanduser('~/voice/talk_count')

try:
    from nav2_msgs.msg import CollisionMonitorState
except ImportError:  # no Nav2 on this machine: the page just never shows the guard
    CollisionMonitorState = None


def default_page() -> str:
    """The installed page, or the one next to this source when run from the tree."""
    try:
        share = get_package_share_directory('jetnano_teleop')
    except Exception:  # noqa: B902 - not installed: use the source tree
        share = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(share, 'web', 'index.html')


def default_dash() -> str:
    return os.path.join(os.path.dirname(default_page()), 'dash.html')


class State:
    """What the page last asked for, shared between the HTTP threads and the node."""

    def __init__(self):
        self.lock = threading.Lock()
        self.linear = 0.0          # -1..1, fraction of max_linear
        self.angular = 0.0         # -1..1, fraction of max_angular
        self.stamp = 0.0           # monotonic time of the last /cmd
        self.e_stop = False
        self.e_stop_changed = False
        self.clients = 0           # /cmd or /status callers in the last few seconds

    def command(self, linear: float, angular: float, now: float) -> None:
        with self.lock:
            self.linear = max(-1.0, min(1.0, float(linear)))
            self.angular = max(-1.0, min(1.0, float(angular)))
            self.stamp = now

    def set_e_stop(self, value: bool) -> None:
        with self.lock:
            if self.e_stop != value:
                self.e_stop_changed = True
            self.e_stop = value
            self.linear = 0.0
            self.angular = 0.0


def make_handler(node: 'WebTeleop'):
    """A request handler class bound to one node (http.server wants a class)."""

    state = node.state

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'
        server_version = 'jetnano_web_teleop/0.1'

        # ------------------------------------------------------------ helpers --

        def _send(self, status, body: bytes, content_type='application/json'):
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            if self.command != 'HEAD':
                self.wfile.write(body)

        def _json(self, payload: dict, status=HTTPStatus.OK):
            self._send(status, json.dumps(payload, separators=(',', ':')).encode())

        def _body(self) -> dict:
            length = int(self.headers.get('Content-Length') or 0)
            if length <= 0 or length > 4096:
                return {}
            try:
                return json.loads(self.rfile.read(length) or b'{}')
            except ValueError:
                return {}

        def log_message(self, fmt, *args):      # keep the console for real news
            node.get_logger().debug(fmt % args)

        # ----------------------------------------------------------- routes --

        def do_GET(self):
            path = self.path.split('?', 1)[0]
            if path in ('/', '/index.html'):
                self._send(HTTPStatus.OK, node.page, 'text/html; charset=utf-8')
            elif path in ('/dash', '/ipad', '/dash.html'):
                self._send(HTTPStatus.OK, node.dash_page(), 'text/html; charset=utf-8')
            elif path == '/dash/state':
                snap = node.dash.snapshot(node.status()) if node.dash else {'error': 'no dashboard'}
                snap['page_v'] = node.dash_page_version()     # the page reloads itself when it changes
                self._json(snap)
            elif path == '/dash/perf':
                self._json({'reports': list(node.dash_perf)})
            elif path == '/dash/map.png':
                png = node.dash.map_image() if node.dash else None
                if png:
                    self._send(HTTPStatus.OK, png, 'image/png')
                else:
                    self._send(HTTPStatus.NO_CONTENT, b'', 'image/png')
            elif path == '/config':
                self._json({
                    'max_linear': node.max_linear,
                    'max_angular': node.max_angular,
                    'command_timeout': node.command_timeout,
                })
            elif path == '/status':
                self._json(node.status())
            elif path == '/favicon.ico':
                self._send(HTTPStatus.NO_CONTENT, b'', 'image/x-icon')
            else:
                self._send(HTTPStatus.NOT_FOUND, b'not found', 'text/plain')

        do_HEAD = do_GET

        def do_POST(self):
            path = self.path.split('?', 1)[0]
            # Always consume the body: the connection is kept alive, and an
            # unread body becomes the start of the next request on it.
            body = self._body()
            if path == '/cmd':
                try:
                    state.command(body.get('lin', 0.0), body.get('ang', 0.0), node.monotonic())
                except (TypeError, ValueError):
                    self._json({'error': 'lin and ang must be numbers'}, HTTPStatus.BAD_REQUEST)
                    return
                self._json(node.status())
            elif path == '/stop':
                state.set_e_stop(True)
                node.get_logger().warning('STOP from the web page: e_stop raised')
                self._json(node.status())
            elif path == '/go':
                state.set_e_stop(False)
                node.get_logger().info('GO from the web page: e_stop cleared')
                self._json(node.status())
            elif path == '/look':
                # the pan-tilt camera, from the dashboard: view degrees, pan + = left, tilt + = up
                try:
                    node.look(float(body.get('pan', 0.0)), float(body.get('tilt', 0.0)))
                except (TypeError, ValueError):
                    self._json({'error': 'pan and tilt must be numbers'}, HTTPStatus.BAD_REQUEST)
                    return
                self._json({'look': node.look_at})
            elif path == '/dash/perf':
                # what the page costs the device showing it, every 10 s (dash.html's beacon)
                node.dash_perf.append({'t': round(time.time(), 1), 'from': self.client_address[0], **body})
                self._json({'ok': True})
            elif path == '/dash/locate':
                # the dashboard's WHERE AM I? button: where_am_i searches; the answer
                # arrives on where_am_i/state and shows in /dash/state
                ok, why = node.dash.locate() if node.dash else (False, 'no dashboard')
                self._json({'ok': ok, 'why': why},
                           HTTPStatus.OK if ok else HTTPStatus.SERVICE_UNAVAILABLE)
            elif path == '/voice':
                # the universal switch (voice_switch.py): load or unload ears/sounds/speak/listen
                on = bool(body.get('on', True))
                threading.Thread(target=node.voice_switch, args=(on,), daemon=True).start()
                node.get_logger().info(f'voice stack {"LOAD" if on else "UNLOAD"} from the web page')
                self._json({'ok': True, 'loading' if on else 'unloading': True})
            elif path == '/talk':
                # TALK: out of every silence, Diane mode, a hello; QUIET: nothing plays; ROBOT: her own sounds, no English
                mode = str(body.get('mode', 'talk' if body.get('on', True) else 'quiet'))
                node.talk(mode)
                self._json(node.status())
            elif path == '/guard':
                ok, why = node.set_guard(bool(body.get('enabled', True)))
                if ok:
                    self._json(node.status())
                else:
                    self._json({'error': why, **node.status()}, HTTPStatus.SERVICE_UNAVAILABLE)
            else:
                self._send(HTTPStatus.NOT_FOUND, b'not found', 'text/plain')

    return Handler


class WebTeleop(Node):

    def __init__(self):
        super().__init__('web_teleop')

        self.declare_parameter('port', 8081)
        self.declare_parameter('page', default_page())
        self.declare_parameter('cmd_vel_topic', 'cmd_vel_web')
        self.declare_parameter('e_stop_topic', 'e_stop_web')
        # Fractions of full throttle / full steering lock, like joysticks.yaml.
        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_angular', 2.4)      # rad/s; 44 deg of lock / 18.33
        # A held knob posts every 0.1 s; anything older than this is a released
        # knob, a closed page or a dead link. 0.8 rather than 0.4: a phone on a
        # weak Wi-Fi spot showed 300-700 ms round trips (2026-09-24), and at
        # 0.4 the robot stopped on every other command. The collision guard
        # is what protects the robot during the coast.
        self.declare_parameter('command_timeout', 0.8)
        self.declare_parameter('publish_rate', 20.0)
        # The collision guard (drive.launch.py) and its zones, for the switch on
        # the page: switching off sets every zone's `enabled` parameter false,
        # which makes the guard pass commands through untouched. It comes back
        # on at every start of the guard, so a reboot never leaves it off.
        self.declare_parameter('guard_node', 'collision_guard')
        self.declare_parameter('guard_zones', ['stop_zone', 'slow_zone'])
        # The iPad dashboard (Steve, 2026-09-27): /dash - cameras, sensors, everything
        # she knows, a STOP and no driving. Its data: dash.py.
        self.declare_parameter('dash_page', default_dash())
        # The pan-tilt from the dashboard (Steve, 2026-09-27: "hand pan and tilt ...
        # right next to the video feed"). View degrees -> servo degrees, as measured
        # 2026-09-26 (pca9685.yaml, motion_watch): pan 74.6 = dead ahead, higher =
        # LEFT; tilt 90 = level, higher = DOWN. The driver clamps to its own limits too.
        self.declare_parameter('pan_topic', '/pca9685/pan/angle')
        self.declare_parameter('tilt_topic', '/pca9685/tilt/angle')
        self.declare_parameter('pan_center_deg', 74.6)
        self.declare_parameter('pan_range_deg', [-69.0, 105.0])     # right .. left
        self.declare_parameter('tilt_center_deg', 90.0)
        self.declare_parameter('tilt_sign', -1.0)
        self.declare_parameter('tilt_range_deg', [-78.0, 78.0])     # down .. up

        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.command_timeout = float(self.get_parameter('command_timeout').value)
        self.page = self._read_page(str(self.get_parameter('page').value))

        self.cmd_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.stop_pub = self.create_publisher(
            Bool, self.get_parameter('e_stop_topic').value, 10)

        self.state = State()
        self.look_at = [0.0, 0.0]            # the pan-tilt, view degrees (pan + = left, tilt + = up)
        self.speak_pub = self.create_publisher(String, 'speak', 10)       # the TALK button's hello
        self.say_pub = self.create_publisher(String, 'say', 10)           # ROBOT ONLY's boop
        self.pan_pub = self.create_publisher(Float64, str(self.get_parameter('pan_topic').value), 10)
        self.tilt_pub = self.create_publisher(Float64, str(self.get_parameter('tilt_topic').value), 10)
        self._quiet_at = 0.0      # when the last stop-burst ends and we go silent
        self._driving = False
        self._last_lock_publish = 0.0

        # What the collision guard did with the last command (drive.launch.py):
        # shown on the page as "slowed" or "blocked" so a robot that will not
        # move is not a mystery. Only meaningful while commands flow; reset
        # when they stop.
        self._guard = 'clear'
        if CollisionMonitorState is not None:
            self.create_subscription(
                CollisionMonitorState, 'collision_guard/state', self._on_guard, 10)

        # The drive watchdog (Steve, 2026-10-01, after drive 16 left her nose-to-couch and the
        # page could not move her: "I want it to be clear when we're in this driving mode"):
        # who holds the wheel (the page, the joystick, Nav2), which locks are on, what the
        # guard blocks and in which direction, and whether a robot told to move is moving -
        # in plain words on the page (status 'drive'), and an "uh-oh" when she is told to
        # move and does not.
        self._wheel = {'nav': 0.0, 'nav_lin': 0.0, 'teleop': 0.0, 'out': 0.0, 'out_lin': 0.0, 'mux': 0.0, 'mux_lin': 0.0}
        self._guard_flow = ('ok', 0.0)      # guard_flow/status: why the guard holds everything, if it does
        self._locks = {}            # lock name -> when it was last seen raised
        self._speed = 0.0
        self._speed_stamp = 0.0
        self._stall_since = None
        self._stall_said = 0.0
        self.create_subscription(Twist, 'cmd_vel_nav', lambda m: self._wheel.update(nav=self.monotonic(), nav_lin=m.linear.x), 10)
        self.create_subscription(Twist, 'cmd_vel_teleop', lambda m: self._wheel.update(teleop=self.monotonic()), 10)
        self.create_subscription(Twist, 'cmd_vel', lambda m: self._wheel.update(out=self.monotonic(), out_lin=m.linear.x), 10)
        self.create_subscription(Twist, 'cmd_vel_mux', lambda m: self._wheel.update(mux=self.monotonic(), mux_lin=m.linear.x), 10)
        self.create_subscription(String, 'guard_flow/status', lambda m: setattr(self, '_guard_flow', (m.data, self.monotonic())), 10)
        self.create_subscription(Odometry, 'odometry/filtered', self._on_odom, 10)
        for name, topic in (('STOP (joystick)', 'e_stop_joy'), ('e-stop', 'e_stop'), ('motion check', 'e_stop_motion')):
            self.create_subscription(Bool, topic, lambda m, name=name: self._locks.__setitem__(name, self.monotonic() if m.data else 0.0), 10)
        self.create_timer(0.5, self._drive_watch)

        # The battery, from battery_monitor: shown in the page header.
        self._battery = None        # (voltage, percentage 0..100 or None, 'ok'|'low'|'flat')
        self._battery_stamp = 0.0
        self.create_subscription(BatteryState, 'battery', self._on_battery, 10)

        # What the watchdog thinks is wrong (jetnano_bringup watchdog): shown
        # under the header so a flaky part is noticed while driving.
        self._health = None         # list of problems, [] when all is well
        self._health_stamp = 0.0
        self.create_subscription(String, 'watchdog/status', self._on_health,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        # The switch: None until the guard has answered once (or if it is not running).
        guard_node = str(self.get_parameter('guard_node').value)
        self._zones = [str(z) for z in self.get_parameter('guard_zones').value]
        self._guard_enabled = None
        self._get_params = self.create_client(GetParameters, f'/{guard_node}/get_parameters')
        self._set_params = self.create_client(SetParameters, f'/{guard_node}/set_parameters')
        self._poll_future = None
        self.create_timer(3.0, self._poll_guard)

        self.dash = None
        self.dash_perf = collections.deque(maxlen=720)     # the pages' own reports, 2 h of them
        try:
            from jetnano_teleop.dash import Dashboard
            self.dash = Dashboard(self)
        except Exception as exc:  # noqa: B902 - the driving page must come up regardless
            self.get_logger().error(f'no dashboard: {type(exc).__name__}: {exc}')

        port = int(self.get_parameter('port').value)
        self.server = ThreadingHTTPServer(('0.0.0.0', port), make_handler(self))
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True,
                         name='web_teleop-http').start()

        rate = float(self.get_parameter('publish_rate').value)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'driving page on http://0.0.0.0:{port}/  (max {self.max_linear:.2f} of full '
            f'throttle, {self.max_angular:.2f} rad/s, commands time out after '
            f'{self.command_timeout:.1f} s)')

    def _on_guard(self, msg) -> None:
        if msg.action_type == CollisionMonitorState.STOP:
            self._guard = 'blocked'
        elif msg.action_type in (CollisionMonitorState.SLOWDOWN, CollisionMonitorState.LIMIT):
            self._guard = 'slowed'
        else:
            self._guard = 'clear'

    def _on_odom(self, msg) -> None:
        v = msg.twist.twist.linear
        self._speed = (v.x * v.x + v.y * v.y) ** 0.5
        self._speed_stamp = self.monotonic()

    def _drive_watch(self) -> None:
        """Told to move (the driver's /cmd_vel) but the odometry says still: a stall, said once."""
        now = self.monotonic()
        told = now - self._wheel['mux'] < 0.5 and abs(self._wheel['mux_lin']) > 0.05   # into the guard
        moving = now - self._speed_stamp < 1.0 and self._speed > 0.03
        if told and not moving:
            if self._stall_since is None:
                self._stall_since = now
            elif now - self._stall_since > 1.5 and now - self._stall_said > 10.0:
                self._stall_said = now
                self.get_logger().warning(f'drive watchdog: told to move ({self._wheel["out_lin"]:+.2f}) for '
                                          f'{now - self._stall_since:.1f} s, the odometry says {self._speed:.2f} m/s')
                self.say_pub.publish(String(data='uhoh'))
        else:
            self._stall_since = None

    def drive_status(self) -> dict:
        """Who drives, what stops it, in words. Called with the state lock held."""
        now = self.monotonic()
        wheel = ('page' if self._driving else 'joystick' if now - self._wheel['teleop'] < 1.0
                 else 'nav2' if now - self._wheel['nav'] < 1.0 else 'nobody')
        locks = [n for n, t in self._locks.items() if now - t < 2.0]
        if self.state.e_stop:
            locks.insert(0, 'STOP (page)')
        blocked = self._guard if (self._driving or wheel == 'nav2') else 'clear'
        ahead = (self.state.linear if self._driving else self._wheel['nav_lin']) >= 0
        stalled = self._stall_since is not None and now - self._stall_since > 1.5
        flow, flow_at = self._guard_flow
        holding = flow.startswith('holding') and now - flow_at < 3.0
        if holding:
            note = 'THE OBSTACLE GUARD IS HOLDING HER: ' + flow[len('holding: '):] + ' - being fixed, or switch the guard OFF'
        elif locks:
            note = 'LOCKED: ' + ', '.join(locks) + (' - press GO' if 'STOP (page)' in locks else '')
        elif blocked == 'blocked':
            note = 'BLOCKED ' + ('AHEAD - back up' if ahead else 'BEHIND - go forward') + ' (obstacle guard)'
        elif stalled:
            note = 'TOLD TO MOVE BUT NOT MOVING - stuck, or no servo power'
        elif wheel == 'nav2':
            note = 'NAV2 IS DRIVING (a lap or a goal) - your stick overrides it'
        elif blocked == 'slowed':
            note = 'slowed: obstacle near'
        else:
            note = ''
        return {'wheel': wheel, 'locks': locks, 'guard': blocked, 'stalled': stalled,
                'speed': round(self._speed, 2), 'note': note}

    def _on_battery(self, msg) -> None:
        if not msg.present:
            self._battery = None
        else:
            pct = None if msg.percentage != msg.percentage else round(msg.percentage * 100)
            if msg.power_supply_health == BatteryState.POWER_SUPPLY_HEALTH_DEAD:
                state = 'flat'
            elif pct is not None and pct <= 20:
                state = 'low'
            else:
                state = 'ok'
            self._battery = (round(msg.voltage, 2), pct, state)
        self._battery_stamp = self.monotonic()

    def _on_health(self, msg) -> None:
        try:
            self._health = list(json.loads(msg.data).get('problems', []))
        except (ValueError, AttributeError):
            return
        self._health_stamp = self.monotonic()

    # ------------------------------------------------------------- the switch --

    def _poll_guard(self) -> None:
        """Ask the guard whether its zones are enabled, so the page shows the truth
        even when someone changed it with ros2 param set."""
        if self._poll_future is not None and not self._poll_future.done():
            return
        if not self._get_params.service_is_ready():
            self._guard_enabled = None
            return
        request = GetParameters.Request(names=[f'{z}.enabled' for z in self._zones])
        self._poll_future = self._get_params.call_async(request)
        self._poll_future.add_done_callback(self._on_guard_params)

    def _on_guard_params(self, future) -> None:
        try:
            values = future.result().values
        except Exception as exc:  # noqa: B902 - the guard went away mid-call
            self.get_logger().debug(f'guard parameters: {exc}')
            self._guard_enabled = None
            return
        flags = [v.bool_value for v in values if v.type == ParameterType.PARAMETER_BOOL]
        self._guard_enabled = bool(flags) and all(flags)

    def set_guard(self, enabled: bool):
        """Switch every zone on or off. Called from an HTTP thread: the request
        goes out asynchronously and the spinning main thread completes it."""
        if not self._set_params.service_is_ready():
            return False, 'the collision guard is not running'
        request = SetParameters.Request(parameters=[
            Parameter(name=f'{z}.enabled',
                      value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=enabled))
            for z in self._zones])
        future = self._set_params.call_async(request)
        deadline = time.monotonic() + 2.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            return False, 'the collision guard did not answer'
        results = future.result().results
        if not all(r.successful for r in results):
            return False, '; '.join(r.reason for r in results if not r.successful) or 'refused'
        self._guard_enabled = enabled
        if enabled:
            self.get_logger().info('obstacle guard switched ON from the web page')
        else:
            self.get_logger().warning('obstacle guard switched OFF from the web page: '
                                      'nothing stops the robot before an obstacle now')
        return True, ''

    def _read_page(self, path: str) -> bytes:
        try:
            with open(path, 'rb') as handle:
                return handle.read()
        except OSError as exc:
            self.get_logger().error(f'cannot read the page {path}: {exc}')
            return b'<h1>jetnano web_teleop: page missing</h1>'

    def look(self, pan: float, tilt: float) -> None:
        """Point the pan-tilt camera: view degrees, clamped to the mount's reach."""
        lo, hi = [float(v) for v in self.get_parameter('pan_range_deg').value]
        tlo, thi = [float(v) for v in self.get_parameter('tilt_range_deg').value]
        pan, tilt = max(lo, min(hi, pan)), max(tlo, min(thi, tilt))
        self.look_at = [round(pan, 1), round(tilt, 1)]
        a, b = Float64(), Float64()
        a.data = float(self.get_parameter('pan_center_deg').value) + pan
        b.data = float(self.get_parameter('tilt_center_deg').value) + float(self.get_parameter('tilt_sign').value) * tilt
        self.pan_pub.publish(a)
        self.tilt_pub.publish(b)

    def dash_page(self) -> bytes:
        # read each time, so the page can be edited without a restart
        return self._read_page(str(self.get_parameter('dash_page').value))

    def dash_page_version(self) -> float:
        try:
            return os.path.getmtime(str(self.get_parameter('dash_page').value))
        except OSError:
            return 0.0

    def monotonic(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def next_hello(self) -> str:
        """The next TALK line in turn."""
        try:
            with open(HELLO_COUNT) as f:
                n = int(f.read().strip() or 0)
        except (OSError, ValueError):
            n = 0
        try:
            with open(HELLO_COUNT, 'w') as f:
                f.write(str(n + 1) + chr(10))
        except OSError:
            pass
        line = HELLOS[n % len(HELLOS)]
        self.get_logger().info(f'TALK hello {n % len(HELLOS) + 1} of {len(HELLOS)}: "{line}"')
        return line

    def voice_loaded(self) -> bool:
        """jetnano-voice.service active? (asked at most every 3 s)"""
        now = self.monotonic()
        if now - getattr(self, '_voice_at', 0.0) > 3.0:
            from jetnano_bringup import voice_switch
            self._voice = voice_switch.loaded()
            self._voice_at = now
        return getattr(self, '_voice', False)

    def voice_switch(self, on: bool) -> None:
        from jetnano_bringup import voice_switch
        voice_switch.switch(on, quiet=True)
        self._voice_at = 0.0

    def talk(self, mode: str) -> None:
        """The driving page's voice button (Steve, 2026-10-01): three modes in a cycle.
        talk:  quiet mode, "over and out" and robot-only off; Diane mode on (listen gives everyone her
               fun personality and answers without her name); then one of the hello lines.
        quiet: nothing out of the speaker at all (~/voice/quiet); Diane and robot-only off.
        robot: her own sounds only, never English (~/voice/robot_only); quiet and Diane off; a boop."""
        os.makedirs(os.path.dirname(QUIET_FLAG), exist_ok=True)
        stamp = time.strftime('%Y-%m-%d %H:%M:%S') + chr(10)

        def put(flag, on):
            if on:
                with open(flag, 'w') as f:
                    f.write(stamp)
            else:
                try:
                    os.remove(flag)
                except FileNotFoundError:
                    pass
        # the hello / boop a moment later - unless the mode has changed again by then (2026-10-01:
        # three taps in half a second, and the hello played in QUIET mode)
        t = getattr(self, 'hello_timer', None)
        if t is not None:
            t.cancel()
        if mode == 'talk':
            put(QUIET_FLAG, False)
            put(VOICE_OFF_FLAG, False)
            put(ROBOT_FLAG, False)
            put(DIANE_FLAG, True)
            delay = 1.5
            if not self.voice_loaded():                # TALK on an unloaded stack loads it: show-off mode
                self.get_logger().info('TALK from the web page: loading the voice stack first')
                threading.Thread(target=self.voice_switch, args=(True,), daemon=True).start()
                delay = 18.0                           # speak needs ~15 s to be ready
            self.get_logger().info('TALK from the web page: quiet off, Diane mode on')
            self.hello_timer = threading.Timer(delay, lambda: os.path.exists(DIANE_FLAG) and not os.path.exists(QUIET_FLAG)
                                               and self.speak_pub.publish(String(data=self.next_hello())))
            self.hello_timer.start()
        elif mode == 'robot':
            put(QUIET_FLAG, False)
            put(VOICE_OFF_FLAG, False)
            put(DIANE_FLAG, False)
            put(ROBOT_FLAG, True)
            self.get_logger().info('ROBOT ONLY from the web page: her own sounds, no English')
            self.hello_timer = threading.Timer(1.2, lambda: os.path.exists(ROBOT_FLAG) and self.say_pub.publish(String(data='boop')))
            self.hello_timer.start()
        else:
            put(DIANE_FLAG, False)
            put(ROBOT_FLAG, False)
            put(QUIET_FLAG, True)
            self.get_logger().info('QUIET from the web page: Diane mode off, quiet on')

    def status(self) -> dict:
        with self.state.lock:
            live = (self.monotonic() - self.state.stamp) <= self.command_timeout
            live = live and not self.state.e_stop
            return {
                'e_stop': self.state.e_stop,
                'driving': self._driving,
                'guard': self._guard if self._driving else 'clear',
                'guard_enabled': self._guard_enabled,
                'battery': (self._battery if self.monotonic() - self._battery_stamp < 5.0 else None),
                'health': (self._health if self.monotonic() - self._health_stamp < 20.0 else None),
                'look': self.look_at,
                'quiet': os.path.exists(QUIET_FLAG),
                'diane': os.path.exists(DIANE_FLAG),
                'robot_only': os.path.exists(ROBOT_FLAG),
                'voice_mode': ('quiet' if os.path.exists(QUIET_FLAG) else 'robot' if os.path.exists(ROBOT_FLAG)
                               else 'talk' if os.path.exists(DIANE_FLAG) else 'normal'),
                'voice_loaded': self.voice_loaded(),
                'linear': self.state.linear if live else 0.0,
                'angular': self.state.angular if live else 0.0,
                'drive': self.drive_status(),
            }

    # ---------------------------------------------------------------- loop --

    def _tick(self) -> None:
        now = self.monotonic()
        with self.state.lock:
            fresh = (now - self.state.stamp) <= self.command_timeout
            linear, angular = self.state.linear, self.state.angular
            e_stop = self.state.e_stop
            lock_changed = self.state.e_stop_changed
            self.state.e_stop_changed = False

        # The lock: on every change, and once a second while raised, so a
        # twist_mux that (re)started still knows it is stopped.
        if lock_changed or (e_stop and now - self._last_lock_publish >= 1.0):
            message = Bool()
            message.data = e_stop
            self.stop_pub.publish(message)
            self._last_lock_publish = now

        twist = Twist()
        if fresh and not e_stop and (linear != 0.0 or angular != 0.0):
            twist.linear.x = linear * self.max_linear
            twist.angular.z = angular * self.max_angular
            self.cmd_pub.publish(twist)
            self._driving = True
            self._quiet_at = now + 0.5
        elif now < self._quiet_at:
            # Just released (or stopped): say "zero" for half a second rather
            # than leaving the robot to time out, then hand control back.
            self.cmd_pub.publish(twist)
            self._driving = False
            self._guard = 'clear'
        else:
            self._driving = False


def main(args=None):
    rclpy.init(args=args)
    node = WebTeleop()
    # rclpy's EventsExecutor, not the default SingleThreadedExecutor: that one rebuilds
    # its wait set of ~40 entities (the dashboard's subscriptions, the parameter
    # services, clients, timers) in Python on every wake-up, which py-spy put at 82-92 %
    # of this node's CPU (2026-09-28: 7.6 % of a core idle, 38.9 % with the dashboard
    # open); the callbacks themselves were under 10 %.
    try:
        from rclpy.experimental import EventsExecutor
        executor = EventsExecutor()
    except ImportError:          # an rclpy without it: the old way
        from rclpy.executors import SingleThreadedExecutor
        executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())
            node.server.shutdown()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
