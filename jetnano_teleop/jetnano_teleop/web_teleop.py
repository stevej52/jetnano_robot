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
shows the camera's MJPEG stream (web_video_server in the Isaac container, port
8080) and a round pad with a knob: up and down is throttle, left and right is
steering, further from the centre is more of it, and any direction in between
is the mix - so the upper right corner is forward and turning right. The knob
springs back to the centre when let go. The page posts the command ten times a
second for as long as the knob (or an arrow key) is held, and this node
publishes on ``cmd_vel_web`` only while those commands keep arriving. When
they stop - finger lifted, page closed, phone asleep, Wi-Fi gone - it publishes
zero for a moment and then goes quiet, so twist_mux lets Nav2 drive again
(``cmd_vel_web`` sits between the joystick and Nav2 in priority) and the
driver's own 0.5 s timeout is the last line of defence.

STOP raises the ``e_stop`` lock, which blocks everything including Nav2, and
stays raised until GO is pressed on the page; the joystick node uses the same
lock. Speeds are fractions of full throttle, like the joystick's.

The page also shows what the collision guard (drive.launch.py) did with the
last command and has a switch for it: off sets the guard's zones' ``enabled``
parameters false, so commands pass through it untouched, until the switch is
put back or the guard restarts (it is on at every boot).

The whole server is Python's http.server: one small JSON API, one HTML page,
nothing to install. It listens on every interface and has no login - it is for
the robot's own network only.
"""

from __future__ import annotations

import json
import os
import threading
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool

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
            self._send(status, json.dumps(payload).encode())

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
            elif path == '/config':
                self._json({
                    'video_url': node.video_url,
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
        self.declare_parameter('e_stop_topic', 'e_stop')
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
        # Empty: the page uses http://<the host it loaded from>:8080/stream?...
        self.declare_parameter('video_url', '')
        # The collision guard (drive.launch.py) and its zones, for the switch on
        # the page: switching off sets every zone's `enabled` parameter false,
        # which makes the guard pass commands through untouched. It comes back
        # on at every start of the guard, so a reboot never leaves it off.
        self.declare_parameter('guard_node', 'collision_guard')
        self.declare_parameter('guard_zones', ['stop_zone', 'slow_zone'])

        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.command_timeout = float(self.get_parameter('command_timeout').value)
        self.video_url = str(self.get_parameter('video_url').value)
        self.page = self._read_page(str(self.get_parameter('page').value))

        self.cmd_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.stop_pub = self.create_publisher(
            Bool, self.get_parameter('e_stop_topic').value, 10)

        self.state = State()
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

        # The battery, from battery_monitor: shown in the page header.
        self._battery = None        # (voltage, percentage 0..100 or None, 'ok'|'low'|'flat')
        self._battery_stamp = 0.0
        self.create_subscription(BatteryState, 'battery', self._on_battery, 10)

        # The switch: None until the guard has answered once (or if it is not running).
        guard_node = str(self.get_parameter('guard_node').value)
        self._zones = [str(z) for z in self.get_parameter('guard_zones').value]
        self._guard_enabled = None
        self._get_params = self.create_client(GetParameters, f'/{guard_node}/get_parameters')
        self._set_params = self.create_client(SetParameters, f'/{guard_node}/set_parameters')
        self._poll_future = None
        self.create_timer(3.0, self._poll_guard)

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

    def monotonic(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

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
                'linear': self.state.linear if live else 0.0,
                'angular': self.state.angular if live else 0.0,
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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
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
