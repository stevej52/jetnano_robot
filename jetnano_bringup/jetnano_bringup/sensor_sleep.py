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

"""Rest the lidar's motor and the camera's laser while she sits doing nothing.

    (in the housekeeping process)
    ros2 topic echo /sensors/state                 {"state": "asleep", "why": "on the bench, idle 2 min", ...}
    ros2 run jetnano_bringup sensors sleep|wake|status

Steve, 2026-10-11, watching her on the bench through a night-vision webcam: the lidar's
laser and the D435's dot projector run full time, for days when she sits on the bench,
and the lidar's motor with them. Both wear: the A1's motor and belt, the projector's laser.

  asleep:  idle `idle_s` (10 min), or `bench_idle_s` (90 s) when she knows she is on the
           bench (where_am_i says bench, or no pack: bench power). The lidar's motor stops
           (/stop_motor) and the projector goes off (emitter_on_off false, then
           emitter_enabled 0: the camera refuses the second while it alternates). The
           camera keeps streaming; /vo may stop (2 times in 4) or run at 90 Hz - either way
           nothing restarts it while she sleeps (sensor_flag.py), she is not moving.
  wake:    any drive command (page, joystick), a mission/park command, a Nav2 goal, or
           /sensor_sleep/wake. The motor starts, the projector comes back; 'waking' until
           the scans have flowed 2 s (about 3-5 s), then 'awake'. Woken off the bench, she
           asks where_am_i again (she may have been carried while blind).
  while asleep the safety gate is inhibited (no lidar) and says "sensors asleep"; the
  watchdogs skip the lidar's streams through ~/sensors/asleep (sensor_flag.py).

Camera keeper (2026-10-12, D435 FW 5.17.3.10 + librealsense 2.59.1): the camera refuses the
projector's alternation before it streams, so its launch starts it with the projector off
and, while awake, this switches alternation on live (CAMERA_ON) whenever the camera reports
otherwise twice in a row - after a boot, a container restart, a driver restart.
"""

import json
import time

from geometry_msgs.msg import PoseStamped, Twist
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger

from action_msgs.msg import GoalStatusArray

from jetnano_bringup import sensor_flag
from jetnano_bringup.quiet_node import QuietNode

ACTIVE_GOAL = (1, 2, 3)          # action_msgs GoalStatus: ACCEPTED, EXECUTING, CANCELING
SWITCH_S = 20.0                  # the camera's gap when the projector mode changes is ~4 s
CAMERA_OFF = (('depth_module.emitter_on_off', False), ('depth_module.emitter_enabled', 0))
CAMERA_ON = (('depth_module.emitter_enabled', 1), ('depth_module.emitter_on_off', True))


def due_to_sleep(idle, busy, bench, idle_s, bench_idle_s):
    """-> why the sensors should go to sleep now, or ''."""
    if busy:
        return ''
    if bench and idle >= bench_idle_s:
        return f'on the bench, idle {idle / 60:.0f} min' if idle >= 60 else f'on the bench, idle {idle:.0f} s'
    if idle >= idle_s:
        return f'idle {idle / 60:.0f} min'
    return ''


def moving(t):
    return any(abs(v) > 1e-3 for v in (t.linear.x, t.linear.y, t.angular.z))


class SensorSleep(QuietNode):

    def __init__(self):
        super().__init__('sensor_sleep')
        self.declare_parameter('enabled', True)
        self.declare_parameter('idle_s', 600.0)
        self.declare_parameter('bench_idle_s', 90.0)
        self.declare_parameter('wake_timeout_s', 20.0)
        self.declare_parameter('lidar', True)
        self.declare_parameter('camera', True)
        self.declare_parameter('camera_node', '/camera')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        now = time.monotonic()
        self.state, self.why, self.since = 'awake', 'starting', now
        self.last_active = now
        self.nav_busy = {}
        self.where = None
        self.pack = None
        self.scan_at = self.scan_since = None
        self.wake_at = self.stop_at = self.flag_at = self.cam_check_at = 0.0
        self.was_bench = False
        self.cam_reoffs = 0
        self.cam_bad = 0                    # keeper: polls in a row the camera was not alternating
        self.cam_changed_at = 0.0
        self.cam_flag_until = 0.0
        self.last_pub = ''
        cam = str(self.get_parameter('camera_node').value).rstrip('/')
        self.stop_motor = self.create_client(Empty, 'stop_motor')
        self.start_motor = self.create_client(Empty, 'start_motor')
        self.cam_set = self.create_client(SetParameters, f'{cam}/set_parameters')
        self.cam_get = self.create_client(GetParameters, f'{cam}/get_parameters')
        self.locate = self.create_client(Trigger, 'where_am_i/locate')
        for topic in ('cmd_vel_teleop', 'cmd_vel_web'):
            self.create_subscription(Twist, topic, lambda m, t=topic: moving(m) and self._activity(t), 10)
        for topic in ('mission/command', 'park/command'):
            self.create_subscription(String, topic, lambda _m, t=topic: self._activity(t), 10)
        self.create_subscription(PoseStamped, 'nav_helper/goal', lambda _m: self._activity('a goal'), 10)
        for action in ('navigate_to_pose', 'navigate_through_poses'):
            self.create_subscription(GoalStatusArray, f'{action}/_action/status',
                                     lambda m, a=action: self._on_nav(a, m), 10)
        for qos in (10, latched):
            self.create_subscription(String, 'where_am_i/state', self._on_where, qos)
            self.create_subscription(String, 'battery/level', self._on_battery, qos)
        self.create_subscription(LaserScan, 'scan_raw', self._on_scan, be, raw=True)
        self.create_service(Trigger, '~/sleep', self._srv_sleep)
        self.create_service(Trigger, '~/wake', self._srv_wake)
        self.state_pub = self.create_publisher(String, 'sensors/state', latched)
        self.create_timer(1.0, self._tick)
        flag = sensor_flag.read()
        if flag is not None and flag.get('state') in ('asleep', 'waking'):
            # restarted while they slept: wake them, the next idle spell puts them back
            self.state = 'asleep'
            self._wake('sensor_sleep restarted while they slept')
        elif flag is not None:
            sensor_flag.clear()
        self._publish()
        self.get_logger().info(
            f'sensor sleep: lidar motor and camera projector rest after {self.gp("idle_s") / 60:.0f} min idle, '
            f'{self.gp("bench_idle_s"):.0f} s on the bench; any drive command wakes them')

    def gp(self, name):
        return self.get_parameter(name).value

    # ---- what she is doing ----
    def bench(self):
        return self.where == 'bench' or self.pack == 'none'

    def busy(self):
        return any(self.nav_busy.values())

    def _activity(self, why):
        self.last_active = time.monotonic()
        if self.state == 'asleep':
            self._wake(why)

    def _on_nav(self, action, msg):
        self.nav_busy[action] = any(s.status in ACTIVE_GOAL for s in msg.status_list)
        if self.nav_busy[action]:
            self._activity(f'a Nav2 goal ({action})')

    def _on_where(self, m):
        try:
            self.where = json.loads(m.data).get('state')
        except (ValueError, AttributeError):
            pass

    def _on_battery(self, m):
        try:
            self.pack = json.loads(m.data).get('level')
        except (ValueError, AttributeError):
            pass

    def _on_scan(self, _raw):
        now = time.monotonic()
        if self.scan_at is None or now - self.scan_at > 1.0:
            self.scan_since = now
        self.scan_at = now

    # ---- sleep and wake ----
    def _sleep(self, why):
        now = time.monotonic()
        self.state, self.why, self.since = 'asleep', why, now
        self.was_bench = self.bench()
        self.cam_reoffs = 0
        sensor_flag.write('asleep', SWITCH_S if self.gp('camera') else 0.0)
        self.flag_at = self.cam_check_at = now
        self.get_logger().info(f'sensors asleep ({why}): lidar motor stopped, camera projector off')
        if self.gp('lidar'):
            self.stop_at = now
            self._call(self.stop_motor, Empty.Request(), 'stop_motor')
        if self.gp('camera'):
            self._camera(CAMERA_OFF, 'off')
        self._publish()

    def _wake(self, why):
        if self.state != 'asleep':
            return
        now = time.monotonic()
        self.state, self.why, self.wake_at = 'waking', why, now
        self.scan_at = self.scan_since = None
        sensor_flag.write('waking', SWITCH_S if self.gp('camera') else 0.0)
        self.flag_at = now
        self.get_logger().info(f'sensors waking ({why})')
        if self.gp('lidar'):
            self._call(self.start_motor, Empty.Request(), 'start_motor')
        if self.gp('camera'):
            self._camera(CAMERA_ON, 'on')
        self._publish()

    def _awake(self, note):
        now = time.monotonic()
        took = now - self.wake_at
        self.state, self.why, self.since, self.last_active = 'awake', note, now, now
        sensor_flag.clear()
        self.get_logger().info(f'sensors awake after {took:.1f} s ({note})')
        if self.was_bench and self.locate.service_is_ready():
            self._call(self.locate, Trigger.Request(), 'where_am_i/locate')   # carried off the bench?
        self._publish()

    def _tick(self):
        now = time.monotonic()
        if self.busy():
            self.last_active = now
        idle = now - self.last_active
        if self.state == 'awake':
            if self.cam_flag_until and now > self.cam_flag_until:
                self.cam_flag_until = 0.0
                sensor_flag.clear()
            if self.gp('camera') and now - self.cam_check_at >= 5.0:
                self.cam_check_at = now
                self._check_camera()
            if self.gp('enabled'):
                why = due_to_sleep(idle, self.busy(), self.bench(), self.gp('idle_s'), self.gp('bench_idle_s'))
                if why:
                    self._sleep(why)
        elif self.state == 'asleep':
            if not self.gp('enabled'):
                self._wake('sensor sleep switched off')
                return
            if now - self.flag_at >= 10.0:
                sensor_flag.write('asleep')
                self.flag_at = now
            # a restarted driver comes back running: put it to rest again
            if (self.gp('lidar') and self.scan_since is not None and now - self.scan_at < 1.0
                    and now - self.scan_since > 3.0 and now - self.stop_at > 15.0):
                self.stop_at = now
                self.get_logger().warning('the lidar is spinning again while asleep (driver restarted?): stopping it')
                self._call(self.stop_motor, Empty.Request(), 'stop_motor')
            if self.gp('camera') and now - self.cam_check_at >= 30.0:
                self.cam_check_at = now
                self._check_camera()
        elif self.state == 'waking':
            if now - self.flag_at >= 10.0:
                sensor_flag.write('waking')
                self.flag_at = now
            flowing = (not self.gp('lidar')) or (
                self.scan_since is not None and now - self.scan_at < 1.0 and now - self.scan_since >= 2.0)
            if flowing:
                self._awake(self.why)
            elif now - self.wake_at > float(self.gp('wake_timeout_s')):
                self.get_logger().warning(
                    f'no lidar scans {now - self.wake_at:.0f} s after waking: handing the lidar back to the watchdog')
                self._awake(f'{self.why}; no scans yet')
        if now - getattr(self, '_pub_at', 0.0) >= 10.0:
            self._publish()

    # ---- the calls ----
    def _call(self, client, req, name):
        if not client.service_is_ready():
            self.get_logger().warning(f'{name}: service not there')
            return
        client.call_async(req)

    def _camera(self, steps, what):
        """Set the camera's parameters one after the other (the order matters)."""
        self.cam_changed_at = time.monotonic()
        if not steps:
            self.get_logger().info(f'camera projector {what}')
            return
        if not self.cam_set.service_is_ready():
            self.get_logger().warning(f'camera parameters not reachable: projector left as it is (wanted {what})')
            return
        name, value = steps[0]
        req = SetParameters.Request()
        req.parameters = [Parameter(name, value=value).to_parameter_msg()]
        self.cam_set.call_async(req).add_done_callback(lambda f: self._camera_step(f, steps, what))

    def _camera_step(self, fut, steps, what):
        name, value = steps[0]
        try:
            res = fut.result().results[0]
            ok, reason = res.successful, res.reason
        except Exception as exc:  # noqa: BLE001 - a dead camera node: say so, the watchdog has it
            ok, reason = False, str(exc)
        if not ok:
            self.get_logger().warning(f'camera {name} -> {value} refused ({reason}): projector {what} not finished')
            return
        self._camera(steps[1:], what)

    def _check_camera(self):
        if not self.cam_get.service_is_ready():
            return
        req = GetParameters.Request()
        req.names = ['depth_module.emitter_enabled', 'depth_module.emitter_on_off']
        self.cam_get.call_async(req).add_done_callback(self._camera_checked)

    def _camera_checked(self, fut):
        try:
            vals = fut.result().values
            on, alternating = vals[0].integer_value != 0, vals[1].bool_value
        except Exception:  # noqa: BLE001
            return
        now = time.monotonic()
        if self.state == 'awake':
            # the keeper: the camera streams but does not alternate (it starts that way)
            self.cam_bad = 0 if (on and alternating) else self.cam_bad + 1
            if self.cam_bad >= 2 and now - self.cam_changed_at > 15.0:
                self.cam_bad = 0
                self.get_logger().info('camera: projector alternation on (the camera started without it)')
                if sensor_flag.read() is None:
                    sensor_flag.write('camera', SWITCH_S)
                    self.cam_flag_until = now + SWITCH_S + 1.0
                self._camera(CAMERA_ON, 'alternating')
            return
        if on and self.state == 'asleep':
            # once per sleep: the camera's launch restarting again and again must not become a loop
            # of switch, gap, restart (2026-10-11, before safety_monitor knew the flag)
            if self.cam_reoffs >= 1:
                if self.cam_reoffs == 1:
                    self.get_logger().warning('the camera projector came back on again: left on until she wakes')
                    self.cam_reoffs += 1
                return
            self.cam_reoffs += 1
            self.get_logger().warning('the camera projector is on again while asleep (driver restarted?): off again')
            sensor_flag.write('asleep', SWITCH_S)
            self._camera(CAMERA_OFF, 'off')

    # ---- services and state ----
    def _srv_sleep(self, _req, res):
        if self.state != 'asleep':
            self._sleep('asked')
        res.success, res.message = True, 'asleep: lidar motor stopped, camera projector off'
        return res

    def _srv_wake(self, _req, res):
        self.last_active = time.monotonic()
        if self.state == 'asleep':
            self._wake('asked')
        res.success, res.message = True, self.state
        return res

    def _publish(self):
        self._pub_at = time.monotonic()
        d = {'state': self.state, 'why': self.why, 'bench': self.bench(),
             'idle_s': round(time.monotonic() - self.last_active), 'enabled': bool(self.gp('enabled'))}
        self.state_pub.publish(String(data=json.dumps(d)))
