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

"""The safety gate: ONE place that decides whether she may drive, and says why.

    ros2 topic echo /safety/state      {"state": "ok" | "degraded" | "stopped" | "inhibited",
                                        "reasons": [...], "speed_pct": 100, "since": ...}
    ros2 topic echo /safety/permit     Bool: ok or degraded

Until 2026-10-02 motion was decided in five places (the mux's locks, the collision guard,
safety_monitor's tilt / motion / VO logic, the battery monitor's lock, guard_flow) and
nothing owned the answer or could explain it (Steve, after drive 16: "it needs to be clear
when we're in this mode that made it so I couldn't move it"). The audit of 2026-10-02:
"own motion permission, battery/IMU validity, sensor freshness and fault latching; start
inhibited until required inputs are valid; maintain unknown/degraded/stopped distinctions."

This node reads what the others already publish and keeps one state machine:
  inhibited  - not enough known to permit: no lidar, no odometry, no battery verdict yet,
               the pack absent (the bench), or where_am_i says bench. At start, always.
  stopped    - a STOP lock (page, joystick, battery, motion check), the collision guard
               holding every command (guard_flow), the tilt guard, a flat pack.
  degraded   - may drive, slower: the camera's obstacles stale (lidar guards alone), VO
               silent, a low pack (home only), the watchdog reporting a part down.
  ok         - everything required fresh and valid.
It raises the mux lock e_stop_gate (twist_mux.yaml) while inhibited or stopped, publishes
/speed_limit while degraded, and the page and predrive read /safety/state. The older
locks stay as defence in depth; this is the one that decides and explains.
"""

import json
import time

from diagnostic_msgs.msg import DiagnosticArray
from nav2_msgs.msg import SpeedLimit

from jetnano_bringup.quiet_node import QuietNode
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

DEGRADED_SPEED_PCT = 50.0
# After this long, a report that never came is a missing part, not one still starting
# (2026-10-12: silence used to read as healthy - a tilt guard that never started let her drive).
GRACE_S = 120.0
LOCKS = (('e_stop_web', 'STOP on the page'), ('e_stop_joy', 'STOP on the joystick'),
         ('e_stop', 'battery stop'), ('e_stop_motion', 'motion check'))


def decide(now, inp):
    """The gate's one decision, pure: -> (state, reasons, speed_pct). `inp` is a dict of the
    latest facts (see SafetyGate._facts); None/missing = unknown."""
    reasons, stopped, degraded = [], [], []
    # required knowledge: without it she stays inhibited
    if inp.get('fresh_report') is False:
        # topic_watch's watchdog/fresh carries the stream ages; without it nothing is known
        reasons.append('stream ages unknown (no freshness report from topic_watch)')
    if inp.get('sensors') in ('asleep', 'waking'):
        # sensor_sleep rests the lidar and the projector while she idles; a drive command wakes them
        reasons.append('sensors asleep: a drive command wakes them' if inp['sensors'] == 'asleep'
                       else 'sensors waking')
    elif inp.get('scan_age') is None or inp['scan_age'] > 1.5:
        reasons.append('no lidar' if inp.get('scan_age') is None else f'lidar silent {inp["scan_age"]:.0f} s')
    if inp.get('odom_age') is None or inp['odom_age'] > 1.0:
        reasons.append('no odometry' if inp.get('odom_age') is None else f'odometry silent {inp["odom_age"]:.0f} s')
    bat = inp.get('battery')
    bench = []
    if bat is None:
        reasons.append('no battery verdict yet')
    elif bat.get('level') == 'none':
        bench.append('no pack (bench power)')
    if inp.get('where') == 'bench':
        bench.append('on the bench')
    # wheels-up tests on the bench are a thing (Steve drives her from the page there): with
    # allow_bench the bench is "degraded: wheels up?" instead of inhibited
    if bench and not inp.get('allow_bench'):
        reasons += bench
    started = inp.get('uptime') is not None and inp['uptime'] > GRACE_S
    if started and inp.get('tilt_age') is None:
        reasons.append('tilt guard has not reported')
    if reasons:
        return 'inhibited', reasons, 0.0
    # stops
    for key, label in LOCKS:
        if inp.get('lock_' + key):
            stopped.append(label)
    if bat.get('level') == 'flat':
        stopped.append(f'pack flat ({bat.get("volts", 0):.1f} V)')
    tilt = inp.get('tilt') or ''                     # safety_monitor: "armed -> recovering: pitch 31 deg"
    if '-> recovering' in tilt:
        stopped.append(f'tilt guard: {tilt.split(":", 1)[-1].strip()}')
    gf = inp.get('guard_flow') or ''
    if gf.startswith('holding'):
        stopped.append(f'collision guard {gf}')
    if stopped:
        return 'stopped', stopped, 0.0
    # degraded
    cam = inp.get('camera') or ''
    cam_age = inp.get('camera_age')
    if started and cam_age is None:
        degraded.append('camera obstacles: no health report ever: lidar only')
    if started and (inp.get('guard_flow_age') is None or inp['guard_flow_age'] > 3.0):
        degraded.append('guard_flow silent: the collision guard is not being watched')
    if started and (inp.get('watchdog_age') is None or inp['watchdog_age'] > 15.0):
        degraded.append('watchdog silent')
    if started and inp.get('fresh_report') and inp.get('vo_age') is None:
        degraded.append('visual odometry never seen')
    if tilt.startswith('off'):
        degraded.append('tilt guard switched off')
    if cam_age is not None and cam_age > 5.0:
        degraded.append(f'camera obstacles: no health report for {cam_age:.0f} s: lidar only')
    elif cam.startswith('stale'):
        degraded.append(f'camera obstacles {cam}: lidar only')
    bat_age = inp.get('battery_age')
    if bat_age is not None and bat_age > 20.0:
        degraded.append(f'battery verdict {bat_age:.0f} s old (the monitor is silent)')
    if inp.get('vo_age') is not None and inp['vo_age'] > 2.0:
        degraded.append(f'visual odometry silent {inp["vo_age"]:.0f} s')
    if bat.get('level') == 'low':
        degraded.append(f'pack low ({bat.get("volts", 0):.1f} V): home only')
    for p in (inp.get('watchdog') or [])[:3]:
        degraded.append(f'watchdog: {p}')
    if bench:
        degraded.append('bench driving allowed (wheels up?)')
    if degraded:
        return 'degraded', degraded, DEGRADED_SPEED_PCT
    return 'ok', [], 100.0


class SafetyGate(QuietNode):

    def __init__(self):
        super().__init__('safety_gate')
        self.declare_parameter('rate_hz', 5.0)
        self.declare_parameter('allow_bench', False)     # ros2 param set /safety_gate allow_bench true
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.fresh, self.fresh_at = {}, None
        self.locks = {}
        self.battery = None
        self.where = None
        self.sensors = None
        self.tilt = ''
        self.guard_flow = ''
        self.camera = ''
        self.watchdog = []
        self.state, self.reasons, self.speed = 'inhibited', ['starting'], 0.0
        self.since = time.monotonic()
        self.started = self.since
        self.tilt_at = self.guard_flow_at = self.watchdog_at = None
        self.last_lock_pub = 0.0
        # The streams' ages come from topic_watch (C++) on watchdog/fresh, 5 times a second: the
        # age of the last message and how far its stamp was behind ROS time on arrival, so a
        # stream that keeps arriving with a frozen stamp is as stale as one that stopped (review
        # 2026-10-03). Until 2026-10-12 this node subscribed to /odometry/filtered (100 Hz), /vo
        # (45 Hz) and /scan itself, ~30 % of a core in rclpy; Steve: "we will always need compute".
        self.create_subscription(DiagnosticArray, 'watchdog/fresh', self._on_fresh, 10)
        for key, _ in LOCKS:
            self.create_subscription(Bool, key, lambda m, key=key: self.locks.__setitem__(key, bool(m.data)), 10)
        self.battery_at = None
        self.camera_at = None
        for qos in (10, latched):
            self.create_subscription(String, 'battery/level', self._on_battery, qos)
            self.create_subscription(String, 'where_am_i/state', self._on_where, qos)
            self.create_subscription(String, 'sensors/state', self._on_sensors, qos)
        for qos in (10, latched):        # latched since 2026-10-12: changes only, and we may start later
            self.create_subscription(String, 'safety_monitor/tilt_status', self._on_tilt, qos)
        self.create_subscription(String, 'guard_flow/status', self._on_guard_flow, 10)
        self.create_subscription(String, 'camera_obstacles/health', self._on_camera, 10)
        self.create_subscription(String, 'watchdog/status', self._on_watchdog, latched)
        self.state_pub = self.create_publisher(String, 'safety/state', latched)
        self.permit_pub = self.create_publisher(Bool, 'safety/permit', latched)
        self.lock_pub = self.create_publisher(Bool, 'e_stop_gate', 10)
        self.speed_pub = self.create_publisher(SpeedLimit, 'speed_limit', 10)
        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self._tick)
        self.get_logger().info('safety gate: inhibited until the lidar, the odometry and a battery verdict are in')

    def _on_fresh(self, m):
        self.fresh_at = time.monotonic()
        fresh = {}
        for st in m.status:
            v = {kv.key: kv.value for kv in st.values}
            try:
                fresh[st.name] = (float(v.get('age_s', '-1')), float(v.get('lag_s', 'nan')))
            except ValueError:
                pass
        self.fresh = fresh

    def _on_camera(self, m):
        self.camera = m.data
        self.camera_at = time.monotonic()

    def _on_battery(self, m):
        self.battery_at = time.monotonic()
        try:
            self.battery = json.loads(m.data)
        except ValueError:
            pass

    def _on_where(self, m):
        try:
            self.where = json.loads(m.data).get('state')
        except ValueError:
            pass

    def _on_sensors(self, m):
        try:
            self.sensors = json.loads(m.data).get('state')
        except ValueError:
            pass

    def _on_tilt(self, m):
        self.tilt, self.tilt_at = m.data, time.monotonic()

    def _on_guard_flow(self, m):
        self.guard_flow, self.guard_flow_at = m.data, time.monotonic()

    def _on_watchdog(self, m):
        self.watchdog_at = time.monotonic()
        try:
            self.watchdog = list(json.loads(m.data).get('problems', []))
        except ValueError:
            pass

    def _facts(self, now):
        report = self.fresh_at is not None and now - self.fresh_at < 1.0
        f = {'scan_age': self._age('/scan', now, report), 'odom_age': self._age('/odometry/filtered', now, report),
             'vo_age': self._age('/vo', now, report), 'fresh_report': report, 'battery': self.battery,
             'battery_age': None if self.battery_at is None else now - self.battery_at,
             'camera_age': None if self.camera_at is None else now - self.camera_at,
             'uptime': now - self.started,
             'tilt_age': None if self.tilt_at is None else now - self.tilt_at,
             'guard_flow_age': None if self.guard_flow_at is None else now - self.guard_flow_at,
             'watchdog_age': None if self.watchdog_at is None else now - self.watchdog_at,
             'where': self.where, 'sensors': self.sensors, 'tilt': self.tilt, 'guard_flow': self.guard_flow, 'camera': self.camera,
             'watchdog': self.watchdog, 'allow_bench': bool(self.get_parameter('allow_bench').value)}
        for key, _ in LOCKS:
            f['lock_' + key] = self.locks.get(key, False)
        return f

    def _age(self, topic, now, report):
        """A stream's age: since it last arrived (topic_watch's figure plus the report's own age),
        plus how far its stamp was behind ROS time then - believed within a minute, else only
        the arrival counts. None = unknown (never seen, or no fresh report)."""
        if not report or topic not in self.fresh:
            return None
        age, lag = self.fresh[topic]
        if age < 0:
            return None
        lag = lag if lag == lag and 0.0 < lag < 60.0 else 0.0     # nan, negative, absurd: arrival only
        return age + (now - self.fresh_at) + lag

    def _tick(self):
        now = time.monotonic()
        state, reasons, speed = decide(now, self._facts(now))
        changed = state != self.state or reasons != self.reasons
        if state != self.state:
            self.since = now
            line = f'safety gate: {self.state} -> {state}' + (': ' + '; '.join(reasons) if reasons else '')
            # two call sites on purpose: rclpy pins a severity to each source line
            # ("Logger severity cannot be changed between calls" killed the process, 2026-10-02)
            if state in ('stopped', 'inhibited'):
                self.get_logger().warning(line)
            else:
                self.get_logger().info(line)
        if speed != self.speed:
            m = SpeedLimit()
            m.percentage = True
            m.speed_limit = speed if speed else 100.0
            self.speed_pub.publish(m)
        self.state, self.reasons, self.speed = state, reasons, speed
        permit = state in ('ok', 'degraded')
        # the mux lock: raised while not permitted, repeated every second (a late mux hears it),
        # released once on the change
        if not permit and (changed or now - self.last_lock_pub > 1.0):
            self.lock_pub.publish(Bool(data=True))
            self.last_lock_pub = now
        elif permit and changed:
            self.lock_pub.publish(Bool(data=False))
        if changed or now - getattr(self, '_pub_at', 0.0) > 1.0:
            self._pub_at = now
            msg = String()
            msg.data = json.dumps({'state': state, 'reasons': reasons, 'speed_pct': speed,
                                   'since_s': round(now - self.since, 1), 'permit': permit})
            self.state_pub.publish(msg)
            self.permit_pub.publish(Bool(data=permit))


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=args)
    node = SafetyGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except (Exception, KeyboardInterrupt):  # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
