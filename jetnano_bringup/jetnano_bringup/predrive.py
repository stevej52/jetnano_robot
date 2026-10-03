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

"""Pre-drive check in one process: GO or NO-GO with every reason, in a few seconds.

    ros2 run jetnano_bringup predrive            -> exit 0 = GO

The same checks as scripts/predrive.sh (everything that stopped a drive on 2026-09-30), but
as ONE node that discovers the graph once and listens to every topic at the same time: the
shell version ran thirteen `ros2` commands in a row, each with its own 4 s of discovery, and
cost 55 s of a 2 min 21 s wait from "go" to wheels (drive 17, 2026-10-01; Steve: "two and a
half minutes is way too long to just be sitting there").

Checks: the robot service is up; she knows where she is (where_am_i: placed, not on the
bench); map->odom is fresh and map->base_footprint resolves; no twist_mux lock; the collision
guard is active AND its output follows its input (a stale source makes it swallow every
command - drive 16); exactly one Nav2, active; visual odometry flows; the motion check is on;
the driver armed the ESC and enabled its outputs this boot; the pack is up to a drive; and
whether a recording runs (a warning). Topics are given 4 s to show up.
"""

import json
import math
import os
import subprocess
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

LISTEN_S = 4.0


class Predrive(Node):

    def __init__(self):
        super().__init__('predrive')
        self.seen = {}                                   # topic key -> last message (or stamp)
        self.mux_cmds = 0
        self.guard_out = 0
        be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # where_am_i speaks once at boot and the battery every few seconds: take them latched
        # and plain, whichever way they are published
        for qos in (10, latched):
            self.create_subscription(String, 'where_am_i/state', lambda m: self._got('where', m.data), qos)
            self.create_subscription(String, 'battery/level', lambda m: self._got('battery', m.data), qos)
        self.create_subscription(Odometry, 'vo', lambda m: self._got('vo', time.monotonic()), be)
        self.create_subscription(DiagnosticArray, 'diagnostics', self._on_diag, 10)
        self.create_subscription(String, 'safety/state', lambda m: self._got('gate', m.data), latched)
        self.create_subscription(Twist, 'cmd_vel_mux', lambda m: self._count('mux_cmds', m), 10)
        self.create_subscription(Twist, 'cmd_vel', lambda m: self._count('guard_out', m), 10)
        self.buf = Buffer()
        self.tf = TransformListener(self.buf, self)
        self.guard_state = self.create_client(GetState, '/collision_guard/get_state')
        self.nav2_active = self.create_client(Trigger, '/lifecycle_manager_navigation/is_active')
        self.motion_param = self.create_client(GetParameters, '/safety_monitor/get_parameters')

    def _got(self, key, value):
        self.seen[key] = value

    def _count(self, key, m):
        if abs(m.linear.x) > 0.02 or abs(m.angular.z) > 0.02:
            setattr(self, key, getattr(self, key) + 1)

    def _on_diag(self, m):
        locks = self.seen.setdefault('locks', {})
        for st in m.status:
            for kv in st.values:
                if kv.key.startswith('lock locks.'):
                    locks[kv.key.split('.', 1)[1]] = kv.value

    def call(self, client, request, timeout=3.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        fut = client.call_async(request)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result() if fut.done() else None


def main(args=None):
    rclpy.init(args=args)
    n = Predrive()
    ok, bad, warn = [], [], []
    t0 = time.monotonic()
    while time.monotonic() - t0 < 1.0:                 # a second of discovery before asking anyone
        rclpy.spin_once(n, timeout_sec=0.05)
    # the services (their answers arrive while the topics are collected); one more try each
    guard = n.call(n.guard_state, GetState.Request()) or n.call(n.guard_state, GetState.Request())
    nav2 = n.call(n.nav2_active, Trigger.Request()) or n.call(n.nav2_active, Trigger.Request())
    motion = n.call(n.motion_param, GetParameters.Request(names=['motion.enabled'])) or n.call(n.motion_param, GetParameters.Request(names=['motion.enabled']))
    while time.monotonic() - t0 < LISTEN_S:
        rclpy.spin_once(n, timeout_sec=0.05)
        # 'gate' too: under UDP-only discovery its latched verdict came a moment after the other four,
        # and predrive had already stopped listening (drive 26, 2026-10-02: "safety gate: no verdict")
        if all(k in n.seen for k in ('where', 'battery', 'vo', 'locks', 'gate')) and n.buf.can_transform('map', 'base_footprint', Time()):
            if time.monotonic() - t0 > 1.5:               # a moment more for the mux/guard counts
                break

    def o(s):
        ok.append(s)
        print(f'  ok    {s}')

    def b(s):
        bad.append(s)
        print(f'  NO-GO {s}')

    def w(s):
        warn.append(s)
        print(f'  warn  {s}')

    up = subprocess.run(['systemctl', 'is-active', '-q', 'jetnano-robot.service']).returncode == 0
    o('robot software up') if up else b('jetnano-robot.service is not active')

    where = n.seen.get('where')
    if where and '"placed"' in where:
        try:
            fit = json.loads(where).get('why', '')
        except ValueError:
            fit = ''
        o(f'placed on the map ({fit.split(",")[0]})')
    elif where and '"bench"' in where:
        b('she is on the bench')
    else:
        b(f'not localised: {where[:60] if where else "no verdict from where_am_i"}')

    try:
        tr = n.buf.lookup_transform('map', 'odom', Time())
        age = n.get_clock().now().nanoseconds / 1e9 - Time.from_msg(tr.header.stamp).nanoseconds / 1e9
        o(f'map->odom fresh ({age:.1f} s)') if age <= 5 else b(f'map->odom is {age:.0f} s old (slam_toolbox stopped re-publishing)')
    except Exception:  # noqa: BLE001
        b('no map->odom transform')
    o('map->base_footprint resolves') if n.buf.can_transform('map', 'base_footprint', Time()) else b('map->base_footprint does not resolve')

    locks = n.seen.get('locks')
    if not locks:
        w('twist_mux diagnostics not seen')
    elif any(v == 'locked' for v in locks.values()):
        b('twist_mux lock engaged: ' + ' '.join(k for k, v in locks.items() if v == 'locked'))
    else:
        o('no twist_mux lock engaged')

    if guard is not None and guard.current_state.label == 'active':
        o('collision guard active')
    else:
        b(f'collision guard: {guard.current_state.label if guard else "no answer"}')
    if n.mux_cmds >= 5 and n.guard_out == 0:
        b(f'the guard swallows every command ({n.mux_cmds} in, 0 out): a stale source - restart it (drive 16, 2026-10-01)')

    launches = subprocess.run(['pgrep', '-fc', '^/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch jetnano_navigation navigation.launch.py'],
                              capture_output=True, text=True).stdout.strip() or '0'
    containers = subprocess.run(['pgrep', '-fc', '^/opt/ros/jazzy/lib/rclcpp_components/component_container_isolated'],
                                capture_output=True, text=True).stdout.strip() or '0'
    nl, nc = int(launches), int(containers)
    if nl == 0 and nc == 0:
        b('Nav2 not running (nav2_ctl.sh start)')
    elif nl > 1 or nc > 1:
        b(f'more than one Nav2 ({nl} launch, {nc} container): nav2_ctl.sh stop, then start')
    elif nav2 is not None and nav2.success:
        o('one Nav2, active')
    else:
        b('Nav2 running but not active')

    o('visual odometry flowing') if 'vo' in n.seen else b('no /vo')

    if motion is not None and motion.values and motion.values[0].bool_value:
        o('motion check on')
    else:
        b('motion check is off or unknown (ros2 param set /safety_monitor motion.enabled true)')

    boot = subprocess.run(['journalctl', '-b', '-o', 'cat'], capture_output=True, text=True).stdout
    arm = [ln for ln in boot.splitlines() if 'pca9685]' in ln and ('ESC armed' in ln or 'outputs ENABLED' in ln or 'outputs DISABLED' in ln)]
    o('ESC armed this boot') if any('ESC armed' in ln for ln in arm) else b('the motor driver never armed the ESC this boot')
    outs = [ln for ln in arm if 'outputs' in ln]
    o('servo outputs enabled') if outs and 'ENABLED' in outs[-1] else b('servo outputs are DISABLED')

    # on the spot? (drive 16, 2026-10-01: started 1 m off it, nose to the couch, and could not move)
    try:
        tr = n.buf.lookup_transform('map', 'base_footprint', Time()).transform
        qx = tr.rotation
        yaw = math.degrees(math.atan2(2 * (qx.w * qx.z + qx.x * qx.y), 1 - 2 * (qx.y * qx.y + qx.z * qx.z)))
        off = math.hypot(tr.translation.x, tr.translation.y)
        line = f'start x {tr.translation.x:+.2f} y {tr.translation.y:+.2f} heading {yaw:+.0f} deg, {off:.2f} m from the spot'
        o(line) if off <= 0.6 else b(line + ' - not on the parking spot (drive her there first, facing the hall)')
    except Exception:  # noqa: BLE001
        b('no map pose for the start check')

    bat = n.seen.get('battery')
    try:
        bj = json.loads(bat) if bat else {}
    except ValueError:
        bj = {}
    volts, level = bj.get('volts'), bj.get('level', '')
    if volts is None:
        w('no battery reading')
    elif level in ('low', 'flat'):
        b(f'battery {level} ({volts} V)')
    elif volts < 10.8:
        w(f'pack at {volts} V: a short drive at most (low line 10.5)')
    else:
        o(f'pack {volts} V ({level})')

    gate = n.seen.get('gate')
    try:
        gj = json.loads(gate) if gate else {}
    except ValueError:
        gj = {}
    if gj.get('state') in ('ok', 'degraded'):
        o(f'safety gate: {gj["state"]}' + (' (' + '; '.join(gj.get('reasons', [])) + ')' if gj.get('reasons') else ''))
    else:
        b(f'safety gate: {gj.get("state", "no verdict")}' + (' - ' + '; '.join(gj.get('reasons', [])) if gj.get('reasons') else ''))

    rec = subprocess.run([os.path.expanduser('~/ros2_ws/src/jetnano_robot/jetnano_bringup/scripts/drive_record.sh'), 'status'],
                         capture_output=True, text=True).stdout.splitlines()
    if rec and rec[0].startswith('recording'):
        o(rec[0])
    else:
        w('not recording (drive_record.sh start full)')

    n.destroy_node()
    rclpy.shutdown()
    print()
    took = time.monotonic() - t0
    if not bad:
        print(f'GO{f" (with {len(warn)} warning(s))" if warn else ""} in {took:.1f} s')
        return 0
    print(f'NO-GO: {len(bad)} reason(s) in {took:.1f} s')
    for s in bad:
        print(f'  - {s}')
    return 1


if __name__ == '__main__':
    sys.exit(main())
