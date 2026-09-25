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

"""Watchdog: keep Rosie's pieces running, cheaply.

    ros2 run jetnano_bringup watchdog         (robot.launch.py starts it, use_watchdog)
    ros2 topic echo /watchdog/events          what it noticed and did
    ros2 topic echo /watchdog/status          the whole picture, JSON, every 5 s
    tail ~/watchdog/events.jsonl              the history, for finding the flaky parts

Every node already respawns when it dies. This catches the rest: a node that
is running but has gone SILENT (a lidar whose motor stalled, an IMU on a
loose wire, a camera stream that froze, a microphone stream that ended, a
listening thread that hung), a node that did not come back, the collision
guard stuck inactive, the web page not answering, the camera container
stopped, and the machine itself (disk, memory, heat, Wi-Fi).

The data rates come from jetnano_watchdog's C++ ``topic_watch``, which counts
the streams without unpacking them and sums them up once a second; this node
reads that summary every two seconds. Measured 2026-09-25: about 2 % of one
core for the pair.

When something goes silent it works up a ladder, one step at a time, waiting
after each for the respawn to land:

    lidar       restart the driver -> reset its USB port (twice)
    lidar filter restart the scan filter
    IMU         restart the driver                     (then: check the wiring)
    odometry    restart the EKF
    visual      leave it to vo_watchdog -> restart the container's launch
      odometry    -> restart the container
    3D map      restart the container's launch
    microphone  restart the ears
    listening   restart listen
    web page    restart web_teleop;   video: restart web_video_server
    container   start it if it stopped
    guard       bring the collision guard back to active

Each item acts at most ``max_actions_per_hour`` times, then gives up and says
so, so it can never loop. It never restarts the whole robot by itself: that
would restart mapping away from the parking spot and spoil the map.

It speaks only when it gives up or something it had given up on comes back
(``announce``: failures | all | none), in English when she is in English
mode ("My lidar stopped and I can't get it back. Please check its cable."),
otherwise a sad sound.
"""

import json
import os
import re
import shutil
import signal
import subprocess
import threading
import time
import urllib.request

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

CONTAINER = 'isaac_vo'
CONTAINER_LAUNCH = r'cuvslam_.*\.launch\.py'
LIDAR_USB = '10c4:ea60'


def proc(path: str) -> str:
    """pgrep -f pattern for an installed ROS executable."""
    return f'/lib/{path}( |$)'


# key: (spoken label, topic, stale_s, min_hz, need, depends, ladder)
#   need: always = must be there from the start; seen = watched once it has
#         published (optional parts: nvblox, battery, cliff sensors); slam =
#         only while jetnano-slam runs
#   ladder: (action, argument, seconds to wait before the next step)
TOPICS = {
    'lidar': ('lidar', '/scan_raw', 2.0, 5.0, 'always', None, [
        ('signal', proc('rplidar_ros/rplidar_composition'), 15.0),
        ('usb_reset', LIDAR_USB, 25.0),          # the known cure for a wedged lidar
        ('usb_reset', LIDAR_USB, 40.0)]),
    'scan_filter': ('lidar filter', '/scan', 2.0, 5.0, 'always', 'lidar', [
        ('signal', proc('laser_filters/scan_to_scan_filter_chain'), 10.0),
        ('signal', proc('laser_filters/scan_to_scan_filter_chain'), 20.0)]),
    'imu': ('I M U', '/imu/data', 1.5, 25.0, 'always', None, [
        ('signal', proc('bno055/bno055'), 15.0),
        ('signal', proc('bno055/bno055'), 30.0)]),
    'odometry': ('odometry', '/odometry/filtered', 1.5, 10.0, 'always', None, [
        ('signal', proc('robot_localization/ekf_node'), 10.0),
        ('signal', proc('robot_localization/ekf_node'), 20.0)]),
    'vo': ('visual odometry', '/vo', 3.0, 10.0, 'seen', None, [
        ('wait', None, 25.0),                 # vo_watchdog restarts the launch after 5 s
        ('container_launch', None, 75.0),
        ('docker_restart', None, 120.0)]),
    'nvblox': ('3D map', '/nvblox_node/static_occupancy_grid', 5.0, 2.0, 'seen', 'vo', [
        ('container_launch', None, 75.0)]),
    'mic': ('microphone', '/sound/audio', 3.0, 5.0, 'seen', None, [
        ('signal', proc('jetnano_bringup/ears'), 12.0),
        ('signal', proc('jetnano_bringup/ears'), 30.0)]),
    'listening': ('listening', '/speech/heartbeat', 8.0, None, 'seen', 'mic', [
        ('signal', proc('jetnano_bringup/listen'), 15.0),
        ('signal', proc('jetnano_bringup/listen'), 30.0)]),
    'battery': ('battery monitor', '/battery', 5.0, None, 'seen', None, [
        ('signal', proc('jetnano_bringup/battery_monitor'), 15.0)]),
    'cliff': ('cliff sensors', '/cliff/ranges', 3.0, None, 'seen', None, [
        ('signal', proc('jetnano_bringup/cliff_guard'), 15.0)]),
    'map': ('map', '/map', 30.0, None, 'slam', None, []),       # report only: never restart mapping
}

# Nodes that respawn by themselves: reported if one stays away.
NODES = ('pca9685', 'twist_mux', 'collision_guard', 'lifecycle_manager_guard', 'robot_state_publisher',
         'ekf_filter_node', 'rplidar', 'scan_filter', 'bno055', 'tilt_guard', 'vo_watchdog', 'web_teleop',
         'sounds', 'speak', 'listen', 'brain', 'ears', 'motion_watch', 'topic_watch')
NODE_LABELS = {'pca9685': 'motor driver', 'twist_mux': 'command mixer', 'collision_guard': 'collision guard',
               'ekf_filter_node': 'odometry', 'rplidar': 'lidar driver', 'bno055': 'I M U driver',
               'web_teleop': 'web page', 'robot_state_publisher': 'robot model'}

HTTP = {
    'web page': ('http://127.0.0.1:8081/status', [('signal', proc('jetnano_teleop/web_teleop'), 12.0)]),
    'video': ('http://127.0.0.1:8080/', [('container_pkill', 'web_video_server', 20.0)]),
}

GIVE_UP_HINT = {
    'lidar': 'Please check its cable.', 'imu': 'Please check its wiring.', 'mic': 'Please check the microphone.',
    'vo': 'Please check the camera cable.', 'cliff': 'Please check their wiring.',
}


class Item:
    """One thing being watched and where it is on its ladder."""

    def __init__(self, key, label, ladder, need='always', depends=None):
        self.key, self.label, self.ladder, self.need, self.depends = key, label, ladder, need, depends
        self.state = 'ok'                # ok | bad | gave_up
        self.since = 0.0
        self.step = 0
        self.next_at = 0.0
        self.actions = []                # times of actions in the last hour
        self.note = ''
        self.seen = False
        self.slow_since = 0.0
        self.slow_said = 0.0
        self.gave_up_at = 0.0


class Watchdog(Node):

    def __init__(self):
        super().__init__('watchdog')
        self.declare_parameter('startup_grace_s', 90.0)
        self.declare_parameter('check_s', 2.0)
        self.declare_parameter('system_check_s', 30.0)
        self.declare_parameter('max_actions_per_hour', 5)
        self.declare_parameter('retry_after_give_up_s', 600.0)
        self.declare_parameter('node_missing_s', 20.0)
        self.declare_parameter('announce', 'failures')         # failures | all | none
        self.declare_parameter('act', True)                    # false: watch and report only
        self.declare_parameter('log_file', os.path.expanduser('~/watchdog/events.jsonl'))
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.grace = float(p('startup_grace_s'))
        self.max_per_hour = int(p('max_actions_per_hour'))
        self.retry_s = float(p('retry_after_give_up_s'))
        self.lock = threading.RLock()
        self.node_missing_s = float(p('node_missing_s'))
        self.announce = str(p('announce'))
        self.act = bool(p('act'))
        self.log_file = os.path.expanduser(str(p('log_file')))
        os.makedirs(os.path.dirname(self.log_file), exist_ok=True)

        self.t_start = time.monotonic()
        self.grace_until = self._grace_end()
        self.topics = {}                 # topic -> (hz, age) from topic_watch
        self.topics_at = 0.0
        self.items = {k: Item(k, v[0], v[6], v[4], v[5]) for k, v in TOPICS.items()}
        self.http_items = {k: Item(k, k, v[1]) for k, v in HTTP.items()}
        self.node_items = {n: Item(n, NODE_LABELS.get(n, n.replace('_', ' ')), []) for n in NODES}
        self.node_seen = {}
        self.sys = {}
        self.english = False
        self.container_busy_until = 0.0  # one container action at a time
        self.slam_active = False
        self.guard_inactive_since = 0.0

        self.events_pub = self.create_publisher(String, 'watchdog/events', 10)
        self.status_pub = self.create_publisher(
            String, 'watchdog/status', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.say_pub = self.create_publisher(String, 'say', 10)
        self.speak_pub = self.create_publisher(String, 'speak', 10)
        self.create_subscription(DiagnosticArray, 'watchdog/topics', self._on_topics, 10)
        self.create_subscription(Bool, 'sound/english', lambda m: setattr(self, 'english', m.data),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._guard_client = None
        self._manage_client = None
        self.create_timer(float(p('check_s')), self._check)
        self.create_timer(float(p('system_check_s')), self._system)
        self.create_timer(10.0, self._slow_checks)
        self.create_timer(5.0, self._publish_status)
        self._event('start', 'watchdog', f'watching {len(self.items)} streams, {len(NODES)} nodes, '
                    f'{len(HTTP)} web endpoints; acting: {self.act}')

    # ---------------------------------------------------------------- input --

    def _on_topics(self, msg: DiagnosticArray) -> None:
        now = time.monotonic()
        for s in msg.status:
            v = {kv.key: kv.value for kv in s.values}
            try:
                self.topics[s.name] = (float(v.get('hz', 0)), float(v.get('age_s', -1)))
            except ValueError:
                continue
        self.topics_at = now

    # --------------------------------------------------------------- checks --

    def _in_grace(self) -> bool:
        # The long grace is for the robot starting up; if only the watchdog
        # restarted, a short one is enough.
        return time.monotonic() < self.grace_until

    def _grace_end(self) -> float:
        now = time.monotonic()
        try:
            out = subprocess.run(['systemctl', 'show', '-p', 'ActiveEnterTimestampMonotonic', '--value',
                                  'jetnano-robot'], capture_output=True, text=True, timeout=5).stdout.strip()
            stack_started = int(out) / 1e6
            if stack_started > 0:
                return max(now + 15.0, stack_started + self.grace)
        except (OSError, ValueError, subprocess.TimeoutExpired):
            pass
        return now + self.grace

    def _check(self) -> None:
        now = time.monotonic()
        if now - self.topics_at > 10.0:
            # the counter itself is gone: its respawn will bring it back
            if not self._in_grace():
                self._bad(self.node_items['topic_watch'], 'no summary from topic_watch', act=False)
            return
        for key, item in self.items.items():
            label, topic, stale, min_hz, need, depends, _ = TOPICS[key]
            hz, age = self.topics.get(topic, (0.0, -1.0))
            if age >= 0:
                item.seen = True
            expected = (need == 'always' or (need == 'seen' and item.seen)
                        or (need == 'slam' and self.slam_active))
            if not expected:
                continue
            if depends and self.items[depends].state != 'ok':
                continue                  # its source is down: fix that first
            silent = age < 0 or age > stale
            if silent:
                if self._in_grace():
                    continue
                why = 'never started' if age < 0 else f'silent for {age:.0f} s'
                self._bad(item, why)
            else:
                self._good(item, f'{hz:.1f} Hz')
                if min_hz and hz < min_hz and hz > 0:
                    item.slow_since = item.slow_since or now
                    if now - item.slow_since > 30.0 and now - item.slow_said > 600.0:
                        item.slow_said = now
                        self._event('slow', item.label, f'{hz:.1f} Hz, expected at least {min_hz:.0f}')
                else:
                    item.slow_since = 0.0

    def _slow_checks(self) -> None:
        """Every 10 s: nodes present, web endpoints, the guard's lifecycle."""
        if self._in_grace():
            return
        now = time.monotonic()
        names = set(self.get_node_names())
        for n, item in self.node_items.items():
            if n == 'topic_watch':
                continue
            if n in names:
                self.node_seen[n] = now
                self._good(item, 'running')
            elif n in self.node_seen and now - self.node_seen[n] > self.node_missing_s:
                self._bad(item, f'gone for {now - self.node_seen[n]:.0f} s', act=False)
        threading.Thread(target=self._check_http, daemon=True).start()
        self._check_guard(names)

    def _check_http(self) -> None:
        for name, (url, _ladder) in HTTP.items():
            item = self.http_items[name]
            try:
                with urllib.request.urlopen(url, timeout=3.0) as r:
                    ok = r.status == 200
            except Exception:      # noqa: BLE001 - any failure is "not answering"
                ok = False
            if ok:
                item.seen = True
                self._good(item, 'answering')
            elif item.seen:
                self._bad(item, 'not answering')

    def _check_guard(self, names) -> None:
        """The collision guard is a lifecycle node: running is not enough, it must be active."""
        if 'collision_guard' not in names:
            return
        if self._guard_client is None:
            from lifecycle_msgs.srv import GetState
            from nav2_msgs.srv import ManageLifecycleNodes
            self._GetState, self._Manage = GetState, ManageLifecycleNodes
            self._guard_client = self.create_client(GetState, '/collision_guard/get_state')
            self._manage_client = self.create_client(ManageLifecycleNodes, '/lifecycle_manager_guard/manage_nodes')
        if not self._guard_client.service_is_ready():
            return
        fut = self._guard_client.call_async(self._GetState.Request())
        fut.add_done_callback(self._on_guard_state)

    def _on_guard_state(self, fut) -> None:
        try:
            state = fut.result().current_state
        except Exception:      # noqa: BLE001
            return
        item = self.node_items['collision_guard']
        now = time.monotonic()
        if state.id == 3:      # active
            self.guard_inactive_since = 0.0
            return
        self.guard_inactive_since = self.guard_inactive_since or now
        if now - self.guard_inactive_since > 15.0:
            self._bad(item, f'{state.label}, not active')
            if self.act and self._allowed(item) and self._manage_client.service_is_ready():
                item.actions.append(now)
                for command in (3, 0):              # RESET, then STARTUP
                    req = self._Manage.Request()
                    req.command = command
                    self._manage_client.call_async(req)
                self._event('act', item.label, 'asked the lifecycle manager to reset and start it')
                self.guard_inactive_since = now

    def _system(self) -> None:
        """Every 30 s: the machine, the container, mapping."""
        s = {}
        try:
            total, used, free = shutil.disk_usage('/')
            s['disk_free_gb'] = round(free / 1e9, 1)
            with open('/proc/meminfo') as f:
                mem = {ln.split(':')[0]: int(ln.split()[1]) for ln in f}
            s['mem_available_mb'] = mem.get('MemAvailable', 0) // 1024
            for zone in sorted(os.listdir('/sys/class/thermal')):
                try:
                    with open(f'/sys/class/thermal/{zone}/type') as f:
                        kind = f.read().strip()
                    with open(f'/sys/class/thermal/{zone}/temp') as f:
                        s[f'temp_{kind}'] = int(f.read().strip()) / 1000.0
                except (OSError, ValueError):
                    continue
        except OSError:
            pass
        s['container'] = self._run(['docker', 'inspect', '-f', '{{.State.Running}}', CONTAINER]).strip() == 'true'
        link = self._run(['iw', 'dev', 'wlP1p1s0', 'link'])
        m = re.search(r'signal: (-?\d+)', link)
        s['wifi_dbm'] = int(m.group(1)) if m else None
        self.slam_active = self._run(['systemctl', 'is-active', 'jetnano-slam']).strip() == 'active'
        s['slam'] = self.slam_active
        self.sys = s
        if self._in_grace():
            return
        warn = []
        if s.get('disk_free_gb', 99) < 3:
            warn.append(f'disk nearly full, {s["disk_free_gb"]} GB free')
        if s.get('mem_available_mb', 9999) < 300:
            warn.append(f'memory low, {s["mem_available_mb"]} MB available')
        hot = max((v for k, v in s.items() if k.startswith('temp_')), default=0)
        if hot > 90:
            warn.append(f'running hot, {hot:.0f} C')
        if s['wifi_dbm'] is None:
            warn.append('Wi-Fi not connected')
        for w in warn:
            self._event('system', 'system', w)
        item = self.http_items.setdefault('container', Item('container', 'camera container', []))
        if s['container']:
            self._good(item, 'running')
        else:
            self._bad(item, 'stopped', act=False)
            if self.act and self._allowed(item) and time.monotonic() > self.container_busy_until:
                item.actions.append(time.monotonic())
                self.container_busy_until = time.monotonic() + 90.0
                self._event('act', item.label, 'docker start')
                threading.Thread(target=self._run, args=(['docker', 'start', CONTAINER],), daemon=True).start()

    # ---------------------------------------------------------------- state --

    def _allowed(self, item) -> bool:
        now = time.monotonic()
        item.actions = [t for t in item.actions if now - t < 3600.0]
        return len(item.actions) < self.max_per_hour

    def _bad(self, item, why: str, act: bool = True) -> None:
        with self.lock:
            now = time.monotonic()
            item.note = why
            if item.state == 'ok':
                item.state, item.since, item.step, item.next_at = 'bad', now, 0, now
                self._event('down', item.label, why)
            if item.state == 'gave_up':
                if now - item.gave_up_at < self.retry_s:
                    return
                item.gave_up_at, item.step, item.next_at = now, 0, now   # a quiet retry: a cable may be back
            if not act or not self.act or now < item.next_at:
                return
            if item.step >= len(item.ladder):
                if item.ladder:
                    self._give_up(item, why)
                return
            if not self._allowed(item):
                self._give_up(item, f'{why}; {self.max_per_hour} tries this hour')
                return
            action, arg, settle = item.ladder[item.step]
            item.step += 1
            item.next_at = now + settle
            if action == 'wait':
                return
            if action in ('container_launch', 'docker_restart', 'container_pkill'):
                if now < self.container_busy_until:
                    item.step -= 1                     # someone else is restarting it: try later
                    item.next_at = self.container_busy_until
                    return
                self.container_busy_until = now + settle
            item.actions.append(now)
            self._event('act', item.label, self._describe(action, arg))
            if self.announce == 'all':
                self._announce(f'My {item.label} stopped. Restarting it.', 'hm')
            threading.Thread(target=self._do, args=(action, arg), daemon=True).start()

    def _good(self, item, note: str) -> None:
        with self.lock:
            item.note = note
            if item.state != 'ok':
                gone = time.monotonic() - item.since
                self._event('back', item.label, f'after {gone:.0f} s')
                if item.state == 'gave_up' or self.announce == 'all':
                    self._announce(f'My {item.label} is back.', 'ok')
                item.state, item.step = 'ok', 0

    def _give_up(self, item, why: str) -> None:
        if item.state == 'gave_up':
            self._event('still_down', item.label, why)          # a retry that did not work: no second announcement
            return
        item.state, item.gave_up_at = 'gave_up', time.monotonic()
        self._event('gave_up', item.label, why)
        hint = GIVE_UP_HINT.get(item.key, '')
        self._announce(f"My {item.label} stopped and I can't get it back. {hint}".strip(), 'sad')

    # -------------------------------------------------------------- actions --

    @staticmethod
    def _describe(action: str, arg) -> str:
        return {'signal': f'restart {str(arg).split("/")[-1].split("(")[0]}',
                'usb_reset': f'reset USB device {arg}',
                'container_launch': 'restart the container launch',
                'docker_restart': 'restart the container',
                'container_pkill': f'restart {arg} in the container'}.get(action, action)

    def _run(self, cmd, timeout=15.0) -> str:
        try:
            return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout).stdout
        except (OSError, subprocess.TimeoutExpired):
            return ''

    def _do(self, action: str, arg) -> None:
        try:
            if action == 'signal':
                self._restart_process(arg)
            elif action == 'usb_reset':
                self._usb_reset(arg)
            elif action == 'container_launch':
                self._run(['docker', 'exec', CONTAINER, 'pkill', '-INT', '-f', CONTAINER_LAUNCH])
            elif action == 'docker_restart':
                self._run(['docker', 'restart', '-t', '20', CONTAINER], timeout=90.0)
            elif action == 'container_pkill':
                self._run(['docker', 'exec', CONTAINER, 'pkill', '-INT', '-f', arg])
        except Exception as exc:      # noqa: BLE001 - never let an action kill the watchdog
            self.get_logger().warning(f'{action} {arg}: {exc}')

    def _pids(self, pattern: str):
        out = self._run(['pgrep', '-f', pattern])
        return [int(x) for x in out.split() if x.isdigit() and int(x) != os.getpid()]

    def _restart_process(self, pattern: str) -> None:
        """SIGINT, then TERM, then KILL; the launch file respawns it."""
        pids = self._pids(pattern)
        if not pids:
            self.get_logger().info(f'{pattern}: not running, its respawn is due')
            return
        for sig, wait in ((signal.SIGINT, 5.0), (signal.SIGTERM, 5.0), (signal.SIGKILL, 0.0)):
            for pid in pids:
                try:
                    os.kill(pid, sig)
                except ProcessLookupError:
                    pass
            deadline = time.monotonic() + wait
            while time.monotonic() < deadline and any(os.path.exists(f'/proc/{p}') for p in pids):
                time.sleep(0.2)
            pids = [p for p in pids if os.path.exists(f'/proc/{p}')]
            if not pids:
                return

    def _usb_reset(self, vid_pid: str) -> None:
        """Re-enumerate a USB device (the fix for a wedged lidar), then restart its driver."""
        vid, pid = vid_pid.split(':')
        base = '/sys/bus/usb/devices'
        for dev in os.listdir(base):
            try:
                with open(f'{base}/{dev}/idVendor') as f:
                    v = f.read().strip()
                with open(f'{base}/{dev}/idProduct') as f:
                    p = f.read().strip()
            except OSError:
                continue
            if (v, p) == (vid, pid):
                auth = f'{base}/{dev}/authorized'
                for value in ('0', '1'):
                    subprocess.run(['sudo', '-n', 'tee', auth], input=value, text=True,
                                   capture_output=True, timeout=10)
                    time.sleep(1.5)
                self.get_logger().info(f'USB {vid_pid} at {dev} re-enumerated')
                time.sleep(2.0)
                self._restart_process(proc('rplidar_ros/rplidar_composition'))
                return
        self.get_logger().warning(f'USB {vid_pid} not found: is it unplugged?')

    # ------------------------------------------------------------ reporting --

    def _event(self, kind: str, what: str, detail: str) -> None:
        rec = {'t': time.strftime('%Y-%m-%d %H:%M:%S'), 'kind': kind, 'what': what, 'detail': detail}
        text = f'{kind}: {what} - {detail}'
        # separate call sites: rclpy refuses one line that logs at two severities
        if kind in ('down', 'gave_up', 'system', 'still_down'):
            self.get_logger().warning(text)
        else:
            self.get_logger().info(text)
        msg = String()
        msg.data = json.dumps(rec)
        self.events_pub.publish(msg)
        try:
            with open(self.log_file, 'a') as f:
                f.write(json.dumps(rec) + '\n')
        except OSError:
            pass

    def _announce(self, words: str, mood: str) -> None:
        if self.announce == 'none':
            return
        msg = String()
        if self.english:
            msg.data = words
            self.speak_pub.publish(msg)
        else:
            msg.data = mood
            self.say_pub.publish(msg)

    def _publish_status(self) -> None:
        problems = []
        out = {}
        for group in (self.items, self.http_items, self.node_items):
            for key, item in group.items():
                if item.state != 'ok':
                    problems.append(f'{item.label}: {item.note}' + (' (gave up)' if item.state == 'gave_up' else ''))
                if group is self.items:
                    hz, age = self.topics.get(TOPICS[key][1], (0.0, -1.0))
                    out[key] = {'state': item.state, 'hz': round(hz, 1), 'age_s': round(age, 1), 'seen': item.seen}
        msg = String()
        msg.data = json.dumps({'ok': not problems, 'problems': problems, 'streams': out, 'system': self.sys,
                               'grace': self._in_grace()})
        self.status_pub.publish(msg)


def main(args=None):
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=args)
    node = Watchdog()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
