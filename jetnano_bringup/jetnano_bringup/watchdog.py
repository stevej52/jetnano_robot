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
    web page    restart web_teleop;   video: restart csi_cameras
    container   start it if it stopped
    guard       bring the collision guard back to active
    GPU         failed to start at boot (its firmware did not load): reboot,
                once - only in the first 15 minutes after boot, never while
                driving or mapping, never twice in a row; otherwise say so and
                ask to be restarted (Steve's go-ahead, 2026-09-26)

Each item acts at most ``max_actions_per_hour`` times, then gives up and says
so, so it can never loop. Apart from that GPU case it never restarts the whole
robot by itself: that would restart mapping away from the parking spot and
spoil the map.

A process that keeps dying is not "starting": after four fresh starts in ten
minutes it is judged like anything else - but not in its first
crash_loop_young_s (10 s), while even a good start is still coming up.

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
from geometry_msgs.msg import Twist
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
#         published (optional parts: battery, cliff sensors); soon = part of
#         every boot but slow to come up: watched once it has published or
#         expect_boot_parts_after_s after the stack started, whichever is first
#         (2026-09-26 the GPU failed at boot, visual odometry never published,
#         and under 'seen' nothing ever looked); slam = only while jetnano-slam runs
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
        ('signal', proc('jetnano_bringup/bno055_lean'), 12.0),
        ('signal', proc('jetnano_bringup/bno055_lean'), 30.0)]),
    'odometry': ('odometry', '/odometry/filtered', 1.5, 10.0, 'always', None, [
        ('signal', proc('robot_localization/ekf_node'), 10.0),
        ('signal', proc('robot_localization/ekf_node'), 20.0)]),
    'vo': ('visual odometry', '/vo', 3.0, 10.0, 'soon', 'gpu', [
        ('wait', None, 25.0),                 # vo_watchdog restarts the launch after 5 s
        ('container_launch', None, 75.0),
        ('docker_restart', None, 120.0)]),
    'nvblox': ('3D map', '/nvblox_node/static_occupancy_grid', 5.0, 2.0, 'soon', 'vo', [
        ('container_launch', None, 75.0)]),
    # What the collision guard reads from the camera (grid_to_points): on 2026-09-27 it
    # dropped off the DDS graph while its process lived on, and the guard - rightly -
    # refused to drive on a stale source; nothing noticed until Steve tried to drive.
    'obstacles': ('camera obstacles for the guard', '/nvblox_node/obstacle_points', 3.0, 2.0, 'soon', 'nvblox', [
        ('signal', proc('jetnano_bringup/grid_to_points'), 10.0),
        ('signal', proc('jetnano_bringup/grid_to_points'), 20.0)]),
    # MOLA (robot.launch.py lidar_odom:=true): its launch respawns it when it dies, and
    # stopping mola-cli ends that launch, so a silent-but-alive one gets the same cure.
    # Judged by MOLA's own output, not the relay's /lidar_odom: since 2026-09-28 the relay
    # withholds whole episodes MOLA got wrong (up to 12 s on the recorded drives), and
    # that silence is not a dead MOLA - restarting it then only made things worse.
    'lidar_odom': ('lidar odometry', '/lidar_odometry/pose', 3.0, 3.0, 'seen', 'scan_filter', [
        ('signal', proc('mola_launcher/mola-cli'), 20.0),
        ('signal', proc('mola_launcher/mola-cli'), 40.0)]),
    'mic': ('microphone', '/sound/audio', 3.0, 5.0, 'seen', None, [
        ('signal', proc('jetnano_bringup/ears'), 12.0),
        ('signal', proc('jetnano_bringup/ears'), 30.0)]),
    # 20 s, not 8: twice on 2026-10-01 (20:08, 20:28 - the second with 92 MB of RAM free) a
    # healthy listen missed a few 2 s beats and was restarted for it (24 s deaf, the model
    # loaded again). A real hang still shows in 20 s.
    'listening': ('listening', '/speech/heartbeat', 20.0, None, 'seen', 'mic', [
        ('signal', proc('jetnano_bringup/listen'), 15.0),
        ('signal', proc('jetnano_bringup/listen'), 30.0)]),
    'battery': ('battery monitor', '/battery', 5.0, None, 'seen', None, [
        ('signal', proc('jetnano_bringup/battery_monitor'), 15.0)]),
    'cliff': ('cliff sensors', '/cliff/ranges', 3.0, None, 'seen', None, [
        ('signal', proc('jetnano_bringup/cliff_guard'), 15.0)]),
    'map': ('map', '/map', 45.0, None, 'slam', None, []),       # report only: never restart mapping
}

# The voice stack (jetnano-voice.service, voice.launch.py) is off unless asked for: while
# ~/voice/off exists (voice_switch.py) its streams and nodes are not watched at all.
VOICE_OFF_FLAG = os.path.expanduser('~/voice/off')
VOICE_STREAMS = ('mic', 'listening')
VOICE_NODES = ('sounds', 'speak', 'listen', 'ears')

# Nodes that respawn by themselves: reported if one stays away.
NODES = ('pca9685', 'twist_mux', 'collision_guard', 'lifecycle_manager_guard', 'robot_state_publisher',
         'ekf_filter_node', 'rplidar', 'scan_filter', 'bno055', 'safety_monitor', 'web_teleop',
         'sounds', 'speak', 'listen', 'brain', 'ears', 'motion_watch', 'topic_watch',
         'lidar_odom_relay')
NODE_LABELS = {'pca9685': 'motor driver', 'twist_mux': 'command mixer', 'collision_guard': 'collision guard',
               'ekf_filter_node': 'odometry', 'rplidar': 'lidar driver', 'bno055': 'I M U driver',
               'web_teleop': 'web page', 'robot_state_publisher': 'robot model',
               'safety_monitor': 'safety monitor', 'lidar_odom_relay': 'lidar odometry relay'}

HTTP = {
    # 2026-10-01 19:40: web_teleop died while its install was being rebuilt, the launch's own
    # respawn gave up ("No such file"), the watchdog found nothing to signal and gave up too -
    # the driving page was dead until the whole service was restarted. Hence the second step:
    # launch the node again ourselves.
    'web page': ('http://127.0.0.1:8081/status', [('signal', proc('jetnano_teleop/web_teleop'), 12.0),
                                                   ('launch', 'web_teleop.launch.py', 30.0)]),
    # the browsers' video (all three cameras) since 2026-09-27; NVIDIA's web_video_server on
    # 8080 is no longer on their path (it hung under load and ignored SIGINT)
    'video': ('http://127.0.0.1:8082/', [('signal', proc('jetnano_bringup/csi_cameras'), 15.0),
                                         ('signal', proc('jetnano_bringup/csi_cameras'), 30.0)]),
}

GIVE_UP_HINT = {
    'lidar': 'Please check its cable.', 'imu': 'Please check its wiring.', 'mic': 'Please check the microphone.',
    'vo': 'Please check the camera cable.', 'cliff': 'Please check their wiring.',
}


def stuck_sound_pids(proc='/proc'):
    """Pids of arecord/aplay processes in state D (uninterruptible sleep) right now."""
    pids = set()
    for name in os.listdir(proc):
        if not name.isdigit():
            continue
        try:
            with open(os.path.join(proc, name, 'stat')) as f:
                stat = f.read()
        except OSError:
            continue
        comm = stat[stat.find('(') + 1:stat.rfind(')')]
        state = stat[stat.rfind(')') + 2:stat.rfind(')') + 3]
        if comm in ('arecord', 'aplay') and state == 'D':
            pids.add(int(name))
    return pids


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
        self.http_fails = 0              # HTTP checks failed in a row
        self.slow_since = 0.0
        self.slow_said = 0.0
        self.gave_up_at = 0.0
        self.starts = set()              # recent start times of its process (crash-loop check)


class Watchdog(Node):

    def __init__(self):
        super().__init__('watchdog')
        self.declare_parameter('startup_grace_s', 90.0)
        self.declare_parameter('check_s', 2.0)
        self.declare_parameter('system_check_s', 30.0)
        self.declare_parameter('max_actions_per_hour', 5)
        self.declare_parameter('retry_after_give_up_s', 600.0)
        self.declare_parameter('node_missing_s', 40.0)
        # A process this young is still starting (listen loads its models for
        # ~10 s): not silent, just new. 2026-09-25 the watchdog fought five
        # deliberate restarts of listen in an hour and gave up on it.
        self.declare_parameter('young_process_s', 40.0)
        # ...but in a crash loop (four starts in ten minutes) only this long. With no
        # grace at all, on 2026-09-28 a series of deliberate EKF restarts made the
        # watchdog kill a respawned EKF one second old, just before it would have
        # published - a second outage for nothing.
        self.declare_parameter('crash_loop_young_s', 10.0)
        self.declare_parameter('expect_boot_parts_after_s', 180.0)
        # what the launch switched on (robot.launch.py vo:= and nvblox:=): a part
        # that is off is only watched once seen (review 2026-09-26: with nvblox
        # off, 'soon' restarted the camera pipeline for nothing)
        self.declare_parameter('expect_vo', True)
        self.declare_parameter('expect_nvblox', True)
        self.declare_parameter('gpu_reboot_within_s', 900.0)   # a GPU dead later than this is not the boot bug
        self.declare_parameter('announce', 'failures')         # failures | all | none
        self.declare_parameter('act', True)                    # false: watch and report only
        self.declare_parameter('log_file', os.path.expanduser('~/watchdog/events.jsonl'))
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.grace = float(p('startup_grace_s'))
        self.max_per_hour = int(p('max_actions_per_hour'))
        self.retry_s = float(p('retry_after_give_up_s'))
        self.lock = threading.RLock()
        self.node_missing_s = float(p('node_missing_s'))
        self.young_s = float(p('young_process_s'))
        self.crash_young_s = float(p('crash_loop_young_s'))
        self.expect_after = float(p('expect_boot_parts_after_s'))
        self.gpu_reboot_within = float(p('gpu_reboot_within_s'))
        self.announce = str(p('announce'))
        self.act = bool(p('act'))
        self.log_file = os.path.expanduser(str(p('log_file')))
        os.makedirs(os.path.dirname(self.log_file), exist_ok=True)

        self.t_start = time.monotonic()
        self.grace_until = self._grace_end()
        self.topics = {}                 # topic -> (hz, age) from topic_watch
        self.topics_at = 0.0
        self.items = {k: Item(k, v[0], v[6], v[4], v[5]) for k, v in TOPICS.items()}
        for key, flag in (('vo', bool(p('expect_vo'))), ('nvblox', bool(p('expect_nvblox'))),
                          ('obstacles', bool(p('expect_nvblox')))):
            if not flag:
                self.items[key].need = 'seen'
        self.http_items = {k: Item(k, k, v[1]) for k, v in HTTP.items()}
        self.node_items = {n: Item(n, NODE_LABELS.get(n, n.replace('_', ' ')), []) for n in NODES}
        self.node_seen = {}
        self.sys = {}
        self.english = False
        self.container_busy_until = 0.0  # one container action at a time
        self._relaunched = {}            # launch file name -> Popen, the 'launch' action's children
        self.stuck_sound = set()         # sound processes asleep in the kernel at the last check
        self.slam_active = False
        self.slam_since = 0.0
        self.guard_inactive_since = 0.0
        self.http_items['gpu'] = Item('gpu', 'graphics processor', [])
        # set when the watchdog reboots for the GPU; removed once the GPU is
        # healthy, so a second failure in a row is reported, not rebooted again
        self.gpu_flag = os.path.join(os.path.dirname(self.log_file), 'gpu_reboot_done')
        self.gpu_rebooting = False
        self.gpu_hold = ''
        self.last_move = 0.0

        self.events_pub = self.create_publisher(String, 'watchdog/events', 10)
        self.status_pub = self.create_publisher(
            String, 'watchdog/status', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.say_pub = self.create_publisher(String, 'say', 10)
        self.speak_pub = self.create_publisher(String, 'speak', 10)
        self.create_subscription(DiagnosticArray, 'watchdog/topics', self._on_topics, 10)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd, 10)
        # the safety monitor's motion check: a stop is an event, so the health page shows it
        # (2026-09-30: it fired on a loose ground wire and nobody saw it for two hours)
        self._motion_state = ''
        self.create_subscription(String, 'motion_check/state', self._on_motion_state, 10)
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

    def _on_cmd(self, msg: Twist) -> None:
        if msg.linear.x != 0.0 or msg.angular.z != 0.0:
            self.last_move = time.monotonic()

    # --------------------------------------------------------------- checks --

    def _in_grace(self) -> bool:
        # The long grace is for the robot starting up; if only the watchdog
        # restarted, a short one is enough.
        return time.monotonic() < self.grace_until

    def _grace_end(self) -> float:
        now = time.monotonic()
        self.stack_started = now
        try:
            out = subprocess.run(['systemctl', 'show', '-p', 'ActiveEnterTimestampMonotonic', '--value',
                                  'jetnano-robot'], capture_output=True, text=True, timeout=5).stdout.strip()
            stack_started = int(out) / 1e6
            if stack_started > 0:
                self.stack_started = stack_started
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
        voice_off = os.path.exists(VOICE_OFF_FLAG)
        for key, item in self.items.items():
            if voice_off and key in VOICE_STREAMS:
                item.state, item.step, item.seen = 'ok', 0, False     # switched off on purpose: not a fault
                continue
            label, topic, stale, min_hz, _need, depends, _ = TOPICS[key]
            need = item.need
            hz, age = self.topics.get(topic, (0.0, -1.0))
            if age >= 0:
                item.seen = True
            expected = (need == 'always' or (need == 'seen' and item.seen)
                        or (need == 'soon' and (item.seen or now - self.stack_started > self.expect_after))
                        or (need == 'slam' and self.slam_active and now - self.slam_since > 60.0))
            if not expected:
                continue
            if self._upstream_down(depends):
                continue                  # its source (or its source's source) is down: fix that first
            silent = age < 0 or age > stale
            if silent:
                if self._in_grace() or self._starting(item):
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
        now = time.monotonic()
        # mapping is started and stopped on request: follow it closely, and
        # give a new mapper a minute before judging its map
        active = self._run(['systemctl', 'is-active', 'jetnano-slam'], timeout=5.0).strip() == 'active'
        if active and not self.slam_active:
            self.slam_since = now
        self.slam_active = active
        if self._in_grace():
            return
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
                item.http_fails = 0
                self._good(item, 'answering')
            elif item.seen:
                # act on the second failure in a row: one slow answer is not a dead
                # server (2026-09-27, one 3 s stall with the dashboard open got
                # web_teleop - and the phone's driving page - killed and restarted)
                item.http_fails += 1
                if item.http_fails >= 2:
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
        s['slam'] = self.slam_active
        self.sys = s
        if self._in_grace():
            return
        # Each is a problem for as long as it lasts: 'down' once when it starts, 'back' when it
        # clears - the sounds node's uh-oh and beeps, and a line on the driving page meanwhile.
        # Until 2026-09-29 each was a 'system' event every 30 s while it lasted, and never
        # said when it was over.
        hot = max((v for k, v in s.items() if k.startswith('temp_')), default=0)
        # Sound stuck in the kernel: an arecord/aplay asleep in state D at two checks in a row.
        # The reSpeaker's USB controller sometimes never hands back the last transfer when a
        # recording stops (2026-09-29: "usb 1-2.4: timeout: still 1 active urbs on EP #81");
        # the process can then never end, the mic and speaker are dead, and only a reboot
        # clears it. The speaker is on the same device, so this one gets no uh-oh: the page
        # and the log have to say it.
        stuck_now = stuck_sound_pids()
        stuck = stuck_now & self.stuck_sound
        self.stuck_sound = stuck_now
        for key, label, bad, why in (
                ('disk', 'disk space', s.get('disk_free_gb', 99) < 3, f'nearly full, {s.get("disk_free_gb")} GB free'),
                ('memory', 'memory', s.get('mem_available_mb', 9999) < 300,
                 f'low, {s.get("mem_available_mb")} MB available'),
                ('heat', 'temperature', hot > 90, f'running hot, {hot:.0f} C'),
                ('wifi', 'Wi-Fi', s['wifi_dbm'] is None, 'not connected'),
                ('sound', 'microphone and speaker', bool(stuck),
                 f'USB sound stuck in the kernel (pids {sorted(stuck)}): a reboot clears it')):
            item = self.http_items.setdefault(key, Item(key, label, []))
            if bad:
                self._bad(item, why, act=False)
            else:
                self._good(item, 'ok')
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
        self._check_gpu()

    def _check_gpu(self) -> None:
        """A healthy GPU has ~15 entries in its device folder; one that failed to start has one."""
        item = self.http_items['gpu']
        try:
            entries = len(os.listdir('/dev/nvgpu/igpu0'))
        except OSError:
            entries = 0
        if entries >= 5:
            self._good(item, 'running')
            self.gpu_hold = ''
            if os.path.exists(self.gpu_flag):
                try:
                    os.remove(self.gpu_flag)      # healthy again: a later failure may reboot once more
                except OSError:
                    pass
            return
        if item.state == 'ok':
            dmesg = self._run(['sudo', '-n', 'dmesg'])
            line = next((ln for ln in dmesg.splitlines()
                         if 'ucode get fail' in ln or 'ACR bootstrap failed' in ln), '')
            if line:
                self._event('system', item.label, line.split(']', 1)[-1].strip()[:160])
            self._bad(item, 'failed to start', act=False)
        self._maybe_reboot_for_gpu(item)

    def _maybe_reboot_for_gpu(self, item) -> None:
        """Only a reboot brings back a GPU whose firmware did not load at boot (2 of 31 boots, 2026-09)."""
        if self.gpu_rebooting:
            return
        try:
            with open('/proc/uptime') as f:
                up = float(f.read().split()[0])
        except (OSError, ValueError):
            up = float('inf')
        why = self._gpu_reboot_blocker(up, first=True)
        if why:
            if why != self.gpu_hold:
                self.gpu_hold = why
                item.note = f'failed to start; not rebooting: {why}'
                self._event('gave_up', item.label, f'not rebooting: {why}')
                self._announce("My graphics processor didn't start. Please restart me.", 'sad')
            return
        self.gpu_rebooting = True
        try:
            with open(self.gpu_flag, 'w') as f:
                f.write(time.strftime('%Y-%m-%d %H:%M:%S') + '\n')
        except OSError:
            pass
        item.note = 'failed to start; rebooting'
        self._event('act', item.label, 'rebooting in 20 s: nothing else brings it back')
        self._announce("My graphics processor didn't start. Restarting myself to fix it.", 'hm')
        threading.Timer(20.0, self._gpu_reboot_fire).start()

    def _gpu_reboot_blocker(self, up: float, first: bool) -> str:
        """Why not to reboot for the GPU right now ('' = go ahead)."""
        if not self.act:
            return 'acting is switched off'
        if up > self.gpu_reboot_within:
            return 'it stopped long after boot'
        if first and os.path.exists(self.gpu_flag):
            return 'a reboot for it already did not help'
        if self.slam_active:
            return 'mapping is running'
        if time.monotonic() - self.last_move < 60.0:
            return 'she is driving'
        return ''

    def _gpu_reboot_fire(self) -> None:
        """The 20 s are up: check everything again first (review 2026-09-26: driving
        or mapping that began during the countdown used to be ignored)."""
        try:
            with open('/proc/uptime') as f:
                up = float(f.read().split()[0])
        except (OSError, ValueError):
            up = float('inf')
        why = self._gpu_reboot_blocker(up, first=False)
        try:
            if len(os.listdir('/dev/nvgpu/igpu0')) >= 5:
                why = 'the GPU came back'
        except OSError:
            pass
        if why:
            self.gpu_rebooting = False
            try:
                os.remove(self.gpu_flag)
            except OSError:
                pass
            self._event('gave_up', 'graphics processor', f'reboot cancelled: {why}')
            return
        self._event('act', 'graphics processor', 'rebooting now')
        self._run(['sudo', '-n', 'systemctl', 'reboot'])

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
                if action == 'container_launch' and self._launch_young():
                    # The safety monitor restarts the launch 5 s into a silence and the wrapper
                    # respawns it 10 s later; it needs ~15 s more to publish. On 2026-09-27 this
                    # step fired 12 s into that relaunch, killed it, and doubled the outage (56 s).
                    item.step -= 1
                    item.next_at = now + 10.0
                    return
                self.container_busy_until = now + settle
            item.actions.append(now)
            self._event('act', item.label, self._describe(action, arg))
            if self.announce == 'all':
                self._announce(f'My {item.label} stopped. Restarting it.', 'hm')
            item.next_at = float('inf')               # set when the action has finished
            threading.Thread(target=self._do, args=(action, arg, item, settle), daemon=True).start()

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
                'launch': f'launch {arg} again',
                'usb_reset': f'reset USB device {arg}',
                'container_launch': 'restart the container launch',
                'docker_restart': 'restart the container',
                'container_pkill': f'restart {arg} in the container'}.get(action, action)

    def _run(self, cmd, timeout=15.0) -> str:
        try:
            return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout).stdout
        except (OSError, subprocess.TimeoutExpired):
            return ''

    def _do(self, action: str, arg, item=None, settle: float = 0.0) -> None:
        try:
            if action == 'signal':
                self._restart_process(arg)
            elif action == 'launch':
                self._relaunch(arg)
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
        finally:
            if item is not None:
                with self.lock:
                    item.next_at = time.monotonic() + settle

    def _upstream_down(self, key) -> bool:
        """True if this source, or anything it depends on, is not ok. The whole chain: on
        2026-09-27 a frozen camera odometry left the 3D map item skipped (so still 'ok') and
        the obstacle feed below it got restarted for a fault that was two levels up."""
        seen = set()
        while key and key not in seen:
            seen.add(key)
            source = self.items.get(key) or self.http_items.get(key)
            if source is None:
                return False
            if source.state != 'ok':
                return True
            key = TOPICS[key][5] if key in TOPICS else None
        return False

    def _launch_young(self) -> bool:
        """True while the camera pipeline's wrapper is between respawns or started under
        young_process_s ago: its launch is still coming up and must not be restarted."""
        pids = self._pids(proc('jetnano_bringup/cuvslam_vo.sh'))
        if not pids:
            return True               # between exit and respawn: odometry.launch.py brings it back
        try:
            with open('/proc/uptime') as f:
                up = float(f.read().split()[0])
            tick = os.sysconf('SC_CLK_TCK')
            starts = []
            for pid in pids:              # the wrapper forks subshells with the same command line:
                with open(f'/proc/{pid}/stat') as f:           # its own start is the oldest
                    starts.append(int(f.read().rsplit(')', 1)[1].split()[19]) / tick)
            return up - min(starts) < self.young_s
        except (OSError, ValueError, IndexError):
            return False

    def _starting(self, item) -> bool:
        """True if the process behind this item was started moments ago."""
        pattern = next((arg for action, arg, _ in item.ladder if action == 'signal'), None)
        if not pattern:
            return False
        try:
            with open('/proc/uptime') as f:
                up = float(f.read().split()[0])
            tick = os.sysconf('SC_CLK_TCK')
            pids = self._pids(pattern)
            if not pids:
                return True          # between death and respawn: the launch brings it back
                                     # (and the node check reports it if that never happens)
            for pid in pids:
                with open(f'/proc/{pid}/stat') as f:
                    start = int(f.read().rsplit(')', 1)[1].split()[19]) / tick
                if up - start < self.young_s:
                    item.starts = {t for t in item.starts if up - t < 600.0} | {round(start)}
                    # four fresh starts in ten minutes is a crash loop, not a start: judged
                    # like anything else, but still not in its first seconds
                    if len(item.starts) < 4:
                        return True
                    return up - start < self.crash_young_s
        except (OSError, ValueError, IndexError):
            pass
        return False

    def _pids(self, pattern: str):
        out = self._run(['pgrep', '-f', pattern])
        return [int(x) for x in out.split() if x.isdigit() and int(x) != os.getpid()]

    @staticmethod
    def _state(pid: int) -> str:
        try:
            with open(f'/proc/{pid}/stat') as f:
                return f.read().rsplit(')', 1)[1].split()[0]
        except (OSError, IndexError):
            return ''

    def _relaunch(self, name: str) -> None:
        """`ros2 launch jetnano_bringup <name>` as our own child when the robot launch's respawn
        has given the node up (web_teleop, 2026-10-01). In the service's cgroup, so a service
        stop takes it with everything else; its output goes to ~/watchdog/relaunch-<name>.log."""
        child = self._relaunched.get(name)
        if child is not None and child.poll() is None:
            self.get_logger().info(f'{name}: already launched again by me (pid {child.pid})')
            return
        log = os.path.expanduser(f'~/watchdog/relaunch-{name}.log')
        os.makedirs(os.path.dirname(log), exist_ok=True)
        stamp = time.strftime('%Y-%m-%d %H:%M:%S')
        with open(log, 'ab') as f:
            f.write(f'== {stamp} ros2 launch jetnano_bringup {name}'.encode() + b'\n')
            self._relaunched[name] = subprocess.Popen(['ros2', 'launch', 'jetnano_bringup', name],
                                                      stdout=f, stderr=subprocess.STDOUT)
        self.get_logger().warning(f'{name}: launched again (pid {self._relaunched[name].pid}), log {log}')

    def _restart_process(self, pattern: str) -> None:
        """SIGINT, then TERM, then KILL; the launch file respawns it."""
        pids = self._pids(pattern)
        if not pids:
            self.get_logger().info(f'{pattern}: not running, its respawn is due')
            return
        frozen = all(self._state(p) == 'T' for p in pids)
        ladder = ((signal.SIGKILL, 0.0),) if frozen else ((signal.SIGINT, 3.0), (signal.SIGTERM, 3.0), (signal.SIGKILL, 0.0))
        for sig, wait in ladder:
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

    def _on_motion_state(self, msg) -> None:
        if msg.data != self._motion_state and msg.data.startswith('stopped'):
            self._event('system', 'motion check', msg.data)
        self._motion_state = msg.data

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
        try:
            node.destroy_node()
        except (Exception, KeyboardInterrupt):  # noqa: BLE001 - the context is gone, or a second SIGINT, after an external shutdown
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
