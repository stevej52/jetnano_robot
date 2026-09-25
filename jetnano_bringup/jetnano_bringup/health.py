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

"""How Rosie is doing, in words.

``Health`` hangs a few light subscriptions on a node (it counts messages, it
does not look at them) and turns what it sees into a spoken report: which
sensors are actually delivering and how fast, whether she is mapping, the
collision guard, the battery, the Wi-Fi link, CPU load and temperature,
uptime, and how noisy the room is. listen says it when someone asks "how are
you?" in English mode.

    from jetnano_bringup.health import Health
    health = Health(node)         # once, at start
    text = health.report()        # any time
"""

import collections
import os
import random
import re
import subprocess
import time

from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, CameraInfo, Imu, LaserScan
from std_msgs.msg import Float32

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Watch:
    """Message times of one topic; alive = something within stale_s."""

    def __init__(self, label: str, stale_s: float = 2.0):
        self.label = label
        self.stale_s = stale_s
        self.times = collections.deque(maxlen=2000)

    def tick(self, _msg=None) -> None:
        self.times.append(time.monotonic())

    def hz(self, window_s: float = 5.0) -> float:
        now = time.monotonic()
        return sum(1 for t in self.times if now - t <= window_s) / window_s

    def alive(self) -> bool:
        return bool(self.times) and time.monotonic() - self.times[-1] <= self.stale_s


class Health:

    def __init__(self, node):
        self.node = node
        self.watch = {}
        for key, label, msg_type, topic, qos, stale in (
                ('lidar', 'lidar', LaserScan, 'scan', qos_profile_sensor_data, 2.0),
                ('imu', 'the I M U', Imu, 'imu/data', qos_profile_sensor_data, 2.0),
                ('camera', 'the camera', CameraInfo, 'camera/color/camera_info', qos_profile_sensor_data, 2.0),
                ('vo', 'visual odometry', Odometry, 'vo', qos_profile_sensor_data, 2.0),
                ('ekf', 'odometry', Odometry, 'odometry/filtered', qos_profile_sensor_data, 2.0),
                ('nvblox', 'the 3D map', OccupancyGrid, 'nvblox_node/static_occupancy_grid', qos_profile_sensor_data, 5.0),
                ('map', 'the map', OccupancyGrid, 'map', LATCHED, 20.0)):
            self.watch[key] = Watch(label, stale)
            node.create_subscription(msg_type, topic, self.watch[key].tick, qos)
        self.battery = None
        self.level = None
        node.create_subscription(BatteryState, 'battery', self._on_battery, 10)
        node.create_subscription(Float32, 'sound/level', self._on_level, 10)

    def _on_battery(self, msg) -> None:
        self.battery = msg

    def _on_level(self, msg) -> None:
        self.level = msg.data

    # ---------------------------------------------------------------- facts --

    @staticmethod
    def wifi():
        """(band GHz, signal dBm, tx Mbit/s) or None."""
        try:
            dev = subprocess.run(['iw', 'dev'], capture_output=True, text=True, timeout=3).stdout
            iface = re.search(r'Interface (\S+)', dev)
            if not iface:
                return None
            link = subprocess.run(['iw', 'dev', iface.group(1), 'link'], capture_output=True, text=True, timeout=3).stdout
            freq = re.search(r'freq: ([\d.]+)', link)
            sig = re.search(r'signal: (-?\d+)', link)
            rate = re.search(r'tx bitrate: ([\d.]+)', link)
            if not (freq and sig):
                return None
            return float(freq.group(1)) / 1000.0, int(sig.group(1)), float(rate.group(1)) if rate else None
        except (OSError, subprocess.TimeoutExpired):
            return None

    @staticmethod
    def cpu():
        """(load percent of all cores, cpu temperature C or None)."""
        load = os.getloadavg()[0] / (os.cpu_count() or 1) * 100.0
        temp = None
        try:
            for zone in os.listdir('/sys/class/thermal'):
                if zone.startswith('thermal_zone'):
                    with open(f'/sys/class/thermal/{zone}/type') as f:
                        if f.read().strip().startswith('cpu'):
                            with open(f'/sys/class/thermal/{zone}/temp') as t:
                                temp = int(t.read().strip()) / 1000.0
                            break
        except OSError:
            pass
        return load, temp

    @staticmethod
    def uptime() -> str:
        try:
            with open('/proc/uptime') as f:
                seconds = float(f.read().split()[0])
        except OSError:
            return ''
        h, m = int(seconds // 3600), int(seconds % 3600 // 60)
        if h == 0:
            return f'{m} minutes'
        return f'{h} hour{"s" if h != 1 else ""} and {m} minutes'

    # --------------------------------------------------------------- report --

    @staticmethod
    def _join(items):
        return items[0] if len(items) == 1 else ', '.join(items[:-1]) + ' and ' + items[-1]

    def report(self, rng=random) -> str:
        names = set(self.node.get_node_names())
        core = ['lidar', 'imu', 'camera', 'vo', 'ekf']
        up = [k for k in core if self.watch[k].alive()]
        down = [k for k in core if not self.watch[k].alive()]
        parts = []
        if not down:
            parts.append(rng.choice(["I'm doing great.", "All good here.", "I'm doing well, thanks."]))
        elif len(down) < len(core):
            parts.append("I'm okay, but not everything is working.")
        else:
            parts.append("Not great. None of my sensors are talking to me.")

        if up:
            labels = [self.watch[k].label for k in up]
            s = f'{self._join(labels)} {"are" if len(up) > 1 else "is"} online'
            if 'vo' in up:
                s += f', visual odometry at {self.watch["vo"].hz():.0f} hertz'
            parts.append(s[0].upper() + s[1:] + '.')
        if down:
            labels = [self.watch[k].label for k in down]
            s = f'{self._join(labels)} {"are" if len(down) > 1 else "is"} offline.'
            parts.append(s[0].upper() + s[1:])

        if self.watch['map'].alive() or 'slam_toolbox' in names:
            parts.append("I'm mapping.")
        else:
            parts.append("I'm not mapping right now.")
        if self.watch['nvblox'].alive():
            parts.append('The 3D map is running.')
        if 'bt_navigator' in names:
            parts.append('Navigation is ready.')
        parts.append('The collision guard is on.' if 'collision_guard' in names else 'The collision guard is off.')

        b = self.battery
        if b is not None and b.present:
            pct = f'{int(round(b.percentage * 100))} percent' if b.percentage == b.percentage else 'unknown'
            parts.append(f'My battery is at {pct}, {b.voltage:.1f} volts.')
        else:
            parts.append("I'm on wall power.")

        w = self.wifi()
        if w:
            ghz, dbm, mbit = w
            strength = 'strong' if dbm > -60 else 'good' if dbm > -70 else 'weak'
            band = '5 gigahertz' if ghz > 4 else '2.4 gigahertz'
            s = f'Wi-Fi is {strength}, {band}'
            if mbit:
                s += f' at {int(mbit)} megabits'
            parts.append(s + '.')
        else:
            parts.append("I'm not on Wi-Fi.")

        load, temp = self.cpu()
        s = f'CPU load is {int(round(load))} percent'
        if temp is not None:
            s += f' at {int(round(temp))} degrees'
        parts.append(s + '.')
        up_for = self.uptime()
        if up_for:
            parts.append(f"I've been up for {up_for}.")
        if self.level is not None:
            parts.append("It's loud in here." if self.level > -30 else
                         "It's a bit noisy in here." if self.level > -40 else "It's nice and quiet.")
        # one line per system: the speak node puts a small pause at each newline
        return '\n'.join(parts)
