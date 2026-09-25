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

"""What the robot says, and when.

    ros2 run jetnano_bringup sounds
    ros2 topic pub --once /say std_msgs/msg/String "{data: hello}"

Plays the voice made by make_voice (``~/sounds/<mood><n>.wav``, a random take
each time) through the USB speaker with aplay, on these events:

    boot (a few seconds after start)          hello
    /e_stop true -> false                     ok
    /e_stop false -> true                     alarm
    /collision_guard/state stops the robot    no        (at most every few seconds)
    /cliff/drop true                          no
    /battery health low / flat                sad / alarm then sleepy
    /say <mood>                               that mood (anyone: the page, a person detector)

One sound at a time; a mood is not repeated within ``min_gap_s``; ``mute``
is a parameter (ros2 param set /sounds mute true) so a droid that chirps at
night can be told to stop without stopping it. With no speaker present it
logs once and keeps trying quietly.
"""

import glob
import os
import queue
import random
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String

try:
    from nav2_msgs.msg import CollisionMonitorState
except ImportError:  # pragma: no cover
    CollisionMonitorState = None


class Sounds(Node):

    def __init__(self):
        super().__init__('sounds')
        self.declare_parameter('sound_dir', os.path.expanduser('~/sounds'))
        # The USB speaker by ALSA card NAME, not number: numbers shuffle when a
        # microphone is plugged in. 'aplay -l' shows the name in brackets.
        self.declare_parameter('card', 'UACDemoV10')
        self.declare_parameter('device', '')          # aplay -D ...; empty = plughw:<card>
        self.declare_parameter('volume_percent', 80)  # set on the card's PCM control at start
        self.declare_parameter('mute', False)
        self.declare_parameter('min_gap_s', 2.5)
        self.declare_parameter('hello_after_s', 4.0)

        self.dir = os.path.expanduser(str(self.get_parameter('sound_dir').value))
        card = str(self.get_parameter('card').value)
        self.device = str(self.get_parameter('device').value) or (f'plughw:{card}' if card else '')
        volume = int(self.get_parameter('volume_percent').value)
        if card:
            for control in ('PCM', 'Speaker', 'Master'):
                subprocess.run(['amixer', '-q', '-c', card, 'sset', control, f'{volume}%'],
                               capture_output=True, timeout=5)
        self.min_gap = float(self.get_parameter('min_gap_s').value)
        self.queue = queue.Queue()
        self.last_said = {}
        self._warned = False
        threading.Thread(target=self._player, daemon=True, name='sounds-player').start()

        self._e_stop = None
        self._guard_stop = False
        self._battery_state = 'ok'
        self._drop = False
        self.create_subscription(String, 'say', lambda m: self.say(m.data), 10)
        self.create_subscription(Bool, 'e_stop', self._on_e_stop, 10)
        self.create_subscription(Bool, 'cliff/drop', self._on_drop, 10)
        self.create_subscription(BatteryState, 'battery', self._on_battery, 10)
        if CollisionMonitorState is not None:
            self.create_subscription(CollisionMonitorState, 'collision_guard/state', self._on_guard, 10)
        self._hello_timer = self.create_timer(float(self.get_parameter('hello_after_s').value), self._hello)
        moods = sorted({os.path.basename(p).rstrip('0123456789.wav') for p in glob.glob(os.path.join(self.dir, '*.wav'))})
        self.get_logger().info((f'{len(moods)} moods in {self.dir}: {", ".join(moods)}' if moods
                                else f'no sounds in {self.dir} (ros2 run jetnano_bringup make_voice {self.dir})')
                               + f' -> {self.device or "default device"} at {volume} %')

    # ----------------------------------------------------------------- speak --

    def say(self, mood: str, force: bool = False) -> None:
        now = time.monotonic()
        if not force and now - self.last_said.get(mood, -1e9) < self.min_gap:
            return
        self.last_said[mood] = now
        self.queue.put(mood)

    def _player(self) -> None:
        while True:
            mood = self.queue.get()
            if bool(self.get_parameter('mute').value):
                continue
            takes = glob.glob(os.path.join(self.dir, f'{mood}[0-9]*.wav'))
            if not takes:
                self.get_logger().warning(f'no sound for "{mood}"')
                continue
            cmd = ['aplay', '-q'] + (['-D', self.device] if self.device else []) + [random.choice(takes)]
            try:
                result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
                if result.returncode != 0 and not self._warned:
                    self.get_logger().warning(f'cannot play sounds: {result.stderr.strip()[:120]} (is the speaker plugged in?)')
                    self._warned = True
                elif result.returncode == 0:
                    self._warned = False
            except (OSError, subprocess.TimeoutExpired) as exc:
                if not self._warned:
                    self.get_logger().warning(f'cannot play sounds: {exc}')
                    self._warned = True

    # ---------------------------------------------------------------- events --

    def _hello(self) -> None:
        self._hello_timer.cancel()
        self.say('hello', force=True)

    def _on_e_stop(self, msg: Bool) -> None:
        if self._e_stop is not None and msg.data != self._e_stop:
            self.say('alarm' if msg.data else 'ok', force=True)
        self._e_stop = msg.data

    def _on_guard(self, msg) -> None:
        stopped = msg.action_type == CollisionMonitorState.STOP
        if stopped and not self._guard_stop:
            self.say('no')
        self._guard_stop = stopped

    def _on_drop(self, msg: Bool) -> None:
        if msg.data and not self._drop:
            self.say('no', force=True)
        self._drop = msg.data

    def _on_battery(self, msg: BatteryState) -> None:
        if not msg.present:
            state = 'ok'
        elif msg.power_supply_health == BatteryState.POWER_SUPPLY_HEALTH_DEAD:
            state = 'flat'
        elif msg.percentage == msg.percentage and msg.percentage <= 0.2:
            state = 'low'
        else:
            state = 'ok'
        if state != self._battery_state:
            if state == 'low':
                self.say('sad', force=True)
            elif state == 'flat':
                self.say('alarm', force=True)
                self.say('sleepy', force=True)
            self._battery_state = state


def main(args=None):
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=args)
    node = Sounds()
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
