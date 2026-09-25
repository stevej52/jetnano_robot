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

"""Ears: how loud the room is, and a reaction to a sudden noise.

    ros2 run jetnano_bringup ears
    ros2 topic echo /sound/level          # dBFS, ten times a second

Reads the USB microphone continuously through arecord (16 kHz mono) and
publishes the level in dBFS on ``sound/level``. A sharp noise - a clap, a
dropped pan, a door - is a jump of ``loud_above_db`` over the room's rolling
background that is also louder than ``loud_min_dbfs``; then ``sound/loud``
goes true for a moment and, if the robot is parked, she says "hm". Nothing
is said while driving (the motor is the loudest thing she hears), and she
never listens to *words* here: this is a level meter, not a microphone feed
to anything.

The mic is named by ALSA card (``arecord -l``); a C-Media "USB PnP Sound
Device" appears as card "Device". Its Auto Gain Control is switched on for
speech-like levels.
"""

import math
import subprocess
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


class Ears(Node):

    def __init__(self):
        super().__init__('ears')
        self.declare_parameter('card', 'Device')
        self.declare_parameter('rate', 16000)
        self.declare_parameter('gain_percent', 80)
        self.declare_parameter('auto_gain', True)
        self.declare_parameter('chunk_s', 0.1)
        self.declare_parameter('background_s', 3.0)      # how slowly the room's level is followed
        self.declare_parameter('loud_above_db', 18.0)
        self.declare_parameter('loud_min_dbfs', -28.0)
        self.declare_parameter('still_after_s', 2.0)
        self.declare_parameter('say_min_gap_s', 6.0)

        self.card = str(self.get_parameter('card').value)
        self.rate = int(self.get_parameter('rate').value)
        self.chunk = float(self.get_parameter('chunk_s').value)
        self.bg_s = float(self.get_parameter('background_s').value)
        self.loud_above = float(self.get_parameter('loud_above_db').value)
        self.loud_min = float(self.get_parameter('loud_min_dbfs').value)
        self.still_after = float(self.get_parameter('still_after_s').value)
        self.say_gap = float(self.get_parameter('say_min_gap_s').value)

        gain = int(self.get_parameter('gain_percent').value)
        for control in ('Mic', 'Capture'):
            subprocess.run(['amixer', '-q', '-c', self.card, 'sset', control, f'{gain}%'], capture_output=True, timeout=5)
        if bool(self.get_parameter('auto_gain').value):
            subprocess.run(['amixer', '-q', '-c', self.card, 'sset', 'Auto Gain Control', 'on'], capture_output=True, timeout=5)

        self.level_pub = self.create_publisher(Float32, 'sound/level', 10)
        self.loud_pub = self.create_publisher(Bool, 'sound/loud', 10)
        self.say_pub = self.create_publisher(String, 'say', 10)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd, 10)
        self.last_cmd = 0.0
        self.last_said = 0.0
        self.background = None
        self._warned = False
        threading.Thread(target=self._listen, daemon=True, name='ears-arecord').start()
        self.get_logger().info(f'listening on card {self.card} at {self.rate} Hz, gain {gain} %')

    def _on_cmd(self, msg: Twist) -> None:
        if msg.linear.x != 0.0 or msg.angular.z != 0.0:
            self.last_cmd = time.monotonic()

    def _listen(self) -> None:
        frames = int(self.rate * self.chunk)
        while rclpy.ok():
            cmd = ['arecord', '-q', '-D', f'plughw:{self.card}', '-f', 'S16_LE', '-r', str(self.rate), '-c', '1', '-t', 'raw']
            try:
                proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            except OSError as exc:
                self._warn(f'cannot start arecord: {exc}')
                time.sleep(10)
                continue
            while rclpy.ok():
                data = proc.stdout.read(frames * 2)
                if len(data) < frames * 2:
                    break
                self._chunk(np.frombuffer(data, dtype=np.int16).astype(np.float32))
            err = proc.stderr.read().decode(errors='replace').strip() if proc.stderr else ''
            proc.kill()
            self._warn(f'microphone stream ended: {err[:100] or "no data"} (retrying)')
            self.background = None
            time.sleep(5)

    def _warn(self, text: str) -> None:
        if not self._warned:
            self.get_logger().warning(text)
            self._warned = True

    def _chunk(self, x: np.ndarray) -> None:
        self._warned = False
        rms = float(np.sqrt(np.mean(x * x))) if len(x) else 0.0
        level = 20.0 * math.log10(max(rms, 1.0) / 32767.0)
        msg = Float32()
        msg.data = level
        self.level_pub.publish(msg)

        if self.background is None:
            self.background = level
            return
        loud = level > self.background + self.loud_above and level > self.loud_min
        # follow the room slowly, and not at all during the bang itself
        if not loud:
            a = self.chunk / self.bg_s
            self.background += a * (level - self.background)
        flag = Bool()
        flag.data = loud
        self.loud_pub.publish(flag)
        if loud:
            now = time.monotonic()
            parked = now - self.last_cmd > self.still_after
            if parked and now - self.last_said > self.say_gap:
                self.last_said = now
                self.get_logger().info(f'loud noise: {level:.0f} dBFS over a room at {self.background:.0f}')
                s = String()
                s.data = 'hm'
                self.say_pub.publish(s)


def main(args=None):
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=args)
    node = Ears()
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
