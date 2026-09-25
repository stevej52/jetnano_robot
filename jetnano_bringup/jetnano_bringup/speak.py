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

"""Speak: Rosie's English voice.

    ros2 run jetnano_bringup speak                  (in ~/venv-voice, like listen)
    ros2 topic pub --once /speak std_msgs/msg/String "{data: 'Hello. I am Rosie.'}"

Text on ``speak`` becomes a wav with a Piper voice (sherpa-onnx, on the CPU,
about half a second per sentence) and goes to the sounds node as
``/say <path>``, so mute, one-sound-at-a-time and the speaking flag all still
apply. Every phrase is kept in ``~/voice/tts_cache`` and synthesised once.
The sounds node uses this for its English mode ("Rosie, speak English");
listen uses it for English replies in a chat.

Each line of a multi-line text (a status report, a news briefing) is its own
sound, sent as soon as it is made, with ``pause_s`` of quiet on its tail: she
starts talking after the first line, there is a breath between systems, and
"stop" (``sound/stop``) can cut in between lines. A ``[mood]`` tag, e.g.
"You're a joke. [laugh]", plays that sound right after the words.
"""

import hashlib
import os
import queue
import re
import threading
import time
import wave

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String

MODEL_DIR = os.path.expanduser('~/voice/models')
LEAD_S, TAIL_S = 0.25, 0.10      # the USB speaker swallows its first 80 ms (voice.py)


def write_wav(path: str, samples, rate: int, volume: float = 0.8, tail_s: float = TAIL_S) -> None:
    y = np.asarray(samples, dtype=np.float32)
    y = y / (float(np.max(np.abs(y))) or 1.0) * volume
    y = np.concatenate([np.zeros(int(rate * LEAD_S), np.float32), y, np.zeros(int(rate * tail_s), np.float32)])
    with wave.open(path, 'wb') as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes((y * 32767).astype(np.int16).tobytes())


def make_tts(model_dir: str, voice: str, threads: int):
    import sherpa_onnx
    d = os.path.join(model_dir, voice)
    onnx = [f for f in os.listdir(d) if f.endswith('.onnx')][0]
    cfg = sherpa_onnx.OfflineTtsConfig(
        model=sherpa_onnx.OfflineTtsModelConfig(
            vits=sherpa_onnx.OfflineTtsVitsModelConfig(
                model=os.path.join(d, onnx), lexicon='', tokens=os.path.join(d, 'tokens.txt'),
                data_dir=os.path.join(d, 'espeak-ng-data')),
            num_threads=threads, provider='cpu'),
        max_num_sentences=1)
    return sherpa_onnx.OfflineTts(cfg)


class Speak(Node):

    def __init__(self):
        super().__init__('speak')
        self.declare_parameter('model_dir', MODEL_DIR)
        self.declare_parameter('voice', 'vits-piper-en_US-hfc_female-medium')
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('threads', 2)
        self.declare_parameter('cache_dir', os.path.expanduser('~/voice/tts_cache'))
        self.declare_parameter('pause_s', 0.45)          # at each newline in the text
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.voice = str(p('voice'))
        self.speed = float(p('speed'))
        self.pause = float(p('pause_s'))
        self.cache = os.path.expanduser(str(p('cache_dir')))
        os.makedirs(self.cache, exist_ok=True)
        t0 = time.monotonic()
        self.tts = make_tts(os.path.expanduser(str(p('model_dir'))), self.voice, int(p('threads')))
        self.say_pub = self.create_publisher(String, 'say', 10)
        self.create_subscription(String, 'speak', lambda m: self.queue.put(m.data), 10)
        self.create_subscription(Empty, 'sound/stop', lambda m: self._stop(), 10)
        self.queue = queue.Queue()
        self._stopped = False
        threading.Thread(target=self._work, daemon=True, name='speak-tts').start()
        self.get_logger().info(f'{self.voice} ready in {time.monotonic() - t0:.1f} s')

    def _stop(self) -> None:
        self._stopped = True
        with self.queue.mutex:
            self.queue.queue.clear()

    def _work(self) -> None:
        while rclpy.ok():
            raw = self.queue.get()
            self._stopped = False
            tags = re.findall(r'\[(\w+)\]', raw)
            text = re.sub(r'\s*\[\w+\]', '', raw).strip()
            lines = [ln.strip() for ln in text.split('\n') if ln.strip()]
            t0 = time.monotonic()
            for i, line in enumerate(lines):
                if self._stopped:
                    break
                tail = self.pause if i < len(lines) - 1 else TAIL_S
                key = hashlib.md5(f'{self.voice}|{self.speed}|{tail}|{line}'.encode()).hexdigest()[:16]
                # long one-offs (a headline, a report line) are not worth keeping
                path = os.path.join(self.cache if len(line) <= 60 else '/tmp', f'rosie_{key}.wav')
                if not os.path.isfile(path):
                    audio = self.tts.generate(line, sid=0, speed=self.speed)
                    if len(audio.samples) == 0:
                        continue
                    write_wav(path, audio.samples, audio.sample_rate, tail_s=tail)
                self._say(path)
            if lines:
                self.get_logger().info(f'"{" | ".join(lines)[:160]}" ({len(lines)} line{"s" if len(lines) != 1 else ""}'
                                       f' in {time.monotonic() - t0:.2f} s{", cut" if self._stopped else ""})')
            for mood in tags:           # after the words: the sounds queue is in order
                if not self._stopped:
                    self._say(mood)

    def _say(self, what: str) -> None:
        msg = String()
        msg.data = what
        self.say_pub.publish(msg)


def main(args=None):
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=args)
    node = Speak()
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
