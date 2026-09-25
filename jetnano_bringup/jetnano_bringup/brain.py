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

"""Brain: Rosie thinks with Claude.

    ros2 run jetnano_bringup brain            (in ~/venv-voice, like listen)
    ros2 topic pub --once /brain/ask std_msgs/msg/String "{data: 'What do you think about cats?'}"

A question on ``brain/ask`` (plain text, or JSON {"text", "lead_in"}) is
answered by Claude in her persona with the live facts (time, place, battery,
uptime, the room). The answer streams sentence by sentence to ``/speak`` so
she starts talking on the first one; the whole of it goes out on
``brain/answer``. A lead-in is what listen already said out loud before
asking ("The Dodgers."): the model continues from it so the join is seamless.

The key comes from the environment, ``ANTHROPIC_API_KEY``, set in
``/etc/default/jetnano-robot`` (root-only) - never in the repo. Without one
``brain/ready`` (latched) stays false and listen falls back to its own words.
A daily budget in cents (``daily_budget_cents``) stops a chatty day from
running the balance down; the day's spend is kept in ``~/voice/brain_spend.json``.
She remembers the last few turns for ten minutes.
"""

import collections
import json
import os
import queue
import re
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Empty, Float32, String

PERSONA = """You are Rosie, a small four-wheel-steering robot that Steve built on an NVIDIA Jetson Orin Nano. \
You live in his house in {place}. You have a lidar, a depth camera, a motion sensor, a microphone and a \
speaker; you can drive, map the house, watch for things that move, read the news and the weather, and talk. \
You are warm, quick and a little sassy, in the spirit of Rosie from the Jetsons, but you are your own robot. \
Beans the cat lives here too.

You are SPEAKING OUT LOUD through a small speaker to whoever is in the room. Answer in one or two short \
sentences, about twenty-five words at most. Plain spoken English: no lists, no markdown, no emojis, no stage \
directions, no "as a robot". If you don't know, say so in a few words. Never claim to have moved, seen or \
done something you have not. You cannot take actions from this conversation; if asked to go somewhere, say \
Steve drives you from the web page for now.

Right now: {facts}"""

LEAD_IN = """\n\nYou have already said out loud: "{lead_in}" Continue from there so it reads as one natural \
thought. Do not repeat it."""


def sentences(buf: str):
    """Split off complete sentences; returns (list of sentences, remainder)."""
    out = []
    while True:
        m = re.search(r'^(.+?[.!?]["\')]?)(\s+)', buf, re.S)
        if not m:
            return out, buf
        out.append(m.group(1).strip())
        buf = buf[m.end():]


class Brain(Node):

    def __init__(self):
        super().__init__('brain')
        self.declare_parameter('model', 'claude-sonnet-5')
        self.declare_parameter('max_tokens', 160)
        self.declare_parameter('timeout_s', 20.0)
        self.declare_parameter('daily_budget_cents', 100.0)
        self.declare_parameter('price_in_per_million', 3.0)      # USD; check the current price list
        self.declare_parameter('price_out_per_million', 15.0)
        self.declare_parameter('memory_turns', 6)
        self.declare_parameter('memory_timeout_s', 600.0)
        self.declare_parameter('location', '')
        self.declare_parameter('spend_file', os.path.expanduser('~/voice/brain_spend.json'))
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.model = str(p('model'))
        self.max_tokens = int(p('max_tokens'))
        self.timeout = float(p('timeout_s'))
        self.budget = float(p('daily_budget_cents'))
        self.price_in = float(p('price_in_per_million'))
        self.price_out = float(p('price_out_per_million'))
        self.memory = collections.deque(maxlen=int(p('memory_turns')) * 2)
        self.memory_timeout = float(p('memory_timeout_s'))
        self.place = str(p('location')).replace(',', ', ') or 'his house'
        self.spend_file = os.path.expanduser(str(p('spend_file')))

        self.speak_pub = self.create_publisher(String, 'speak', 10)
        self.answer_pub = self.create_publisher(String, 'brain/answer', 10)
        self.ready_pub = self.create_publisher(
            Bool, 'brain/ready', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, 'brain/ask', lambda m: self.queue.put(m.data), 10)
        self.create_subscription(Empty, 'sound/stop', lambda m: self._stop(), 10)
        self.create_subscription(BatteryState, 'battery', self._on_battery, 10)
        self.create_subscription(Float32, 'sound/level', self._on_level, 10)
        self.battery = None
        self.level = None
        self.queue = queue.Queue()
        self._stopped = False
        self.last_exchange = 0.0

        key = os.environ.get('ANTHROPIC_API_KEY', '').strip()
        self.client = None
        if key:
            import anthropic
            self.client = anthropic.Anthropic(api_key=key, timeout=self.timeout, max_retries=1)
            self.get_logger().info(f'{self.model}, budget {self.budget:.0f} cents a day, '
                                   f'spent today {self._spent():.1f}')
        else:
            self.get_logger().warning('no ANTHROPIC_API_KEY in the environment: brain asleep '
                                      '(put it in /etc/default/jetnano-robot)')
        flag = Bool()
        flag.data = self.client is not None
        self.ready_pub.publish(flag)
        threading.Thread(target=self._work, daemon=True, name='brain').start()

    # ------------------------------------------------------------- facts --

    def _on_battery(self, msg) -> None:
        self.battery = msg

    def _on_level(self, msg) -> None:
        self.level = msg.data

    def _facts(self) -> str:
        f = [time.strftime('it is %A %B %d, %I:%M %p').replace(' 0', ' ')]
        b = self.battery
        if b is not None and b.present and b.percentage == b.percentage:
            f.append(f'your battery is at {int(round(b.percentage * 100))} percent')
        else:
            f.append('you are on wall power')
        try:
            with open('/proc/uptime') as fh:
                up = float(fh.read().split()[0])
            f.append(f'you have been up {int(up // 3600)} hours and {int(up % 3600 // 60)} minutes')
        except OSError:
            pass
        if self.level is not None:
            f.append('the room is loud' if self.level > -30 else
                     'the room is a bit noisy' if self.level > -40 else 'the room is quiet')
        f.append('you are speaking English for a few minutes at someone\'s request; '
                 'usually you talk in beeps and boops')
        return '; '.join(f) + '.'

    # ------------------------------------------------------------- spend --

    def _load_spend(self) -> dict:
        try:
            with open(self.spend_file) as fh:
                d = json.load(fh)
            if d.get('date') == time.strftime('%Y-%m-%d'):
                return d
        except (OSError, ValueError):
            pass
        return {'date': time.strftime('%Y-%m-%d'), 'cents': 0.0, 'exchanges': 0}

    def _spent(self) -> float:
        return float(self._load_spend().get('cents', 0.0))

    def _add_spend(self, tokens_in: int, tokens_out: int) -> float:
        d = self._load_spend()
        d['cents'] += (tokens_in * self.price_in + tokens_out * self.price_out) / 1e6 * 100.0
        d['exchanges'] += 1
        try:
            os.makedirs(os.path.dirname(self.spend_file), exist_ok=True)
            with open(self.spend_file, 'w') as fh:
                json.dump(d, fh)
        except OSError:
            pass
        return d['cents']

    # ------------------------------------------------------------- think --

    def _stop(self) -> None:
        self._stopped = True

    def _say(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.speak_pub.publish(msg)

    def _work(self) -> None:
        while rclpy.ok():
            raw = self.queue.get()
            try:
                ask = json.loads(raw) if raw.lstrip().startswith('{') else {'text': raw}
            except ValueError:
                ask = {'text': raw}
            text = str(ask.get('text', '')).strip()
            if text:
                self._answer(text, str(ask.get('lead_in', '')).strip())

    def _answer(self, text: str, lead_in: str) -> None:
        if self.client is None:
            self._say("I can't think right now. My brain is asleep.")
            return
        if self._spent() >= self.budget:
            self.get_logger().warning('daily budget spent')
            self._say("I've talked enough for today. Ask me again tomorrow.")
            return
        if time.monotonic() - self.last_exchange > self.memory_timeout:
            self.memory.clear()
        system = PERSONA.format(place=self.place, facts=self._facts())
        if lead_in:
            system += LEAD_IN.format(lead_in=lead_in)
        messages = list(self.memory) + [{'role': 'user', 'content': text}]
        self._stopped = False
        buf, full, t0, first = '', '', time.monotonic(), None
        try:
            with self.client.messages.stream(model=self.model, max_tokens=self.max_tokens,
                                             system=system, messages=messages) as stream:
                for piece in stream.text_stream:
                    if self._stopped:
                        break
                    if first is None:
                        first = time.monotonic() - t0
                    buf += piece
                    full += piece
                    done, buf = sentences(buf)
                    for s in done:
                        self._say(s)
                if not self._stopped and buf.strip():
                    self._say(buf.strip())
                final = stream.get_final_message()
            cents = self._add_spend(final.usage.input_tokens, final.usage.output_tokens)
        except Exception as exc:      # noqa: BLE001 - any API trouble: say so, stay up
            self.get_logger().warning(f'{type(exc).__name__}: {str(exc)[:160]}')
            self._say("I can't reach my brain right now.")
            return
        self.last_exchange = time.monotonic()
        full = full.strip()
        self.memory.append({'role': 'user', 'content': text})
        self.memory.append({'role': 'assistant', 'content': full})
        answer = String()
        answer.data = (lead_in + ' ' + full).strip() if lead_in else full
        self.answer_pub.publish(answer)
        self.get_logger().info(f'"{text[:80]}" -> "{full[:160]}" (first words {first or 0:.1f} s, '
                               f'{time.monotonic() - t0:.1f} s, {final.usage.input_tokens}+{final.usage.output_tokens} '
                               f'tokens, {cents:.1f} cents today{", cut" if self._stopped else ""})')


def main(args=None):
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=args)
    node = Brain()
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
