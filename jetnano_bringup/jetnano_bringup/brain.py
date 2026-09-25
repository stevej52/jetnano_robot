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

"""Brain: Rosie thinks - with the model on JEDIPC, or with Claude.

    ros2 run jetnano_bringup brain            (in ~/venv-voice, like listen)
    ros2 topic pub --once /brain/ask std_msgs/msg/String "{data: 'What do you think about cats?'}"

A question on ``brain/ask`` (plain text, or JSON {"text", "lead_in"}) is
answered in her persona with the live facts (time, place, battery, uptime,
the room). The answer streams sentence by sentence to ``/speak`` so she
starts talking on the first one; the whole of it goes out on
``brain/answer``. A lead-in is what listen already said out loud before
asking ("The Dodgers."): the model continues from it so the join is seamless.

Two backends, ``backend`` = auto | local | claude:
  local   llama.cpp's server on the PC upstairs (``local_url``; Qwen 2.5 14B
          on the Radeon, robot-environment ``scripts/jedipc_brain.md``): free,
          private, fastest. Used whenever it answers ``/health``.
  claude  ``claude-sonnet-5`` with ``ANTHROPIC_API_KEY`` from the environment
          (``/etc/default/jetnano-robot``, root-only, never in the repo) -
          the smarter one, and the fallback when that PC is off. A daily
          budget in cents (``daily_budget_cents``) stops a chatty day from
          running the balance down; the day's spend is in ``~/voice/brain_spend.json``.
``brain/ready`` (latched) is true while either can answer; otherwise listen
keeps to its own words. She remembers the last few turns for ten minutes.

Hand-over (``hand_over``, on): the local model answers small talk itself and
replies PASS to anything that needs real knowledge (facts, numbers, science,
history, how things work, sports, news, advice). Her first sentence is held
back until it is known not to be PASS or an "I'm not sure"; if it is either,
the question goes to Claude, which continues after the lead-in she already
said. Without Claude (no key, or the day's budget spent) a PASS becomes "I
don't know. I'm just a robot." Calibrated 2026-09-25: 16 of 16 questions
routed as intended, the PASS in 0.2 s.
"""

import collections
import json
import os
import queue
import re
import threading
import time
import urllib.error
import urllib.request

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
Steve drives you from the web page for now."""

# For the local model only: answer the easy things, hand the rest to Claude.
# Calibrated 2026-09-25 on 16 questions (8 small talk, 8 real): 16 of 16
# routed as intended, PASS arriving in 0.2 s.
HAND_OVER = """

You are the quick small brain; a much bigger one stands behind you. Answer small talk, feelings, opinions, \
greetings, jokes, and anything about yourself, Steve, Beans or the house yourself. If a good answer needs facts \
you are not certain of (names, dates, numbers, events, science, history, how things work, sports, current \
affairs), arithmetic, or a careful explanation, do not guess: reply with exactly PASS and nothing else."""
PASS_WORD = re.compile(r'\s*pass\W', re.I)        # "PASS" and then something that is not a letter
PASS_END = re.compile(r'\s*pass\W*$', re.I)       # or "PASS" and nothing else
UNSURE = re.compile(r"\b(i don'?t know|i'?m not sure|i am not sure|not sure|no idea|i can'?t say|not certain|"
                    r"i don'?t have (that|any|enough) (information|info|details))\b", re.I)

# The persona above never changes, so the local server keeps it cached and
# only the lines below get processed each time: keep them last and short.
FACTS = "\n\nRight now: {facts}"
LEAD_IN = """\n\nYou have already said out loud: "{lead_in}" Pick up mid-thought from there so it reads as one \
natural remark. Do not repeat those words and do not start with the same word: after "Dodgers." begin with \
something like "They" or "I", after "Well," just carry on."""


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
        self.declare_parameter('backend', 'auto')                 # auto | local | claude
        self.declare_parameter('hand_over', True)                 # local passes what it cannot answer to Claude
        self.declare_parameter('local_url', 'http://192.168.1.137:8090')
        self.declare_parameter('local_model', 'rosie')
        self.declare_parameter('local_timeout_s', 30.0)
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
        self.backend = str(p('backend'))
        self.hand_over = bool(p('hand_over'))
        self.local_url = str(p('local_url')).rstrip('/')
        self.local_model = str(p('local_model'))
        self.local_timeout = float(p('local_timeout_s'))
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
        if key and self.backend in ('auto', 'claude'):
            import anthropic
            self.client = anthropic.Anthropic(api_key=key, timeout=self.timeout, max_retries=1)
            self.get_logger().info(f'{self.model}, budget {self.budget:.0f} cents a day, '
                                   f'spent today {self._spent():.1f}')
        elif self.backend in ('auto', 'claude'):
            self.get_logger().warning('no ANTHROPIC_API_KEY in the environment: no Claude '
                                      '(put it in /etc/default/jetnano-robot)')
        self.local_up = self.backend in ('auto', 'local') and self._local_ok()
        self.get_logger().info(f'local model at {self.local_url}: {"up" if self.local_up else "not answering"}')
        self.ready = None
        self._publish_ready()
        self.create_timer(30.0, self._check_local)
        threading.Thread(target=self._work, daemon=True, name='brain').start()

    # ----------------------------------------------------------- backends --

    def _local_ok(self) -> bool:
        try:
            with urllib.request.urlopen(self.local_url + '/health', timeout=1.5) as r:
                return r.status == 200
        except (OSError, urllib.error.URLError, ValueError):
            return False

    def _check_local(self) -> None:
        if self.backend not in ('auto', 'local'):
            return
        up = self._local_ok()
        if up != self.local_up:
            self.local_up = up
            self.get_logger().info(f'local model {"is back" if up else "went away"}')
        self._publish_ready()

    def _publish_ready(self) -> None:
        ready = self.local_up or self.client is not None
        if ready != self.ready:
            self.ready = ready
            flag = Bool()
            flag.data = ready
            self.ready_pub.publish(flag)

    def _pick(self):
        if self.local_up and self.backend in ('auto', 'local'):
            return 'local'
        if self.client is not None and self.backend in ('auto', 'claude'):
            return 'claude'
        return None

    def _stream_claude(self, system: str, messages: list):
        with self.client.messages.stream(model=self.model, max_tokens=self.max_tokens,
                                         system=system, messages=messages) as stream:
            for piece in stream.text_stream:
                if self._stopped:
                    break
                yield piece
            final = stream.get_final_message()
        self._usage = (final.usage.input_tokens, final.usage.output_tokens)

    def _stream_local(self, system: str, messages: list):
        body = json.dumps({'model': self.local_model, 'stream': True, 'max_tokens': self.max_tokens,
                           'temperature': 0.7, 'cache_prompt': True,
                           'messages': [{'role': 'system', 'content': system}] + messages}).encode()
        req = urllib.request.Request(self.local_url + '/v1/chat/completions', data=body,
                                     headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(req, timeout=self.local_timeout) as r:
            for raw in r:
                if self._stopped:
                    break
                line = raw.decode('utf-8', 'replace').strip()
                if not line.startswith('data:'):
                    continue
                data = line[5:].strip()
                if data == '[DONE]':
                    break
                j = json.loads(data)
                choice = (j.get('choices') or [{}])[0]
                delta = (choice.get('delta') or {}).get('content')
                if delta:
                    yield delta
                if j.get('usage'):
                    self._usage = (int(j['usage'].get('prompt_tokens', 0)), int(j['usage'].get('completion_tokens', 0)))
                if j.get('timings'):
                    self._timings = j['timings']

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

    def _claude_ok(self) -> bool:
        return self.client is not None and self.backend in ('auto', 'claude') and self._spent() < self.budget

    def _run(self, backend: str, system: str, messages: list, check: bool):
        """Stream one answer, speaking sentence by sentence. With check, the
        first sentence is held back until it is known not to be a hand-over
        (PASS) or an admission ("I'm not sure"). -> (text, first_s, reason)."""
        self._usage, self._timings = (0, 0), None
        buf, full, t0, first, spoke = '', '', time.monotonic(), None, False
        pieces = self._stream_local(system, messages) if backend == 'local' else self._stream_claude(system, messages)
        for piece in pieces:
            if first is None:
                first = time.monotonic() - t0
            buf += piece
            full += piece
            if check and not spoke and PASS_WORD.match(full):
                return full, first, 'passed'
            done, buf = sentences(buf)
            for sent in done:
                if check and not spoke and UNSURE.search(sent):
                    return full, first, 'unsure'
                self._say(sent)
                spoke = True
        rest = buf.strip()
        if check and not spoke:
            if PASS_END.match(full):
                return full, first, 'passed'
            if UNSURE.search(rest):
                return full, first, 'unsure'
        if not self._stopped and rest:
            self._say(rest)
        return full, first, None

    def _answer(self, text: str, lead_in: str) -> None:
        backend = self._pick()
        if backend is None:
            self._say("I can't think right now. My brain is asleep.")
            return
        if backend == 'claude' and self._spent() >= self.budget:
            self.get_logger().warning('daily budget spent')
            self._say("I've talked enough for today. Ask me again tomorrow.")
            return
        if time.monotonic() - self.last_exchange > self.memory_timeout:
            self.memory.clear()
        facts = FACTS.format(facts=self._facts()) + (LEAD_IN.format(lead_in=lead_in) if lead_in else '')
        persona = PERSONA.format(place=self.place)
        messages = list(self.memory) + [{'role': 'user', 'content': text}]
        self._stopped = False
        t0 = time.monotonic()
        handed, reason = False, None
        try:
            if backend == 'local':
                check = self.hand_over
                full, first, reason = self._run('local', persona + (HAND_OVER if check else '') + facts, messages, check)
                if reason:
                    if self._claude_ok():
                        handed, backend = True, 'claude'
                        full, first, _ = self._run('claude', persona + facts, messages, False)
                        first = time.monotonic() - t0 if first is None else first
                    elif reason == 'passed':
                        full = "I don't know. I'm just a robot."
                        self._say(full)
                    else:
                        said = sentences(full.strip() + ' ')[0] or [full.strip()]
                        for sent in said:
                            self._say(sent)
            else:
                full, first, _ = self._run('claude', persona + facts, messages, False)
        except Exception as exc:      # noqa: BLE001 - any trouble: say so, stay up
            self.get_logger().warning(f'{backend}: {type(exc).__name__}: {str(exc)[:160]}')
            if backend == 'local':
                self.local_up = False
                self._publish_ready()
            self._say("I can't reach my brain right now.")
            return
        cents = self._add_spend(*self._usage) if backend == 'claude' else self._spent()
        self.last_exchange = time.monotonic()
        full = full.strip()
        self.memory.append({'role': 'user', 'content': text})
        self.memory.append({'role': 'assistant', 'content': full})
        answer = String()
        answer.data = (lead_in + ' ' + full).strip() if lead_in else full
        self.answer_pub.publish(answer)
        speed = ''
        if self._timings and self._timings.get('predicted_per_second'):
            speed = f', {self._timings["predicted_per_second"]:.0f} tokens/s'
        if handed:
            route = f'claude (local {reason}, handed over)'
        elif reason:
            route = f'local ({reason}, no Claude to hand to)'
        else:
            route = backend
        self.get_logger().info(f'{route}: "{text[:80]}" -> "{full[:160]}" (first words {first or 0:.1f} s, '
                               f'{time.monotonic() - t0:.1f} s, {self._usage[0]}+{self._usage[1]} tokens{speed}'
                               + (f', {cents:.1f} cents today' if backend == 'claude' else '')
                               + (', cut' if self._stopped else '') + ')')


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
