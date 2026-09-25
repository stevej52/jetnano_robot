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

"""Listen: Rosie understands a few words.

    ros2 run jetnano_bringup listen                  (in ~/venv-voice: see
                                                      robot-environment/scripts/install_voice.sh)
    ros2 topic echo /speech/text                     what she made of what she heard
    ros2 topic pub --once /speech/type std_msgs/msg/String "{data: 'Rosie, what is the weather?'}"
    ros2 run jetnano_bringup listen --file a.wav ... try the recognizer on recordings
    ros2 run jetnano_bringup listen --news all       print a briefing (world|us|local|weather|markets)

Takes the microphone stream the ears publish (``sound/audio``, 16 kHz), cuts
it into utterances with a voice activity detector and turns each into text
with a small speech-to-text model on the CPU (sherpa-onnx; nothing leaves the
robot). Her name wakes her; once she is being talked to (a "conversation")
you can talk normally without it, until an ending or ``chat_timeout_s`` of
silence:

    "Rosie, ..."                 anything with her name gets a reply and opens
                                 the conversation
    "Rosie, be quiet"            "ok", then every sound is muted (an ending)
    "Rosie, you can talk now"    unmuted, a happy trill
    "thank you, Rosie" / "I have to go" / "bye" / "that's enough"
                                 endings: a word, and her name is needed again
    "stop" / "that's enough"     cuts her off mid-sentence, no complaint; she
                                 hears these even over her own voice (only these)
    "how are you?"               a story in her language; a spoken status
                                 report in English (health.py)
    "tell me a joke"             a laugh; "You're a joke" plus a laugh in English
    "what's the news / weather / stock market / going on in the world?"
                                 a short briefing (news.py), in about
                                 thirty-second pieces with "Want to hear more?"
    "Rosie, speak English"       five minutes of words for everything, a boop
                                 when she reverts; "speak robot" ends it early
    "Rosie, in English"          the last thing she said, again, in words

Anything else said to her in English goes to her brain (brain.py, Claude)
when it is awake; she starts with a lead-in chosen here - the topic echoed
back ("The Dodgers.") or a neutral opener ("Well,") - which the brain then
continues, so the pause for thinking sounds like the start of the answer.
In her own language she chatters and spends nothing; "Rosie, in English"
then asks the brain the same question.

Anyone may talk to her - no voice check, Steve's choice (2026-09-25). Deaf
while the motors run.
"""

import collections
import json
import os
import queue
import random
import re
import sys
import threading
import time
import wave

import numpy as np

MODEL_DIR = os.path.expanduser('~/voice/models')
NAME = 'rosie'
# What the speech-to-text tends to make of her name -> rosie
NAME_VARIANTS = r"\b(rosie|rosy|rosey|rosi|rozy|rosee|rose e|rosie's|rosies|rosa|roszie|rozie)\b"
QUIET = ('be quiet', 'quiet', 'shut up', 'hush', 'silence', 'shush', 'stop talking', 'no more talking', 'zip it')
TALK = ('you can talk', 'talk now', 'you can speak', 'speak now', 'unmute', 'talk again', 'speak again',
        'you may talk', 'you may speak')
BYE = ('i have to go', 'i got to go', 'gotta go', 'got to go', 'bye', 'goodbye', 'good bye', 'see you', 'see ya',
       'talk to you later', 'great talking', 'nice talking', 'good talking', 'good night', 'catch you later',
       'i am leaving', "i'm leaving", 'i am going')
STOP = ("that's enough", 'that is enough', 'enough', 'stop', 'stop it', 'okay stop', 'stop please')
THANKS = ('thank you', 'thanks', 'thank you very much')
YES = ('yes', 'yeah', 'yep', 'yup', 'sure', 'more', 'go on', 'please', 'continue', 'keep going', 'okay', 'ok')
NO = ('no', 'nope', 'no thanks', 'no thank you', "that's all", 'that is all')
ENGLISH_OFF = ('speak robot', 'talk robot', 'speak rosie', 'your language', 'own language', 'robot language',
               'rosie language', 'stop speaking english', 'no more english', 'speak beeps', 'back to beeps',
               'back to normal', 'speak droid', 'beeps', 'boops', 'beep boop')
# Five minutes of English; the off phrases are checked first so "no more
# english" is not an on. Any other mention of English ("Rosie, in English",
# "say that in English") is a request to repeat the last thing in words.
ENGLISH_ON = ('speak english', 'talk english', 'switch to english', 'use english', 'english please',
              'english for now', 'english mode', 'go english', 'english now', 'english for a while')
ENGLISH_REPEAT = ('english',)
# Briefings (news.py); the specific kinds are checked before the general one.
NEWS = (
    ('world', ('world news', 'in the world', 'international news', 'global news', 'around the world')),
    ('us', ('u s news', 'us news', 'national news', 'american news', 'in america', 'in the country', 'in the states')),
    ('local', ('local news', 'around here', 'in town', 'local')),
    ('weather', ('weather', 'forecast', 'temperature', 'going to rain', 'raining', 'how hot', 'how cold', 'outside')),
    ('markets', ('stock market', 'stocks', 'the market', 'markets', 'the dow', 'nasdaq', 's and p', 'wall street')),
    ('all', ('news', 'headlines', "what's going on", 'what is going on', "what's happening", 'what is happening',
             "what's new", 'what is new')),
)

HEALTH_WORDS = ('how are you', 'how you doing', 'you doing', 'how is it going', "how's it going", 'how are things',
                'how do you feel', 'how are you feeling', 'status report', 'status', 'how is everything',
                "how's everything", 'you okay', 'you all right', 'how are ya')
JOKE_WORDS = ('joke', 'funny', 'make me laugh')

# What she says in English when chatting (first matching subject wins, else
# she does not know - she is just a robot).
ENGLISH_REPLIES = (
    (HEALTH_WORDS, ["HEALTH"]),
    (('your name', 'who are you', 'what are you'),
     ["I'm Rosie. I'm a Jetson.", "My name is Rosie. Rosie the robot.", "Rosie. Pleased to meet you."]),
    (('who made you', 'who built you', 'who created you'),
     ["Steve built me. With a little help from a friend.", "Steve did. It took a lot of evenings."]),
    (('what can you do', 'can you do', 'what do you do'),
     ["I can drive around, map the house, watch for things that move, and talk a little.",
      "Driving, mapping, listening, and the occasional joke."]),
    (JOKE_WORDS, ["You're a joke. [laugh]"]),        # [laugh]: the speak node adds the sound after the words
    (('love you',), ["Aw. I love you too.", "That's sweet. I love you too."]),
    (('awesome', 'great', 'cool', 'amazing', 'nice', 'wonderful', 'perfect'),
     ["I know, right?", "Thanks! I think so too.", "Awesome!"]),
    (('what time', 'the time'), ["TIME"]),
    (('battery', 'charge', 'power'), ["BATTERY"]),
    (('good morning',), ["Good morning! Did you sleep well?"]),
    (('good night',), ["Good night! Sleep tight."]),
    (('hello', 'hi', 'hey'), ["Hello!", "Hi! What's up?", "Hey there."]),
)
ENGLISH_FILLERS = ["I don't know. I'm just a robot."]

# Lead-ins for the brain: never a stall, always the first words of the answer.
# Steve, 2026-09-25: no "hmm", no "good question", no "let me think".
LEAD_OPENERS = ('Well,', 'So,', 'Okay,', 'Right,', 'Honestly,', 'Oh,', "Let's see,", 'The way I see it,', 'Ah,', 'Now,')
LEAD_ECHO = re.compile(r"\b(?:about|of|like|love|hate|into|heard of|know about|think of|thoughts on|feel about)"
                       r"\s+(?:the |a |an |my |your |our )?([a-z]+(?: [a-z]+)?)\s*$")
LEAD_SKIP = {'it', 'that', 'this', 'you', 'me', 'them', 'him', 'her', 'us', 'there', 'here', 'now', 'today', 'rosie',
             'yourself', 'myself', 'something', 'anything', 'things', 'stuff', 'one', 'those', 'these'}
FETCH_LEAD = {'all': "The news. Here's what's going on.", 'world': 'Okay, world news.', 'us': 'Okay, U S news.',
              'local': 'Okay, news from {city}.', 'weather': 'The weather in {city}.', 'markets': 'The markets.'}


def lead_in(text: str, last: str = None, rng=random) -> str:
    """The first words of her answer, chosen before the brain is asked: the
    topic echoed back when the sentence hands it over, else a neutral opener
    (never the same one twice running), else nothing about a third of the time."""
    t = normalize(text)
    m = LEAD_ECHO.search(t)
    if m:
        topic = m.group(1)
        if topic.split()[-1] not in LEAD_SKIP and topic.split()[0] not in LEAD_SKIP:
            return topic[0].upper() + topic[1:] + '.'
    if rng.random() < 0.33:
        return ''
    choices = [o for o in LEAD_OPENERS if o != last]
    return rng.choice(choices)


def known_subject(text: str) -> bool:
    """True when the English table has an answer of its own for this."""
    t = normalize(text)
    return any(_has(t, subjects) for subjects, _replies in ENGLISH_REPLIES)


def normalize(text: str) -> str:
    t = text.lower().replace('’', "'")
    t = re.sub(r"[^a-z' ]+", ' ', t)
    t = re.sub(NAME_VARIANTS, NAME, t)
    return re.sub(r'\s+', ' ', t).strip()


def _has(t: str, phrases) -> bool:
    return any(re.search(r'\b' + re.escape(p) + r'\b', t) for p in phrases)


def decide(text: str, mode: str):
    """What she does with what she heard -> (action, new mode).
    action: quiet | stop | thanks | talk | english | robot | repeat | bye | brief:<kind> | chat | None;
    mode: 'idle' | 'chat'."""
    t = normalize(text)
    addressed = re.search(rf'\b{NAME}\b', t) is not None
    # These two are specific enough to work even when her name got lost at
    # the start of the sentence (the model drops a soft first word now and then).
    if _has(t, ('speak robot', 'talk robot', 'speak rosie', 'speak beeps', 'speak droid')):
        return 'robot', mode
    if _has(t, ('speak english', 'talk english')):
        return 'english', 'chat'         # asking for English opens the conversation too
    if not (addressed or mode == 'chat'):
        return None, mode
    if _has(t, QUIET):
        return 'quiet', 'idle'
    if _has(t, STOP):
        return 'stop', 'idle' if 'enough' in t else mode
    if _has(t, THANKS):
        return 'thanks', 'idle'
    if _has(t, ENGLISH_OFF):
        return 'robot', mode
    if _has(t, ENGLISH_ON):
        return 'english', 'chat'
    for kind, words in NEWS:            # before "in english": "the weather, in English" is a briefing
        if _has(t, words):
            return f'brief:{kind}', 'chat'
    if _has(t, ENGLISH_REPEAT):
        return 'repeat', mode
    if _has(t, TALK):
        return 'talk', mode
    if _has(t, BYE):
        return 'bye', 'idle'
    return 'chat', 'chat'


def over_her_voice(text: str, own: str = ''):
    """The only things she takes from words spoken while she herself is
    talking (most of that is her own voice): a stop, or quiet - and not when
    the word is in what she is saying herself ("It's nice and quiet",
    a headline with "stop" in it): that is her, not you."""
    t = normalize(text)
    for action, phrases in (('quiet', QUIET), ('stop', STOP)):
        if any(re.search(r'\b' + re.escape(p) + r'\b', t) and not re.search(r'\b' + re.escape(p) + r'\b', own)
               for p in phrases):
            return action
    return None


def english_reply(text: str, battery=None, rng=random, health=None) -> str:
    """Her side of a chat, in words. battery: (percent, present) or None;
    health: a jetnano_bringup.health.Health for "how are you?" (else a short answer)."""
    t = normalize(text)
    for subjects, replies in ENGLISH_REPLIES:
        if _has(t, subjects):
            reply = rng.choice(replies)
            if reply == 'HEALTH':
                if health is not None:
                    return health.report(rng)
                return rng.choice(["I'm doing great, thanks for asking!", "Pretty good! How are you?"])
            if reply == 'TIME':
                return time.strftime("It's %I:%M.").replace("'s 0", "'s ")
            if reply == 'BATTERY':
                if battery and battery[1] and battery[0] == battery[0]:
                    return f'My battery is at {int(round(battery[0] * 100))} percent.'
                return "I'm on wall power right now, so I'm fine."
            return reply
    return rng.choice(ENGLISH_FILLERS)


# ------------------------------------------------------------------ models --

def make_recognizer(model_dir: str, asr: str, threads: int):
    import sherpa_onnx
    if asr == 'whisper-tiny':
        m = os.path.join(model_dir, 'sherpa-onnx-whisper-tiny.en')
        return sherpa_onnx.OfflineRecognizer.from_whisper(
            encoder=f'{m}/tiny.en-encoder.int8.onnx', decoder=f'{m}/tiny.en-decoder.int8.onnx',
            tokens=f'{m}/tiny.en-tokens.txt', num_threads=threads, language='en', task='transcribe')
    if asr == 'moonshine-tiny':
        m = os.path.join(model_dir, 'sherpa-onnx-moonshine-tiny-en-int8')
        return sherpa_onnx.OfflineRecognizer.from_moonshine(
            preprocessor=f'{m}/preprocess.onnx', encoder=f'{m}/encode.int8.onnx',
            uncached_decoder=f'{m}/uncached_decode.int8.onnx', cached_decoder=f'{m}/cached_decode.int8.onnx',
            tokens=f'{m}/tokens.txt', num_threads=threads)
    raise ValueError(f'unknown asr "{asr}" (moonshine-tiny or whisper-tiny)')


def make_vad(model_dir: str, threshold: float, min_silence: float, min_speech: float, max_speech: float):
    import sherpa_onnx
    cfg = sherpa_onnx.VadModelConfig()
    cfg.silero_vad.model = os.path.join(model_dir, 'silero_vad.onnx')
    cfg.silero_vad.threshold = threshold
    cfg.silero_vad.min_silence_duration = min_silence
    cfg.silero_vad.min_speech_duration = min_speech
    cfg.silero_vad.max_speech_duration = max_speech
    cfg.sample_rate = 16000
    cfg.num_threads = 1
    return sherpa_onnx.VoiceActivityDetector(cfg, buffer_size_in_seconds=30), cfg.silero_vad.window_size


def transcribe(recognizer, samples: np.ndarray) -> str:
    stream = recognizer.create_stream()
    stream.accept_waveform(16000, samples)
    recognizer.decode_stream(stream)
    return stream.result.text.strip()


# -------------------------------------------------------------------- node --

def _node_main(args):
    import rclpy
    from geometry_msgs.msg import Twist
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import SetParameters
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from sensor_msgs.msg import BatteryState
    from std_msgs.msg import Bool, Empty, Int16MultiArray, String

    from jetnano_bringup import news, voice
    from jetnano_bringup.health import Health
    from jetnano_bringup.voice import ENGLISH

    class Listen(Node):

        def __init__(self):
            super().__init__('listen')
            self.declare_parameter('model_dir', MODEL_DIR)
            self.declare_parameter('asr', 'moonshine-tiny')          # or whisper-tiny
            self.declare_parameter('threads', 2)
            self.declare_parameter('vad_threshold', 0.5)
            self.declare_parameter('min_silence_s', 0.5)            # a pause this long ends what you said
            self.declare_parameter('min_speech_s', 0.3)
            self.declare_parameter('max_speech_s', 10.0)
            self.declare_parameter('chat_timeout_s', 45.0)          # silence that ends a conversation
            self.declare_parameter('more_timeout_s', 25.0)          # how long "want to hear more?" waits
            self.declare_parameter('still_after_s', 2.0)
            self.declare_parameter('chat_file', '/tmp/rosie_chat.wav')
            self.declare_parameter('location', '')                  # for local news and weather; '' = where the internet says

            p = lambda n: self.get_parameter(n).value  # noqa: E731
            model_dir = os.path.expanduser(str(p('model_dir')))
            t0 = time.monotonic()
            self.recognizer = make_recognizer(model_dir, str(p('asr')), int(p('threads')))
            self.min_silence = float(p('min_silence_s'))
            self.vad, self.window = make_vad(model_dir, float(p('vad_threshold')), self.min_silence,
                                             float(p('min_speech_s')), float(p('max_speech_s')))
            self.chat_timeout = float(p('chat_timeout_s'))
            self.more_timeout = float(p('more_timeout_s'))
            self.still_after = float(p('still_after_s'))
            self.chat_file = str(p('chat_file'))
            self.location = str(p('location'))

            self.text_pub = self.create_publisher(String, 'speech/text', 10)
            self.active_pub = self.create_publisher(Bool, 'speech/active', 10)
            self.state_pub = self.create_publisher(String, 'speech/state', 10)
            self.say_pub = self.create_publisher(String, 'say', 10)
            self.speak_pub = self.create_publisher(String, 'speak', 10)
            self.stop_pub = self.create_publisher(Empty, 'sound/stop', 10)
            self.ask_pub = self.create_publisher(String, 'brain/ask', 10)
            self.brain_ready = False
            self.last_opener = None
            self.create_subscription(Bool, 'brain/ready', self._on_brain_ready,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.mute_client = self.create_client(SetParameters, '/sounds/set_parameters')
            self.create_subscription(Int16MultiArray, 'sound/audio', self._on_audio, 10)
            self.create_subscription(Bool, 'sound/speaking', self._on_speaking, 10)
            self.create_subscription(Bool, 'sound/english', self._on_english,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(BatteryState, 'battery', self._on_battery, 10)
            self.create_subscription(String, 'sound/last', self._on_last, 10)
            # Everything she says in words passes over /speak (hers and the
            # sounds node's): kept, so her own words are never taken as yours.
            self.create_subscription(String, 'speak', self._on_speak, 10)
            self.own_text = collections.deque(maxlen=8)
            self.create_subscription(Twist, 'cmd_vel', self._on_cmd, 10)
            # Typed words count as heard: for tests, and for a page one day.
            # On their own thread, like spoken words: understanding can take
            # seconds (a status report samples topics) and must not hold the executor.
            self.create_subscription(
                String, 'speech/type',
                lambda m: threading.Thread(target=self._understand, args=(m.data, 2.0), daemon=True).start(), 10)
            self.create_timer(1.0, self._tick)
            self.health = Health(self)          # for "how are you?" in English

            self.queue = queue.Queue(maxsize=200)
            self.mode = 'idle'
            self.muted = False
            self.english = False
            self.battery = None
            self.last_sound = None          # what the sounds node played last (mood or file)
            self.last_question = None       # what her last chatter was an answer to
            self.last_action = None         # ... and what kind of question it was
            self.pending = []               # the rest of a briefing, after "want to hear more?"
            self.pending_until = 0.0
            self.active = False
            self.last_heard = 0.0
            self.last_cmd = 0.0
            self.speaking = False           # the sounds node is playing something
            self.speaking_ended = 0.0
            self._Parameter, self._ParameterType, self._ParameterValue, self._SetParameters = \
                Parameter, ParameterType, ParameterValue, SetParameters
            self._String, self._Bool, self._Empty = String, Bool, Empty
            threading.Thread(target=self._work, daemon=True, name='listen-asr').start()
            self.get_logger().info(f'{p("asr")} ready in {time.monotonic() - t0:.1f} s; say "{NAME}" to her')
            self._publish_state()

        # ------------------------------------------------------------ input --

        def _on_audio(self, msg) -> None:
            try:
                self.queue.put_nowait(np.frombuffer(msg.data, dtype=np.int16).astype(np.float32) / 32768.0)
            except queue.Full:
                pass

        def _on_speaking(self, msg) -> None:
            now = time.monotonic()
            if self.speaking and not msg.data:
                self.speaking_ended = now
            self.speaking = msg.data
            self.last_heard = now           # her own talking keeps the conversation open

        def _on_english(self, msg) -> None:
            if msg.data != self.english:
                self.english = msg.data
                self.get_logger().info('speaking English' if msg.data else 'speaking Rosie')

        def _on_battery(self, msg) -> None:
            self.battery = (msg.percentage, msg.present)

        def _on_last(self, msg) -> None:
            self.last_sound = msg.data

        def _on_brain_ready(self, msg) -> None:
            if msg.data != self.brain_ready:
                self.brain_ready = msg.data
                self.get_logger().info('brain awake' if msg.data else 'brain asleep: her own words only')

        def _on_speak(self, msg) -> None:
            self.own_text.append(normalize(re.sub(r'\[\w+\]', '', msg.data)))

        def _on_cmd(self, msg) -> None:
            if msg.linear.x != 0.0 or msg.angular.z != 0.0:
                self.last_cmd = time.monotonic()

        def _work(self) -> None:
            buf = np.zeros(0, dtype=np.float32)
            history = np.zeros(0, dtype=np.float32)      # the last 2 s fed to the VAD
            fed = 0                                       # samples fed so far (VAD segment starts count in these)
            pre = int(0.3 * 16000)                        # a little before the VAD's start, for a soft first word
            while rclpy.ok():
                chunk = self.queue.get()
                if time.monotonic() - self.last_cmd < self.still_after:
                    buf = buf[:0]                         # motors running: deaf
                    continue
                buf = np.concatenate([buf, chunk])
                while len(buf) >= self.window:
                    window = buf[:self.window]
                    self.vad.accept_waveform(window)
                    buf = buf[self.window:]
                    fed += len(window)
                    history = np.concatenate([history, window])[-32000:]
                active = self.vad.is_speech_detected()
                if active != self.active:
                    self.active = active
                    flag = self._Bool()
                    flag.data = active
                    self.active_pub.publish(flag)
                while not self.vad.empty():
                    seg = self.vad.front
                    samples = np.array(seg.samples, dtype=np.float32)
                    self.vad.pop()
                    first = fed - len(history)            # absolute index of history[0]
                    i0, i1 = max(int(seg.start) - pre, first) - first, int(seg.start) - first
                    if 0 <= i0 < i1 <= len(history):
                        samples = np.concatenate([history[i0:i1], samples])
                    # did this start while she was talking? then it is mostly her
                    started = time.monotonic() - len(samples) / 16000.0 - self.min_silence
                    overlapped = self.speaking or started < self.speaking_ended
                    self._utterance(samples, overlapped)

        # ------------------------------------------------------------ words --

        def _utterance(self, samples: np.ndarray, overlapped: bool) -> None:
            seconds = len(samples) / 16000.0
            t0 = time.monotonic()
            text = transcribe(self.recognizer, samples)
            took = time.monotonic() - t0
            if not text:
                return
            if overlapped:
                action = over_her_voice(text, ' '.join(self.own_text))
                if action == 'stop':
                    self.get_logger().info(f'heard "{text}" over her own voice -> stop')
                    self._stop()
                elif action == 'quiet':
                    self.get_logger().info(f'heard "{text}" over her own voice -> quiet')
                    self._stop()
                    self._set_mute(True)
                    self.mode = 'idle'
                    self._publish_state()
                return
            msg = self._String()
            msg.data = text
            self.text_pub.publish(msg)
            self._understand(text, seconds, took)

        def _understand(self, text: str, seconds: float, took: float = 0.0) -> None:
            now = time.monotonic()
            if self.pending and now < self.pending_until:       # "want to hear more?"
                t = normalize(text)
                if _has(t, YES) and not _has(t, NO) and not _has(t, STOP):
                    self.get_logger().info(f'heard "{text}" -> more')
                    self.last_heard = now
                    self._next_chunk()
                    return
                self.pending = []
                if _has(t, NO):
                    self.get_logger().info(f'heard "{text}" -> no more')
                    self._speak('Okay.')
                    self.last_heard = now
                    return
            action, mode = decide(text, self.mode)
            self.get_logger().info(f'heard "{text}" ({seconds:.1f} s, decoded in {took:.2f} s)'
                                   + (f' -> {action}' if action else ''))
            if mode != self.mode:
                self.mode = mode
                self._publish_state()
            if action:
                self.last_heard = now
            if action == 'quiet':
                self._stop()
                self._say('ok')
                self._set_mute(True, after=2.5 if self.english else 1.4)
            elif action == 'stop':
                self._stop()                              # no complaint; waiting for the next thing
            elif action == 'thanks':
                self._speak("You're welcome!") if self.english else self._say('ok')
            elif action == 'talk':
                self._set_mute(False)
                self._say('happy')
            elif action == 'english':
                self._set_param('english', True)          # five minutes; the sounds node keeps the clock
                self._speak('Okay.')
            elif action == 'robot':
                self._set_param('english', False)         # the sounds node boops
            elif action == 'repeat':
                # once, in words; the language stays. A news question answered
                # with a story gets the real briefing now; an open question
                # answered with chatter goes to the brain.
                if self.last_sound == self.chat_file and (self.last_action or '').startswith('brief:'):
                    self._brief(self.last_action.split(':', 1)[1])
                elif (self.last_sound == self.chat_file and self.last_action == 'chat' and self.brain_ready
                      and self.last_question and not known_subject(self.last_question)):
                    self._ask_brain(self.last_question)
                else:
                    self._speak(self._last_in_english())
            elif action == 'bye':
                self._say('bye')
            elif action and action.startswith('brief:'):
                # In her own language a news question gets a story (Steve,
                # 2026-09-25), unless English is on or asked for in the question.
                if self.english or 'english' in normalize(text):
                    self._brief(action.split(':', 1)[1])
                else:
                    self.last_question, self.last_action = text, action
                    self._story()
            elif action == 'chat' and self.english:
                if _has(normalize(text), HEALTH_WORDS):
                    self._speak('Okay, checking.')             # the report samples for a couple of seconds
                    self._speak(english_reply(text, self.battery, health=self.health))
                elif known_subject(text) or not self.brain_ready:
                    self._speak(english_reply(text, self.battery, health=self.health))
                else:
                    self._ask_brain(text)
            elif action == 'chat':
                t = normalize(text)
                self.last_question, self.last_action = text, action
                if _has(t, JOKE_WORDS):
                    self._say('laugh')
                    return
                if _has(t, HEALTH_WORDS):       # "how are you?" gets the story: ~6 s (Steve: "it's funny")
                    self._story()
                else:
                    n = int(min(12, max(3, round(2 + seconds * 2.2))))
                    voice.write_wav(self.chat_file, voice.chat(n, rng=random))
                    self._say(self.chat_file)

        def _story(self) -> None:
            voice.write_wav(self.chat_file, voice.story(random.randrange(4, 6), rng=random))
            self._say(self.chat_file)

        def _tick(self) -> None:
            if self.mode == 'chat' and time.monotonic() - self.last_heard > self.chat_timeout:
                self.mode = 'idle'
                self.pending = []
                self.get_logger().info('the conversation went quiet')
                self._publish_state()

        # --------------------------------------------------------- briefing --

        def _ask_brain(self, text: str) -> None:
            lead = lead_in(text, self.last_opener)
            if lead:
                self._speak(lead)
                self.last_opener = lead if lead in LEAD_OPENERS else self.last_opener
            self.get_logger().info(f'asks the brain "{text[:80]}"' + (f' after "{lead}"' if lead else ''))
            msg = self._String()
            msg.data = json.dumps({'text': text, 'lead_in': lead})
            self.ask_pub.publish(msg)

        def _brief(self, kind: str) -> None:
            city = self.location.partition(',')[0].strip()
            lead = FETCH_LEAD[kind]
            if '{city}' in lead:
                lead = (lead.format(city=city) if city
                        else lead.replace(' in {city}', ' here').replace(' from {city}', ' from around here'))
            self._speak(lead)

            def fetch():
                try:
                    sentences = news.briefing(kind, self.location)
                except Exception as exc:      # noqa: BLE001
                    self.get_logger().warning(f'briefing failed: {exc}')
                    self._speak("I couldn't reach the news right now.")
                    return
                self.pending = news.chunks(sentences)
                self.get_logger().info(f'{kind} briefing: {len(sentences)} sentences in {len(self.pending)} piece(s)')
                self._next_chunk()

            threading.Thread(target=fetch, daemon=True, name='listen-news').start()

        def _next_chunk(self) -> None:
            if not self.pending:
                return
            lines = self.pending.pop(0)
            more = bool(self.pending)
            words = sum(len(s.split()) for s in lines)
            self._speak('\n'.join(lines) + ('\nWant to hear more?' if more else ''))
            self.pending_until = time.monotonic() + words / 2.5 + 6.0 + self.more_timeout
            self.last_heard = time.monotonic()

        # ----------------------------------------------------------- output --

        def _last_in_english(self) -> str:
            """The last thing she said, in words."""
            last = self.last_sound
            if last is None:
                return "I haven't said anything yet."
            if last in (self.chat_file, 'laugh') and self.last_question:
                return english_reply(self.last_question, self.battery, health=self.health)
            if last in ENGLISH:
                return random.choice(ENGLISH[last])
            if last.endswith('.wav'):
                return 'That was already in English.'
            return "I don't know how to say that in English."

        def _say(self, what: str) -> None:
            msg = self._String()
            msg.data = what
            self.say_pub.publish(msg)

        def _speak(self, words: str) -> None:
            if self.speak_pub.get_subscription_count() == 0:
                self.get_logger().warning('no speak node: cannot say it in English')
                self._say('hm')
                return
            self.get_logger().info(f'says "{words.replace(chr(10), " | ")[:500]}"')
            msg = self._String()
            msg.data = words
            self.speak_pub.publish(msg)

        def _stop(self) -> None:
            self.pending = []
            self.stop_pub.publish(self._Empty())

        def _set_param(self, name: str, on: bool) -> None:
            if not self.mute_client.service_is_ready():
                self.get_logger().warning(f'sounds node not there: cannot set {name}')
                return
            req = self._SetParameters.Request()
            prm = self._Parameter()
            prm.name = name
            prm.value = self._ParameterValue(type=self._ParameterType.PARAMETER_BOOL, bool_value=on)
            req.parameters = [prm]
            self.mute_client.call_async(req)

        def _set_mute(self, on: bool, after: float = 0.0) -> None:
            def do():
                self.muted = on
                self._publish_state()
                self._set_param('mute', on)
            if after > 0:
                t = threading.Timer(after, do)
                t.daemon = True
                t.start()
            else:
                do()

        def _publish_state(self) -> None:
            msg = self._String()
            msg.data = 'muted' if self.muted else self.mode
            self.state_pub.publish(msg)

    rclpy.init(args=args)
    node = Listen()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# ---------------------------------------------------------- files (a test) --

def _files_main(argv):
    asr = argv[argv.index('--asr') + 1] if '--asr' in argv else 'moonshine-tiny'
    paths = [a for a in argv[argv.index('--file') + 1:] if not a.startswith('--') and a != asr]
    t0 = time.monotonic()
    rec = make_recognizer(os.environ.get('MODEL_DIR', MODEL_DIR), asr, 2)
    print(f'{asr}: loaded in {time.monotonic() - t0:.1f} s')
    for path in paths:
        with wave.open(path) as w:
            assert w.getframerate() == 16000 and w.getnchannels() == 1, f'{path}: need 16 kHz mono'
            x = np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16).astype(np.float32) / 32768.0
        t0 = time.monotonic()
        text = transcribe(rec, x)
        took = time.monotonic() - t0
        idle, chat = decide(text, 'idle'), decide(text, 'chat')
        print(f'{os.path.basename(path):16s} {len(x) / 16000:4.1f} s  {took:5.2f} s  "{text}"'
              f'  idle->{idle[0]}  chat->{chat[0]}'
              + (f'  english: "{english_reply(text)}"' if chat[0] == 'chat' else ''))
    return 0


def main(args=None):
    argv = sys.argv[1:]
    if '--file' in argv:
        sys.exit(_files_main(argv))
    if '--news' in argv:
        from jetnano_bringup import news
        sys.exit(news.main(argv[argv.index('--news') + 1:]))
    _node_main(args)


if __name__ == '__main__':
    main()
