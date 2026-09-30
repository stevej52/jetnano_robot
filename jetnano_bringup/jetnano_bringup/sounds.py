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
    /battery/level soon / low / flat          lowsoon once / lowbat (the countdown, again every 5 min)
                                              / alarm then sleepy
    /watchdog/events down / slow              uhoh      (anything dropped, or not right)
    /watchdog/events back                     beeps     (whatever the uh-oh was is back)
    /say <mood>                               that mood (anyone: the page, a person detector)
    /say /path/to/file.wav                    that file (listen's freshly made chatter)

One sound at a time; a mood is not repeated within ``min_gap_s``; ``mute``
is a parameter (ros2 param set /sounds mute true) so a droid that chirps at
night can be told to stop without stopping it. ``quiet_file`` (~/voice/quiet)
is the same across reboots - "Diane is sleeping" mode, ``ros2 run
jetnano_bringup quiet on|off|status``; the power-on and goodbye hooks honour
it too. With no speaker present it logs once and keeps trying quietly.

English mode ("Rosie, speak English", or ros2 param set /sounds english true):
for ``english_for_s`` (five minutes) every mood is said in words instead
(voice.ENGLISH, through the ``speak`` node); then she goes back to her own
language by herself, with a boopity boop. ``sound/english`` (latched) says
which it is. ``sound/last`` is whatever she played last (a mood or a file),
for "Rosie, in English". ``sound/stop`` (Empty) cuts the current sound and
drops the queue: "stop" / "that's enough".
"""

import glob
import json
import os
import queue
import random
import subprocess
import threading
import time
import wave

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Empty, String

from jetnano_bringup.voice import ENGLISH

try:
    from nav2_msgs.msg import CollisionMonitorState
except ImportError:  # pragma: no cover
    CollisionMonitorState = None


# What she says when the watchdog reports something (its /watchdog/events "kind"). Steve,
# 2026-09-29: "anytime anything goes wrong ... a robot uh-oh ... and then when it comes back on
# ... a high three beeps like at the end of the boot up" - heard as it happens, not told after
# the fact; "if I hear a whole bunch of them, then I can kind of determine that something is
# going wrong". 'act' (a restart) and 'still_down' are the same trouble again, so no new sound;
# 'gave_up' has the watchdog's own announcement (sad).
WATCHDOG_MOODS = {'down': 'uhoh', 'slow': 'uhoh', 'back': 'beeps'}
# the watchdog's name for the reSpeaker stuck in the kernel: the speaker is on it, so an
# aplay now would only get stuck with it
SOUND_DEVICE = 'microphone and speaker'


def watchdog_mood(event_json: str):
    """The mood for one /watchdog/events message, or None."""
    try:
        rec = json.loads(event_json)
        kind, what = rec.get('kind'), rec.get('what')
    except (ValueError, AttributeError):
        return None
    if what == SOUND_DEVICE and kind != 'back':
        return None
    return WATCHDOG_MOODS.get(kind)


def _duration(path: str) -> float:
    try:
        with wave.open(path) as w:
            return w.getnframes() / float(w.getframerate())
    except (OSError, wave.Error, EOFError):
        return 5.0


class Sounds(Node):

    def __init__(self):
        super().__init__('sounds')
        self.declare_parameter('sound_dir', os.path.expanduser('~/sounds'))
        # The USB speaker by ALSA card NAME, not number: numbers shuffle when a
        # microphone is plugged in. 'aplay -l' shows the name in brackets.
        # 2026-09-28: the speaker hangs off the reSpeaker Flex (card L16K6Ch; the old
        # USB speaker was UACDemoV10), so the mic's echo canceller hears what she says
        self.declare_parameter('card', 'L16K6Ch')
        self.declare_parameter('device', '')          # aplay -D ...; empty = plughw:<card>
        # Set on the card's PCM control at start. The reSpeaker has two volume controls
        # in series, PCM,0 and PCM,1 (both boot at -20 dB); the second is held at full
        # ('full_control') so this one alone sets her loudness. Measured 2026-09-28 with
        # the reSpeaker's own mics: 90 % (-6 dB) is loud and the echo canceller still
        # takes ~18 dB of her voice out; at 100 % the little speaker distorts and only
        # ~12 dB comes out. Steve, by ear: 100 % "5 % distorted", 95 % (-3 dB) it is -
        # the speaker is a 4 ohm 3 W driver and the reSpeaker's amp on USB power gives
        # it about 3 W at 0 dB, ~1.5 W here.
        self.declare_parameter('volume_percent', 95)
        self.declare_parameter('full_control', 'PCM,1')   # '' = none
        # "over and out" (listen) leaves this flag: she starts silent after a reboot
        # 'over and out' (listen's voice_off flag) no longer mutes her sounds: Steve,
        # 2026-09-26, wants her noises to carry on with only the talking switched off
        self.declare_parameter('mute', False)
        # "Diane is sleeping" mode (Steve, 2026-09-30): while this file exists nothing plays -
        # sounds, spoken replies (they come back here as /say <wav>), the watchdog's uh-ohs -
        # and the power-on and goodbye hooks stay silent too. Survives a reboot, unlike mute.
        #     ros2 run jetnano_bringup quiet on | off | status
        self.declare_parameter('quiet_file', os.path.expanduser('~/voice/quiet'))
        self._quiet = None
        self.declare_parameter('min_gap_s', 2.5)
        self.declare_parameter('hello_after_s', 4.0)
        self.declare_parameter('english', False)         # speak English (for english_for_s, then back)
        self.declare_parameter('english_for_s', 300.0)
        self.declare_parameter('watchdog_sounds', True)  # uh-oh / beeps on the watchdog's events
        self.declare_parameter('low_battery_repeat_s', 300.0)   # the countdown again while it stays low

        self.dir = os.path.expanduser(str(self.get_parameter('sound_dir').value))
        card = str(self.get_parameter('card').value)
        self.device = str(self.get_parameter('device').value) or (f'plughw:{card}' if card else '')
        volume = int(self.get_parameter('volume_percent').value)
        if card:
            for control in ('PCM', 'Speaker', 'Master'):
                subprocess.run(['amixer', '-q', '-c', card, 'sset', control, f'{volume}%'],
                               capture_output=True, timeout=5)
            full = str(self.get_parameter('full_control').value)
            if full:
                subprocess.run(['amixer', '-q', '-c', card, 'sset', full, '100%'],
                               capture_output=True, timeout=5)
        self.min_gap = float(self.get_parameter('min_gap_s').value)
        self.queue = queue.Queue()
        self.last_said = {}
        self._warned = False
        # True while a sound plays: the ears must not take her own voice for a noise.
        self.speaking_pub = self.create_publisher(Bool, 'sound/speaking', 10)
        # English mode: words go to the speak node; the state is latched for listen.
        self.speak_pub = self.create_publisher(String, 'speak', 10)
        self.english_pub = self.create_publisher(
            Bool, 'sound/english', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.last_pub = self.create_publisher(String, 'sound/last', 10)
        self.proc = None
        self._stopped = False
        self.create_subscription(Empty, 'sound/stop', lambda m: self.stop(), 10)
        self.english_until = 0.0
        self.add_on_set_parameters_callback(self._on_params)
        self.create_timer(1.0, self._tick)
        self._publish_english()
        threading.Thread(target=self._player, daemon=True, name='sounds-player').start()

        self._e_stop = None
        self._guard_stop = False
        self._battery_state = 'ok'
        self._low_said = 0.0
        self._drop = False
        self.create_subscription(String, 'say', lambda m: self.say(m.data, force=m.data.endswith('.wav')), 10)
        for topic in ('e_stop', 'e_stop_web', 'e_stop_joy'):      # one lock per source since 2026-09-26
            self.create_subscription(Bool, topic, self._on_e_stop, 10)
        self.create_subscription(Bool, 'cliff/drop', self._on_drop, 10)
        self.create_subscription(String, 'battery/level', self._on_battery_level,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, 'watchdog/events', self._on_watchdog, 20)
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

    def stop(self) -> None:
        """Cut whatever is playing and forget whatever is waiting."""
        with self.queue.mutex:
            self.queue.queue.clear()
        proc = self.proc
        if proc is not None and proc.poll() is None:
            self._stopped = True
            proc.terminate()

    # --------------------------------------------------------------- english --

    @property
    def english(self) -> bool:
        return time.monotonic() < self.english_until

    def _on_params(self, params):
        for prm in params:
            if prm.name == 'english':
                was_on = self.english_until > 0.0
                self.english_until = (time.monotonic() + float(self.get_parameter('english_for_s').value)
                                      if prm.value else 0.0)
                self._publish_english(prm.value)
                if was_on and not prm.value:
                    self.get_logger().info('back to her own language')
                    self.say('boop', force=True)
        return SetParametersResult(successful=True)

    @property
    def quiet(self) -> bool:
        return bool(self._quiet)

    def _tick(self) -> None:
        if self.english_until and not self.english:       # the five minutes are up
            self.set_parameters([Parameter('english', Parameter.Type.BOOL, False)])
        quiet = os.path.exists(str(self.get_parameter('quiet_file').value))
        if quiet != self._quiet:
            if self._quiet is not None or quiet:
                self.get_logger().info('quiet mode ' + ('ON: nothing plays' if quiet else 'off'))
            self._quiet = quiet
            if quiet:
                self.stop()

    def _publish_english(self, on=None) -> None:
        msg = Bool()
        msg.data = bool(self.english if on is None else on)
        self.english_pub.publish(msg)

    # ---------------------------------------------------------------- player --

    def _player(self) -> None:
        while True:
            mood = self.queue.get()
            if bool(self.get_parameter('mute').value) or self.quiet:
                continue
            if mood.endswith('.wav') and os.path.isfile(mood):
                takes = [mood]
            elif self.english and mood in ENGLISH and self.speak_pub.get_subscription_count() > 0:
                words = String()
                words.data = random.choice(ENGLISH[mood])
                self.speak_pub.publish(words)      # comes back as /say <path>
                continue
            else:
                takes = glob.glob(os.path.join(self.dir, f'{mood}[0-9]*.wav'))
            if not takes:
                self.get_logger().warning(f'no sound for "{mood}"')
                continue
            path = random.choice(takes)
            cmd = ['aplay', '-q'] + (['-D', self.device] if self.device else []) + [path]
            self._speaking(True)
            try:
                for attempt in range(5):
                    self._stopped = False
                    self.proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
                    try:
                        # a status report runs half a minute: give a file its length plus a margin
                        _, err = self.proc.communicate(timeout=_duration(path) + 8.0)
                    except subprocess.TimeoutExpired:
                        self.proc.kill()
                        _, err = self.proc.communicate()
                    rc = self.proc.returncode
                    self.proc = None
                    if self._stopped:
                        rc, err = 0, ''            # cut on purpose: not an error, not "said"
                        break
                    # someone else (a test, a tune) has the speaker: wait for them
                    if rc == 0 or 'busy' not in err or attempt == 4:
                        break
                    time.sleep(0.4)
                if rc != 0 and not self._warned:
                    self.get_logger().warning(f'cannot play sounds: {err.strip()[:120]} (is the speaker plugged in?)')
                    self._warned = True
                elif rc == 0:
                    self._warned = False
                    if not self._stopped:
                        last = String()
                        last.data = mood
                        self.last_pub.publish(last)
            except OSError as exc:
                if not self._warned:
                    self.get_logger().warning(f'cannot play sounds: {exc}')
                    self._warned = True
            finally:
                self.proc = None
                self._speaking(False)

    def _speaking(self, on: bool) -> None:
        msg = Bool()
        msg.data = on
        self.speaking_pub.publish(msg)

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

    def _on_watchdog(self, msg: String) -> None:
        # not forced: a burst (the camera container takes three streams down at once) is one
        # uh-oh, and their return one set of beeps (min_gap_s)
        mood = watchdog_mood(msg.data)
        if mood and bool(self.get_parameter('watchdog_sounds').value):
            self.say(mood)

    def _on_battery_level(self, msg: String) -> None:
        """battery_monitor's verdict (Steve's minute rule): soon -> three notes once; low -> the
        five-note countdown, again every low_battery_repeat_s; flat -> alarm, then sleepy."""
        try:
            state = json.loads(msg.data).get('level', 'ok')
        except ValueError:
            return
        now = time.monotonic()
        if state != self._battery_state:
            if state == 'soon':
                self.say('lowsoon', force=True)
            elif state == 'low':
                self.say('lowbat', force=True)       # the countdown (Steve, 2026-09-30)
                self._low_said = now
            elif state == 'flat':
                self.say('alarm', force=True)
                self.say('sleepy', force=True)
            self._battery_state = state
        elif state == 'low' and now - self._low_said > float(self.get_parameter('low_battery_repeat_s').value):
            self.say('lowbat', force=True)           # still low: say so again
            self._low_said = now


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
