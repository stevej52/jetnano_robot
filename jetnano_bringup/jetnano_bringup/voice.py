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

"""The robot's voice: one timbre, many moods, synthesised from nothing.

    ros2 run jetnano_bringup make_voice ~/sounds          # writes <mood>.wav files
    ros2 run jetnano_bringup make_voice ~/sounds --play   # and plays each (aplay)

Every sound is the same instrument - a warbling two-oscillator tone with a
slightly nasal formant, quick to speak and quick to stop - saying something
different. The moods and where the sounds node uses them:

    hello     three notes rising, a lift at the end      boot finished
    ok        two quick blips, up                        GO, command accepted
    no        two low blips, same pitch, a little rough  guard blocks, drop ahead
    alarm     fast two-tone, four times                  e-stop
    sad       a long slide down, vibrato slowing         battery low
    happy     a trill, up and over                       goal reached
    curious   a rising slide with a question step        person spotted
    sleepy    three notes falling, fading                shutting down / flat battery
    hm        one soft rising blip                       acknowledgement, looking
    huh       a cat's "mrrp?" - rolled onset, rising     something moved, a bang (motion_watch, ears)
    bye       three notes down with a little wave        "I have to go" ends a chat (listen)
    chat      a run of quick notes, a question or a      her side of a conversation; listen makes
              statement at the end                       one fresh each time, as long as yours

Each mood is written with three slightly different takes (pitch and timing
jitter), so she does not say exactly the same thing twice in a row.
"""

import math
import os
import random
import struct
import subprocess
import sys
import wave

import numpy as np

RATE = 22050
BASE = 467.0        # her middle: half an octave under A5, where Steve liked it (2026-09-24)
# The USB speaker swallows the first ~80 ms after it wakes: measured with her
# own mic 2026-09-25, "hm" came out as 0.08 s of 0.16 s and 8 dB quieter;
# with a lead-in of silence all of it came through. So every file starts quiet.
LEAD_S = 0.25
TAIL_S = 0.10


def tone(freq_curve, seconds, vibrato_hz=6.0, vibrato_depth=0.02, rough=0.0, bright=0.5):
    """A note whose pitch follows freq_curve(t in 0..1). The timbre: a sine
    plus a softer octave and a touch of the fifth, formant-ish, with vibrato;
    'rough' adds a low-rate wobble that makes it sound unhappy."""
    n = int(RATE * seconds)
    t = np.arange(n) / RATE
    u = t / seconds
    f = np.array([freq_curve(x) for x in u])
    vib = 1.0 + vibrato_depth * np.sin(2 * math.pi * vibrato_hz * t)
    if rough:
        vib *= 1.0 + rough * np.sign(np.sin(2 * math.pi * 28.0 * t)) * 0.5
    phase = 2 * math.pi * np.cumsum(f * vib) / RATE
    y = (np.sin(phase)
         + bright * 0.45 * np.sin(2 * phase)
         + bright * 0.20 * np.sin(3 * phase)
         + 0.12 * np.sin(1.5 * phase))
    # envelope: 8 ms attack, gentle decay, 25 ms release
    env = np.ones(n)
    a, r = int(0.008 * RATE), int(0.025 * RATE)
    env[:a] = np.linspace(0, 1, a)
    env[-r:] = np.linspace(1, 0, r)
    env *= np.exp(-1.2 * u)
    return y * env


def rest(seconds):
    return np.zeros(int(RATE * seconds))


def glide(f0, f1, curve=1.0):
    return lambda x: f0 * (f1 / f0) ** (x ** curve)


def flat(f):
    return lambda x: f


# ---- the moods --------------------------------------------------------------
def hello(j):
    k = 2 ** (j / 12)
    return np.concatenate([
        tone(flat(BASE * k), 0.13), rest(0.03),
        tone(flat(BASE * k * 1.26), 0.13), rest(0.03),
        tone(glide(BASE * k * 1.5, BASE * k * 2.0, 0.6), 0.30, vibrato_depth=0.03)])


def ok(j):
    k = 2 ** (j / 12)
    return np.concatenate([tone(flat(BASE * k * 1.2), 0.08), rest(0.04), tone(flat(BASE * k * 1.8), 0.11)])


def no(j):
    k = 2 ** (j / 12)
    low = BASE * k * 0.55
    return np.concatenate([tone(flat(low), 0.14, rough=0.5, bright=0.3), rest(0.06),
                           tone(flat(low), 0.16, rough=0.5, bright=0.3)])


def alarm(j):
    k = 2 ** (j / 12)
    parts = []
    for _ in range(4):
        parts += [tone(flat(BASE * k * 1.6), 0.09, vibrato_depth=0.0, bright=0.8),
                  tone(flat(BASE * k * 2.2), 0.09, vibrato_depth=0.0, bright=0.8)]
    return np.concatenate(parts)


def sad(j):
    k = 2 ** (j / 12)
    return tone(glide(BASE * k * 1.3, BASE * k * 0.5, 1.4), 0.95, vibrato_hz=3.5, vibrato_depth=0.04, bright=0.3)


def happy(j):
    k = 2 ** (j / 12)
    parts = []
    for f in (1.0, 1.26, 1.5, 2.0, 1.5, 2.0, 2.4):
        parts += [tone(flat(BASE * k * f), 0.07, vibrato_depth=0.0), rest(0.015)]
    parts.append(tone(glide(BASE * k * 2.4, BASE * k * 3.0), 0.18, vibrato_depth=0.03))
    return np.concatenate(parts)


def curious(j):
    k = 2 ** (j / 12)
    return np.concatenate([tone(glide(BASE * k * 0.9, BASE * k * 1.4, 0.8), 0.32),
                           rest(0.05),
                           tone(glide(BASE * k * 1.6, BASE * k * 2.1, 0.5), 0.16, vibrato_depth=0.03)])


def sleepy(j):
    k = 2 ** (j / 12)
    return np.concatenate([tone(flat(BASE * k * 1.2), 0.25, vibrato_hz=4.0, bright=0.35), rest(0.06),
                           tone(flat(BASE * k * 0.95), 0.30, vibrato_hz=3.5, bright=0.3), rest(0.06),
                           tone(glide(BASE * k * 0.75, BASE * k * 0.55), 0.55, vibrato_hz=3.0, bright=0.25)])


def hm(j):
    k = 2 ** (j / 12)
    return tone(glide(BASE * k * 1.1, BASE * k * 1.35), 0.16, bright=0.4)


def huh(j):
    """A cat's "mrrp?": a rolled, purring onset that lifts into a question.
    The roll is a fast amplitude flutter (about 24 Hz, a cat's trill rate)
    over the first half, fading out as the pitch rises; the end steps up a
    fourth and stops short, like a raised eyebrow."""
    k = 2 ** (j / 12)
    # Tuned with Steve, 2026-09-24/25: one continuous rise - no separate note at
    # the end (that read as "boop, boop") - the pitch just accelerates upward
    # over the last third; a deep purr through most of it.
    seconds = 0.65          # 0.75 was "perfect but 15 % faster" (Steve, 2026-09-25)
    y = tone(glide(BASE * k * 0.85, BASE * k * 1.6, 1.6), seconds, vibrato_hz=5.0, vibrato_depth=0.012, bright=0.35)
    n = len(y)
    t = np.arange(n) / RATE
    u = t / seconds
    flutter = 1.0 - 0.9 * np.clip(1.0 - u * 1.15, 0.0, 1.0) * (0.5 + 0.5 * np.sin(2 * math.pi * 24.0 * t))
    return y * flutter


def bye(j):
    """Bye-bye: two notes stepping down and a longer one that lifts a little
    at the end, a small wave rather than a sad one (sleepy is the sad one)."""
    k = 2 ** (j / 12)
    return np.concatenate([tone(flat(BASE * k * 1.5), 0.14), rest(0.03),
                           tone(flat(BASE * k * 1.26), 0.12), rest(0.05),
                           tone(glide(BASE * k * 1.0, BASE * k * 1.12, 0.5), 0.35, vibrato_depth=0.03, bright=0.4)])


CHAT_SCALE = (0.75, 0.84, 1.0, 1.12, 1.26, 1.5, 1.68, 2.0)


def chat(n=6, j=0.0, rng=random):
    """Chatter: n quick notes wandering over a happy scale, some sliding, an
    uneven rhythm, and a last note that goes up (a question) or down (a
    statement). Every call is different; listen makes one per reply."""
    k = 2 ** (j / 12)
    parts = []
    i = rng.randrange(2, 6)
    for _ in range(n):
        i = max(0, min(len(CHAT_SCALE) - 1, i + rng.choice([-2, -1, -1, 1, 1, 2, 3])))
        f = BASE * k * CHAT_SCALE[i]
        d = rng.choice([0.06, 0.07, 0.09, 0.11, 0.14])
        if rng.random() < 0.3:
            parts.append(tone(glide(f, f * rng.choice([0.84, 1.19, 1.26]), 0.8), d + 0.05, vibrato_depth=0.0, bright=0.5))
        else:
            parts.append(tone(flat(f), d, vibrato_depth=0.0, bright=0.5))
        parts.append(rest(rng.choice([0.015, 0.025, 0.04, 0.08])))
    f = BASE * k * CHAT_SCALE[i]
    up = rng.random() < 0.5
    parts.append(tone(glide(f, f * (1.5 if up else 0.67), 0.7), 0.18, vibrato_depth=0.03))
    return np.concatenate(parts)


MOODS = {'hello': hello, 'ok': ok, 'no': no, 'alarm': alarm, 'sad': sad,
         'happy': happy, 'curious': curious, 'sleepy': sleepy, 'hm': hm, 'huh': huh,
         'bye': bye, 'chat': lambda j: chat(random.randrange(4, 9), j)}


def write_wav(path, samples, volume=0.6):
    y = samples / (np.max(np.abs(samples)) or 1.0) * volume
    y = np.concatenate([np.zeros(int(RATE * LEAD_S)), y, np.zeros(int(RATE * TAIL_S))])
    data = struct.pack('<%dh' % len(y), *(int(v * 32767) for v in y))
    with wave.open(path, 'wb') as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(RATE)
        w.writeframes(data)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    out = os.path.expanduser(sys.argv[1])
    play = '--play' in sys.argv
    os.makedirs(out, exist_ok=True)
    random.seed(7)
    for mood, fn in MOODS.items():
        for take in range(3):
            jitter = random.uniform(-1.5, 1.5) if take else 0.0     # semitones
            path = os.path.join(out, f'{mood}{take + 1}.wav')
            write_wav(path, fn(jitter))
            if play and take == 0:
                print(mood)
                subprocess.run(['aplay', '-q', path], check=False)
    print(f'{len(MOODS) * 3} sounds in {out}: ' + ', '.join(MOODS))
    return 0


if __name__ == '__main__':
    sys.exit(main())
