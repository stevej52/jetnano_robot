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


MOODS = {'hello': hello, 'ok': ok, 'no': no, 'alarm': alarm, 'sad': sad,
         'happy': happy, 'curious': curious, 'sleepy': sleepy, 'hm': hm}


def write_wav(path, samples, volume=0.6):
    y = samples / (np.max(np.abs(samples)) or 1.0) * volume
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
