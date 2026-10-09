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

"""Buzz bench: which stop routine quiets the steering? A bench tool: WHEELS UP or ESC unplugged.

    voice on                                        # the ears publish sound/audio
    ros2 run jetnano_bringup buzz_source routines --wheels-up [--rounds 5]
    ros2 run jetnano_bringup buzz_source find --wheels-up       # cut one output at a time

Steve's plan (2026-10-08): use her microphone ONCE, on the bench, to find a stop routine that
quiets the steering servo, then run that routine at every stop with no microphone. Tyres on
something grippy so a steer winds them up as a real stop does.

routines: each trial provokes (steer 15 deg one way for 1 s, back to centre, as a stop after a
turn), listens, runs one candidate, listens again. Candidates run in a shuffled order each round,
with 'wait' (do nothing) as the control: the buzz comes and goes by itself, so a routine only
counts if it beats waiting. Steve listens too; the meter has been wrong before.

find: cut one output's pulse (pca9685 / head_pwm <name>/pulse_width 0) at a time, then all.

The mic stream lags. settle judged a 1 s release by one half second of audio, which may well
have been the buzz from before the release ("not the steering", 10-03 on, while Steve says it is
the steering). So this measures the lag first (a steering snap is loud) and listens that much
later. The buzz measure is settle's (top three bins over the band median per half second) in
the 1.6 kHz band and the 5.5-6.7 kHz cluster seen on 2026-09-27. Spectra go to
~/calibration/buzz/<mode>-<time>.npz.
"""

import argparse
import math
import os
import random
import threading
import time

import numpy as np
import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Float64, Int16MultiArray
from std_srvs.srv import Trigger

RATE_HZ = 16000
CHUNK = 1600
BANDS = {'1.6k': (1400.0, 1800.0), '6k': (5500.0, 6700.0)}
STEER = ['steering', 'rear_steering']
PCA = STEER + ['throttle']
HEAD = ['pan', 'tilt']
PROVOKE_DEG = 15.0
BUZZ_DB = 8.0          # settle's: room 3-6, faint buzz 8-10, loud 15-18


def prominence(power, mask):
    """settle's buzz measure: the three loudest bins over the band's median, dB."""
    band = np.sort(power[mask])
    return float(10.0 * math.log10(max(band[-3:].mean(), 1e-20) / max(float(np.median(band)), 1e-20)))


def verdict(base, cut, room=6.0):
    """One output's rounds: 'SOURCE' when cutting it brings a buzz down to the room."""
    pairs = [(b, c) for b, c in zip(base, cut) if b is not None and c is not None]
    if not pairs:
        return 'no data'
    buzzing = [(b, c) for b, c in pairs if b > room + 2.0]
    if not buzzing:
        return 'no buzz heard to judge by'
    quieted = sum(1 for b, c in buzzing if c <= room + 1.0 and b - c >= 4.0)
    if quieted == len(buzzing):
        return f'SOURCE (quiet every time it buzzed, {quieted}/{len(buzzing)})'
    if quieted:
        return f'part ({quieted}/{len(buzzing)} quieted)'
    return f'not it (0/{len(buzzing)})'


def score(trials):
    """A routine's trials [(before, after)] -> (buzzing before, of those quiet after,
    quiet after of all, trials, median after); None without data."""
    ok = [(b, a) for b, a in trials if b is not None and a is not None]
    if not ok:
        return None
    buzzed = [(b, a) for b, a in ok if b >= BUZZ_DB]
    return (len(buzzed), sum(1 for _, a in buzzed if a < BUZZ_DB),
            sum(1 for _, a in ok if a < BUZZ_DB), len(ok), float(np.median([a for _, a in ok])))


class Ears:
    def __init__(self, node):
        freqs = np.fft.rfftfreq(CHUNK, 1.0 / RATE_HZ)
        self.masks = {k: (freqs >= lo) & (freqs <= hi) for k, (lo, hi) in BANDS.items()}
        self.window = np.hanning(CHUNK).astype(np.float32)
        self.lock = threading.Lock()
        self.chunks = []          # (arrival t, rms, power)
        self.voice = 0.0          # last time someone (or she) was talking
        self.lag = 0.3            # s from a sound to its chunk arriving (measure_lag)
        node.create_subscription(Int16MultiArray, 'sound/audio', self.on_audio, qos_profile_sensor_data)
        for topic in ('speech/active', 'sound/speaking'):
            node.create_subscription(Bool, topic, self.on_voice, 10)

    def on_audio(self, m):
        if len(m.data) < CHUNK:
            return
        x = np.asarray(m.data[:CHUNK], dtype=np.float32) * (1.0 / 32768.0)
        p = np.abs(np.fft.rfft(x * self.window)) ** 2
        with self.lock:
            self.chunks.append((time.monotonic(), float(np.sqrt(np.mean(x * x))), p))
            del self.chunks[:-300]

    def on_voice(self, m):
        if m.data:
            self.voice = time.monotonic()

    def measure_lag(self, snap):
        """snap() makes a loud servo move. Lag = its first loud chunk's arrival - the command -
        one chunk (a chunk is sent when it is full). Median of three."""
        lags = []
        for _ in range(3):
            time.sleep(1.5)
            t0 = time.monotonic()
            snap()
            time.sleep(2.0)
            with self.lock:
                before = [r for t, r, _ in self.chunks if t0 - 1.0 <= t < t0]
                after = [(t, r) for t, r, _ in self.chunks if t >= t0]
            if len(before) < 5 or not after:
                continue
            floor = float(np.median(before))
            loud = [t for t, r in after if r > 4.0 * floor]
            if loud:
                lags.append(loud[0] - t0 - CHUNK / RATE_HZ)
        if lags:
            self.lag = max(0.0, float(np.median(lags)))
        return lags

    def listen(self, seconds):
        """dB per band (median of half seconds) over the next seconds of SOUND (arrivals shifted
        by the lag), plus the mean spectrum; (None, None) if someone talked."""
        t0 = time.monotonic() + self.lag + CHUNK / RATE_HZ
        time.sleep(seconds + self.lag + 0.25)
        with self.lock:
            got = [p for t, _, p in self.chunks if t0 <= t <= t0 + seconds]
        if time.monotonic() - self.voice < seconds + self.lag + 1.0 or len(got) < 5:
            return None, None
        halves = [np.mean(got[i:i + 5], axis=0) for i in range(0, len(got) - 4, 5)]
        db = {k: float(np.median([prominence(h, m) for h in halves])) for k, m in self.masks.items()}
        return db, np.mean(got, axis=0)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('mode', choices=['routines', 'find'])
    ap.add_argument('--wheels-up', action='store_true', help='confirm: wheels up or the ESC/motor unplugged')
    ap.add_argument('--rounds', type=int, default=5)
    ap.add_argument('--cut', type=float, default=3.0, help='find: seconds each output is cut')
    ap.add_argument('--only', default='', help='routines: comma list of candidates (wait always runs)')
    a = ap.parse_args()
    if not a.wheels_up:
        print('This drives the outputs directly, past every safety layer. Wheels up (or the ESC / motor\n'
              'unplugged), tyres on something grippy so stops are reproduced, then add --wheels-up.')
        return 2

    rclpy.init()
    node = rclpy.create_node('buzz_source')
    ears = Ears(node)
    pulse = {n: node.create_publisher(Float64, f'/pca9685/{n}/pulse_width', 10) for n in PCA + HEAD}
    angle = {n: node.create_publisher(Float64, f'/pca9685/{n}/angle', 10) for n in STEER + HEAD}
    head_last = {}
    for n in HEAD:
        node.create_subscription(Float64, f'/pca9685/{n}/angle', lambda m, j=n: head_last.__setitem__(j, m.data), 10)
    arm = node.create_client(Trigger, '/pca9685/throttle/arm')
    params = node.create_client(GetParameters, '/pca9685/get_parameters')
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    home = {'steering': 81.0, 'rear_steering': 87.0}      # capture.py's, if pca9685 does not say
    if not params.wait_for_service(timeout_sec=3.0):
        print('pca9685 not answering: is the robot stack up?')
        return 1
    f = params.call_async(GetParameters.Request(names=[f'{n}.home' for n in STEER]))
    t = time.monotonic()
    while not f.done() and time.monotonic() - t < 3.0:
        time.sleep(0.05)
    if f.done() and f.result():
        for n, v in zip(STEER, f.result().values):
            if v.type in (2, 3):
                home[n] = float(v.double_value if v.type == 3 else v.integer_value)
    print(f'steering home: front {home["steering"]:g}, rear {home["rear_steering"]:g}')

    time.sleep(2.0)
    if not ears.chunks:
        print('no sound/audio: load the voice first (voice on, or the page LOAD)')
        return 1

    def steer(d, which=STEER):
        for n in which:
            angle[n].publish(Float64(data=home[n] + (d if n == 'steering' else -d)))

    def hold(d, s):
        end = time.monotonic() + s
        while time.monotonic() < end:
            steer(d)
            time.sleep(0.1)

    def cut(names):
        for n in names:
            pulse[n].publish(Float64(data=0.0))

    def restore(n):
        if n in STEER:
            angle[n].publish(Float64(data=home[n]))
        elif n == 'throttle':
            if arm.wait_for_service(timeout_sec=1.0):
                arm.call_async(Trigger.Request())
            time.sleep(2.2)                                   # the arming neutral
        elif n in head_last:
            angle[n].publish(Float64(data=head_last[n]))

    def snap():
        hold(PROVOKE_DEG, 0.3)
        steer(0.0)

    lags = ears.measure_lag(snap)
    print(f'mic lag: {ears.lag:.2f} s (snaps: {", ".join(f"{x:.2f}" for x in lags) or "not heard, using 0.3"})')

    spectra = {}
    try:
        if a.mode == 'find':
            outputs = PCA + HEAD
            rec = {n: {k: ([], []) for k in BANDS} for n in outputs + ['all']}
            for r in range(a.rounds):
                hold(PROVOKE_DEG if r % 2 == 0 else -PROVOKE_DEG, 1.0)
                steer(0.0)
                time.sleep(1.5)
                order = outputs[:]
                random.shuffle(order)
                for n in order + ['all']:
                    names = outputs if n == 'all' else [n]
                    base, _ = ears.listen(2.0)
                    cut(names)
                    time.sleep(0.5)
                    during, spec = ears.listen(a.cut - 0.5)
                    for m in names:
                        restore(m)
                    time.sleep(1.5)                          # the servo moving back is loud
                    for k in BANDS:
                        rec[n][k][0].append(base[k] if base else None)
                        rec[n][k][1].append(during[k] if during else None)
                    if spec is not None:
                        spectra[f'{n}_{r}'] = spec
                    b = ' '.join(f'{k} {base[k]:4.1f}' for k in BANDS) if base else 'talking'
                    d = ' '.join(f'{k} {during[k]:4.1f}' for k in BANDS) if during else 'talking'
                    print(f'round {r + 1} {n:>13}: on [{b}]  cut [{d}] dB', flush=True)
            print('\nverdicts (room is 3-6 dB):')
            for n in outputs + ['all']:
                print(f'  {n:>13}: ' + '; '.join(f'{k} {verdict(*rec[n][k])}' for k in BANDS))
        else:
            def release(names, s):
                cut(names)
                time.sleep(s)
                for n in names:
                    restore(n)

            def from_side(side):                             # past centre the other way, then back
                hold(-side * 3.0, 0.3)
                steer(0.0)

            def nudge(side):
                for d in (2.0, -2.0):
                    hold(d, 0.15)
                steer(0.0)

            routines = {
                'wait': lambda side: None,
                'release1': lambda side: release(STEER, 1.0),
                'release3': lambda side: release(STEER, 3.0),
                'front_off2': lambda side: release(['steering'], 2.0),
                'rear_off2': lambda side: release(['rear_steering'], 2.0),
                'from_side3': from_side,
                'nudge2': nudge,
                'offset1': lambda side: steer(side * 1.0),   # stay 1 deg toward the turn
            }
            if a.only:
                routines = {k: v for k, v in routines.items() if k in a.only.split(',') or k == 'wait'}
            rec = {k: [] for k in routines}
            trial = 0
            for r in range(a.rounds):
                order = list(routines)
                random.shuffle(order)
                for name in order:
                    trial += 1
                    side = 1.0 if trial % 2 else -1.0
                    hold(side * PROVOKE_DEG, 1.0)            # a turn ...
                    steer(0.0)                               # ... and the stop
                    time.sleep(1.0)
                    before, _ = ears.listen(1.5)
                    routines[name](side)
                    time.sleep(0.5)
                    after, spec = ears.listen(3.0)
                    k = '1.6k'
                    rec[name].append((before[k] if before else None, after[k] if after else None))
                    if spec is not None:
                        spectra[f'{name}_{r}'] = spec
                    nan = float('nan')
                    print(f'trial {trial:2d} {name:>11}: before {before[k] if before else nan:5.1f}'
                          f'  after {after[k] if after else nan:5.1f} dB'
                          + ('' if before and after else '  (talking: not counted)'), flush=True)
                    steer(0.0)
            print(f'\n1.6 kHz; buzzing = {BUZZ_DB:g} dB or more. A routine counts only if it beats wait.')
            print(f'  {"routine":>11}  buzzed->quiet  quiet after  median after')
            for name in routines:
                sc = score(rec[name])
                if sc is None:
                    print(f'  {name:>11}  no data')
                    continue
                nb, nq, aq, n, med = sc
                print(f'  {name:>11}  {nq:>5}/{nb:<7}  {aq:>5}/{n:<5}  {med:6.1f} dB')
    finally:
        steer(0.0)
        for n in HEAD:
            if n in head_last:
                angle[n].publish(Float64(data=head_last[n]))
        time.sleep(0.3)

    d = os.path.expanduser('~/calibration/buzz')
    os.makedirs(d, exist_ok=True)
    path = os.path.join(d, time.strftime(f'{a.mode}-%Y%m%d-%H%M%S.npz'))
    np.savez(path, freqs=np.fft.rfftfreq(CHUNK, 1.0 / RATE_HZ), lag=ears.lag, **spectra)
    print('spectra:', path)
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
