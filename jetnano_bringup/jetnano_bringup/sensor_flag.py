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

"""The sensors-asleep flag: sensor_sleep writes it, the watchdogs read it.

~/sensors/asleep exists while the lidar's motor is stopped (asleep, and while it spins up
again). JSON: {"state": "asleep"|"waking", "since": wall s, "switching_until": wall s}.
Switching the camera's projector stops /vo: on 2026-10-11 it came back by itself 2 times
in 4 when the projector went off (once after ~4 s) and every time it came back on. So
while asleep the camera's streams are not judged at all (she does not move; the safety
monitor holds the EKF still), and after waking only until switching_until.
sensor_sleep rewrites it every 10 s; one older than 60 s belongs to a writer that died
and is ignored, so a crash can never blind the watchdog.
"""

import json
import os
import time

FLAG = os.path.expanduser('~/sensors/asleep')
STALE_S = 60.0
LIDAR_STREAMS = ('lidar', 'scan_filter', 'lidar_odom')      # watchdog TOPICS keys
CAMERA_STREAMS = ('vo', 'nvblox', 'obstacles')


def read(path=FLAG, now=None):
    """-> the flag's dict, or None when there is no fresh flag."""
    now = time.time() if now is None else now
    try:
        if now - os.stat(path).st_mtime > STALE_S:
            return None
        with open(path) as f:
            d = json.loads(f.read() or '{}')
    except (OSError, ValueError):
        return None
    return d if isinstance(d, dict) else {}


def not_judged(path=FLAG, now=None):
    """-> the watchdog streams that are quiet on purpose right now."""
    now = time.time() if now is None else now
    d = read(path, now)
    if d is None:
        return ()
    skip = LIDAR_STREAMS
    if d.get('state') == 'asleep' or now < float(d.get('switching_until', 0) or 0):
        skip += CAMERA_STREAMS
    return skip


def write(state, switching_s=0.0, path=FLAG, now=None):
    now = time.time() if now is None else now
    old = read(path, now) or {}
    d = {'state': state, 'since': old.get('since', now) if old.get('state') == state else now,
         'switching_until': max(float(old.get('switching_until', 0) or 0), now + switching_s)}
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        f.write(json.dumps(d))
    os.replace(tmp, path)


def clear(path=FLAG):
    try:
        os.remove(path)
    except OSError:
        pass
