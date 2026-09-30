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

"""Places that are not on the house map, known by what the lidar sees from them: the bench.

Steve, 2026-09-30: "Can you take some measurements on the bench to localize when she's
there?" - so that where_am_i says "on the bench" instead of "not on the map (best fit
0.63)", and nothing tries to drive her off it.

A remembered place is one lidar scan (its ranges) saved under a name in a JSON file
(~/maps/places.json). A live scan is compared beam for beam, over a range of rotations,
and the score is the share of beams that agree (within 15 cm or 8 %, whichever is larger,
because she is never put down in exactly the same spot). No ROS here, so it can be tested.
"""

import json
import os
import time

import numpy as np

MAX_RANGE_M = 10.0


def place_match(ranges, ref, max_shift=90, tol_m=0.15, tol_frac=0.08):
    """How well a scan matches a remembered one, 0..1, at the best rotation within
    +-max_shift beams (90 beams = 45 deg at the A1M8's 0.5 deg)."""
    r = np.asarray(ranges, dtype=np.float32)
    q = np.asarray(ref, dtype=np.float32)
    n = min(len(r), len(q))
    if n == 0:
        return 0.0
    r, q = r[:n], q[:n]
    ok_q = np.isfinite(q) & (q > 0.05) & (q < MAX_RANGE_M)
    ok_r = np.isfinite(r) & (r > 0.05) & (r < MAX_RANGE_M)
    tol = np.maximum(tol_m, tol_frac * np.where(ok_q, q, 0.0))
    best = 0.0
    for s in range(-max_shift, max_shift + 1):
        rr, okr = np.roll(r, s), np.roll(ok_r, s)
        both = okr & ok_q
        seen = int(both.sum())
        if seen < 0.3 * n:
            continue
        agree = int((both & (np.abs(rr - q) <= tol)).sum())
        best = max(best, agree / seen)
    return float(best)


def load_places(path):
    """{name: {'ranges': [...], 'saved': text, ...}} or {} if there is no file."""
    try:
        with open(os.path.expanduser(path)) as f:
            data = json.load(f)
    except (OSError, ValueError):
        return {}
    return data if isinstance(data, dict) else {}


def save_place(path, name, ranges, note=''):
    """Remember `ranges` under `name`; other places in the file are kept."""
    path = os.path.expanduser(path)
    places = load_places(path)
    places[name] = {'ranges': [None if not np.isfinite(v) else round(float(v), 3) for v in ranges],
                    'saved': time.strftime('%Y-%m-%d %H:%M:%S'), 'note': note}
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    with open(path + '.new', 'w') as f:
        json.dump(places, f)
    os.replace(path + '.new', path)
    return places


def best_place(ranges, places):
    """(name, score) of the remembered place that fits best, or (None, 0.0)."""
    best = (None, 0.0)
    for name, rec in places.items():
        ref = [np.nan if v is None else v for v in rec.get('ranges', [])]
        score = place_match(ranges, ref)
        if score > best[1]:
            best = (name, score)
    return best
