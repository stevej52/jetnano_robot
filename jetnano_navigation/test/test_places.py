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

"""Recognising a remembered place (the bench) from one lidar scan."""

import math

import numpy as np

from jetnano_navigation import places


def room_scan(n=720, w=3.0, d=2.0, seed=0, noise=0.01):
    """A lidar in a w x d room, off-centre, with a bit of noise."""
    rng = np.random.default_rng(seed)
    a = np.linspace(-math.pi, math.pi, n, endpoint=False)
    x0, y0 = 0.4, -0.3
    r = np.full(n, np.inf)
    for ang_i, ang in enumerate(a):
        c, s = math.cos(ang), math.sin(ang)
        cands = []
        if c > 1e-9:
            cands.append((w / 2 - x0) / c)
        if c < -1e-9:
            cands.append((-w / 2 - x0) / c)
        if s > 1e-9:
            cands.append((d / 2 - y0) / s)
        if s < -1e-9:
            cands.append((-d / 2 - y0) / s)
        r[ang_i] = min(v for v in cands if v > 0)
    r += rng.normal(0, noise, n)
    r[rng.random(n) < 0.03] = np.nan            # dropped beams
    return r


def test_same_place_scores_high():
    ref = room_scan(seed=1)
    again = room_scan(seed=2)
    assert places.place_match(again, ref) > 0.9


def test_turned_a_little_still_matches():
    ref = room_scan(seed=1)
    turned = np.roll(room_scan(seed=3), 40)      # 20 deg
    assert places.place_match(turned, ref) > 0.85


def test_another_room_does_not():
    ref = room_scan(seed=1)
    other = room_scan(w=5.0, d=4.0, seed=4)
    assert places.place_match(other, ref) < 0.4


def test_save_load_and_best(tmp_path):
    path = str(tmp_path / 'places.json')
    ref = room_scan(seed=1)
    ref[5] = np.nan
    places.save_place(path, 'bench', ref, 'the bench')
    places.save_place(path, 'hall', room_scan(w=6.0, d=1.5, seed=9))
    loaded = places.load_places(path)
    assert set(loaded) == {'bench', 'hall'}
    assert loaded['bench']['ranges'][5] is None
    name, score = places.best_place(room_scan(seed=2), loaded)
    assert name == 'bench' and score > 0.9
    assert places.best_place(room_scan(w=5.0, d=4.0, seed=4), loaded)[1] < 0.5
    assert places.load_places(str(tmp_path / 'missing.json')) == {}
