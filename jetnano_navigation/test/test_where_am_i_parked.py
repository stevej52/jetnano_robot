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

"""where_am_i, 2026-11-01: where she parked is written once she rests and searched first
at boot; a tap on the dashboard's map is searched near the tap; a hand placement must be
on open floor. The node's methods on a stand-in (no ROS)."""

from collections import deque
import json
import math
import time
from types import SimpleNamespace

from jetnano_navigation.global_locate import MapModel
from jetnano_navigation import where_am_i
from jetnano_navigation.where_am_i import WhereAmI
import pytest

from test_global_locate import cast, RES, room


class Log:
    def __init__(self):
        self.lines = []

    def info(self, m, **kw):
        self.lines.append(m)

    warning = error = info


def stand_in(**kw):
    log = Log()
    me = SimpleNamespace(accept=0.55, margin=1.15, home=(0.0, 0.0), home_radius=1.5,
                         home_accept=0.50, home_margin=1.15, remember=True, last_radius=1.0,
                         hint_radius=0.75, hint_tol=math.radians(45), rival_ratio=1.02, saved=None,
                         model_mtime=100.0, mapping_unit='jetnano-slam', log=log,
                         get_logger=lambda: log)
    me.__dict__.update(kw)
    me._near = lambda *a: WhereAmI._near(me, *a)
    return me


@pytest.fixture(scope='module')
def l_room():
    occ, free = room('L')
    return occ, MapModel(occ, free, RES, 0.0, 0.0)


@pytest.mark.parametrize('drift, still', [(0.0, True), (0.05, False)])
def test_still_reads_the_stamped_odometry(drift, still):
    # the odometry as _on_odom keeps it: (monotonic, x, y, yaw, stamp), 8 Hz for 2.5 s
    now = time.monotonic()
    recent = deque((now - k / 8, drift * k / 20, 0.0, 0.0, 1000.0 - k / 8) for k in range(20, -1, -1))
    assert WhereAmI._still(SimpleNamespace(recent=recent, still_s=2.0)) is still


def test_rest_is_written_once(tmp_path):
    f = tmp_path / 'last.json'
    me = stand_in(fix=((1.0, 2.0, math.pi / 2), (0.0, 0.0, 0.0)),
                  odom=(0.0, 0.5, 0.0, 0.0, 0.0),           # 0.5 m forward since the fix
                  last_file=str(f), _still=lambda: True, _unit_active=lambda unit: False)
    WhereAmI._save_parked(me)
    d = json.loads(f.read_text())
    assert (d['x'], d['y'], d['yaw']) == pytest.approx((1.0, 2.5, math.pi / 2), abs=1e-3)
    assert d['map_mtime'] == 100.0
    f.write_text('{}')                                      # not written again: same place
    WhereAmI._save_parked(me)
    assert f.read_text() == '{}'
    assert WhereAmI._read_last(SimpleNamespace(last_file=str(f))) is None


def test_not_written_while_moving_or_mapping(tmp_path):
    f = tmp_path / 'last.json'
    base = dict(fix=((1.0, 2.0, 0.0), (0.0, 0.0, 0.0)), odom=(0.0, 0.0, 0.0, 0.0, 0.0),
                last_file=str(f))
    WhereAmI._save_parked(stand_in(**base, _still=lambda: False, _unit_active=lambda u: False))
    assert not f.exists()
    me = stand_in(**base, _still=lambda: True, _unit_active=lambda u: True)
    WhereAmI._save_parked(me)
    assert not f.exists() and me.fix is None                # a new map's frame: forgotten


def test_boot_looks_where_she_parked_first(l_room):
    occ, model = l_room
    x, y, yaw = 2.37, 1.81, math.radians(37)
    pts = cast(occ, x, y, yaw)
    me = stand_in(model=model, saved={'x': 2.0, 'y': 2.2, 'yaw': 0.0, 'map_mtime': 100.0})
    verdict, why, cands, accept, _ = WhereAmI._search(me, 'boot', pts, None)
    assert verdict == 'placed' and why.startswith('where she parked') and accept == 0.50
    assert math.hypot(cands[0][1] - x, cands[0][2] - y) < 0.06
    # parked on an older map: the whole map, at the full bar
    me = stand_in(model=model, saved={'x': 2.0, 'y': 2.2, 'yaw': 0.0, 'map_mtime': 99.0})
    verdict, why, cands, accept, _ = WhereAmI._search(me, 'boot', pts, None)
    assert verdict == 'placed' and not why.startswith('where she parked') and accept == 0.55
    assert any('older map' in m for m in me.log.lines)


def test_boot_falls_back_when_carried_away(l_room):
    occ, model = l_room
    x, y, yaw = 2.37, 1.81, math.radians(37)
    me = stand_in(model=model, saved={'x': 6.5, 'y': 1.0, 'yaw': 0.0, 'map_mtime': 100.0})
    verdict, why, cands, _, _ = WhereAmI._search(me, 'boot', cast(occ, x, y, yaw), None)
    assert verdict == 'placed' and not why.startswith('where she parked')
    assert math.hypot(cands[0][1] - x, cands[0][2] - y) < 0.06
    assert any('not where she parked' in m for m in me.log.lines)


def test_a_tap_with_a_heading():
    occ, free = room('box')                         # 0 and 180 degrees alike in the middle
    model = MapModel(occ, free, RES, 0.0, 0.0)
    pts = cast(occ, 4.0, 3.0, math.radians(10))
    me = stand_in(model=model)
    assert WhereAmI._search(me, 'hint', pts, (4.1, 2.9, None))[0] == 'unsure'
    verdict, why, cands, _, _ = WhereAmI._search(me, 'hint', pts, (4.1, 2.9, math.radians(-20)))
    assert verdict == 'placed' and 'tapped' in why
    assert abs(math.degrees(math.remainder(cands[0][3] - math.radians(10), 2 * math.pi))) < 3.0


def test_a_tap_settles_a_look_alike_elsewhere():
    occ, free = room('box')                         # (2, 2) facing 0 looks like (6, 4) facing 180
    model = MapModel(occ, free, RES, 0.0, 0.0)
    pts = cast(occ, 2.0, 2.0, 0.0)
    me = stand_in(model=model)
    assert WhereAmI._search(me, 'locate', pts, None)[0] == 'unsure'         # the whole map cannot tell
    verdict, why, cands, _, _ = WhereAmI._search(me, 'hint', pts, (2.2, 1.9, None))
    assert verdict == 'placed', why
    assert math.hypot(cands[0][1] - 2.0, cands[0][2] - 2.0) < 0.06


@pytest.mark.parametrize('rival, verdict', [(0.90, 'unsure'), (0.62, 'unsure'), (0.61, 'placed')])
def test_a_clearly_better_fit_elsewhere_wins(monkeypatch, rival, verdict):
    # round the spot 0.60, and another place, outside the circle, fits better (or as well)
    monkeypatch.setattr(where_am_i, 'locate', lambda *a, **kw: [(0.60, 0.1, 0.1, 0.0), (0.40, 0.4, -0.3, 2.0)])
    me = stand_in(model=None)
    got, why, cands = WhereAmI._near(me, None, (0.0, 0.0), 0.75, None, lambda: [(rival, 5.0, 5.0, 0.0)])
    assert got == verdict, why
    if verdict == 'unsure':
        assert 'elsewhere' in why and cands[1][1:3] == (5.0, 5.0)


def test_by_hand_only_on_open_floor(l_room):
    _, model = l_room
    me = SimpleNamespace(model=model)
    assert WhereAmI._free_floor(me, 2.0, 2.0)
    assert not WhereAmI._free_floor(me, 0.01, 2.0)          # the wall
    assert not WhereAmI._free_floor(me, 6.5, 4.5)           # the L's missing corner
    assert not WhereAmI._free_floor(me, -1.0, 2.0)          # off the map


def test_main_spins_on_after_the_subscription_race(monkeypatch):
    # _fresh_scan's subscription dropped by a search thread while the executor built its wait set
    from rclpy._rclpy_pybind11 import InvalidHandle
    spins = []

    class Executor:
        def add_node(self, node):
            pass

        def spin(self):
            spins.append(1)
            if len(spins) == 1:
                raise InvalidHandle('Destroyable is invalid')

    def race(node):
        raise InvalidHandle('Destroyable is invalid')
    log = Log()
    monkeypatch.setattr(where_am_i, 'WhereAmI', lambda: SimpleNamespace(get_logger=lambda: log,
                                                                       destroy_node=lambda: None))
    monkeypatch.setattr(where_am_i, 'SingleThreadedExecutor', Executor, raising=False)
    for name, fn in (('init', lambda args=None: None), ('ok', lambda: True), ('try_shutdown', lambda: None),
                     ('spin', race)):
        monkeypatch.setattr(where_am_i.rclpy, name, fn)
    where_am_i.main()
    assert len(spins) == 2 and any('spinning on' in m for m in log.lines)
