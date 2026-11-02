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

"""guard_flow, 2026-11-01 review: its container restart (27 s into a hold) waits while the camera
launch is still coming up after a restart, as the watchdog's has since 2026-09-27. A real
process stands in for the wrapper (cuvslam_vo.sh); docker is never called."""

import os
import subprocess
import sys
import time
from types import SimpleNamespace

from jetnano_bringup import guard_flow, launch_age
from jetnano_bringup.guard_flow import GuardFlow
from jetnano_bringup.watchdog import Watchdog
import pytest


class Log:
    def __init__(self):
        self.lines = []

    def info(self, m, **kw):
        self.lines.append(m)

    warning = error = info


def stand_in(young_s):
    log = Log()
    return SimpleNamespace(fix_step=2, fix_at={'grid_to_points': 0.0, 'container': time.monotonic()},
                           log=log, get_logger=lambda: log, _pids=getattr(GuardFlow, '_pids', None),
                           get_parameter=lambda name: SimpleNamespace(value={'launch_young_s': young_s}[name]))


@pytest.fixture
def docker(monkeypatch):
    calls, real = [], subprocess.run

    def run(cmd, *a, **kw):
        if cmd[0] == 'docker':
            calls.append(cmd)
            return SimpleNamespace(returncode=0)
        return real(cmd, *a, **kw)
    monkeypatch.setattr(guard_flow.subprocess, 'run', run)
    return calls


@pytest.fixture
def wrapper(monkeypatch):
    tag = f'guard_flow_test_wrapper_{os.getpid()}'
    p = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)', tag])
    monkeypatch.setattr(guard_flow, 'CAMERA_WRAPPER', tag, raising=False)
    time.sleep(0.3)
    yield p
    p.kill()
    p.wait()


def test_a_launch_just_respawned_is_left_to_come_up(docker, wrapper):
    me = stand_in(40.0)
    GuardFlow._restart_container_launch(me)
    assert docker == []
    assert me.fix_step == 1 and time.monotonic() - me.fix_at['container'] > 100.0     # again in 10 s
    assert any('s old' in m and 'not restarting' in m for m in me.log.lines)


def test_an_old_launch_is_restarted(docker, wrapper):
    me = stand_in(0.0)
    GuardFlow._restart_container_launch(me)
    assert docker and docker[0][:5] == ['docker', 'exec', 'isaac_vo', 'pkill', '-INT']


def test_between_respawns_it_waits(docker, monkeypatch):
    monkeypatch.setattr(guard_flow, 'CAMERA_WRAPPER', f'no_such_wrapper_{os.getpid()}_x', raising=False)
    me = stand_in(40.0)
    GuardFlow._restart_container_launch(me)
    assert docker == [] and any('between respawns' in m for m in me.log.lines)


def test_launch_age():
    assert launch_age.young([], 40.0)                       # between exit and respawn
    assert launch_age.started_ago([os.getpid()]) > 0.0
    assert launch_age.young([os.getpid()], 1e9)
    assert not launch_age.young([os.getpid()], 0.0)
    assert not launch_age.young([2 ** 22 + 12345], 40.0)    # no such process: unreadable, not young


def test_the_watchdog_asks_the_same(monkeypatch):
    me = SimpleNamespace(young_s=40.0, _pids=lambda pattern: [])
    assert Watchdog._launch_young(me)
    me._pids = lambda pattern: [os.getpid()]
    me.young_s = 0.0
    assert not Watchdog._launch_young(me)
