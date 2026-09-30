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

"""battery_monitor's power-off: 3.3 V a cell for 60 s shuts her down, a recovery cancels it."""

from types import SimpleNamespace

from jetnano_bringup import battery_monitor
from jetnano_bringup.battery_monitor import BatteryMonitor


class Log:

    def __init__(self):
        self.lines = []

    def error(self, text):
        self.lines.append(('error', text))

    def warning(self, text):
        self.lines.append(('warning', text))


class Pub:

    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg.data)


def monitor(on=True):
    log = Log()
    return SimpleNamespace(
        poweroff_on=on, poweroff_v=9.9, poweroff_after=60.0, cells=3,
        poweroff_cmd='sudo -n systemctl poweroff', _powering_off=False, _low_since=None,
        stop_pub=Pub(), get_logger=lambda: log, log=log)


def run(m, readings, monkeypatch):
    """Feed (seconds, volts) readings; returns the commands it tried to run."""
    ran = []
    clock = [0.0]
    monkeypatch.setattr(battery_monitor.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(battery_monitor.subprocess, 'Popen', lambda cmd: ran.append(cmd))
    for t, v in readings:
        clock[0] = t
        BatteryMonitor._poweroff_check(m, v)
    return ran


def readings(volts, dt=0.5):
    return [(i * dt, v) for i, v in enumerate(volts)]


def test_level_needs_a_full_minute():
    assert battery_monitor.judge_level(readings([10.0] * 20), 10.5) == 'ok'


def test_level_low_when_never_above_for_a_minute():
    """Steve: "low for over a minute straight with no sag, then it's low"."""
    assert battery_monitor.judge_level(readings([10.4, 10.3, 10.2] * 41), 10.5) == 'low'


def test_level_soon_when_sagging_under_but_recovering():
    """"low for over a minute with sag, then it's gonna be low pretty soon": under on average,
    over the line between sags."""
    v = ([10.7, 10.6] + [10.0] * 6) * 16          # peaks 10.7, average ~10.2
    assert battery_monitor.judge_level(readings(v), 10.5) == 'soon'


def test_level_ok_when_only_dipping():
    v = ([10.9] * 10 + [10.2]) * 12                # one dip in eleven: average well over
    assert battery_monitor.judge_level(readings(v), 10.5) == 'ok'


def test_low_for_the_whole_minute_powers_off(monkeypatch):
    m = monitor()
    ran = run(m, [(0, 9.8), (30, 9.7), (59.9, 9.7), (60.0, 9.6), (61, 9.6), (90, 9.5)], monkeypatch)
    assert ran == [['sudo', '-n', 'systemctl', 'poweroff']]       # once, not every reading after
    assert m.stop_pub.sent == [True]                                # the robot is stopped first
    assert m._powering_off


def test_a_dip_that_recovers_does_not(monkeypatch):
    m = monitor()
    # under 9.9 V for 40 s (a hill), then back over 9.9 + 0.05 x 3 = 10.05 V: the minute restarts
    ran = run(m, [(0, 9.8), (40, 9.8), (41, 10.2), (70, 9.8), (129, 9.8)], monkeypatch)
    assert ran == []
    assert m._low_since == 70
    assert any('not powering off' in text for _, text in m.log.lines)
    assert run(m, [(130, 9.8)], monkeypatch) == [['sudo', '-n', 'systemctl', 'poweroff']]


def test_hovering_just_over_the_line_keeps_counting(monkeypatch):
    m = monitor()
    # 10.0 V is over 9.9 but inside the 0.05 V/cell margin: not a recovery
    ran = run(m, [(0, 9.85), (20, 10.0), (40, 9.9), (60, 9.9)], monkeypatch)
    assert ran == [['sudo', '-n', 'systemctl', 'poweroff']]


def test_off_means_off(monkeypatch):
    m = monitor(on=False)
    assert run(m, [(0, 9.0), (100, 9.0)], monkeypatch) == []
    assert m.stop_pub.sent == []
