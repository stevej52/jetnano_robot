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

"""The uh-oh and the beeps: what they sound like, and which watchdog events play them."""

import json

import numpy as np
import pytest

from jetnano_bringup import voice


def test_beeps_are_the_end_of_on():
    """The same beep-beep-beep that ends the power-on sound (take 1, no jitter)."""
    b = voice.beeps(0.0)
    assert np.allclose(voice.on(0.0)[-len(b):], b)


def segments(y, floor=0.2):
    """(start, end) sample ranges where the 5 ms loudness is above `floor` of its peak."""
    w = int(0.005 * voice.RATE)
    e = np.sqrt(np.convolve(y * y, np.ones(w) / w, 'same'))
    loud = np.concatenate([[0], (e > floor * e.max()).astype(int), [0]])
    d = np.diff(loud)
    return list(zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1)))


def pitch(y):
    """Rough frequency from zero crossings."""
    return np.count_nonzero(np.diff(np.signbit(y))) / 2 / (len(y) / voice.RATE)


def test_beeps_are_three_and_high():
    b = voice.beeps(1.5)                     # jitter is ignored: always the same beeps
    assert np.allclose(b, voice.beeps(0.0))
    parts = segments(b)
    assert len(parts) == 3
    assert all(pitch(b[s:e]) > 1200 for s, e in parts)
    assert len(b) / voice.RATE < 0.4


def test_uhoh_two_parts_second_lower_and_longer():
    y = voice.uhoh(0.0)
    parts = segments(y, floor=0.1)
    assert len(parts) == 2                   # uh, a catch, oh
    (s1, e1), (s2, e2) = parts
    assert e2 - s2 > 2 * (e1 - s1)
    assert pitch(y[s2:e2]) < 0.9 * pitch(y[s1:e1])
    assert 0.5 < len(y) / voice.RATE < 0.8


def test_new_moods_are_made_and_have_english():
    for mood in ('uhoh', 'beeps'):
        assert mood in voice.MOODS and mood in voice.ENGLISH
        assert np.isfinite(voice.MOODS[mood](0.0)).all()


@pytest.mark.parametrize('kind, mood', [('down', 'uhoh'), ('slow', 'uhoh'), ('back', 'beeps'),
                                        ('act', None), ('still_down', None), ('gave_up', None),
                                        ('start', None), ('system', None)])
def test_watchdog_events_to_moods(kind, mood):
    sounds = pytest.importorskip('jetnano_bringup.sounds')          # needs ROS
    msg = json.dumps({'t': '2026-09-29 23:10:00', 'kind': kind, 'what': 'visual odometry', 'detail': 'x'})
    assert sounds.watchdog_mood(msg) == mood


def test_no_uhoh_on_the_stuck_speaker_itself():
    sounds = pytest.importorskip('jetnano_bringup.sounds')
    msg = json.dumps({'kind': 'down', 'what': sounds.SOUND_DEVICE, 'detail': 'stuck'})
    assert sounds.watchdog_mood(msg) is None


def test_stuck_sound_pids(tmp_path):
    watchdog = pytest.importorskip('jetnano_bringup.watchdog')
    for pid, comm, state in ((10, 'arecord', 'D'), (11, 'arecord', 'S'), (12, 'aplay', 'D'),
                             (13, 'python3', 'D'), (14, 'a) b', 'D')):
        (tmp_path / str(pid)).mkdir()
        (tmp_path / str(pid) / 'stat').write_text(f'{pid} ({comm}) {state} 1 2 3 4\n')
    (tmp_path / 'self').mkdir()
    assert watchdog.stuck_sound_pids(str(tmp_path)) == {10, 12}


def test_watchdog_garbage_is_ignored():
    sounds = pytest.importorskip('jetnano_bringup.sounds')
    assert sounds.watchdog_mood('not json') is None
    assert sounds.watchdog_mood('[1, 2]') is None
