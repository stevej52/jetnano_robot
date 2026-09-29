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

"""Voice pitch (who gets the fun personality) and Smart Turn's features, on made-up sound."""

import os

from jetnano_bringup.speaker import pitch_hz
from jetnano_bringup.turn import DEFAULT_MODEL, features, last_seconds, RATE
import numpy as np
import pytest


def voice(f0: float, seconds: float = 2.0, rate: int = RATE, noise: float = 0.02) -> np.ndarray:
    """A buzzy vowel: f0 and its harmonics, falling off like a voice, with a little hiss."""
    t = np.arange(int(seconds * rate)) / rate
    x = sum(np.sin(2 * np.pi * f0 * k * t) / k for k in range(1, 12) if f0 * k < rate / 2)
    rng = np.random.default_rng(0)
    return (0.3 * x / np.abs(x).max() + noise * rng.standard_normal(len(t))).astype(np.float32)


@pytest.mark.parametrize('f0', [95.0, 120.0, 150.0, 180.0, 220.0, 250.0])
def test_pitch_hz_finds_the_voice(f0):
    got = pitch_hz(voice(f0))
    assert got is not None
    assert abs(got - f0) / f0 < 0.03


def test_pitch_sides_of_the_fun_line():
    # listen's fun_pitch_hz is 165: a man's voice sits under it, a woman's over it
    assert pitch_hz(voice(120.0)) < 165.0 < pitch_hz(voice(210.0))


def test_pitch_hz_none_without_a_voice():
    rng = np.random.default_rng(1)
    assert pitch_hz(0.1 * rng.standard_normal(RATE * 2)) is None       # hiss
    assert pitch_hz(np.zeros(RATE * 2)) is None                           # silence
    assert pitch_hz(voice(150.0, seconds=0.1)) is None                    # too short


def test_last_seconds_pads_and_trims():
    short = last_seconds(np.ones(RATE))
    assert short.shape == (8 * RATE,)
    assert short[0] == 0.0 and short[-1] == 1.0
    long = last_seconds(np.arange(10 * RATE, dtype=np.float32))
    assert long.shape == (8 * RATE,)
    assert long[-1] == 10 * RATE - 1


def test_features_shape_and_range():
    f = features(voice(140.0, seconds=3.0))
    assert f.shape == (1, 80, 800)
    assert f.dtype == np.float32
    assert np.isfinite(f).all()
    # Whisper's normalisation: log10 clamped to 8 below the peak, then (x + 4) / 4
    assert f.max() - f.min() <= 2.0 + 1e-5


@pytest.mark.skipif(not os.path.exists(DEFAULT_MODEL), reason='no Smart Turn model here')
def test_smart_turn_gives_a_probability():
    pytest.importorskip('onnxruntime')
    from jetnano_bringup.turn import SmartTurn
    p = SmartTurn().complete(voice(140.0, seconds=3.0))
    assert 0.0 <= p <= 1.0
