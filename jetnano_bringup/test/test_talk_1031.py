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

"""Steve's demo, 2026-10-31: her name inside a long turn, and the brain's cool-off from a
starved local model."""

import time
from types import SimpleNamespace

from jetnano_bringup.brain import Brain
from jetnano_bringup.listen import addressed_part
import pytest

DEMO = ("And now you can talk to it. And there's a delay. Every time I work on it, I try to get the "
        "delay down, but it's hard, I don't have super fast processors, so a one or two second delay "
        "happens. Rosie, give me a status update. And that")


def test_her_name_inside_a_long_turn_is_the_request():
    assert addressed_part(DEMO) == 'Rosie, give me a status update.'


def test_just_her_name_takes_the_next_sentence():
    text = ' '.join(['word'] * 40) + '. Rosie. What time is it? And then we went out.'
    assert addressed_part(text) == 'Rosie. What time is it?'


@pytest.mark.parametrize('text', ['Rosie, what time is it?',                     # short: as it is
                                  ' '.join(['we talked about the weather'] * 10)])  # long, no name
def test_left_alone(text):
    assert addressed_part(text) == text


def brain(backend='auto'):
    logged = []
    b = SimpleNamespace(local_up=True, backend=backend, local_cooldown=180.0, _local_cool_until=0.0,
                        get_logger=lambda: SimpleNamespace(info=logged.append))
    return b, logged


def test_a_slow_call_sends_the_next_ones_to_claude():
    b, logged = brain()
    assert Brain._local_usable(b)
    Brain._cool(b, 'TimeoutError')
    assert not Brain._local_usable(b)
    assert len(logged) == 1 and '180 s' in logged[0]
    Brain._cool(b, 'TimeoutError')            # still cooling: extended, not logged again
    assert len(logged) == 1
    b._local_cool_until = time.monotonic() - 1.0
    assert Brain._local_usable(b)


def test_local_only_never_cools():
    b, _ = brain('local')
    Brain._cool(b, 'TimeoutError')
    assert Brain._local_usable(b)             # nothing else to answer with


def test_down_is_down():
    b, _ = brain()
    b.local_up = False
    assert not Brain._local_usable(b)
