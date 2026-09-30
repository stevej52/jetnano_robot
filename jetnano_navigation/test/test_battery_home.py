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

"""When a low battery sends her home, and when it must not."""

import pytest

pytest.importorskip('rclpy')

from jetnano_navigation.battery_home import should_go_home  # noqa: E402


def test_low_out_in_the_house_goes_home():
    assert should_go_home('low', 'placed', 4.2, False) == (True, 'battery low: heading home')


def test_only_low_counts():
    for level in ('ok', 'soon', 'flat', 'none'):
        assert should_go_home(level, 'placed', 4.2, False)[0] is False


def test_not_from_the_bench():
    assert should_go_home('low', 'bench', None, False)[0] is False


def test_not_when_already_home_or_already_going():
    assert should_go_home('low', 'placed', 0.3, False)[0] is False
    assert should_go_home('low', 'placed', 4.2, True)[0] is False


def test_no_position_yet_still_goes():
    """/pose may not have arrived: better to try than to sit and go flat."""
    assert should_go_home('low', None, None, False)[0] is True
