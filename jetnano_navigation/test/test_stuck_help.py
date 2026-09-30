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

"""nav_helper's non-ROS parts: backing out along her track, the pictures, reading the answer."""

import math

import numpy as np
import pytest

pytest.importorskip('cv2')

from jetnano_navigation import stuck_help as sh  # noqa: E402

MAX_CURVATURE = 1 / 0.40          # her tightest turn


def drive_in(steps):
    """A car's track: forward along the given (distance, curvature) pieces, 2 cm at a time."""
    x = y = yaw = 0.0
    track = [(x, y)]
    for dist, k in steps:
        for _ in range(int(dist / 0.02)):
            yaw += 0.02 * k
            x += 0.02 * math.cos(yaw)
            y += 0.02 * math.sin(yaw)
            track.append((x, y))
    return track, (x, y, yaw)


def back_out(track, pose, metres, lookahead=0.35):
    """nav_helper's retrace loop on a bicycle model that can steer no tighter than 0.40 m."""
    i = len(track) - 1
    x, y, yaw = pose
    done, worst = 0.0, 0.0
    while done < metres:
        i = sh.nearest_behind(track, (x, y), i)
        tgt = sh.track_behind(track, (x, y), lookahead, i)
        if tgt is None:
            break
        k = max(-MAX_CURVATURE, min(MAX_CURVATURE, sh.reverse_curvature((x, y, yaw), tgt[1])))
        v, dt = -0.15, 0.05
        yaw += v * k * dt
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        done += abs(v) * dt
        worst = max(worst, min(math.hypot(px - x, py - y) for px, py in track))
    return (x, y, yaw), done, worst


@pytest.mark.parametrize('pieces', [
    [(1.5, 0.0)],                                   # straight in
    [(0.6, 0.0), (0.8, 1.8), (0.4, 0.0)],           # a left bend
    [(0.5, 0.0), (0.6, -2.2), (0.5, 0.0)],          # a tight right bend
    [(0.4, 1.5), (0.5, -1.5), (0.4, 0.0)],          # an S
])
def test_backs_out_along_the_track_it_came_in_on(pieces):
    track, pose = drive_in(pieces)
    end, done, worst = back_out(track, pose, 1.0)
    assert done >= 0.99                              # went the whole way asked
    assert worst < 0.05                              # never more than 5 cm off her own track


def test_stops_at_the_start_of_the_track():
    track, pose = drive_in([(0.5, 0.0)])
    end, done, worst = back_out(track, pose, 3.0)
    assert done < 0.5                                # there was only half a metre of track


def test_reverse_curvature_sign():
    # a target behind and to her left: backing towards it turns her tail left (yaw rate
    # = negative speed x curvature: curvature > 0 -> she turns clockwise, tail goes left)
    assert sh.reverse_curvature((0.0, 0.0, 0.0), (-1.0, 0.5)) > 0
    assert sh.reverse_curvature((0.0, 0.0, 0.0), (-1.0, -0.5)) < 0
    assert sh.reverse_curvature((0.0, 0.0, 0.0), (-1.0, 0.0)) == 0


def test_pictures_have_what_they_should():
    grid = np.zeros((100, 100), np.int16)
    grid[:, 0] = 100
    house = sh.render_house(grid, (0.05, 0.0, 0.0), (2.5, 2.5, 0.5), track=[(1.5, 2.5), (2.5, 2.5)],
                            plan=[(2.5, 2.5), (4.0, 4.0)], goal=(4.0, 4.0), half=2.0)
    assert house.shape == (240, 240, 3)
    close = sh.render_close(scan_pts=[(1.0, 0.0)], camera_pts=[(0.5, 0.3)], track=[(0, 0), (-1, 0)])
    assert close.shape == (480, 480, 3)
    sheet = sh.contact_sheet([('ahead', np.zeros((480, 640, 3), np.uint8)), ('left', None)], cols=2)
    assert sheet.shape == (326, 800, 3)


def test_scan_points_face_backwards():
    # the lidar faces backwards: its angle 0 is behind her
    pts = sh.scan_points([1.0], 0.0, 0.1, 0.1, 12.0)
    assert pts[0][0] < -0.9 and abs(pts[0][1]) < 1e-9


@pytest.mark.parametrize('text, action', [
    ('{"what": "open dishwasher door", "temporary": true, "action": "retrace", "retrace_m": 0.8, '
     '"say": "The dishwasher is open.", "why": "the way in is clear"}', 'retrace'),
    ('Here you go:\n{"action": "via", "x": 3.2, "y": -5.6, "what": "door"}', 'via'),
    ('{"action": "wait", "wait_s": 500}', 'wait'),
    ('{"action": "give_up", "say": "I cannot get past."}', 'give_up'),
])
def test_parse_advice(text, action):
    a = sh.parse_advice(text)
    assert a['action'] == action
    if action == 'wait':
        assert a['wait_s'] == 60.0                  # clamped
    if action == 'retrace':
        assert a['retrace_m'] == 0.8 and a['temporary']


@pytest.mark.parametrize('text', ['no json here', '{"action": "spin"}', '{"action": "via", "x": "left"}',
                                  '{"action": "via"}', '{bad json'])
def test_parse_advice_refuses(text):
    assert sh.parse_advice(text) is None
