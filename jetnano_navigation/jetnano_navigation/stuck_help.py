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

"""What nav_helper needs that is not ROS: backing out along her own track, the pictures she
sends when she is stuck, and reading the answer. numpy and OpenCV only, so it runs (and is
tested) anywhere.

Steve, 2026-09-29: when she comes to an obstacle and is not sure what to do, send the AI a
picture - of the map, where she is, the obstacle map, and a panorama from the pan-tilt.
"""

import json
import math
import re

import cv2
import numpy as np

# her outline, tyre to tyre (nav2.yaml footprint), metres, x forward, y left
HALF_L, HALF_W = 0.222, 0.148
LIDAR_X, LIDAR_YAW = -0.005, math.pi          # jetnano.urdf.xacro: the lidar faces backwards
ACTIONS = ('retrace', 'via', 'wait', 'give_up')

SYSTEM = """You help Rosie, a small robot car in Steve's house, when she is stuck.
She is 0.44 m long and 0.30 m wide, steers all four wheels (tightest turn: 0.40 m radius) and
CANNOT turn on the spot; she drives forward and backward. Her own planner and controller
could not get her to her goal, and backing out a little and trying again has not worked.

You get:
1. HOUSE MAP around her, north up, grid lines every metre labelled in metres (map frame):
   black = walls and furniture, white = floor, grey = unknown, red = low obstacles her depth
   camera sees now that the map does not have. Her outline is blue with a white nose line
   showing where she faces, the route her planner wanted is purple, her goal is a green star,
   and the track she has just driven is orange.
2. CLOSE-UP of what is around her, drawn with HER FRONT AT THE TOP and her left on the left:
   blue dots = lidar hits (the lidar sees only at 26 cm height), red = obstacles the depth
   camera sees between 10 and 35 cm high (low things the lidar misses), green = her outline,
   orange = the track she came in on. Grid lines every 0.5 m.
3. PHOTOS from her cameras, each labelled with where it looks.

Choose ONE action. Her safety layers stop her before any contact whatever you choose, and
she must never push anything. Reply with ONE JSON object and nothing else:
{"what": "what is in her way, a few words",
 "temporary": true or false (likely to move or be put away soon, e.g. an open door, a person, a pet, a bag),
 "action": "retrace" | "via" | "wait" | "give_up",
 "retrace_m": metres to back out along her own track (0.2 to 1.5; action retrace),
 "x": ..., "y": ... (action via: a point in open floor on the HOUSE MAP, in its metres, to drive to
                     first before going on to the goal - e.g. to go round the other side),
 "wait_s": seconds (5 to 60; action wait: something that will move by itself, like a person),
 "say": "one short plain sentence she can say out loud about it",
 "why": "one sentence"}"""

PROMPT = ('She is stuck on the way to her goal: {situation}. Look at the house map, the '
          'close-up and the photos, and choose what she should do.')


# ------------------------------------------------------------------ backing out --

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def track_behind(track, pose, lookahead, start):
    """The target for backing out along `track` (oldest -> newest, (x, y) in odom), from
    index `start` (the point she is nearest, walking backwards): the first point at least
    `lookahead` from her; nearer the track's beginning than that, its first point, until she is
    within 5 cm of it. -> (index of the target, target (x, y)), or None at the track's end."""
    x, y = pose[0], pose[1]
    for i in range(start, -1, -1):
        if math.hypot(track[i][0] - x, track[i][1] - y) >= lookahead:
            return i, track[i]
    if track and math.hypot(track[0][0] - x, track[0][1] - y) > 0.05:
        return 0, track[0]
    return None


def nearest_behind(track, pose, start, window=40):
    """Index of the track point nearest her, searching backwards from `start` only (she is
    retracing, so the nearest point only moves towards the track's beginning)."""
    lo = max(0, start - window)
    best = min(range(lo, start + 1), key=lambda i: (track[i][0] - pose[0]) ** 2 + (track[i][1] - pose[1]) ** 2)
    return best


def reverse_curvature(pose, target):
    """Pure pursuit for backing towards `target` (odom x, y): the curvature, where yaw rate =
    speed * curvature with a NEGATIVE speed. Same formula as forward (2 y / L^2 in her frame):
    reversing onto a point behind-left turns her tail left, which is the right way."""
    dx, dy = target[0] - pose[0], target[1] - pose[1]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    xr, yr = c * dx + s * dy, -s * dx + c * dy
    d2 = xr * xr + yr * yr
    return 0.0 if d2 < 1e-9 else 2.0 * yr / d2


# ------------------------------------------------------------------ pictures --

def _put(img, text, org, scale=0.45, color=(40, 40, 200), thick=1):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), thick + 2, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)


def outline(pose):
    """Her four corners in the frame of `pose` (x, y, yaw)."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return [(pose[0] + c * px - s * py, pose[1] + s * px + c * py)
            for px, py in ((HALF_L, HALF_W), (HALF_L, -HALF_W), (-HALF_L, -HALF_W), (-HALF_L, HALF_W))]


def render_house(grid, info, pose, track=(), plan=(), goal=None, half=4.0, px_per_m=60, obstacles=()):
    """The house map around `pose` (map frame), north up. grid: 2-D int array of the
    OccupancyGrid (row 0 = the map's bottom), info: (resolution, origin_x, origin_y).
    obstacles: (x, y) map points the camera sees that the map does not have, drawn red."""
    res, ox, oy = info
    size = int(2 * half * px_per_m)
    x0, y0 = pose[0] - half, pose[1] - half
    img = np.full((size, size, 3), 200, np.uint8)                  # unknown: grey
    # sample the grid at every pixel's centre
    js, is_ = np.meshgrid(np.arange(size), np.arange(size))
    wx = x0 + (js + 0.5) / px_per_m
    wy = y0 + (size - 1 - is_ + 0.5) / px_per_m
    cu = np.floor((wx - ox) / res).astype(int)
    cv = np.floor((wy - oy) / res).astype(int)
    inside = (cu >= 0) & (cu < grid.shape[1]) & (cv >= 0) & (cv < grid.shape[0])
    vals = np.full((size, size), -1, np.int16)
    vals[inside] = grid[cv[inside], cu[inside]]
    img[vals == 0] = (255, 255, 255)
    img[vals >= 65] = (0, 0, 0)

    def to_px(x, y):
        return int(round((x - x0) * px_per_m)), int(round(size - 1 - (y - y0) * px_per_m))

    for g in range(int(math.floor(x0)), int(math.ceil(x0 + 2 * half)) + 1):
        u = to_px(g, 0)[0]
        cv2.line(img, (u, 0), (u, size), (235, 190, 150), 1)
        _put(img, f'x{g}', (u + 2, 14))
    for g in range(int(math.floor(y0)), int(math.ceil(y0 + 2 * half)) + 1):
        v = to_px(0, g)[1]
        cv2.line(img, (0, v), (size, v), (235, 190, 150), 1)
        _put(img, f'y{g}', (2, v - 3))
    cell = max(2, int(0.05 * px_per_m))
    for x, y in obstacles:
        u, v = to_px(x, y)
        if 0 <= u < size and 0 <= v < size:
            cv2.rectangle(img, (u - cell // 2, v - cell // 2), (u + cell // 2, v + cell // 2), (40, 40, 230), -1)
    if len(plan) > 1:
        cv2.polylines(img, [np.array([to_px(*p) for p in plan], np.int32)], False, (200, 60, 170), 2, cv2.LINE_AA)
    if len(track) > 1:
        cv2.polylines(img, [np.array([to_px(*p) for p in track], np.int32)], False, (0, 140, 255), 2, cv2.LINE_AA)
    if goal is not None:
        u, v = to_px(goal[0], goal[1])
        cv2.drawMarker(img, (u, v), (0, 170, 0), cv2.MARKER_STAR, 22, 2)
        _put(img, 'goal', (u + 10, v - 8), color=(0, 130, 0))
    pts = np.array([to_px(*p) for p in outline(pose)], np.int32)
    cv2.fillPoly(img, [pts], (230, 120, 40))
    nose = to_px(pose[0] + HALF_L * math.cos(pose[2]), pose[1] + HALF_L * math.sin(pose[2]))
    cv2.line(img, to_px(pose[0], pose[1]), nose, (255, 255, 255), 2)
    _put(img, 'HOUSE MAP - north up, metres', (6, size - 8), 0.5, (0, 0, 0))
    return img


def render_close(scan_pts=(), camera_pts=(), track=(), half=2.0, px_per_m=120):
    """What is around her, in HER frame (x forward, y left), drawn front up, left on the left.
    scan_pts, camera_pts and track are (x, y) in her frame."""
    size = int(2 * half * px_per_m)
    img = np.full((size, size, 3), 255, np.uint8)

    def to_px(x, y):                                   # forward = up, left = left
        return int(round(size / 2 - y * px_per_m)), int(round(size / 2 - x * px_per_m))

    step = 0.5
    for k in range(-int(half / step), int(half / step) + 1):
        u = to_px(0, -k * step)[0]
        v = to_px(k * step, 0)[1]
        cv2.line(img, (u, 0), (u, size), (235, 225, 215), 1)
        cv2.line(img, (0, v), (size, v), (235, 225, 215), 1)
    cell = max(2, int(0.05 * px_per_m))
    for x, y in camera_pts:
        u, v = to_px(x, y)
        cv2.rectangle(img, (u - cell // 2, v - cell // 2), (u + cell // 2, v + cell // 2), (40, 40, 230), -1)
    for x, y in scan_pts:
        cv2.circle(img, to_px(x, y), 3, (220, 90, 20), -1, cv2.LINE_AA)
    if len(track) > 1:
        cv2.polylines(img, [np.array([to_px(*p) for p in track], np.int32)], False, (0, 140, 255), 3, cv2.LINE_AA)
    pts = np.array([to_px(*p) for p in outline((0.0, 0.0, 0.0))], np.int32)
    cv2.polylines(img, [pts], True, (40, 160, 40), 3, cv2.LINE_AA)
    cv2.line(img, to_px(0, 0), to_px(HALF_L, 0), (40, 160, 40), 2)
    _put(img, 'FRONT', (size // 2 - 24, 18), 0.55, (40, 120, 40), 2)
    _put(img, 'BEHIND', (size // 2 - 30, size - 8), 0.55, (40, 120, 40), 2)
    _put(img, 'LEFT', (4, size // 2), 0.55, (40, 120, 40), 2)
    _put(img, 'RIGHT', (size - 60, size // 2), 0.55, (40, 120, 40), 2)
    _put(img, 'CLOSE-UP, her front up, grid 0.5 m', (6, size - 28), 0.45, (0, 0, 0))
    return img


def scan_points(ranges, angle_min, angle_inc, rmin, rmax, max_range=2.5):
    """The lidar's hits in her frame (the lidar faces backwards, jetnano.urdf.xacro)."""
    out = []
    for i, r in enumerate(ranges):
        if rmin <= r <= min(rmax, max_range) and math.isfinite(r):
            a = angle_min + i * angle_inc + LIDAR_YAW
            out.append((r * math.cos(a) + LIDAR_X, r * math.sin(a)))
    return out


def to_robot(points, pose):
    """(x, y) points in the pose's frame (odom or map) -> her frame."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return [(c * (x - pose[0]) + s * (y - pose[1]), -s * (x - pose[0]) + c * (y - pose[1])) for x, y in points]


def grid_cells(grid, info, threshold=65):
    """Centres (x, y) of the occupied cells of an OccupancyGrid array, in its frame."""
    res, ox, oy = info
    rows, cols = np.nonzero(grid >= threshold)
    return list(zip(ox + (cols + 0.5) * res, oy + (rows + 0.5) * res))


def contact_sheet(views, cols=3, cell_w=400, cell_h=300):
    """[(label, BGR image or None)] -> one labelled grid."""
    rows = max(1, math.ceil(len(views) / cols))
    sheet = np.full((rows * (cell_h + 26), cols * cell_w, 3), 255, np.uint8)
    for k, (label, img) in enumerate(views):
        r, c = divmod(k, cols)
        y, x = r * (cell_h + 26), c * cell_w
        if img is not None:
            sheet[y + 26:y + 26 + cell_h, x:x + cell_w] = cv2.resize(img, (cell_w, cell_h), interpolation=cv2.INTER_AREA)
        else:
            _put(sheet, '(no picture)', (x + 10, y + 26 + cell_h // 2), 0.6, (80, 80, 80))
        _put(sheet, label, (x + 6, y + 19), 0.6, (0, 0, 0), 2)
    return sheet


# ------------------------------------------------------------------ the answer --

def parse_advice(text):
    """Claude's reply -> a checked dict, or None if there is no usable one. Numbers are
    clamped to what she can safely do; anything else is dropped."""
    m = re.search(r'\{.*\}', text or '', re.S)
    if not m:
        return None
    try:
        raw = json.loads(m.group(0))
    except ValueError:
        return None
    action = raw.get('action')
    if action not in ACTIONS:
        return None
    out = {'action': action, 'what': str(raw.get('what', ''))[:120], 'temporary': bool(raw.get('temporary', False)),
           'say': str(raw.get('say', ''))[:200], 'why': str(raw.get('why', ''))[:300]}
    try:
        if action == 'retrace':
            out['retrace_m'] = max(0.2, min(1.5, float(raw.get('retrace_m', 0.6))))
        elif action == 'via':
            out['x'], out['y'] = float(raw['x']), float(raw['y'])
            if not (math.isfinite(out['x']) and math.isfinite(out['y'])):
                return None
        elif action == 'wait':
            out['wait_s'] = max(5.0, min(60.0, float(raw.get('wait_s', 15))))
    except (KeyError, TypeError, ValueError):
        return None
    return out
