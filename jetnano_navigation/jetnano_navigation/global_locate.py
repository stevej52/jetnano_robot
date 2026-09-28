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

"""Where on the saved map is she? One lidar scan against the whole map, no starting guess.

slam_toolbox only refines a guess it is given (the parking spot, map_start_at_dock).
This finds the guess: every free spot of the map on a 10 cm grid, at every heading
in 5 degree steps, is scored by how close the scan's points fall to the map's walls;
the best few hundred are refined to 3 cm and 1.5 degrees.

The score of a pose is the mean, over the scan's points, of exp(-d^2 / 2 sigma^2),
d = the distance from the point to the nearest wall cell: 1.0 if every point lies
on a wall, 0 if none is near one. The coarse pass scores against a blurred copy
(sigma 0.25 m) so that a pose 5 degrees off - 44 cm at 5 m - still scores well
enough to be refined; the fine pass uses sigma 0.08 m.

Tried on the bench, 2026-09-28 (1 m up, the lidar seeing the room above the
furniture): the true spot scored 0.64 and two wrong spots in other rooms 0.66 and
0.64 - so the answer comes with a verdict, not just a pose: see judge().
~6 s on one Jetson core for the 64 m2 house map.
"""

from __future__ import annotations

import math
import os

import numpy as np


def read_map_yaml(path: str) -> dict:
    """map_saver's .yaml: flat 'key: value' lines, origin as [x, y, yaw]."""
    meta = {}
    with open(path) as f:
        for line in f:
            key, _, value = line.partition(':')
            key, value = key.strip(), value.split('#', 1)[0].strip()
            if not key or not value:
                continue
            if value.startswith('['):
                meta[key] = [float(x) for x in value.strip('[]').split(',')]
            else:
                try:
                    meta[key] = float(value)
                except ValueError:
                    meta[key] = value
    return meta


def read_pgm(path: str) -> np.ndarray:
    """An 8-bit binary (P5) PGM as a height x width array, top row first."""
    with open(path, 'rb') as f:
        data = f.read()
    tokens, i = [], 0
    while len(tokens) < 4:
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            i = data.index(b'\n', i) + 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        tokens.append(data[i:j])
        i = j
    if tokens[0] != b'P5' or int(tokens[3]) > 255:
        raise ValueError('not an 8-bit binary PGM')
    w, h = int(tokens[1]), int(tokens[2])
    return np.frombuffer(data, dtype=np.uint8, count=w * h, offset=i + 1).reshape(h, w)


class MapModel:
    """A saved map ready to score scans against. Rows run upward (row 0 = lowest y)."""

    def __init__(self, occupied: np.ndarray, free: np.ndarray, res: float, x0: float, y0: float,
                 sigma: float = 0.08, sigma_coarse: float = 0.25, cap: float = 0.6):
        self.occupied, self.free = occupied, free
        self.res, self.x0, self.y0 = float(res), float(x0), float(y0)
        self.h, self.w = occupied.shape
        d = _wall_distance(occupied, self.res, cap)
        self.like = np.exp(-(d * d) / (2 * sigma * sigma)).astype(np.float32)
        self.like_coarse = np.exp(-(d * d) / (2 * sigma_coarse * sigma_coarse)).astype(np.float32)

    @classmethod
    def load(cls, base: str, **kw) -> 'MapModel':
        """<base>.yaml and the image it names (map_saver's output)."""
        base = os.path.expanduser(base)
        meta = read_map_yaml(base + '.yaml')
        pixels = read_pgm(os.path.join(os.path.dirname(base), str(meta['image'])))
        p = pixels.astype(np.float32) / 255.0
        occ_p = p if int(meta.get('negate', 0)) else 1.0 - p
        occupied = (occ_p > float(meta.get('occupied_thresh', 0.65)))[::-1]
        free = (occ_p < float(meta.get('free_thresh', 0.196)))[::-1]
        x0, y0 = meta['origin'][0], meta['origin'][1]
        return cls(occupied, free, float(meta['resolution']), x0, y0, **kw)

    def score(self, xs, ys, yaws, pts: np.ndarray, coarse: bool = False) -> np.ndarray:
        """Mean wall likelihood of the scan ``pts`` (N x 2, her own frame) at each pose."""
        like = self.like_coarse if coarse else self.like
        out = np.empty(len(xs), dtype=np.float32)
        step = max(1, 400000 // max(1, len(pts)))
        for i in range(0, len(xs), step):
            c = np.cos(yaws[i:i + step])[:, None]
            s = np.sin(yaws[i:i + step])[:, None]
            mx = xs[i:i + step, None] + c * pts[None, :, 0] - s * pts[None, :, 1]
            my = ys[i:i + step, None] + s * pts[None, :, 0] + c * pts[None, :, 1]
            ci = np.floor((mx - self.x0) / self.res).astype(np.int32)
            ri = np.floor((my - self.y0) / self.res).astype(np.int32)
            inside = (ci >= 0) & (ci < self.w) & (ri >= 0) & (ri < self.h)
            v = like[np.clip(ri, 0, self.h - 1), np.clip(ci, 0, self.w - 1)]
            out[i:i + step] = np.where(inside, v, 0.0).mean(axis=1)
        return out


def _wall_distance(occupied: np.ndarray, res: float, cap: float) -> np.ndarray:
    """Distance (m) from every cell to the nearest wall cell, capped (8-neighbour chamfer)."""
    h, w = occupied.shape
    d = np.where(occupied, 0.0, cap).astype(np.float32)
    for _ in range(int(cap / res) + 2):
        before = d.copy()
        for dy, dx, k in ((0, 1, 1.0), (1, 0, 1.0), (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2))):
            for sgn in (1, -1):
                ay, ax = sgn * dy, sgn * dx
                src = d[max(0, -ay):h - max(0, ay), max(0, -ax):w - max(0, ax)]
                dst = (slice(max(0, ay), h - max(0, -ay)), slice(max(0, ax), w - max(0, -ax)))
                d[dst] = np.minimum(d[dst], src + k * res)
        if np.array_equal(before, d):
            break
    return np.minimum(d, cap)


def _distinct(a, b, dist_m: float, angle_rad: float) -> bool:
    return (math.hypot(a[1] - b[1], a[2] - b[2]) > dist_m
            or abs(math.remainder(a[3] - b[3], 2 * math.pi)) > angle_rad)


def locate(model: MapModel, pts: np.ndarray, coarse_points: int = 120,
           keep_per_heading: int = 60, seeds: int = 150, top: int = 3) -> list:
    """The ``top`` best clearly different poses: [(score, x, y, yaw), ...], best first.

    ``pts``: the scan's points (N x 2) in her own frame (base_footprint), metres.
    """
    if len(pts) < 20:
        return []
    # coarse: every free cell on a 10 cm grid, 5 degree headings, a subset of the points
    fr, fc = np.nonzero(model.free[::2, ::2])
    cx = model.x0 + (fc * 2 + 1) * model.res
    cy = model.y0 + (fr * 2 + 1) * model.res
    sub = pts[np.linspace(0, len(pts) - 1, min(coarse_points, len(pts))).astype(int)]
    best = []
    for yaw in np.radians(np.arange(0.0, 360.0, 5.0)):
        sc = model.score(cx, cy, np.full(len(cx), yaw), sub, coarse=True)
        for i in np.argsort(sc)[-keep_per_heading:]:
            best.append((float(sc[i]), float(cx[i]), float(cy[i]), float(yaw)))
    best.sort(reverse=True)
    # seeds: the best, at most one per 30 cm and 15 degrees
    chosen = []
    for c in best:
        if all(_distinct(c, q, 0.3, math.radians(15)) for q in chosen):
            chosen.append(c)
            if len(chosen) == seeds:
                break
    # fine: 3 cm and 1.5 degrees around each seed, every point
    g = np.arange(-0.12, 0.121, 0.03)
    a = np.radians(np.arange(-6.0, 6.01, 1.5))
    refined = []
    for _, bx, by, byaw in chosen:
        gx, gy, gyaw = np.meshgrid(bx + g, by + g, byaw + a, indexing='ij')
        gx, gy, gyaw = gx.ravel(), gy.ravel(), gyaw.ravel()
        sc = model.score(gx, gy, gyaw, pts)
        i = int(np.argmax(sc))
        refined.append((float(sc[i]), float(gx[i]), float(gy[i]),
                        math.remainder(float(gyaw[i]), 2 * math.pi)))
    refined.sort(reverse=True)
    out = []
    for c in refined:
        if all(_distinct(c, q, 1.0, math.radians(30)) for q in out):
            out.append(c)
            if len(out) == top:
                break
    return out


def judge(candidates: list, accept: float, margin: float) -> tuple[str, str]:
    """('placed' | 'unsure' | 'not_on_map', why) for locate()'s answer.

    placed: the best fits at least ``accept`` and beats the best clearly different
    place by the factor ``margin``.
    """
    if not candidates:
        return 'not_on_map', 'too few lidar points'
    best = candidates[0][0]
    if best < accept:
        return 'not_on_map', f'best fit {best:.2f}, needs {accept:.2f}'
    if len(candidates) > 1 and best < margin * candidates[1][0]:
        return 'unsure', (f'{len(candidates)} places fit alike: {best:.2f} vs '
                          f'{candidates[1][0]:.2f} (needs x{margin:.2f})')
    return 'placed', f'fit {best:.2f}' + (f', next place {candidates[1][0]:.2f}'
                                          if len(candidates) > 1 else '')
