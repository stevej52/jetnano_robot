"""nav_goal's clear_goal on a recorded planner map (maps_at_times.py's NAME-map+camera.pgm/yaml),
with the global costmap rebuilt the way Nav2 inflates it (nav2.yaml: footprint 0.444 x 0.296,
padding 0.03, inflation 0.31, scaling 6.0). Shows which goals pass and where they get moved.

    python goal_check_replay.py MAP.yaml "x y" ["x y" ...]
"""
import ast
import math
import os
import sys

import cv2
import numpy as np
import yaml



def load_clear_goal(path=os.path.join(os.path.dirname(__file__), '..', '..', 'jetnano_navigation',
                                      'jetnano_navigation', 'nav_goal.py')):
    """nav_goal's constants and clear_goal alone (the rest needs ROS)."""
    tree = ast.parse(open(path, encoding='utf-8').read())
    keep = [n for n in tree.body if isinstance(n, ast.Assign) and n.targets[0].id.isupper()
            or isinstance(n, ast.FunctionDef) and n.name == 'clear_goal']
    ns = {'math': math, 'np': np}
    exec(compile(ast.Module(body=keep, type_ignores=[]), path, 'exec'), ns)
    return ns['clear_goal']

INSCRIBED = 0.148 + 0.03
INFLATION, SCALING = 0.31, 6.0


class Info:
    pass


class Grid:
    pass


def costmap(yaml_path):
    y = yaml.safe_load(open(yaml_path))
    img = cv2.imread(os.path.join(os.path.dirname(yaml_path), y['image']), cv2.IMREAD_UNCHANGED)[::-1]
    res = y['resolution']
    lethal = img < 50
    # distance from every cell to the nearest obstacle cell, in metres
    d = cv2.distanceTransform((~lethal).astype(np.uint8), cv2.DIST_L2, 5) * res
    c = np.zeros(img.shape, np.int16)
    band = (d <= INFLATION)
    c[band] = np.round(1 + 97 * ((252 * np.exp(-SCALING * (d[band] - INSCRIBED))).clip(1, 252) - 1) / 251)
    c[d <= INSCRIBED] = 99
    c[lethal] = 100
    c[img == 205] = -1
    c[lethal] = 100
    g = Grid()
    g.info = Info()
    g.info.resolution, g.info.width, g.info.height = res, img.shape[1], img.shape[0]
    g.info.origin = Info()
    g.info.origin.position = Info()
    g.info.origin.position.x, g.info.origin.position.y = y['origin'][0], y['origin'][1]
    g.data = c.reshape(-1).tolist()
    return g


def main(path, goals):
    g = costmap(path)
    clear_goal = load_clear_goal()
    for s in goals:
        gx, gy = (float(v) for v in s.split())
        found = clear_goal(g, gx, gy)
        print(f'({gx:+.2f}, {gy:+.2f}):', 'REFUSED' if found is None else
              f'ok at ({found[0]:+.2f}, {found[1]:+.2f}), moved {found[2] * 100:.0f} cm')


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2:])
