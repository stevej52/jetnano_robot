#!/usr/bin/env python3
"""Draw the tour on the house map: out/tour.png + out/summary.json.

Each goal is a dot where she was sent, coloured by how it went:
  green  = got there cleanly (<= 2 direction changes, no recovery)
  amber  = got there, but struggled (3+ direction changes or a Nav2 recovery)
  red    = did not get there (failed or out of time); a black X marks where she was left
Her track is drawn faint blue; bumps into walls (sim) are magenta.
"""
import json
import os
import math

import numpy as np
import yaml
from PIL import Image, ImageDraw, ImageFont

D = os.path.dirname(os.path.realpath(__file__))
S = 6   # pixels per map cell (5 cm)


def grade(r):
    if r['outcome'] != 'SUCCEEDED':
        return 'red'
    if r.get('reversals', 0) >= 3 or r.get('recoveries', 0) > 0:
        return 'amber'
    return 'green'


def main():
    m = yaml.safe_load(open(f'{D}/map/home.yaml'))
    img = np.array(Image.open(f'{D}/map/{m["image"]}'))
    res, ox, oy = m['resolution'], m['origin'][0], m['origin'][1]
    h, w = img.shape
    rgb = np.stack([np.where(img == 254, 250, np.where(img == 0, 40, 200))] * 3, -1).astype(np.uint8)
    im = Image.fromarray(rgb).resize((w * S, h * S), Image.NEAREST)
    dr = ImageDraw.Draw(im, 'RGBA')

    def px(x, y):
        return ((x - ox) / res * S, (h - (y - oy) / res) * S)

    track = [json.loads(line) for line in open(f'{D}/out/track.jsonl')]
    pts = [px(t['x'], t['y']) for t in track]
    for a, b in zip(pts, pts[1:]):
        if math.dist(a, b) < 0.6 / res * S:          # skip the teleports
            dr.line([a, b], fill=(40, 110, 230, 70), width=2)
    rs = [json.loads(line) for line in open(f'{D}/out/results.jsonl')]
    col = {'green': (30, 160, 60, 255), 'amber': (235, 150, 20, 255), 'red': (215, 40, 40, 255)}
    seen_bumps = set()
    prev = None
    for t in track:
        if t.get('last_bump') and tuple(t['last_bump']) != prev:
            prev = tuple(t['last_bump'])
            seen_bumps.add(prev)
    for bx, by in seen_bumps:
        x, y = px(bx, by)
        dr.ellipse([x - 7, y - 7, x + 7, y + 7], outline=(200, 0, 200, 255), width=3)
    for r in rs:
        x, y = px(r['goal'][0], r['goal'][1])
        g = grade(r)
        hd = math.radians(r['goal'][2])
        dr.line([x, y, x + 16 * math.cos(hd), y - 16 * math.sin(hd)], fill=col[g], width=3)
        dr.ellipse([x - 8, y - 8, x + 8, y + 8], fill=col[g], outline=(0, 0, 0, 255))
        if g == 'red':
            ex, ey = px(r['end'][0], r['end'][1])
            dr.line([ex - 9, ey - 9, ex + 9, ey + 9], fill=(0, 0, 0, 255), width=4)
            dr.line([ex - 9, ey + 9, ex + 9, ey - 9], fill=(0, 0, 0, 255), width=4)
            dr.text((x + 10, y - 8), str(r['n']), fill=(0, 0, 0, 255))
    x0, y0 = px(0, 0)
    dr.rectangle([x0 - 10, y0 - 10, x0 + 10, y0 + 10], outline=(0, 0, 0, 255), width=3)
    im.save(f'{D}/out/tour.png', optimize=True)
    summ = {'goals': len(rs), 'green': sum(grade(r) == 'green' for r in rs),
            'amber': sum(grade(r) == 'amber' for r in rs), 'red': sum(grade(r) == 'red' for r in rs),
            'bumps': len(seen_bumps), 'sim_seconds': round(sum(r.get('seconds', 0) for r in rs)),
            'driven_m': round(sum(r.get('driven_m', 0) for r in rs), 1),
            'failed': [r for r in rs if grade(r) == 'red'],
            'struggled': [r for r in rs if grade(r) == 'amber']}
    json.dump(summ, open(f'{D}/out/summary.json', 'w'), indent=1)
    print({k: v for k, v in summ.items() if k not in ('failed', 'struggled')})


if __name__ == '__main__':
    main()
