"""Sim world from a real drive: the old map, corrected wherever her lidar looked on that drive.

    python3 make_world.py REF.yaml SCAN.npz OUT_PREFIX [diff.png]

OUT_PREFIX.pgm/.yaml = what the sim's lidar sees and she bumps into (tall things).
Rules: a cell her rays crossed >= 4 times and hit < 10 % of the time is open floor; a cell hit
>= 3 times and >= 40 % of the time is solid (0.25 smeared walls inward with her localisation jitter). Everything else keeps the old map.
"""
import sys
import numpy as np
import yaml
from PIL import Image

ref, scan, outp = sys.argv[1:4]
m = yaml.safe_load(open(ref))
d = ref.rsplit('/', 1)[0] if '/' in ref else '.'
img = np.array(Image.open(d + '/' + m['image']))
z = np.load(scan)
hit, pas = z['hit'].astype(float), z['pas'].astype(float)
ratio = hit / np.maximum(hit + pas, 1)
free = (pas >= 4) & (ratio < 0.10)
solid = (hit >= 3) & (ratio >= 0.40)
new = img.copy()
new[free] = 254
new[solid] = 0
Image.fromarray(new).save(outp + '.pgm')
m2 = dict(m)
m2['image'] = outp.rsplit('/', 1)[-1] + '.pgm'
yaml.safe_dump(m2, open(outp + '.yaml', 'w'))
opened = (img != 254) & (new == 254)
closed = (img == 254) & (new != 254)
print(f'opened {int(opened.sum())} cells, closed {int(closed.sum())}')
if len(sys.argv) > 4:
    rgb = np.stack([new] * 3, -1).astype(np.uint8)
    rgb[img == 205] = rgb[img == 205]
    rgb[opened] = (60, 200, 60)
    rgb[closed] = (220, 40, 40)
    # crop to where she drove, x4
    ys, xs = np.nonzero((pas > 0) | (hit > 0))
    r0, r1, c0, c1 = max(ys.min() - 10, 0), ys.max() + 10, max(xs.min() - 10, 0), xs.max() + 10
    crop = Image.fromarray(rgb[r0:r1, c0:c1]).resize(((c1 - c0) * 4, (r1 - r0) * 4), Image.NEAREST)
    crop.save(sys.argv[4])
    ox, oy, res = m['origin'][0], m['origin'][1], m['resolution']
    H = img.shape[0]
    print('crop x %.2f..%.2f  y %.2f..%.2f' % (ox + c0 * res, ox + c1 * res, oy + (H - r1) * res, oy + (H - r0) * res))
