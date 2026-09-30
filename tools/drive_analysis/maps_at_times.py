"""The planner's map at given moments of a drive: /map (SLAM's) and /map with the camera grid
(/nvblox_node/map_grid, already laid on /map's cells) stamped on, as map_server pgm/yaml pairs.

    python maps_at_times.py BAG.mcap OUTDIR NAME=2026-09-29T23:17:01Z [NAME=...]
"""
import os
import sys
from datetime import datetime, timezone

import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory


def when(s):
    return datetime.strptime(s, '%Y-%m-%dT%H:%M:%SZ').replace(tzinfo=timezone.utc).timestamp()


def save(out, name, grid, info):
    h, w = info.height, info.width
    img = np.full((h, w), 205, np.uint8)                 # unknown
    img[grid == 0] = 254                                 # free
    img[grid >= 65] = 0                                  # occupied
    with open(os.path.join(out, name + '.pgm'), 'wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode())
        f.write(img[::-1].tobytes())                     # pgm rows run top-down
    o = info.origin.position
    with open(os.path.join(out, name + '.yaml'), 'w') as f:
        f.write(f'image: {name}.pgm\nmode: trinary\nresolution: {info.resolution}\n'
                f'origin: [{o.x}, {o.y}, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')


def main(bag, out, pairs):
    os.makedirs(out, exist_ok=True)
    wanted = {name: when(t) for name, t in (p.split('=') for p in pairs)}
    latest = {'/map': None, '/nvblox_node/map_grid': None}
    got = {}
    dec = DecoderFactory()
    decoders = {}
    pending = sorted(wanted.items(), key=lambda kv: kv[1])
    with open(bag, 'rb') as f:
        for schema, channel, msg in make_reader(f).iter_messages(topics=list(latest), log_time_order=True):
            t = msg.log_time * 1e-9
            while pending and t > pending[0][1]:
                name, _ = pending.pop(0)
                got[name] = dict(latest)
            if not pending:
                break
            d = decoders.get(channel.id) or decoders.setdefault(channel.id, dec.decoder_for('cdr', schema))
            latest[channel.topic] = (t, d(msg.data))
    for name, (t_want) in wanted.items():
        g = got.get(name)
        if not g or g['/map'] is None:
            print(name, 'no /map before then')
            continue
        tm, m = g['/map']
        base = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        save(out, f'{name}-map', base, m.info)
        line = f'{name}: /map {m.info.width}x{m.info.height} ({wanted[name] - tm:.1f} s old)'
        if g['/nvblox_node/map_grid'] is not None:
            tc, c = g['/nvblox_node/map_grid']
            cam = np.array(c.data, dtype=np.int16).reshape(c.info.height, c.info.width)
            if cam.shape == base.shape:
                merged = base.copy()
                merged[cam >= 65] = 100
                save(out, f'{name}-map+camera', merged, m.info)
                line += f'; camera grid {wanted[name] - tc:.1f} s old, {(cam >= 65).sum()} occupied cells, ' \
                        f'{((cam >= 65) & (base < 65)).sum()} of them not in /map'
            else:
                line += f'; camera grid a different size {cam.shape} vs {base.shape}'
        print(line)


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2], sys.argv[3:])
