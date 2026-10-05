#!/usr/bin/env python3
"""Replay a tour's failed trips from the pose each one started at (fast A/B of a fix).

    python3 replay.py BASE_RESULTS.jsonl OUT.jsonl
Start pose of trip n = where trip n-1 ended if it succeeded, else trip n-1's goal (the reset).
"""
import json
import sys

import rclpy

from tour import Tour


def cases(path):
    rs = [json.loads(line) for line in open(path)]
    out = []
    for i, r in enumerate(rs):
        if r['outcome'] == 'SUCCEEDED':
            continue
        if i == 0:
            start = [0.0, 0.0, 0.0]
        else:
            p = rs[i - 1]
            start = p['end'] if p['outcome'] == 'SUCCEEDED' else p['goal']
        out.append((r['n'], start, r['goal']))
    return out


def main():
    base, dst = sys.argv[1], sys.argv[2]
    cs = cases(base)
    rclpy.init()
    t = Tour()
    t.nav.wait_for_server()
    while t.stats is None:
        rclpy.spin_once(t, timeout_sec=0.1)
    out = open(dst, 'w')
    ok = 0
    for n, s, g in cs:
        t.reset_to(*s)
        t.spin_for(1.0)
        rec, _ = t.go(*g)
        rec['n'], rec['start'] = n, s
        ok += rec['outcome'] == 'SUCCEEDED'
        out.write(json.dumps(rec) + '\n')
        out.flush()
        print(f"#{n:3d} from ({s[0]:+.2f},{s[1]:+.2f},{s[2]:+4.0f}) -> {g}: {rec['outcome']:9s} "
              f"err {rec.get('error_code')} {rec.get('seconds', 0):5.1f} s rev {rec.get('reversals', 0)} "
              f"rec {rec['recoveries']}", flush=True)
    print(f'replay: {ok}/{len(cs)} succeeded', flush=True)


if __name__ == '__main__':
    main()
