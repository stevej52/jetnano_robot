#!/usr/bin/env python3
"""What crashed during a drive: drive_crashes.py START_EPOCH END_EPOCH OUT_JSON

drive.sh runs it after a lap (2026-10-12, Steve: crashes per drive on the scorecard). From the
journal between the two times: processes that died (ros2 launch's "process has died", with the
exit code), and Python tracebacks, by process; from ~/watchdog/events.jsonl: the watchdog's
"down" events, by what. A kernel crash ends the drive itself and leaves no crashes.json - the
scorecard then shows an empty cell, not a zero (the black box keeps the crash record).
"""

import collections
import json
import os
import re
import subprocess
import sys
import time


def main():
    t0, t1, out = float(sys.argv[1]), float(sys.argv[2]), sys.argv[3]
    deaths, tracebacks, downs = collections.Counter(), collections.Counter(), collections.Counter()
    try:
        text = subprocess.run(['journalctl', '--since', f'@{int(t0)}', '--until', f'@{int(t1) + 1}',
                               '--no-pager', '-q', '-o', 'cat'],
                              capture_output=True, text=True, timeout=60).stdout
    except (OSError, subprocess.TimeoutExpired):
        text = ''
    for ln in text.splitlines():
        m = re.search(r'\[ERROR\] \[([\w.\-]+?)-\d+\]: process has died \[pid \d+, exit code (-?\d+)', ln)
        if m:
            deaths[f'{m.group(1)} ({m.group(2)})'] += 1
            continue
        m = re.match(r'\[([\w.\-]+?)-\d+\] Traceback \(most recent call last\)', ln)
        if m:
            tracebacks[m.group(1)] += 1
    try:
        with open(os.path.expanduser('~/watchdog/events.jsonl')) as f:
            for ln in f:
                try:
                    e = json.loads(ln)
                    t = time.mktime(time.strptime(e['t'], '%Y-%m-%d %H:%M:%S'))
                except (ValueError, KeyError):
                    continue
                if t0 <= t <= t1 + 1 and e.get('kind') == 'down':
                    downs[e.get('what', '?')] += 1
    except OSError:
        pass
    j = {'from': time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime(t0)),
         'to': time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime(t1)),
         'process_deaths': sum(deaths.values()), 'deaths': dict(deaths),
         'tracebacks': sum(tracebacks.values()), 'traceback_by': dict(tracebacks),
         'watchdog_downs': sum(downs.values()), 'downs': dict(downs)}
    with open(out, 'w') as f:
        json.dump(j, f, indent=1)
    what = ', '.join(list(deaths) + [f'{k} down' for k in downs]) or 'nothing'
    print(f'crashes: {j["process_deaths"]} process deaths, {j["tracebacks"]} tracebacks, '
          f'{j["watchdog_downs"]} watchdog downs ({what})')


if __name__ == '__main__':
    main()
