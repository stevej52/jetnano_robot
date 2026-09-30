"""Score a replayed EKF's heading against SLAM's truth from the same drive.

    python replay_compare.py heading.npz replay_ekf_yaw.csv

Truth, as in heading_drift.py: map->odom yaw + the original EKF's odom yaw (both recorded).
The replayed EKF started its odom frame at its own zero, so only changes are compared: over
20 s windows (scale and bias fit), and the accumulated error over the drive and over any
5-minute span - the drift that placed nvblox's obstacles wrongly.
"""
import sys

import numpy as np


def main(npz, csv):
    d = np.load(npz)
    ekf, mo = d['ekf'], d['mapodom']
    mo = mo[np.argsort(mo[:, 0])]
    ekf = ekf[np.argsort(ekf[:, 0])]
    r = np.loadtxt(csv, delimiter=',', skiprows=1)
    r = r[np.argsort(r[:, 0])]
    t0 = max(ekf[0, 0], mo[0, 0], r[0, 0])
    t1 = min(ekf[-1, 0], mo[-1, 0], r[-1, 0])
    tt = np.arange(t0, t1, 1.0)
    truth = np.interp(tt, mo[:, 0], np.unwrap(mo[:, 3])) + np.interp(tt, ekf[:, 0], np.unwrap(ekf[:, 3]))
    old = np.interp(tt, ekf[:, 0], np.unwrap(ekf[:, 3]))
    new = np.interp(tt, r[:, 0], np.unwrap(r[:, 3]))
    print(f'{(t1 - t0) / 60:.1f} min compared ({len(r)} replayed poses)')
    print(f'{"":22s} {"end err":>8s} {"worst 5 min":>11s} {"p95 5 min":>9s} {"scale":>8s}')
    for name, y in (('EKF on the drive', old), ('EKF replayed (new)', new)):
        e = np.degrees((y - y[0]) - (truth - truth[0]))
        w5 = np.abs(e[300:] - e[:-300]) if len(e) > 300 else np.abs(e - e[0])
        dt, dy = np.diff(truth[::20]), np.diff(y[::20])
        ok = np.abs(dy - dt) < np.radians(60)
        k = np.polyfit(dt[ok], dy[ok], 1)[0] if ok.sum() > 2 else float('nan')
        print(f'{name:22s} {e[-1]:+7.1f}d {w5.max():10.1f}d {np.percentile(w5, 95):8.1f}d {100 * (k - 1):+7.2f}%')


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
