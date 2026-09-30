"""Which heading source drifted? From extract_heading.py's .npz.

    python heading_drift.py heading.npz

Truth is SLAM's: the map-frame heading = map->odom yaw + the EKF's odom yaw. For every
source, its turn over consecutive 20 s windows is compared with the truth's turn, and a
least-squares fit gives   source_turn = (1 + scale) * true_turn + bias * seconds.
Sources: the BNO055's gyro (integrated), the BNO055's own orientation, the camera's yaw rate
(integrated - what the EKF fuses) and pose, MOLA's pose, and the EKF itself.
"""
import sys

import numpy as np

WIN = 20.0


def unwrap_series(t, yaw):
    return t, np.unwrap(yaw)


def integrate(t, rate):
    dt = np.diff(t, prepend=t[0])
    dt[(dt < 0) | (dt > 0.5)] = 0.0             # gaps: nothing integrated across them
    return t, np.cumsum(rate * dt)


def at(t_src, y_src, t):
    return np.interp(t, t_src, y_src)


def main(path):
    d = np.load(path)
    imu, vo, mola, ekf, mo = d['imu'], d['vo'], d['mola'], d['ekf'], d['mapodom']
    mo = mo[np.argsort(mo[:, 0])]
    ekf = ekf[np.argsort(ekf[:, 0])]
    t0 = max(ekf[0, 0], mo[0, 0])
    t1 = min(ekf[-1, 0], mo[-1, 0])
    mo_yaw = np.unwrap(mo[:, 3])
    ekf_yaw = np.unwrap(ekf[:, 3])
    print(f'{(t1 - t0) / 60:.1f} min; map->odom yaw went {np.degrees(mo_yaw[0]):+.1f} -> '
          f'{np.degrees(mo_yaw[-1]):+.1f} deg (the EKF heading error SLAM had to absorb)')
    print('map->odom (x, y, yaw) every 5 min:')
    for tt in np.arange(t0, t1, 300.0):
        i = np.searchsorted(mo[:, 0], tt)
        i = min(i, len(mo) - 1)
        print(f'   {(tt - t0) / 60:5.1f} min  ({mo[i, 1]:+6.2f}, {mo[i, 2]:+6.2f}) {np.degrees(mo_yaw[i]):+7.1f} deg')

    edges = np.arange(t0, t1, WIN)
    mid = edges[:-1]
    truth = at(mo[:, 0], mo_yaw, edges) + at(ekf[:, 0], ekf_yaw, edges)
    dtrue = np.diff(truth)
    moving = np.abs(dtrue) > np.radians(3)
    still = np.abs(dtrue) < np.radians(0.5)
    srcs = {
        'BNO055 gyro (integrated)': integrate(imu[:, 0], imu[:, 1]),
        'BNO055 orientation': unwrap_series(imu[:, 0], imu[:, 2]),
        'camera yaw rate (integrated)': integrate(vo[:, 0], vo[:, 2]),
        'EKF (odom yaw)': (ekf[:, 0], ekf_yaw),
        'MOLA pose': unwrap_series(mola[:, 0], mola[:, 3]),
    }
    total_turn = np.degrees(np.sum(np.abs(dtrue)))
    print(f'\n{len(dtrue)} windows of {WIN:.0f} s; she turned {total_turn:.0f} deg in all '
          f'({moving.sum()} turning windows, {still.sum()} still ones)')
    print(f'{"source":30s} {"scale":>8s} {"bias":>11s}  {"err(sum)":>9s}  {"err still":>9s}  {"err turning":>11s}')
    for name, (ts, ys) in srcs.items():
        order = np.argsort(ts)
        ts, ys = ts[order], ys[order]
        v = at(ts, ys, edges)
        dsrc = np.diff(v)
        # windows the source covers without a gap of more than 1 s
        ok = np.ones(len(dsrc), bool)
        for k in range(len(dsrc)):
            a, b = np.searchsorted(ts, [edges[k], edges[k + 1]])
            seg = ts[max(a - 1, 0):b + 1]
            ok[k] = len(seg) > 5 and np.max(np.diff(seg)) < 1.0
        err = dsrc - dtrue
        # no jump over 60 deg in 20 s is real: a restart or a wrap bug
        ok &= np.abs(err) < np.radians(60)
        A = np.stack([dtrue[ok], np.full(ok.sum(), WIN)], 1)
        coef, *_ = np.linalg.lstsq(A, dsrc[ok], rcond=None)
        scale, bias = coef[0] - 1.0, coef[1]
        print(f'{name:30s} {scale * 100:+7.2f}% {np.degrees(bias) * 60:+8.2f}d/m  '
              f'{np.degrees(err[ok].sum()):+8.1f}d  {np.degrees(err[ok & still].sum()):+8.1f}d  '
              f'{np.degrees(err[ok & moving].sum()):+10.1f}d   ({ok.sum()} windows)')


if __name__ == '__main__':
    main(sys.argv[1])
