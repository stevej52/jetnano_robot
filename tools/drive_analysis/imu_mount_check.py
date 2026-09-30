"""Does the BNO055's orientation, turned into base_link by the URDF's imu_rpy, give the robot's
heading with the right sign and scale, and read level? (Before fusing it in the EKF.)

    python imu_mount_check.py BAG.mcap heading.npz
"""
import math
import sys

import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

IMU_RPY = (2.9698, 0.0552, 1.5680)          # jetnano.urdf.xacro imu_rpy (base_link -> imu_link)


def q_from_rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r / 2), math.sin(r / 2), math.cos(p / 2), math.sin(p / 2),
                              math.cos(y / 2), math.sin(y / 2))
    return np.array([cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                     cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy])   # w x y z


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def rpy(q):
    w, x, y, z = q
    return (math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
            math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))),
            math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def main(bag, npz):
    mount_inv = q_from_rpy(*IMU_RPY) * np.array([1, -1, -1, -1])
    dec = DecoderFactory()
    decoder = None
    rows = []
    n = 0
    with open(bag, 'rb') as f:
        for schema, channel, msg in make_reader(f).iter_messages(topics=['/imu/data']):
            n += 1
            if n % 10:
                continue
            decoder = decoder or dec.decoder_for(channel.message_encoding, schema)
            m = decoder(msg.data)
            o = m.orientation
            q_imu = np.array([o.w, o.x, o.y, o.z])
            q_base = qmul(q_imu, mount_inv)            # world<-imu * imu<-base
            r, p, y = rpy(q_base)
            rows.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, r, p, y))
    a = np.array(rows)
    print(f'{len(a)} samples; base roll {np.degrees(np.median(a[:, 1])):+.1f} deg, pitch '
          f'{np.degrees(np.median(a[:, 2])):+.1f} deg (median; level floor should read ~0)')
    d = np.load(npz)
    ekf, mo = d['ekf'], d['mapodom']
    mo = mo[np.argsort(mo[:, 0])]
    ekf = ekf[np.argsort(ekf[:, 0])]
    t0, t1 = max(ekf[0, 0], mo[0, 0], a[0, 0]), min(ekf[-1, 0], mo[-1, 0], a[-1, 0])
    edges = np.arange(t0, t1, 20.0)
    truth = np.interp(edges, mo[:, 0], np.unwrap(mo[:, 3])) + np.interp(edges, ekf[:, 0], np.unwrap(ekf[:, 3]))
    yb = np.interp(edges, a[:, 0], np.unwrap(a[:, 3]))
    dt, dy = np.diff(truth), np.diff(yb)
    ok = np.abs(dy - dt) < math.radians(60)
    k = np.polyfit(dt[ok], dy[ok], 1)
    print(f'base yaw from the IMU vs truth over 20 s windows: slope {k[0]:+.4f} (want +1), '
          f'total error {math.degrees((dy - dt)[ok].sum()):+.1f} deg over {len(dt)} windows')


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
