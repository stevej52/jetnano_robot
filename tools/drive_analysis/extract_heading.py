"""Pull every heading source out of a drive recording into one small .npz, for drift analysis.

    python extract_heading.py BAG.mcap OUT.npz

Reads the MCAP directly (mcap + mcap-ros2-support, no ROS needed) and keeps, as arrays of
(stamp seconds, values): the BNO055 (imu/data: yaw-rate and its own orientation yaw), the
camera odometry (/vo: yaw and yaw-rate), MOLA (/lidar_odometry/pose: yaw), the relay's
lidar speeds (/lidar_odom), the EKF (/odometry/filtered: x, y, yaw, speeds, yaw-rate),
SLAM's map -> odom correction (/tf), and the drive commands (/cmd_vel_nav: throttle, steer).
/tf is checked for "map" in the raw bytes before decoding: 160 messages a second otherwise.
"""
import math
import sys
import time

import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def main(bag, out):
    rows = {k: [] for k in ('imu', 'vo', 'mola', 'lidar', 'ekf', 'mapodom', 'cmd')}
    t0 = time.time()
    dec = DecoderFactory()
    decoders = {}
    with open(bag, 'rb') as f:
        r = make_reader(f)
        topics = ['/imu/data', '/vo', '/lidar_odometry/pose', '/lidar_odom', '/odometry/filtered', '/tf',
                  '/cmd_vel_nav']
        for schema, channel, msg in r.iter_messages(topics=topics):
            if channel.topic == '/tf' and b'map\x00' not in msg.data:
                continue
            d = decoders.get(channel.id)
            if d is None:
                d = decoders[channel.id] = dec.decoder_for(channel.message_encoding, schema)
            m = d(msg.data)
            tp = channel.topic
            if tp == '/imu/data':
                rows['imu'].append((stamp(m.header), m.angular_velocity.z, yaw_of(m.orientation)))
            elif tp == '/vo':
                rows['vo'].append((stamp(m.header), yaw_of(m.pose.pose.orientation), m.twist.twist.angular.z,
                                   m.twist.twist.linear.x))
            elif tp == '/lidar_odometry/pose':
                p = m.pose.pose
                rows['mola'].append((stamp(m.header), p.position.x, p.position.y, yaw_of(p.orientation)))
            elif tp == '/lidar_odom':
                rows['lidar'].append((stamp(m.header), m.twist.twist.linear.x, m.twist.twist.linear.y,
                                      m.twist.twist.angular.z))
            elif tp == '/odometry/filtered':
                p = m.pose.pose
                rows['ekf'].append((stamp(m.header), p.position.x, p.position.y, yaw_of(p.orientation),
                                    m.twist.twist.linear.x, m.twist.twist.angular.z))
            elif tp == '/cmd_vel_nav':
                rows['cmd'].append((msg.log_time * 1e-9, m.linear.x, m.angular.z))
            elif tp == '/tf':
                for tr in m.transforms:
                    if tr.header.frame_id.lstrip('/') == 'map' and tr.child_frame_id.lstrip('/') == 'odom':
                        t = tr.transform
                        rows['mapodom'].append((stamp(tr.header), t.translation.x, t.translation.y,
                                                yaw_of(t.rotation)))
    arrays = {k: np.array(v, dtype=np.float64) for k, v in rows.items()}
    np.savez_compressed(out, **arrays)
    print(f'{time.time() - t0:.0f} s: ' + ', '.join(f'{k} {len(v)}' for k, v in arrays.items()))


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
