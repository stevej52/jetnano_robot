# Copyright 2026 stevej52
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Is the servo rail live? Nothing on her measures it (the one INA219 is on the Jetson's side of
the battery switch, and Steve keeps it there, 2026-10-01), so she looks: a head-camera frame,
a small pan, another frame. If the picture moved, the servos have power.

    ros2 run jetnano_bringup rail_check          -> "servo rail LIVE" / "servo rail DEAD", exit 0 / 1
    from jetnano_bringup.rail_check import rail_live;  rail_live(node) -> True / False / None

Head only, never the wheels. About 1.5 s when the head camera is already streaming (someone
watching the dashboard), 3-4 s cold (csi_cameras starts the pipeline on the first viewer).
"""

import sys
import time
import urllib.request

import cv2
import numpy as np
import rclpy
from std_msgs.msg import Float64

HEAD_URL = 'http://127.0.0.1:8082/front.mjpg'
PAN_HOME = 74.6            # pca9685.yaml: dead ahead, higher = left
PAN_STEP = 15.0            # degrees: enough to move the picture ~130 px, small enough to be quick
MOVED_PX = 15.0            # a pan of 15 deg shifts the picture far more than this; still is < 1
SETTLE_S = 0.45            # a hobby servo covers 15 deg in ~0.1 s; the stream lags ~0.2 s


def frame(timeout_s=8.0):
    """One greyscale frame out of the head camera's MJPEG stream, or None."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            with urllib.request.urlopen(HEAD_URL, timeout=4) as r:
                buf = b''
                while len(buf) < 600000 and time.monotonic() < deadline:
                    buf += r.read(8192)
                    a = buf.find(b'\xff\xd8')
                    b = buf.find(b'\xff\xd9', a + 2) if a >= 0 else -1
                    if a >= 0 and b > a:
                        img = cv2.imdecode(np.frombuffer(buf[a:b + 2], np.uint8), cv2.IMREAD_GRAYSCALE)
                        if img is not None:
                            return img
                        buf = buf[b + 2:]
        except Exception:  # noqa: BLE001
            time.sleep(0.3)
    return None


def shift_px(a, b):
    """How far the picture moved sideways between two frames (phase correlation, pixels)."""
    a = cv2.resize(a, (320, 240)).astype(np.float32)
    b = cv2.resize(b, (320, 240)).astype(np.float32)
    (dx, _dy), _resp = cv2.phaseCorrelate(a, b)
    return abs(dx) * 2


def rail_live(node, log=print):
    """True if the head moves, False if it does not, None if there was no picture to judge by."""
    pub = node.create_publisher(Float64, '/pca9685/pan/angle', 10)
    t0 = time.monotonic()
    before = frame()
    if before is None:
        log('rail_check: no head camera picture')
        return None
    t1 = time.monotonic()
    pub.publish(Float64(data=PAN_HOME + PAN_STEP))
    time.sleep(SETTLE_S)
    after = frame()
    pub.publish(Float64(data=PAN_HOME))
    if after is None:
        return None
    moved = shift_px(before, after)
    live = moved > MOVED_PX
    log(f'rail_check: picture moved {moved:.0f} px on a {PAN_STEP:.0f} deg pan -> servo rail '
        f'{"LIVE" if live else "DEAD"} ({t1 - t0:.1f} s for the first frame, {time.monotonic() - t0:.1f} s in all)')
    time.sleep(SETTLE_S)      # home before anyone else moves it
    node.destroy_publisher(pub)
    return live


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('rail_check')
    try:
        live = rail_live(node)
    finally:
        try:
            node.destroy_node()
        except (Exception, KeyboardInterrupt):  # noqa: BLE001 - the context is gone, or a second SIGINT, after an external shutdown
            pass
        rclpy.shutdown()
    print('servo rail ' + ('LIVE' if live else 'DEAD' if live is False else 'UNKNOWN (no picture)'))
    return 0 if live else 1


if __name__ == '__main__':
    sys.exit(main())
