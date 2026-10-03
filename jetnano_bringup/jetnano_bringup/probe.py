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

"""Look at the robot without hurting it.

    ros2 run jetnano_bringup probe rates /odometry/filtered /vo /scan      Hz over --seconds (default 5)
    ros2 run jetnano_bringup probe echo /safety/state [--field data]      one message, as text, within --seconds
    ros2 run jetnano_bringup probe tf map base_footprint                  x y heading (deg), age
    ros2 run jetnano_bringup probe nodes                                  the graph's node names
    ros2 run jetnano_bringup probe cost X Y                               the planner's costmap at a map point

Why this and not `ros2 topic hz` / `echo` / `tf2_echo` in a shell with `timeout`:
2026-10-02 13:55 a probe killed by `timeout` left its Fast-DDS shared-memory port locked,
and the EKF's publisher hung behind that dead reader for five minutes. The CLI also measured
nothing at all while that port was locked ("Failed init_port ... open_and_lock_file"). This
tool subscribes with a QoS that matches any publisher (best effort, volatile), runs under
the UDP-only profile whatever the shell has, finishes on its own deadline, and shuts rclpy
down cleanly. Never run it under `timeout`; give it --seconds instead.
"""

import argparse
import json
import math
import os
import sys
import time

PROFILE = '/etc/jetnano/fastdds_udp_only.xml'
if os.path.exists(PROFILE):
    os.environ.setdefault('FASTRTPS_DEFAULT_PROFILES_FILE', PROFILE)

import rclpy  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402

ANY = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
# reliable on purpose: a late joiner only gets the latched sample over the reliable protocol
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def spin_for(node, seconds, until=lambda: False):
    end = time.monotonic() + seconds
    while time.monotonic() < end and not until():
        rclpy.spin_once(node, timeout_sec=0.05)


def topic_type(node, topic, wait_s):
    end = time.monotonic() + wait_s
    while time.monotonic() < end:
        for name, types in node.get_topic_names_and_types():
            if name == topic and types:
                return types[0]
        rclpy.spin_once(node, timeout_sec=0.2)
    return None


def rates(node, topics, seconds):
    counts = {t: 0 for t in topics}
    subs = []
    for t in topics:
        typ = topic_type(node, t, 5.0)
        if typ is None:
            print(f'{t}: no publisher found in 5 s')
            continue
        msg_cls = get_message(typ)
        subs.append(node.create_subscription(msg_cls, t, (lambda t: lambda m: counts.__setitem__(t, counts[t] + 1))(t), ANY))
        subs.append(node.create_subscription(msg_cls, t, (lambda t: lambda m: None)(t), LATCHED))   # a latched one too
    spin_for(node, seconds)
    for t in topics:
        print(f'{t}: {counts[t] / seconds:.1f} Hz ({counts[t]} in {seconds:g} s)')


def echo(node, topic, field, seconds):
    typ = topic_type(node, topic, min(seconds, 5.0))
    if typ is None:
        print(f'{topic}: no publisher found')
        return 1
    got = []
    msg_cls = get_message(typ)
    for qos in (ANY, LATCHED):
        node.create_subscription(msg_cls, topic, lambda m: got.append(m), qos)
    spin_for(node, seconds, lambda: bool(got))
    if not got:
        print(f'{topic}: nothing in {seconds:g} s')
        return 1
    m = got[0]
    for part in (field.split('.') if field else []):
        m = getattr(m, part)
    from rosidl_runtime_py import message_to_yaml
    print(message_to_yaml(m) if hasattr(m, 'get_fields_and_field_types') else m)
    return 0


def tf(node, parent, child, seconds):
    import tf2_ros
    buf = tf2_ros.Buffer()
    lis = tf2_ros.TransformListener(buf, node)
    spin_for(node, seconds, lambda: buf.can_transform(parent, child, rclpy.time.Time()))
    if not buf.can_transform(parent, child, rclpy.time.Time()):
        print(f'{parent} -> {child}: not available in {seconds:g} s')
        lis.unregister()
        return 1
    t = buf.lookup_transform(parent, child, rclpy.time.Time())
    q = t.transform.rotation
    yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
    age = node.get_clock().now().nanoseconds * 1e-9 - (t.header.stamp.sec + t.header.stamp.nanosec * 1e-9)
    print(f'{parent} -> {child}: x {t.transform.translation.x:+.3f}  y {t.transform.translation.y:+.3f}  heading {yaw:+.1f} deg  '
          f'(stamp {age:.2f} s old)')
    lis.unregister()
    return 0


def cost(node, x, y, seconds):
    from nav_msgs.msg import OccupancyGrid
    got = []
    node.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda m: got.append(m), LATCHED)
    spin_for(node, seconds, lambda: bool(got))
    if not got:
        print('no /global_costmap/costmap (is Nav2 up?)')
        return 1
    g = got[-1]
    i = g.info
    c, r = int((x - i.origin.position.x) / i.resolution), int((y - i.origin.position.y) / i.resolution)
    if not (0 <= c < i.width and 0 <= r < i.height):
        print(f'({x:+.2f}, {y:+.2f}) is outside the costmap')
        return 1
    v = g.data[r * i.width + c]
    name = {-1: 'unknown', 100: 'lethal', 99: 'inscribed'}.get(v, 'free' if v == 0 else f'cost {v}')
    print(f'({x:+.2f}, {y:+.2f}): {v} ({name}); costmap {i.width}x{i.height} at ({i.origin.position.x:.2f}, {i.origin.position.y:.2f})')
    return 0


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('what', choices=['rates', 'echo', 'tf', 'nodes', 'cost'])
    p.add_argument('args', nargs='*')
    p.add_argument('--seconds', type=float, default=5.0)
    p.add_argument('--field', default='')
    a = p.parse_args(argv)
    rclpy.init()
    node = rclpy.create_node(f'probe_{os.getpid()}')
    rc = 0
    try:
        if a.what == 'rates':
            rates(node, a.args, a.seconds)
        elif a.what == 'echo':
            rc = echo(node, a.args[0], a.field, a.seconds)
        elif a.what == 'tf':
            rc = tf(node, a.args[0], a.args[1], a.seconds)
        elif a.what == 'nodes':
            spin_for(node, 2.0)
            names = sorted(node.get_node_names())
            print(f'{len(names)} nodes: ' + ' '.join(names))
        elif a.what == 'cost':
            rc = cost(node, float(a.args[0]), float(a.args[1]), a.seconds)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
