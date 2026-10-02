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

"""How fast is each stream really running?

    ros2 run jetnano_bringup rates                    the watchdog's live numbers, every stream
    ros2 run jetnano_bringup rates /vo /scan          also measure these topics directly

Without arguments it prints what the watchdog measures all the time (its C++
counter is always connected, so its numbers are the ones to trust).

With topics it also measures them itself, the careful way. `ros2 topic hz`
prints "does not appear to be published yet" while it is still discovering
the publisher; a check that stops at that first line measures nothing
(2026-09-26: that is how a healthy visual odometry was reported as silent).
This waits for each topic's publisher, subscribes with the publisher's own
reliability, then counts messages for --seconds.
"""

import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String


def watchdog_numbers(node: Node, wait_s: float = 8.0):
    got = {}
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, '/watchdog/status', lambda m: got.setdefault('s', m.data), qos)
    end = time.monotonic() + wait_s
    while 's' not in got and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    return json.loads(got['s']) if 's' in got else None


def measure(node: Node, topic: str, seconds: float, discover_s: float = 15.0) -> str:
    end = time.monotonic() + discover_s
    pubs = []
    while time.monotonic() < end:
        pubs = node.get_publishers_info_by_topic(topic)
        if pubs:
            break
        rclpy.spin_once(node, timeout_sec=0.2)
    if not pubs:
        return f'no publisher found in {discover_s:.0f} s'
    msg_type = get_message(pubs[0].topic_type)
    reliable = all(p.qos_profile.reliability == ReliabilityPolicy.RELIABLE for p in pubs)
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT)
    stamps = []
    sub = node.create_subscription(msg_type, topic, lambda _m: stamps.append(time.monotonic()), qos,
                                   raw=True)
    # let the first message arrive before the clock starts
    settle = time.monotonic() + 5.0
    while not stamps and time.monotonic() < settle:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not stamps:
        node.destroy_subscription(sub)
        return f'publisher found ({len(pubs)}), but no message within 5 s'
    stamps.clear()
    start = time.monotonic()
    while time.monotonic() - start < seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    return f'{len(stamps) / seconds:.1f} Hz ({len(stamps)} messages in {seconds:.0f} s, ' \
           f'{"reliable" if reliable else "best effort"})'


def main(args=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('topics', nargs='*', help='topics to measure directly')
    ap.add_argument('--seconds', type=float, default=5.0)
    a = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init(args=args)
    node = Node('rates')
    try:
        status = watchdog_numbers(node)
        if status is None:
            print('watchdog: no status (is it running?)')
        else:
            print(f'watchdog says: {"OK" if status.get("ok") else "PROBLEMS"}'
                  + ('' if status.get('ok') else ' - ' + '; '.join(status.get('problems', []))))
            for key, s in status.get('streams', {}).items():
                age = s.get('age_s', -1)
                seen = 'never seen' if age < 0 else f'last {age:.1f} s ago'
                print(f'  {key:12s} {s.get("state", "?"):8s} {s.get("hz", 0):6.1f} Hz   {seen}')
        for topic in a.topics:
            print(f'{topic}: {measure(node, topic, a.seconds)}')
    finally:
        try:
            node.destroy_node()
        except (Exception, KeyboardInterrupt):  # noqa: BLE001 - the context is gone, or a second SIGINT, after an external shutdown
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
