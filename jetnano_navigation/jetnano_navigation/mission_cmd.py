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

"""A client of the mission controller (mission.py): send one command, print its story
(/mission/log: what nav_route printed - a line a second, waypoints passed, detours,
pictures) and the result line, exit 0 on SUCCEEDED.

    ros2 run jetnano_navigation mission_cmd goal X Y HEADING_DEG [TIMEOUT_S] [--exact]   (--exact: arrive facing HEADING)
    ros2 run jetnano_navigation mission_cmd route X Y H  X Y H ...  [--timeout S] [--park X Y H]
        --park: the lap's end is handed to the parking server while she rolls (her spot: 0 0 0)
    ros2 run jetnano_navigation mission_cmd cancel | resume | status
"""

import json
import sys
import time
import uuid

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if not argv:
        print(__doc__)
        return 2
    do = argv[0]
    timeout = None
    if '--timeout' in argv:
        i = argv.index('--timeout')
        timeout = float(argv[i + 1])
        del argv[i:i + 2]
    park = None
    if '--park' in argv:
        i = argv.index('--park')
        park = {'x': float(argv[i + 1]), 'y': float(argv[i + 2]), 'heading_deg': float(argv[i + 3])}
        del argv[i:i + 4]
    exact = '--exact' in argv          # arrive facing exactly HEADING (the old goal); else any heading
    if exact:
        argv.remove('--exact')
    if do == 'goal':
        x, y, h = (float(v) for v in argv[1:4])
        cmd = {'do': 'goal', 'x': x, 'y': y, 'heading_deg': h}
        if exact:
            cmd['exact'] = True
        if len(argv) > 4:
            timeout = float(argv[4])
    elif do == 'route':
        nums = [float(v) for v in argv[1:]]
        if not nums or len(nums) % 3:
            print('a route is X Y HEADING_DEG triplets: ' + ' '.join(argv[1:]))
            return 2
        cmd = {'do': 'route', 'waypoints': [nums[i:i + 3] for i in range(0, len(nums), 3)]}
        if park:
            cmd['then_park'] = park
    elif do in ('cancel', 'resume', 'status'):
        cmd = {'do': do}
    else:
        print(__doc__)
        return 2
    if timeout is not None:
        cmd['timeout_s'] = timeout
    mid = uuid.uuid4().hex[:8]
    cmd['id'] = mid

    rclpy.init()
    n = rclpy.create_node('mission_cmd')
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    got = {'status': None, 'result': None, 'logged_result': False}
    n.create_subscription(String, 'mission/status', lambda m: got.__setitem__('status', json.loads(m.data)), latched)

    def on_result(m):
        r = json.loads(m.data)
        if r.get('id') == mid:
            got['result'] = r

    def on_log(m):
        line = json.loads(m.data)
        if line.get('id') == mid:
            print(line['line'], flush=True)
            got['logged_result'] = got['logged_result'] or line['line'].startswith('result ')

    n.create_subscription(String, 'mission/result', on_result, 10)
    n.create_subscription(String, 'mission/log', on_log, 50)
    pub = n.create_publisher(String, 'mission/command', 10)
    end = time.monotonic() + 5.0
    while pub.get_subscription_count() == 0 and time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        print('no mission controller is listening (is Nav2 up? navigation.launch.py starts it)', flush=True)
        return 3
    pub.publish(String(data=json.dumps(cmd)))
    print(f'-> {json.dumps(cmd)}', flush=True)
    if do in ('cancel', 'resume', 'status'):
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            rclpy.spin_once(n, timeout_sec=0.1)
        print(json.dumps(got['status']) if got['status'] else 'no status', flush=True)
        return 0
    t0 = time.monotonic()
    last = 0.0
    outcome = None
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.1)
        r = got['result']
        if r is not None:
            if not got['logged_result']:
                print(r['line'], flush=True)          # a route's result line came through the log already
            outcome = r['outcome']
            break
        if do == 'goal' and time.monotonic() - last >= 1.0:
            last = time.monotonic()
            s = got['status'] or {}
            fb = s.get('feedback') or {}
            held = s.get('held')
            print(f'{time.monotonic() - t0:5.1f} s  {s.get("mission", "?")}/{s.get("phase", "?")}'
                  + (f'  at x {fb["x"]:+.2f} y {fb["y"]:+.2f} h {fb["h_deg"]:+4.0f}  {fb["dist_m"]:.2f} m' if 'x' in fb else '')
                  + (f'  HELD {held["since_s"]:.0f} s: {held["why"]} - {held["next"]}' if held else '')
                  + f'  gate {s.get("gate", "?")}', flush=True)
    n.destroy_node()
    rclpy.shutdown()
    return 0 if outcome == 'SUCCEEDED' else 1


if __name__ == '__main__':
    sys.exit(main())
