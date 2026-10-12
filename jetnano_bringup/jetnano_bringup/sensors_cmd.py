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

"""Ask sensor_sleep to rest or wake the lidar motor and the camera projector.

    ros2 run jetnano_bringup sensors status
    ros2 run jetnano_bringup sensors sleep
    ros2 run jetnano_bringup sensors wake        (waits until the scans flow again; drive.sh uses it)

Exits 0 when sensor_sleep is not running, so a drive is never blocked by it.
"""

import argparse
import json
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger


def main(args=None):
    ap = argparse.ArgumentParser(description='rest or wake the lidar motor and the camera projector')
    ap.add_argument('what', choices=('status', 'sleep', 'wake'), nargs='?', default='status')
    ap.add_argument('--timeout', type=float, default=25.0)
    opts, ros_args = ap.parse_known_args(args)
    rclpy.init(args=ros_args)
    n = rclpy.create_node('sensors_cmd')
    state = {}
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    n.create_subscription(String, 'sensors/state', lambda m: state.update(json.loads(m.data)), latched)
    try:
        if opts.what != 'status':
            cli = n.create_client(Trigger, f'sensor_sleep/{opts.what}')
            if not cli.wait_for_service(timeout_sec=5.0):
                print('sensors: sensor_sleep is not running (sensors stay as they are)')
                return
            fut = cli.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(n, fut, timeout_sec=5.0)
        want = {'sleep': 'asleep', 'wake': 'awake'}.get(opts.what)
        end = time.monotonic() + (opts.timeout if opts.what == 'wake' else 5.0)
        while time.monotonic() < end:
            rclpy.spin_once(n, timeout_sec=0.2)
            if state and (want is None or state.get('state') == want):
                break
        if not state:
            print('sensors: no state from sensor_sleep')
        else:
            print(f'sensors {state.get("state")}: {state.get("why", "")}'
                  + (' (on the bench)' if state.get('bench') else '') + f', idle {state.get("idle_s", 0)} s')
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
