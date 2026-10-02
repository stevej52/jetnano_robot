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

"""Her small housekeeping nodes in ONE process: motion_watch, guard_flow, battery_monitor.

    ros2 run jetnano_bringup housekeeping --ros-args --params-file config/housekeeping.yaml

Every Python ROS process costs ~80 MB and an executor of its own before it does anything;
ten of them were 940 MB (benchmark 2026-10-01). These three are light, timer-driven and
thread-free, so they share one executor here. Their node names, topics and parameters
are unchanged (the params file has a section per node), so the watchdog, the pages and
the tools see exactly what they saw before - only the process is one.
Steve, 2026-10-01: "to have 24 different watchdogs so that it can bring up stuff that
crashes just shows how often stuff crashes... fewer nodes."
"""

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

from jetnano_bringup.battery_monitor import BatteryMonitor
from jetnano_bringup.guard_flow import GuardFlow
from jetnano_bringup.motion_watch import MotionWatch


def main(args=None):
    rclpy.init(args=args)
    nodes = [MotionWatch(), GuardFlow(), BatteryMonitor()]
    ex = MultiThreadedExecutor(num_threads=3)
    for n in nodes:
        ex.add_node(n)
    nodes[0].get_logger().info('housekeeping: motion_watch, guard_flow and battery_monitor in one process')
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        for n in nodes:
            try:
                n.destroy_node()
            except (Exception, KeyboardInterrupt):  # noqa: BLE001 - the context is gone after an external shutdown
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
