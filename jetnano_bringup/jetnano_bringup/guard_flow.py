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

"""Does the collision guard let commands through? If not, why not, and fix it.

    ros2 run jetnano_bringup guard_flow        (robot.launch.py starts it)
    ros2 topic echo /guard_flow/status         "ok" | "holding: camera obstacles stale 73 s" | ...

Drive 16 (2026-10-01 19:31): the guard's camera source (/nvblox_node/obstacle_points) kept
arriving but with stamps frozen 73 s and more in the past; the guard - rightly - stopped
"due to invalid source" and from then on swallowed EVERY command, Steve's reverse on the
driving page included, for twelve minutes. The watchdog counts arrivals, so it saw nothing
wrong; nothing told Steve. This node judges the guard by what matters:

* flow: commands going in (/cmd_vel_mux) with nothing coming out (/cmd_vel) for 2 s;
* the camera source's stamps: older than 3 s while messages still arrive.

Either, held for 2 s: an "uh-oh", the reason on /guard_flow/status (the driving page shows it),
and the fix the watchdog would apply - restart grid_to_points (the launch respawns it);
if that does not clear it within 25 s, restart the launch inside the Isaac container
(nvblox's stamps come from the depth frames it integrates). At most one of each a minute.
"""

import os
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav2_msgs.msg import CollisionMonitorState
from sensor_msgs.msg import CameraInfo, PointCloud2
from std_msgs.msg import String

CONTAINER, CONTAINER_LAUNCH = 'isaac_vo', r'cuvslam_.*\.launch\.py'     # cuvslam_vo.sh picks the file
GRID_TO_POINTS = '/lib/jetnano_watchdog/grid_to_points'                    # the C++ one runs (drive.launch.py)


class GuardFlow(Node):

    def __init__(self):
        super().__init__('guard_flow')
        self.declare_parameter('act', True)
        self.declare_parameter('hold_s', 2.0)
        self.declare_parameter('stale_s', 3.0)
        self.mux_at = 0.0           # last non-zero command into the guard
        self.out_at = 0.0           # last non-zero command out of it
        self.pts_at = 0.0           # last obstacle_points arrival (monotonic)
        self.pts_age = 0.0          # its stamp's age then (s)
        self.bad_since = None
        self.why = ''
        self.fix_at = {'grid_to_points': 0.0, 'container': 0.0}
        self.fix_step = 0
        be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Twist, 'cmd_vel_mux', lambda m: self._cmd('mux_at', m), 10)
        self.create_subscription(Twist, 'cmd_vel', lambda m: self._cmd('out_at', m), 10)
        self.create_subscription(PointCloud2, '/nvblox_node/obstacle_points', self._on_points, be)
        # the depth stream behind nvblox, raw (the header's stamp is bytes 4-12 of the CDR; no
        # deserialising): when the grid goes stale this tells WHICH of three things happened -
        # no depth frames (the camera's stream died), frames with old stamps (the camera's clock
        # drifted off ROS time and nvblox refuses them), or fresh frames (nvblox itself stalled).
        # 2026-10-01 19:31: twelve minutes frozen, and the log could not say which.
        # ... from camera_info, which carries the same stamps at a few hundred bytes a frame (the
        # audit of 2026-10-02: the raw image was ~14 MB/s across the host for eight bytes)
        self.depth_n = 0
        self.depth_lag = 0.0
        self.depth_at = 0.0
        self.create_subscription(CameraInfo, '/camera/depth/camera_info', self._on_depth_info, be)
        # the guard's own verdict: a STOP for an obstacle is her doing her job, not a fault
        # the monitor publishes its state on CHANGE, not as a heartbeat (review 2026-10-03), so the
        # last state is held until the next; the polygon name tells an obstacle stop (a named
        # polygon) from a stop for an invalid source (no polygon)
        self.guard_action = 0
        self.guard_polygon = ''
        self.guard_at = 0.0
        self.create_subscription(CollisionMonitorState, 'collision_guard/state', self._on_guard_state, 10)
        # the camera's obstacles: stale or not is the C++ grid_to_points' verdict (camera_obstacles/
        # health), since it replaces a stale cloud with an empty one stamped now - the cloud's own
        # stamp no longer says anything about staleness
        self.camera_health = ''
        self.camera_health_at = 0.0
        self.create_subscription(String, 'camera_obstacles/health', self._on_camera_health, 10)
        # (the speed cap while the camera's obstacles are stale is the safety gate's, since
        # 2026-10-02 08:00: one owner of /speed_limit; this node reports)
        self.status_pub = self.create_publisher(String, 'guard_flow/status', 10)
        self.say_pub = self.create_publisher(String, 'say', 10)
        self.create_timer(0.5, self._tick)
        self.get_logger().info('watching the guard: commands in vs out, and its camera source\'s stamps')

    def _cmd(self, key, m):
        if abs(m.linear.x) > 0.02 or abs(m.angular.z) > 0.02:
            setattr(self, key, time.monotonic())

    def _on_depth_info(self, m):
        self.depth_n += 1
        self.depth_at = time.monotonic()
        if self.depth_n % 10 == 0:                 # every 10th frame is plenty for a lag figure
            self.depth_lag = self.get_clock().now().nanoseconds / 1e9 - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)

    def _on_guard_state(self, m):
        self.guard_action = int(m.action_type)
        self.guard_polygon = str(getattr(m, 'polygon_name', '') or '')
        self.guard_at = time.monotonic()

    def _on_camera_health(self, m):
        self.camera_health = m.data
        self.camera_health_at = time.monotonic()

    def depth_words(self) -> str:
        now = time.monotonic()
        if not self.depth_at:
            return 'no depth frames ever'
        if now - self.depth_at > 2.0:
            return f'no depth frames for {now - self.depth_at:.0f} s (the camera depth stream died)'
        if abs(self.depth_lag) > 2.0:
            return f'depth frames arrive but stamped {self.depth_lag:+.1f} s off ROS time (the camera clock)'
        return f'depth frames arrive, stamps fresh ({self.depth_lag * 1000:+.0f} ms): nvblox itself stalled'

    def _on_points(self, m):
        self.pts_at = time.monotonic()
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.pts_age = self.get_clock().now().nanoseconds / 1e9 - stamp

    def _tick(self):
        now = time.monotonic()
        hold = float(self.get_parameter('hold_s').value)
        why = ''
        obstacle_stop = self.guard_action == CollisionMonitorState.STOP and bool(self.guard_polygon)
        source_stop = self.guard_action == CollisionMonitorState.STOP and not self.guard_polygon
        if now - self.mux_at < 1.0 and now - self.out_at > hold and not obstacle_stop:
            why = ('commands go in, nothing comes out: the guard stopped for an invalid source' if source_stop
                   else 'commands go in, nothing comes out (and the guard reports no obstacle stop)')
        camera_stale = now - self.camera_health_at < 5.0 and self.camera_health.startswith('stale')
        if camera_stale:
            why = (why + '; ' if why else '') + f'camera obstacles {self.camera_health} - {self.depth_words()}'
        if why:
            if self.bad_since is None:
                self.bad_since = now
            if now - self.bad_since >= hold:
                self._bad(why, now)
        else:
            if self.bad_since is not None and now - self.bad_since >= hold:
                self.get_logger().info(f'the guard flows again ({self.why}) after {now - self.bad_since:.0f} s')
            self.bad_since, self.why, self.fix_step = None, '', 0
        msg = String()
        msg.data = 'ok' if not why else f'holding: {why}'
        self.status_pub.publish(msg)

    def _bad(self, why, now):
        if why != self.why:
            self.why = why
            self.get_logger().warning(f'the guard is holding everything: {why}')
            self.say_pub.publish(String(data='uhoh'))
        if not bool(self.get_parameter('act').value):
            return
        down = now - self.bad_since
        if self.fix_step == 0 and down >= 2.0 and now - self.fix_at['grid_to_points'] > 60.0:
            self.fix_step, self.fix_at['grid_to_points'] = 1, now
            self._signal(GRID_TO_POINTS, 'restarting grid_to_points (the launch respawns it)')
        elif self.fix_step == 1 and down >= 27.0 and now - self.fix_at['container'] > 120.0:
            self.fix_step, self.fix_at['container'] = 2, now
            self.get_logger().warning('still holding: restarting the launch in the Isaac container')
            try:
                r = subprocess.run(['docker', 'exec', CONTAINER, 'pkill', '-INT', '-f', CONTAINER_LAUNCH], timeout=15)
                if r.returncode != 0:
                    self.get_logger().error(f'container restart: pkill matched nothing for {CONTAINER_LAUNCH} (rc {r.returncode})')
            except (OSError, subprocess.TimeoutExpired) as exc:
                self.get_logger().warning(f'container restart: {exc}')

    def _signal(self, pattern, what):
        pids = subprocess.run(['pgrep', '-f', pattern], capture_output=True, text=True).stdout.split()
        self.get_logger().warning(f'{what}: pids {pids or "none (its respawn is due)"}')
        for p in pids:
            try:
                os.kill(int(p), signal.SIGINT)
            except (ProcessLookupError, ValueError):
                pass


def main(args=None):
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=args)
    node = GuardFlow()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except (Exception, KeyboardInterrupt):  # noqa: BLE001 - the context is gone, or a second SIGINT, after an external shutdown
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
