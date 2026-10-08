#!/usr/bin/env python3
"""Drive Steve's house lap (drive.sh's LAP) in the sim and record it.

    python3 lap.py TAG [--laps N]      -> out/lap-TAG-K.jsonl per lap, summary line per lap

Like route_run: the Through planner, the lap cut into two stretches, the second sent when the first
ends (route_run swaps them on the move; here both laps stop once at waypoint 5). Records odometry, the commands
the sim receives and every new plan, in sim time.
"""
import argparse, json, math, os, time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from nav2_msgs.action import NavigateThroughPoses
from nav2_msgs.msg import CollisionMonitorState
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

D = os.path.dirname(os.path.abspath(__file__))
LAP = [(2.94, -3.48, -90), (1.2, -4.4, 180), (-0.45, -5.6, -90), (1.5, -7.3, 0), (4.5, -7.5, 0),
       (6.5, -7.7, 20), (7.05, -6.5, 90), (5.54, -5.49, 180), (3.14, -4.33, 110), (0.85, -0.81, 134)]
if os.environ.get('LAP'):                 # drive.sh's form: "x y h  x y h ..."
    v = [float(t) for t in os.environ['LAP'].split()]
    LAP = [tuple(v[i:i + 3]) for i in range(0, len(v), 3)]
START = tuple(float(v) for v in os.environ.get("START", "0.85 -0.81 -46").split())
_S = next((i for i, w in enumerate(LAP) if abs(w[0] - 4.5) < 0.01 and abs(w[1] + 7.5) < 0.01), len(LAP) // 2)   # split on the straight
STRETCHES = [LAP[:_S + 1], LAP[_S:]]


class Lap(Node):
    def __init__(self, out):
        super().__init__('lap', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.nav = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.tele = self.create_publisher(PoseStamped, '/sim/teleport', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.sel = self.create_publisher(String, '/planner_selector', latched)
        self.sel.publish(String(data='Through'))
        self.f = open(out, 'w')
        self.stats = {}
        self.create_subscription(Odometry, '/odometry/filtered', self.on_odom, 20)
        self.create_subscription(Twist, '/cmd_vel_nav_mps', self.on_cmd, 20)
        self.create_subscription(Path, '/plan', self.on_plan, 5)
        self.create_subscription(CollisionMonitorState, '/collision_monitor_state', self.on_mon, 10)   # its stops
        # what Nav2 ASKED (before the monitor) and the last scan, so a stop says what was in the zone
        self.create_subscription(Twist, '/cmd_vel_smoothed',
                                 lambda m: self.w({'k': 'a', 't': self.t(), 'v': round(m.linear.x, 3), 'wz': round(m.angular.z, 3)}), 20)
        self.scan = None
        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan', m), 5)
        self.create_subscription(String, '/sim/stats', lambda m: self.stats.update(json.loads(m.data)), 10)

    def t(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def w(self, rec):
        self.f.write(json.dumps(rec) + '\n')

    def on_odom(self, m):
        q = m.pose.pose.orientation
        self.w({'k': 'o', 't': m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                'x': round(m.pose.pose.position.x, 4), 'y': round(m.pose.pose.position.y, 4),
                'th': round(math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z), 5),
                'v': round(m.twist.twist.linear.x, 4), 'wz': round(m.twist.twist.angular.z, 4)})

    def on_mon(self, m):
        rec = {'k': 'm', 't': self.t(), 'a': int(m.action_type)}
        if m.action_type == 1 and self.scan is not None:   # points within 0.6 m ahead/behind, |y| < 0.25 (base frame)
            sc = self.scan
            pts = []
            for i, r in enumerate(sc.ranges):
                if math.isfinite(r):
                    a = sc.angle_min + i * sc.angle_increment
                    x, y = -0.005 + r * math.cos(a), r * math.sin(a)
                    if abs(x) < 0.6 and abs(y) < 0.25:
                        pts.append((round(x, 3), round(y, 3)))
            rec['pts'] = pts
        self.w(rec)

    def on_cmd(self, m):
        self.w({'k': 'c', 't': self.t(), 'v': round(m.linear.x, 4), 'wz': round(m.angular.z, 4)})

    def on_plan(self, m):
        self.w({'k': 'p', 't': self.t(), 'xy': [(round(p.pose.position.x, 3), round(p.pose.position.y, 3)) for p in m.poses[::2]]})

    def pose(self, x, y, h):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.pose.position.x, p.pose.position.y = float(x), float(y)
        p.pose.orientation.z, p.pose.orientation.w = math.sin(math.radians(h) / 2), math.cos(math.radians(h) / 2)
        return p

    def spin_for(self, s):
        end = time.monotonic() + s
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def send(self, stretch, fb=None):
        self.sel.publish(String(data='Through'))
        self.spin_for(0.5)
        g = NavigateThroughPoses.Goal()
        g.poses = [self.pose(*w) for w in stretch]
        f = self.nav.send_goal_async(g, feedback_callback=fb)
        while not f.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        return f.result()

    def run(self, limit_s=400.0):
        while self.tele.get_subscription_count() == 0:   # a teleport sent before the sim listens is lost
            self.spin_for(0.1)
        for _ in range(20):
            self.tele.publish(self.pose(*START))
            self.spin_for(0.3)
            if self.stats and math.hypot(self.stats['x'] - START[0], self.stats['y'] - START[1]) < 0.05:
                break
        self.spin_for(2.0)
        self.nav.wait_for_server()
        t0 = self.t()
        b0 = self.stats.get('bumps', 0)
        for k, stretch in enumerate(STRETCHES):   # one after the other (route_run swaps them on the move)
            h = self.send(stretch)
            res = h.get_result_async()
            while not res.done() and self.t() - t0 < limit_s:
                rclpy.spin_once(self, timeout_sec=0.05)
            if not res.done() or res.result().status != 4:
                break
        st = res.result().status if res.done() else -1
        return {'ok': st == 4, 'status': st, 't': round(self.t() - t0, 1), 'bumps': self.stats.get('bumps', 0) - b0}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('tag')
    ap.add_argument('--laps', type=int, default=1)
    a = ap.parse_args()
    rclpy.init()
    for k in range(a.laps):
        n = Lap(f'{D}/out/lap-{a.tag}-{k}.jsonl')
        r = n.run()
        n.f.close()
        print(json.dumps({'tag': a.tag, 'lap': k, **r}), flush=True)
        n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
