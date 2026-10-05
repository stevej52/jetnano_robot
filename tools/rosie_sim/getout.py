#!/usr/bin/env python3
"""getout: a multi-point "wiggle out of a tight spot" recovery for a car-like robot.

Nav2's BackUp only reverses straight; wedged nose-in between furniture that is often blocked
too, and a car cannot turn on the spot. This serves the BackUp action (nav2_msgs/BackUp) under
its own name, so the behaviour tree calls it with <BackUp server_name="getout" .../>:

  up to MOVES times: read the local costmap, try short arcs she can really drive (forward and
  reverse, full lock left / straight / full lock right, 0.15-0.40 m), keep those whose swept
  footprint stays off lethal cells, score each by the clearance it ends with and by how well it
  lines up with the current global plan, drive the best one on odometry, stop.

Drives on cmd_vel (Nav2's chain: velocity smoother, collision monitor) like Nav2's own behaviours.
"""
import math
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.action import BackUp
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from scipy import ndimage

R_MIN = 0.42                     # a little wider than her 0.40 lock: margin for the servos
HALF_L, HALF_W = 0.222 + 0.02, 0.148 + 0.02
SPEED = 0.15
MOVES = 3
LENGTHS = (0.15, 0.25, 0.40)


class GetOut(Node):
    def __init__(self):
        super().__init__('getout')
        cb = ReentrantCallbackGroup()
        self.grid = None
        self.odom = None
        self.plan = None
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.on_grid, 1, callback_group=cb)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_odom, 10, callback_group=cb)
        self.create_subscription(Path, '/plan', self.on_plan, 1, callback_group=cb)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        ActionServer(self, BackUp, 'getout', self.execute, callback_group=cb,
                     cancel_callback=lambda _: CancelResponse.ACCEPT)
        per = []
        for s in np.arange(-HALF_L, HALF_L + 1e-6, 0.04):
            per += [(s, HALF_W), (s, -HALF_W)]
        for s in np.arange(-HALF_W, HALF_W + 1e-6, 0.04):
            per += [(HALF_L, s), (-HALF_L, s)]
        self.per = np.array(per)
        self.lock = threading.Lock()

    def on_grid(self, m):
        with self.lock:
            self.grid = m

    def on_odom(self, m):
        q = m.pose.pose.orientation
        self.odom = (m.pose.pose.position.x, m.pose.pose.position.y, 2 * math.atan2(q.z, q.w))

    def on_plan(self, m):
        self.plan = m

    # --------------------------------------------------------------- choosing --
    def choose(self):
        with self.lock:
            g = self.grid
        if g is None or self.odom is None:
            return None, 'no costmap or odometry'
        w, h, res = g.info.width, g.info.height, g.info.resolution
        ox, oy = g.info.origin.position.x, g.info.origin.position.y
        a = np.array(g.data, dtype=np.int16).reshape(h, w)
        hard = a >= 100
        clear = ndimage.distance_transform_edt(~hard) * res       # to real obstacles, not the inflation band
        x0, y0, t0 = self.odom
        want = self.plan_heading(x0, y0)

        def cells(px, py):
            c = ((px - ox) / res).astype(int)
            r = ((py - oy) / res).astype(int)
            ok = (c >= 0) & (c < w) & (r >= 0) & (r < h)
            return r.clip(0, h - 1), c.clip(0, w - 1), ok

        def footprint_hits(x, y, t):
            ct, st = math.cos(t), math.sin(t)
            px = x + self.per[:, 0] * ct - self.per[:, 1] * st
            py = y + self.per[:, 0] * st + self.per[:, 1] * ct
            r, c, ok = cells(px, py)
            return bool(np.any(~ok | hard[r, c]))

        def end_clear(x, y, t):
            ct, st = math.cos(t), math.sin(t)
            px = x + self.per[:, 0] * ct - self.per[:, 1] * st
            py = y + self.per[:, 0] * st + self.per[:, 1] * ct
            r, c, ok = cells(px, py)
            return float(np.min(np.where(ok, clear[r, c], 0.0)))

        best = None
        here = end_clear(x0, y0, t0)
        for d in (1, -1):
            for k in (-1 / R_MIN, 0.0, 1 / R_MIN):
                for L in LENGTHS:
                    x, y, t = x0, y0, t0
                    okp = True
                    for s in np.arange(0.02, L + 1e-6, 0.02):
                        ds = 0.02 * d
                        t2 = t + ds * k
                        x += ds * math.cos((t + t2) / 2)
                        y += ds * math.sin((t + t2) / 2)
                        t = t2
                        if footprint_hits(x, y, t):
                            okp = False
                            break
                    if not okp:
                        continue
                    cl = end_clear(x, y, t)
                    align = math.cos(t - want) if want is not None else 0.0
                    score = 2.0 * min(cl, 0.5) + 0.4 * align + 0.05 * L
                    if best is None or score > best[0]:
                        best = (score, d, k, L, cl, align)
        if best is None:
            return None, f'no free move (clearance here {here:.2f} m)'
        return best, f'clearance {here:.2f} -> {best[4]:.2f} m'

    def plan_heading(self, x0, y0):
        p = self.plan
        if p is None or len(p.poses) < 2:
            return None
        pts = np.array([(q.pose.position.x, q.pose.position.y) for q in p.poses])
        i = int(np.argmin(np.hypot(pts[:, 0] - x0, pts[:, 1] - y0)))
        j = min(len(pts) - 1, i + 20)                        # ~1 m on: where the path is going
        if j == i:
            return None
        return math.atan2(pts[j, 1] - pts[i, 1], pts[j, 0] - pts[i, 0])

    # --------------------------------------------------------------- driving --
    def drive(self, d, k, L, goal_handle, deadline):
        x0, y0, _ = self.odom
        tw = Twist()
        tw.linear.x = d * SPEED
        tw.angular.z = d * SPEED * k
        moved = 0.0
        while moved < L and time.monotonic() < deadline and rclpy.ok():
            if goal_handle.is_cancel_requested:
                break
            self.pub.publish(tw)
            time.sleep(0.05)
            moved = math.hypot(self.odom[0] - x0, self.odom[1] - y0)
        self.pub.publish(Twist())
        return moved

    def execute(self, goal_handle):
        tl = goal_handle.request.time_allowance
        allow = tl.sec + tl.nanosec * 1e-9 or 20.0
        deadline = time.monotonic() + allow
        t0 = time.monotonic()
        total = 0.0
        # a fresh local costmap first: the tree clears it just before calling us
        called = self.get_clock().now().nanoseconds
        while time.monotonic() - t0 < 3.0:
            g = self.grid
            if g is not None and g.header.stamp.sec * 10**9 + g.header.stamp.nanosec > called + 300_000_000:
                break
            time.sleep(0.05)
        for i in range(MOVES):
            best, why = self.choose()
            if best is None:
                self.get_logger().warn(f'getout: move {i + 1}: {why}')
                break
            _, d, k, L, _, _ = best
            self.get_logger().info(f'getout: move {i + 1}: {"forward" if d > 0 else "reverse"} '
                                   f'{"left" if k > 0 else "right" if k < 0 else "straight"} {L:.2f} m ({why})')
            total += self.drive(d, k, L, goal_handle, deadline)
            time.sleep(0.3)
            if best[4] >= 0.35 or goal_handle.is_cancel_requested or time.monotonic() > deadline:
                break
        res = BackUp.Result()
        el = time.monotonic() - t0
        res.total_elapsed_time.sec = int(el)
        res.total_elapsed_time.nanosec = int((el % 1) * 1e9)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return res
        if total < 0.05:
            if hasattr(res, 'error_code'):
                res.error_code = 1
            goal_handle.abort()
            return res
        goal_handle.succeed()
        return res


def main():
    rclpy.init()
    n = GetOut()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(n)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
