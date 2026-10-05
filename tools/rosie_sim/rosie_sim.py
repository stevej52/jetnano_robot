#!/usr/bin/env python3
"""Rosie in a box: a kinematic stand-in for the crawler on the house map, for Nav2 tours.

Takes what Nav2's collision monitor sends her (cmd_vel_nav_mps, m/s and rad/s), drives a
car with her turning circle (0.40 m, four-wheel steering, no turning on the spot), lags the
speed and the steering like the real ones, refuses motion into the map's walls (a "bump"),
and publishes a sped-up /clock, odometry + TF and a lidar scan ray-cast in the map.

Runs on its own ROS domain, localhost only (tour.sh sets it), so it can never reach Rosie.
"""
import json
import math
import threading
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from PIL import Image
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

R_MIN = 0.40            # m, her turning circle (planner's minimum_turning_radius)
K_MAX = 1.0 / R_MIN
TAU_V = 0.25            # s, speed lag
K_RATE = 8.0            # 1/m per s: the steering servos' slew
HALF_L, HALF_W = 0.222, 0.148   # her body (footprint without padding)
LIDAR_X = -0.005
CMD_STALE = 0.5         # s of sim time without a command = stop


class Sim(Node):
    def __init__(self):
        super().__init__('rosie_sim')
        self.speed = float(self.declare_parameter('speed', 3.0).value)
        map_yaml = self.declare_parameter('map', '/home/steve/rosie-sim/map/home.yaml').value
        self.x = float(self.declare_parameter('x', 0.0).value)
        self.y = float(self.declare_parameter('y', 0.0).value)
        self.th = math.radians(float(self.declare_parameter('heading_deg', 0.0).value))
        m = yaml.safe_load(open(map_yaml))
        img = np.array(Image.open(map_yaml.rsplit('/', 1)[0] + '/' + m['image']))
        self.res = float(m['resolution'])
        self.ox, self.oy = float(m['origin'][0]), float(m['origin'][1])
        self.h, self.w = img.shape
        self.blocked = img != 254            # walls and unknown both stop her
        self.v = self.k = 0.0
        self.cmd = (0.0, 0.0, -1e9)
        self.t = 0.0
        self.odo = 0.0
        self.reversals = 0
        self.last_sign = 0
        self.bumps = 0
        self.bumping = False
        self.last_bump = None
        self.lock = threading.Lock()
        perim = []
        for s in np.arange(-HALF_L, HALF_L + 1e-6, 0.03):
            perim += [(s, HALF_W), (s, -HALF_W)]
        for s in np.arange(-HALF_W, HALF_W + 1e-6, 0.03):
            perim += [(HALF_L, s), (-HALF_L, s)]
        self.perim = np.array(perim)
        self.ang = np.linspace(-math.pi, math.pi, 360, endpoint=False)
        self.rr = np.arange(0.12, 8.0, 0.025)

        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.odom_pubs = [self.create_publisher(Odometry, t, 10) for t in ('/odom', '/odometry/filtered')]
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.stats_pub = self.create_publisher(String, '/sim/stats', 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Twist, '/cmd_vel_nav_mps', self.on_cmd, 10)
        self.create_subscription(PoseStamped, '/sim/teleport', self.on_teleport, 10)
        st = StaticTransformBroadcaster(self)
        st.sendTransform([self._tf('map', 'odom', 0, 0, 0, 0), self._tf('base_footprint', 'base_link', 0, 0, 0, 0),
                          self._tf('base_link', 'sim_laser', LIDAR_X, 0, 0.198, 0)])
        threading.Thread(target=self.loop, daemon=True).start()

    def _tf(self, parent, child, x, y, z, th, stamp=None):
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = parent, child
        if stamp is not None:
            t.header.stamp = stamp
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = float(x), float(y), float(z)
        t.transform.rotation.z, t.transform.rotation.w = math.sin(th / 2), math.cos(th / 2)
        return t

    def on_cmd(self, m):
        with self.lock:
            self.cmd = (m.linear.x, m.angular.z, self.t)

    def on_teleport(self, m):
        q = m.pose.orientation
        with self.lock:
            self.x, self.y = m.pose.position.x, m.pose.position.y
            self.th = 2 * math.atan2(q.z, q.w)
            self.v = self.k = 0.0
            self.cmd = (0.0, 0.0, -1e9)
            self.bumping = False

    def cells(self, xs, ys):
        c = np.floor((xs - self.ox) / self.res).astype(int)
        r = self.h - 1 - np.floor((ys - self.oy) / self.res).astype(int)
        inside = (c >= 0) & (c < self.w) & (r >= 0) & (r < self.h)
        return r.clip(0, self.h - 1), c.clip(0, self.w - 1), inside

    def hits(self, x, y, th):
        ct, s = math.cos(th), math.sin(th)
        px = x + self.perim[:, 0] * ct - self.perim[:, 1] * s
        py = y + self.perim[:, 0] * s + self.perim[:, 1] * ct
        r, c, inside = self.cells(px, py)
        return bool(np.any(~inside | self.blocked[r, c]))

    def step(self, dt):
        vc, wc, at = self.cmd
        if self.t - at > CMD_STALE:
            vc, wc = 0.0, 0.0
        self.v += (vc - self.v) * min(1.0, dt / TAU_V)
        if abs(vc) < 0.02 and abs(self.v) < 0.03:
            self.v = 0.0
        if abs(vc) > 0.01:
            kc = wc / vc
        elif abs(wc) > 1e-3:
            kc = math.copysign(K_MAX, wc)     # wheels turn while standing, she does not
        else:
            kc = self.k
        kc = max(-K_MAX, min(K_MAX, kc))
        dk = max(-K_RATE * dt, min(K_RATE * dt, kc - self.k))
        self.k += dk
        dth = self.v * self.k * dt
        nth = self.th + dth
        nx = self.x + self.v * dt * math.cos(self.th + dth / 2)
        ny = self.y + self.v * dt * math.sin(self.th + dth / 2)
        if self.v != 0.0 and self.hits(nx, ny, nth):
            if not self.bumping:
                self.bumps += 1
                self.last_bump = (round(self.x, 2), round(self.y, 2))
            self.bumping = True
            self.v = 0.0
            return
        self.bumping = False
        sign = (self.v > 0.02) - (self.v < -0.02)
        if sign and self.last_sign and sign != self.last_sign:
            self.reversals += 1
        if sign:
            self.last_sign = sign
        self.odo += abs(self.v) * dt
        self.x, self.y, self.th = nx, ny, math.atan2(math.sin(nth), math.cos(nth))

    def scan(self, stamp):
        lx = self.x + LIDAR_X * math.cos(self.th)
        ly = self.y + LIDAR_X * math.sin(self.th)
        a = self.ang + self.th
        xs = lx + self.rr[:, None] * np.cos(a)[None, :]
        ys = ly + self.rr[:, None] * np.sin(a)[None, :]
        r, c, inside = self.cells(xs, ys)
        hit = inside & self.blocked[r, c]
        first = np.argmax(hit, axis=0)
        any_hit = hit[first, np.arange(len(self.ang))]
        ranges = np.where(any_hit, self.rr[first], np.inf).astype(np.float32)
        m = LaserScan()
        m.header.stamp, m.header.frame_id = stamp, 'sim_laser'
        m.angle_min, m.angle_max = float(self.ang[0]), float(self.ang[-1])
        m.angle_increment = float(self.ang[1] - self.ang[0])
        m.range_min, m.range_max = 0.12, 8.0
        m.scan_time = 0.1
        m.ranges = ranges.tolist()
        self.scan_pub.publish(m)

    def loop(self):
        wall_dt = 0.01
        dt = wall_dt * self.speed
        n = 0
        nxt = time.monotonic()
        while rclpy.ok():
            with self.lock:
                self.step(dt)
                self.t += dt
                x, y, th, v, w = self.x, self.y, self.th, self.v, self.v * self.k
            sec = int(self.t)
            stamp = rclpy.time.Time(seconds=sec, nanoseconds=int((self.t - sec) * 1e9)).to_msg()
            self.clock_pub.publish(Clock(clock=stamp))
            if n % 2 == 0:
                self.tf.sendTransform(self._tf('odom', 'base_footprint', x, y, 0, th, stamp))
                o = Odometry()
                o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, 'odom', 'base_footprint'
                o.pose.pose.position.x, o.pose.pose.position.y = x, y
                o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
                o.twist.twist.linear.x, o.twist.twist.angular.z = v, w
                for p in self.odom_pubs:
                    p.publish(o)
            if n % max(1, round(0.1 / dt)) == 0:
                self.scan(stamp)
            if n % max(1, round(0.5 / dt)) == 0:
                self.stats_pub.publish(String(data=json.dumps({
                    't': round(self.t, 2), 'x': round(x, 3), 'y': round(y, 3), 'th': round(th, 3),
                    'odo': round(self.odo, 2), 'reversals': self.reversals, 'bumps': self.bumps,
                    'last_bump': self.last_bump})))
            n += 1
            nxt += wall_dt
            time.sleep(max(0.0, nxt - time.monotonic()))


def main():
    rclpy.init()
    node = Sim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
