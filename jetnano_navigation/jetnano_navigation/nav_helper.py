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

"""Help for when Rosie is stuck: back out along her own track, and ask Claude with pictures.

Runs beside Nav2 (navigation.launch.py). Services (std_srvs/Trigger):

    ~/retrace    back out along the track she drove in on, `retrace_m` metres (a parameter),
                 on Nav2's own command topic (cmd_vel_nav_raw): the velocity smoother, the
                 collision monitor, the translator, twist_mux, the e-stop lock and the
                 collision guard all apply, and with Nav2 not running nothing moves. The track
                 is always free - she was just there - which a blind straight reverse is not
                 (2026-09-29: wedged at the kitchen island's far corner, "backup failed").
    ~/snapshot   the pictures only, in `out_dir`/<time>/: the house map round her, a close-up of
                 the obstacles (lidar and camera, her front up) and a sheet of camera views - a
                 pan-tilt sweep, the depth camera, the rear camera.
    ~/ask        the pictures to Claude through the brain (brain/look: the key and the day's
                 budget stay there), with what she was trying to do (nav_helper/goal,
                 nav_helper/situation); the reply is checked (stuck_help.parse_advice) and
                 returned as JSON in the response message.

Steve, 2026-09-29: "when she comes to an obstacle and she's not sure what to do ... upload it
to her AI ... a picture of the map and where it's at and the obstacle map ... pan and tilt ...
several pictures". nav_goal --rescue uses all three.
"""

import collections
import json
import math
import os
import threading
import time
import urllib.error
import urllib.request
import uuid

from action_msgs.msg import GoalStatus, GoalStatusArray
import cv2
from geometry_msgs.msg import PoseStamped, Twist
from jetnano_navigation import stuck_help as sh
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import numpy as np
import rclpy
from rclpy._rclpy_pybind11 import InvalidHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_action_status_default,
                       qos_profile_sensor_data)
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

MAP_FRAME_CDR = b'\x04\x00\x00\x00map\x00'           # a TFMessage naming "map": worth decoding


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def compose(a, b):
    """Pose b (x, y, yaw) given in frame a -> in a's parent."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], sh.wrap(a[2] + b[2]))


class NavHelper(Node):

    def __init__(self):
        super().__init__('nav_helper')
        p = self.declare_parameter
        p('retrace_m', 0.8)
        p('retrace_speed', 0.15)                 # m/s, backwards
        p('lookahead_m', 0.35)
        p('max_curvature', 2.5)                  # 1 / 0.40 m, her tightest turn
        p('track_step_m', 0.05)
        p('track_max_m', 12.0)
        p('cmd_topic', 'cmd_vel_nav_raw')
        p('web_url', 'http://127.0.0.1:8081')    # web_teleop: /look, /status
        p('cameras_url', 'http://127.0.0.1:8082')  # csi_cameras: front.jpg (pan-tilt), rear.jpg, d435.jpg
        p('sweep_pan_deg', [-69.0, -35.0, 0.0, 40.0, 85.0, 0.0])    # view degrees, + = left
        p('sweep_tilt_deg', [0.0, 0.0, 0.0, 0.0, 0.0, -40.0])       # + = up
        p('sweep_settle_s', 0.9)
        p('sweep', True)                         # false: no pan-tilt sweep (quiet, or no pan-tilt)
        p('out_dir', '/tmp/nav_helper')
        p('ask_timeout_s', 150.0)
        p('ask_effort', 'medium')
        p('odom_linger_s', 30.0)                 # odometry kept this long after the last Nav2 goal
        self.gp = lambda name: self.get_parameter(name).value

        # 2026-09-29: subscribed to everything all the time, the node took 30-36 % of a core
        # parked - its executor woke 145 times a second (odometry 100/s, the camera grid 36/s,
        # the scan 8/s). Now the scan and the grid are fetched when a packet is built, and the
        # odometry (her track) is followed only while Nav2 has a goal, and a while after.
        self.subs = MutuallyExclusiveCallbackGroup()
        subs = self.subs
        self.work = ReentrantCallbackGroup()
        self.pose = None                         # odom (x, y, yaw), <= 20 a second
        self._pose_t = 0.0
        self.track = collections.deque()         # odom (x, y), oldest first
        self.raw = {}                            # the latest plan, still serialized
        self.map = None
        self.goal = None
        self.situation = ''
        self.lock = False
        self.answers = {}
        self._odom_sub = None
        self._fresh = False
        self._odom_lock = threading.Lock()
        # one pan-tilt sweep at a time: drive 41 (2026-10-06) ran two snapshots' sweeps at once, the
        # second took the first's 85 deg left as "where it was" and left the head there
        self._sweep_lock = threading.Lock()
        self._d435_rotate = None                # from the camera server's config.json, on first use
        self._active_t = -1e9                    # when Nav2 last had a goal (monotonic)
        self._busy = 0                           # retrace / packet in progress: keep the odometry
        self.cmd = self.create_publisher(Twist, str(self.gp('cmd_topic')), 10)
        self.look_pub = self.create_publisher(String, 'brain/look', 10)
        self.create_subscription(GoalStatusArray, 'navigate_to_pose/_action/status', self._on_status,
                                 qos_profile_action_status_default, callback_group=subs)
        self.create_timer(2.0, self._odom_idle, callback_group=subs)
        self.create_subscription(Path, 'plan', lambda r: self.raw.__setitem__('plan', r), 2, raw=True,
                                 callback_group=subs)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, 'map', lambda m: setattr(self, 'map', m), latched,
                                 callback_group=subs)
        self.create_subscription(PoseStamped, 'nav_helper/goal', self._on_goal, 10, callback_group=subs)
        self.create_subscription(String, 'nav_helper/situation', lambda m: setattr(self, 'situation', m.data), 10,
                                 callback_group=subs)
        self.create_subscription(Bool, 'e_stop_motion', lambda m: setattr(self, 'lock', bool(m.data)), 10,
                                 callback_group=subs)
        self.create_subscription(String, 'brain/look_answer', self._on_answer, 10, callback_group=subs)
        self.create_service(Trigger, '~/retrace', self._retrace, callback_group=self.work)
        self.create_service(Trigger, '~/snapshot', self._snapshot, callback_group=self.work)
        self.create_service(Trigger, '~/ask', self._ask, callback_group=self.work)
        self.get_logger().info('ready: ~/retrace, ~/snapshot, ~/ask')

    # ---------------------------------------------------------------- inputs --

    def _on_goal(self, msg):
        self.goal = msg
        self._active_t = time.monotonic()
        self._odom_on()

    def _on_status(self, msg):
        if any(s.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING)
               for s in msg.status_list):
            self._active_t = time.monotonic()
            self._odom_on()

    def _odom_on(self):
        with self._odom_lock:
            if self._odom_sub is None:
                self._fresh = True
                self._odom_sub = self.create_subscription(Odometry, 'odometry/filtered', self._on_odom, 20,
                                                          raw=True, callback_group=self.subs)

    def _odom_idle(self):
        with self._odom_lock:
            if (self._odom_sub is not None and not self._busy
                    and time.monotonic() - self._active_t > float(self.gp('odom_linger_s'))):
                self.destroy_subscription(self._odom_sub)
                self._odom_sub = None
                self.pose = None                 # stale from here on

    def _fresh_pose(self, wait_s=2.0):
        """Odometry on (if Nav2 had no goal lately) and a pose from it."""
        self._odom_on()
        end = time.monotonic() + wait_s
        while self.pose is None and time.monotonic() < end:
            time.sleep(0.05)
        return self.pose

    def _on_odom(self, raw):
        now = time.monotonic()
        if now - self._pose_t < 0.05:
            return
        self._pose_t = now
        m = deserialize_message(raw, Odometry)
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        tr = self.track
        if self._fresh:                          # first pose after a gap: did she move meanwhile?
            self._fresh = False
            if tr and math.hypot(tr[-1][0] - self.pose[0], tr[-1][1] - self.pose[1]) > 0.3:
                tr.clear()                       # driven by hand: that track no longer leads here
        if not tr or math.hypot(tr[-1][0] - self.pose[0], tr[-1][1] - self.pose[1]) >= float(self.gp('track_step_m')):
            tr.append((self.pose[0], self.pose[1]))
            while len(tr) * float(self.gp('track_step_m')) > float(self.gp('track_max_m')):
                tr.popleft()

    def _on_answer(self, msg):
        try:
            a = json.loads(msg.data)
        except ValueError:
            return
        ev = self.answers.get(a.get('id'))
        if isinstance(ev, threading.Event):
            self.answers[a['id']] = a
            ev.set()

    def _map_to_odom(self, wait_s=2.0):
        """SLAM's map -> odom, from /tf for a moment (a standing TF listener costs a Python node
        about a third of a core, 2026-09-28)."""
        found = []

        def on_tf(raw):
            if MAP_FRAME_CDR in raw:
                for t in deserialize_message(raw, TFMessage).transforms:
                    if t.header.frame_id.lstrip('/') == 'map' and t.child_frame_id.lstrip('/') == 'odom':
                        found.append((t.transform.translation.x, t.transform.translation.y,
                                      yaw_of(t.transform.rotation)))
        sub = self.create_subscription(TFMessage, '/tf', on_tf, 50, raw=True, callback_group=self.work)
        end = time.monotonic() + wait_s
        while not found and time.monotonic() < end:
            time.sleep(0.05)
        self.destroy_subscription(sub)
        return found[-1] if found else None

    def _latest(self, msg_type, topic, qos, wait_s=2.0):
        """One message from `topic`, subscribed only for that moment; None if none came."""
        got = []
        sub = self.create_subscription(msg_type, topic, lambda r: got.append(r), qos, raw=True,
                                       callback_group=self.work)
        end = time.monotonic() + wait_s
        while not got and time.monotonic() < end:
            time.sleep(0.05)
        self.destroy_subscription(sub)
        return deserialize_message(got[-1], msg_type) if got else None

    # ---------------------------------------------------------------- retrace --

    def _send(self, v, w):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.cmd.publish(t)

    def _retrace(self, request, response):
        self._busy += 1
        try:
            return self._retrace_work(response)
        finally:
            self._busy -= 1

    def _retrace_work(self, response):
        metres = float(self.gp('retrace_m'))
        speed, look = float(self.gp('retrace_speed')), float(self.gp('lookahead_m'))
        kmax = float(self.gp('max_curvature'))
        self._fresh_pose()
        track = list(self.track)
        if self.pose is None or len(track) < 3:
            response.success, response.message = False, 'no track to back out along yet'
            return response
        i = len(track) - 1
        done, last = 0.0, self.pose
        t0 = time.monotonic()
        moved_at, moved_from = t0, self.pose
        why = None
        while done < metres:
            if self.lock:
                why = 'the motion lock stopped her'
                break
            if time.monotonic() - t0 > 30.0:
                why = 'took over 30 s'
                break
            pose = self.pose
            i = sh.nearest_behind(track, pose, i)
            tgt = sh.track_behind(track, pose, look, i)
            if tgt is None:
                why = 'reached the start of her track'
                break
            k = max(-kmax, min(kmax, sh.reverse_curvature(pose, tgt[1])))
            self._send(-speed, -speed * k)
            time.sleep(0.05)
            now = self.pose
            done += math.hypot(now[0] - last[0], now[1] - last[1])
            last = now
            if math.hypot(now[0] - moved_from[0], now[1] - moved_from[1]) > 0.03:
                moved_at, moved_from = time.monotonic(), now
            elif time.monotonic() - moved_at > 2.5:
                why = 'not moving (blocked, or Nav2 not running)'
                break
        for _ in range(5):
            self._send(0.0, 0.0)
            time.sleep(0.05)
        # what she backed over is behind her now: the next retrace carries on from here
        self.track = collections.deque(track[:max(i, 1)])
        response.success = done >= 0.5 * metres
        response.message = f'backed out {done:.2f} m of {metres:.2f}' + (f': {why}' if why else '')
        self.get_logger().info(f'retrace: {response.message}')
        return response

    # ---------------------------------------------------------------- pictures --

    def _http(self, url, body=None, timeout=10.0):
        data = None if body is None else json.dumps(body).encode()
        req = urllib.request.Request(url, data=data, headers={'Content-Type': 'application/json'} if data else {})
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.read()

    def _picture(self, name, width=640):
        try:
            raw = self._http(f'{self.gp("cameras_url")}/{name}.jpg?w={width}&q=85')
            img = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
        except (OSError, urllib.error.URLError, ValueError):
            return None
        # the RealSense hangs upside down (2026-09-27); the server says how the pages turn it,
        # and the stop pictures turn it the same (Steve, 2026-10-09: "make it right side up")
        if name == 'd435' and img is not None:
            if self._d435_rotate is None:
                try:
                    self._d435_rotate = int(json.loads(self._http(f'{self.gp("cameras_url")}/config.json'))
                                            .get('d435_rotate', 0)) % 360
                except (OSError, urllib.error.URLError, ValueError):
                    return img
            if self._d435_rotate == 180:
                img = cv2.rotate(img, cv2.ROTATE_180)
        return img

    def _sweep(self):
        """The pan-tilt camera round the sweep, then back where it was; plus the fixed cameras."""
        with self._sweep_lock:
            return self._sweep_locked()

    def _sweep_locked(self):
        web = str(self.gp('web_url'))
        views = []
        try:
            before = json.loads(self._http(web + '/status')).get('look', [0.0, 0.0])
        except (OSError, urllib.error.URLError, ValueError):
            before = None
        if before is not None and self.gp('sweep'):
            for pan, tilt in zip(self.gp('sweep_pan_deg'), self.gp('sweep_tilt_deg')):
                try:
                    self._http(web + '/look', {'pan': pan, 'tilt': tilt})
                except (OSError, urllib.error.URLError):
                    break
                time.sleep(float(self.gp('sweep_settle_s')))
                side = 'straight ahead' if abs(pan) < 5 else f'{abs(pan):.0f} deg {"left" if pan > 0 else "right"}'
                if tilt < -5:
                    side += f', tilted {abs(tilt):.0f} deg down (the floor just ahead)'
                views.append((f'pan-tilt: {side}', self._picture('front')))
            try:
                self._http(web + '/look', {'pan': before[0], 'tilt': before[1]})
            except (OSError, urllib.error.URLError):
                pass
        views.append(('depth camera, fixed, ahead', self._picture('d435')))
        views.append(('rear camera, behind her', self._picture('rear')))
        # a camera that is off (the rear one since 2026-09-30, broken) or failed gives None:
        # not a blank tile for Claude to wonder about
        return [v for v in views if v[1] is not None]

    def build_packet(self, sweep=True):
        """-> (directory, [{path, label}], facts text)."""
        self._busy += 1
        try:
            return self._build_packet(sweep)
        finally:
            self._busy -= 1

    def _build_packet(self, sweep):
        d = os.path.join(str(self.gp('out_dir')), time.strftime('%Y%m%d-%H%M%S'))
        os.makedirs(d, exist_ok=True)
        images, facts = [], []
        pose = self._fresh_pose()
        camera = self._latest(OccupancyGrid, '/nvblox_node/static_occupancy_grid', 2)
        scan_msg = self._latest(LaserScan, 'scan', qos_profile_sensor_data)
        mo = self._map_to_odom()
        pose_map = compose(mo, pose) if (mo and pose) else None
        track = list(self.track)
        if pose_map is not None and self.map is not None:
            m = self.map
            grid = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
            info = (m.info.resolution, m.info.origin.position.x, m.info.origin.position.y)
            track_map = [compose(mo, (x, y, 0.0))[:2] for x, y in track]
            plan = []
            if 'plan' in self.raw:
                pm = deserialize_message(self.raw['plan'], Path)
                plan = [(q.pose.position.x, q.pose.position.y) for q in pm.poses]
            goal = None
            if self.goal is not None:
                goal = (self.goal.pose.position.x, self.goal.pose.position.y)
                facts.append(f'her goal is at map ({goal[0]:+.2f}, {goal[1]:+.2f}), '
                             f'{math.hypot(goal[0] - pose_map[0], goal[1] - pose_map[1]):.1f} m from her')
            facts.append(f'she is at map ({pose_map[0]:+.2f}, {pose_map[1]:+.2f}) facing '
                         f'{math.degrees(pose_map[2]):+.0f} deg (0 = east / +x, 90 = north / +y)')
            obstacles = []
            if camera is not None:
                g = camera
                cells = sh.grid_cells(np.array(g.data, dtype=np.int16).reshape(g.info.height, g.info.width),
                                      (g.info.resolution, g.info.origin.position.x, g.info.origin.position.y))
                obstacles = [compose(mo, (x, y, 0.0))[:2] for x, y in cells]
            path = os.path.join(d, 'house_map.png')
            cv2.imwrite(path, sh.render_house(grid, info, pose_map, track_map, plan, goal, obstacles=obstacles))
            images.append({'path': path, 'label': '1. HOUSE MAP around her (north up, metres)'})
        else:
            facts.append('no house map or no position on it right now')
        if pose is not None:
            scan = []
            if scan_msg is not None:
                s = scan_msg
                scan = sh.scan_points(s.ranges, s.angle_min, s.angle_increment, s.range_min, s.range_max)
            cam = []
            if camera is not None:
                g = camera
                cells = sh.grid_cells(np.array(g.data, dtype=np.int16).reshape(g.info.height, g.info.width),
                                      (g.info.resolution, g.info.origin.position.x, g.info.origin.position.y))
                cam = [c for c in sh.to_robot(cells, pose) if abs(c[0]) < 2.0 and abs(c[1]) < 2.0]
            back = sh.to_robot(track[-60:], pose)
            path = os.path.join(d, 'close_up.png')
            cv2.imwrite(path, sh.render_close(scan, cam, back))
            images.append({'path': path, 'label': '2. CLOSE-UP of what is around her (her front up)'})
            length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(track, track[1:]))
            facts.append(f'she has {length:.1f} m of her own track behind her to back out along')
        if sweep:
            views = self._sweep()
            if any(img is not None for _, img in views):
                path = os.path.join(d, 'cameras.jpg')
                cv2.imwrite(path, sh.contact_sheet(views), [cv2.IMWRITE_JPEG_QUALITY, 85])
                images.append({'path': path, 'label': '3. PHOTOS from her cameras, each labelled'})
        if self.situation:
            facts.insert(0, self.situation)
        with open(os.path.join(d, 'facts.txt'), 'w') as fh:
            fh.write('\n'.join(facts) + '\n')
        return d, images, '; '.join(facts)

    def _snapshot(self, request, response):
        d, images, facts = self.build_packet()
        response.success = bool(images)
        response.message = json.dumps({'dir': d, 'images': [i['path'] for i in images], 'facts': facts})
        return response

    # ---------------------------------------------------------------- asking --

    def _ask(self, request, response):
        d, images, facts = self.build_packet()
        if not images:
            response.success, response.message = False, json.dumps({'error': 'no pictures', 'dir': d})
            return response
        rid = uuid.uuid4().hex[:12]
        ev = threading.Event()
        self.answers[rid] = ev
        msg = String()
        msg.data = json.dumps({'id': rid, 'system': sh.SYSTEM, 'prompt': sh.PROMPT.format(situation=facts),
                               'images': images, 'effort': str(self.gp('ask_effort')), 'max_tokens': 4000})
        self.look_pub.publish(msg)
        t0 = time.monotonic()
        got = ev.wait(float(self.gp('ask_timeout_s')))
        a = self.answers.pop(rid, None)
        if not got or not isinstance(a, dict):
            response.success, response.message = False, json.dumps({'error': 'no answer from the brain', 'dir': d})
            return response
        advice = sh.parse_advice(a.get('text', '')) if 'text' in a else None
        out = {'advice': advice, 'error': a.get('error'), 'raw': a.get('text', '')[:2000], 'dir': d,
               'seconds': round(time.monotonic() - t0, 1)}
        with open(os.path.join(d, 'answer.json'), 'w') as fh:
            json.dump(out, fh, indent=1)
        self.get_logger().info(f'ask: {json.dumps(advice) if advice else out.get("error") or "no usable advice"} '
                               f'({out["seconds"]} s, {d})')
        response.success = advice is not None
        response.message = json.dumps(out)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = NavHelper()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        while rclpy.ok():
            try:
                executor.spin()
                break
            except InvalidHandle as exc:
                # a short-lived subscription (_latest, _map_to_odom, odometry idle) was destroyed by a
                # worker thread while this thread built the wait set: rclpy's race. Drive 41
                # (2026-10-06) lost the whole node - and Claude's answer - to it mid-ask. The
                # entity is gone now; spin on and the worker's reply still goes out.
                node.get_logger().warning(f'executor: {exc} (a temporary subscription closed mid-wait): spinning on')
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except (Exception, KeyboardInterrupt):  # noqa: BLE001 - the context is gone, or a second SIGINT, after an external shutdown
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
