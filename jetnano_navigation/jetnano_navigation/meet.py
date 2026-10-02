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

"""Meet a person: see them, drive up to them, look at them, talk to them.

    ros2 run jetnano_navigation meet [--stand 1.2] [--timeout 60] [--hold 30] [--no-drive]
                                     [--hello "Hi there! I'm Rosie."] [--wait 15]

(Steve, 2026-10-01: "Recognize a person, drive up to them, look at them and talk to them.")

1. See: waits (--wait s) for /people (jetnano_bringup people: YOLOv8-pose on the D435) to show
   a person with a distance; takes the nearest.
2. Drive: with the person on the map (localised), a Nav2 goal --stand metres short of them on
   the line from her to them, facing them (nav_goal, with its costmap and battery checks), unless
   they are already within --stand + 0.3 m. If they move more than 0.5 m while she drives, one
   more goal. On the bench, or with no map, she stays put and says so (--no-drive does the same).
3. Look: the whole time, the pan-tilt head follows the face the detector sees (face_deg: the
   bearing/elevation from the fixed camera, the head is close enough to it), smoothed; recentred
   at the end. First, in a thread while she waits, rail_check pans the head and watches the head
   camera: with no servo power (the pack unplugged on the bench) she says so instead of staring.
4. Talk: the TALK button's mode (quiet, "over and out" and robot-only off, Diane mode on, so
   listen answers anyone with her fun personality and without her name), then the hello line
   through /speak. She keeps looking at them for --hold s, then the head recentres and this exits;
   the conversation itself is listen's.
"""

import argparse
import json
import math
import os
import subprocess
import sys
import threading
import time

import rclpy
import tf2_ros
from rclpy.node import Node
from std_msgs.msg import Float64, String

from jetnano_bringup.rail_check import rail_live

QUIET_FLAG = os.path.expanduser('~/voice/quiet')
DIANE_FLAG = os.path.expanduser('~/voice/diane')
VOICE_OFF_FLAG = os.path.expanduser('~/voice/voice_off')
ROBOT_FLAG = os.path.expanduser('~/voice/robot_only')

# the pan-tilt head (web_teleop, pca9685.yaml): pan 74.6 = ahead, higher = LEFT; tilt 90 = level, higher = DOWN
PAN_CENTER, PAN_RANGE = 74.6, (-69.0, 105.0)       # view degrees right .. left
TILT_CENTER, TILT_RANGE = 90.0, (-78.0, 78.0)      # view degrees down .. up
HEAD_GAIN = 0.5                                    # of the remaining angle, each 0.1 s step
HEAD_STALE_S = 1.5                                 # no face this long: hold still


def nearest(msg: dict):
    """The nearest person with a distance in a /people message, or None."""
    people = [q for q in msg.get('people', []) if q.get('dist_m') is not None]
    return min(people, key=lambda q: q['dist_m']) if people else None


def stand_off(robot_xy, person_xy, stand_m):
    """Where to stop: `stand_m` short of the person on the line from the robot, facing them
    -> (x, y, heading_rad)."""
    dx, dy = person_xy[0] - robot_xy[0], person_xy[1] - robot_xy[1]
    d = math.hypot(dx, dy)
    if d < 1e-6:
        return robot_xy[0], robot_xy[1], 0.0
    ux, uy = dx / d, dy / d
    back = min(stand_m, d)                         # never past her own spot
    return person_xy[0] - ux * back, person_xy[1] - uy * back, math.atan2(uy, ux)


def head_step(current, target, gain=HEAD_GAIN):
    """One smoothed step of the head toward the target (view degrees), clamped to its reach."""
    pan = current[0] + gain * (target[0] - current[0])
    tilt = current[1] + gain * (target[1] - current[1])
    return (max(PAN_RANGE[0], min(PAN_RANGE[1], pan)), max(TILT_RANGE[0], min(TILT_RANGE[1], tilt)))


class Meet(Node):

    def __init__(self):
        super().__init__('meet')
        self.latest = None                         # last /people message (dict)
        self.latest_at = 0.0
        self.create_subscription(String, 'people', self._on_people, 10)
        self.speak_pub = self.create_publisher(String, 'speak', 10)
        self.pan_pub = self.create_publisher(Float64, '/pca9685/pan/angle', 10)
        self.tilt_pub = self.create_publisher(Float64, '/pca9685/tilt/angle', 10)
        self.buf = tf2_ros.Buffer()
        self.tf = tf2_ros.TransformListener(self.buf, self)
        self.head = (0.0, 0.0)                     # view degrees (pan + = left, tilt + = up)
        self.head_target = (0.0, 0.0)
        self.face_at = 0.0
        self.next_head = 0.0
        # does the head have power? (rail_check: a head-camera frame, a small pan, a frame;
        # 0.6-1.9 s) - in a thread while she waits for someone, so it costs the meet nothing
        self.head_ok = None
        self.head_check = threading.Thread(target=self._check_head, daemon=True)
        self.head_check.start()

    def _check_head(self):
        self.head_ok = rail_live(self, log=lambda s: print('   ' + s, flush=True))

    def _on_people(self, m: String) -> None:
        try:
            self.latest = json.loads(m.data)
        except ValueError:
            return
        self.latest_at = time.monotonic()
        q = nearest(self.latest) or next(iter(self.latest.get('people', [])), None)
        if q and q.get('face_deg'):
            self.head_target = (float(q['face_deg'][0]), float(q['face_deg'][1]))
            self.face_at = self.latest_at

    def spin(self, s: float) -> None:
        """Spin for `s` seconds, moving the head toward the face every 0.1 s."""
        end = time.monotonic() + s
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            now = time.monotonic()
            if now >= self.next_head:
                self.next_head = now + 0.1
                if now - self.face_at < HEAD_STALE_S and self.head_ok is not False and not self.head_check.is_alive():
                    self.look(head_step(self.head, self.head_target))

    def look(self, view) -> None:
        self.head = view
        a, b = Float64(), Float64()
        a.data = PAN_CENTER + view[0]
        b.data = TILT_CENTER - view[1]
        self.pan_pub.publish(a)
        self.tilt_pub.publish(b)

    def recentre(self) -> None:
        if self.head_ok is False:
            return
        for _ in range(8):
            self.look(head_step(self.head, (0.0, 0.0)))
            rclpy.spin_once(self, timeout_sec=0.1)
        self.look((0.0, 0.0))

    def person(self, wait_s: float):
        """The nearest person seen in the last second, waiting up to `wait_s` for one."""
        end = time.monotonic() + wait_s
        while time.monotonic() < end:
            self.spin(0.1)
            if self.latest and time.monotonic() - self.latest_at < 1.0:
                q = nearest(self.latest)
                if q:
                    return q
        return None

    def on_map(self, q):
        """The person's camera-frame point on the map -> (x, y) or None (not localised)."""
        if not q.get('pos') or not q.get('frame'):
            return None
        try:
            from geometry_msgs.msg import PointStamped
            from tf2_geometry_msgs import do_transform_point
            pt = PointStamped()
            pt.header.frame_id = q['frame']
            pt.point.x, pt.point.y, pt.point.z = [float(v) for v in q['pos']]
            tr = self.buf.lookup_transform('map', q['frame'], rclpy.time.Time())
            out = do_transform_point(pt, tr)
            return (out.point.x, out.point.y)
        except Exception as exc:  # noqa: BLE001
            print(f'   (no map for {q["frame"]}: {str(exc)[:80]})', flush=True)
            return None

    def map_pose(self):
        for _ in range(20):
            if self.buf.can_transform('map', 'base_footprint', rclpy.time.Time()):
                tr = self.buf.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform
                q = tr.rotation
                return (tr.translation.x, tr.translation.y,
                        math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
            self.spin(0.1)
        return None

    def nav_goal(self, x, y, h_deg, timeout) -> bool:
        """nav_goal as a child, spun while it runs so the head keeps following the face."""
        cmd = [sys.executable, '-m', 'jetnano_navigation.nav_goal', f'{x:.3f}', f'{y:.3f}', f'{h_deg:.1f}', str(timeout)]
        print(f'-- nav_goal {x:+.2f} {y:+.2f} {h_deg:+.0f}', flush=True)
        child = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        lines = []
        while child.poll() is None:
            self.spin(0.2)
        for ln in child.stdout.read().splitlines():
            if ln.startswith(('result', 'goal', 'start', 'refused')):
                lines.append(ln)
        print('\n'.join('   ' + ln for ln in lines), flush=True)
        return any(ln.startswith('result SUCCEEDED') for ln in lines)

    def say(self, text: str) -> None:
        """Through /speak; wait for its listeners first (a fresh publisher's first message is lost
        until all three - speak, listen, the dashboard - have matched, 2026-10-01)."""
        end = time.monotonic() + 5.0
        while self.speak_pub.get_subscription_count() < 3 and time.monotonic() < end:
            self.spin(0.1)
        self.spin(0.3)
        self.speak_pub.publish(String(data=text))
        print(f'-- say: {text}', flush=True)

    @staticmethod
    def talk_mode() -> None:
        """The web page's TALK button, in flags (web_teleop.talk('talk'))."""
        os.makedirs(os.path.dirname(QUIET_FLAG), exist_ok=True)
        for flag in (QUIET_FLAG, VOICE_OFF_FLAG, ROBOT_FLAG):
            try:
                os.remove(flag)
            except FileNotFoundError:
                pass
        with open(DIANE_FLAG, 'w') as f:
            f.write(time.strftime('%Y-%m-%d %H:%M:%S') + '\n')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--stand', type=float, default=1.2, help='metres short of the person to stop (default 1.2)')
    ap.add_argument('--timeout', type=float, default=60.0, help='seconds for each Nav2 goal')
    ap.add_argument('--hold', type=float, default=30.0, help='seconds to keep looking at them after the hello')
    ap.add_argument('--wait', type=float, default=15.0, help='seconds to wait for someone to appear')
    ap.add_argument('--hello', default="Hi there! I'm Rosie. What's your name?")
    ap.add_argument('--no-drive', action='store_true', help='look and talk only')
    args = ap.parse_args(argv)

    rclpy.init()
    node = Meet()
    try:
        q = node.person(args.wait)
        if q is None:
            print('nobody seen', flush=True)
            node.say("I don't see anyone.")
            node.spin(2.0)
            return 1
        print(f'person at {q["dist_m"]:.2f} m, {q["frame"]} {q["pos"]}, face {q["face_deg"]}, eyes {q["eyes"]}', flush=True)
        drove = False
        node.spin(0.5)                             # let the TF listener hear the map
        here, there = (None, None) if args.no_drive else (node.map_pose(), node.on_map(q))
        if not args.no_drive and (here is None or there is None):
            print('not localised: staying put', flush=True)
        elif not args.no_drive:
            for attempt in range(2):
                if q['dist_m'] <= args.stand + 0.3:
                    print(f'already close ({q["dist_m"]:.2f} m)', flush=True)
                    break
                gx, gy, gh = stand_off(here[:2], there, args.stand)
                print(f'   she is at {here[0]:+.2f} {here[1]:+.2f}, they are at {there[0]:+.2f} {there[1]:+.2f}', flush=True)
                ok = node.nav_goal(gx, gy, math.degrees(gh), args.timeout)
                drove = True
                if not ok:
                    print('could not get there', flush=True)
                    break
                q2 = node.person(2.0)
                here2, there2 = node.map_pose(), None if q2 is None else node.on_map(q2)
                if q2 is None or here2 is None or there2 is None:
                    break
                moved = math.hypot(there2[0] - there[0], there2[1] - there[1])
                q, here, there = q2, here2, there2
                if moved < 0.5 or attempt == 1:
                    break
                print(f'they moved {moved:.2f} m: once more', flush=True)
        node.talk_mode()
        node.head_check.join(timeout=3.0)
        hello = args.hello if drove or q['dist_m'] <= args.stand + 0.3 else args.hello + " I can't come over right now."
        if node.head_ok is False:
            hello += " My head has no power, so I can't look at you."
        node.say(hello)
        node.spin(args.hold)
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        try:
            node.recentre()
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
