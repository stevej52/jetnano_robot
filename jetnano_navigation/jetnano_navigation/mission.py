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

"""The mission controller: the one node that talks to Nav2.

    ros2 topic pub --once /mission/command std_msgs/String '{data: "{\\"do\\": \\"goal\\", \\"x\\": 2.0, \\"y\\": -2.0, \\"heading_deg\\": 135}"}'
    ros2 topic echo /mission/status      {"mission": "goal", "phase": "driving", "goal": "a1b2...", "since_s": 4.2,
                                          "feedback": {"dist_m": 3.1, "left": 1, "x": .., "y": .., "h_deg": ..},
                                          "held": null | {"since_s": 12, "why": "STOP on the page"}, "gate": "ok"}
    ros2 topic echo /mission/result      one line per finished mission, as nav_goal printed it
    ros2 run jetnano_navigation mission_cmd goal X Y H [TIMEOUT]   (a client that waits and prints)

Commands (JSON on /mission/command):
    {"do": "goal", "x", "y", "heading_deg", "timeout_s": 45, "planner": "GridBased"}
    {"do": "route", "waypoints": [[x, y, heading_deg], ...], "timeout_s": 300}     (NavigateThroughPoses)
    {"do": "cancel"}      {"do": "resume"}      {"do": "status"}

Until 2026-10-02 every script (nav_goal, nav_route, nav_park, meet, battery_home) opened its
own Nav2 connection; nobody held the list of accepted goals, a safety stop cancelled the one
action it knew about, and the parker could start on top of a lap. This node owns the Nav2
action clients and the goal ids, and it is the one the safety gate (safety_gate.py) talks to:
  gate stopped/inhibited while a mission runs -> cancel what it owns, wait for Nav2 to confirm,
      hold with the reason;
  gate ok/degraded again -> resume on its own if the hold lasted under resume_within_s (30),
      else stay held and say "resume to continue" (Steve's choice, 2026-10-02).
Step 1 (this file): goal, route, cancel, resume, status, the gate. The stretch handover, the
detour guard and the pictures of nav_route, and nav_park's arcs, move in next (steps 2-3);
until then the scripts keep working beside it.
"""

import json
import math
import sys
import time
import uuid

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from jetnano_navigation.nav_goal import yaw
from jetnano_navigation.nav_route import pose_msg

STATUS_NAMES = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
                GoalStatus.STATUS_CANCELED: 'CANCELED'}


def resume_policy(held_s, resume_within_s=30.0):
    """After the gate permits again: 'resume' if the hold was short, else 'ask'."""
    return 'resume' if held_s <= resume_within_s else 'ask'


def remaining_route(waypoints, left):
    """The waypoints still to go, from Nav2's number_of_poses_remaining."""
    if left is None or left <= 0 or left > len(waypoints):
        return list(waypoints)
    return list(waypoints[len(waypoints) - left:])


class Mission(Node):

    def __init__(self):
        super().__init__('mission')
        self.declare_parameter('resume_within_s', 30.0)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.to_pose = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.through = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.planner_pub = self.create_publisher(String, 'planner_selector', latched)
        self.status_pub = self.create_publisher(String, 'mission/status', latched)
        self.result_pub = self.create_publisher(String, 'mission/result', 10)
        self.create_subscription(String, 'mission/command', self._on_command, 10)
        self.create_subscription(String, 'safety/state', self._on_gate, latched)
        self.gate = None                 # the gate's latest verdict (dict)
        self.active = None               # the mission in hand (dict), or None
        self.held = None                 # {'since': monotonic, 'why': str, 'mission': dict} while stopped by the gate
        self.cancelling = None           # future of a cancel we asked for
        self.create_timer(0.5, self._tick)
        self.get_logger().info('mission controller: goal / route / cancel / resume on /mission/command; the safety gate is obeyed')

    # ---------------------------------------------------------------- inputs --

    def _on_gate(self, m):
        try:
            self.gate = json.loads(m.data)
        except ValueError:
            return

    def _on_command(self, m):
        try:
            cmd = json.loads(m.data)
        except ValueError:
            self.get_logger().warning(f'command is not JSON: {m.data[:80]}')
            return
        do = cmd.get('do')
        if do == 'status':
            self._publish_status()
        elif do == 'cancel':
            self.held = None
            self._cancel('cancelled on request')
        elif do == 'resume':
            self._resume('on request')
        elif do in ('goal', 'route'):
            if self.active is not None:
                self._cancel('replaced by a new command')
            self.held = None
            self._start(cmd)
        else:
            self.get_logger().warning(f'unknown command: {do}')

    # --------------------------------------------------------------- missions --

    def _start(self, cmd):
        permit = (self.gate or {}).get('permit', False)
        if not permit:
            why = '; '.join((self.gate or {}).get('reasons') or ['no verdict from the safety gate'])
            self._finish({'do': cmd.get('do'), 'id': uuid.uuid4().hex[:8], 't0': time.monotonic(), 'cmd': cmd},
                         'REFUSED', f'the safety gate does not permit: {why}')
            return
        mission = {'do': cmd['do'], 'id': uuid.uuid4().hex[:8], 't0': time.monotonic(), 'cmd': cmd,
                   'timeout_s': float(cmd.get('timeout_s', 45.0 if cmd['do'] == 'goal' else 300.0)),
                   'feedback': {}, 'handle': None, 'goal_uuid': None, 'result_future': None, 'phase': 'sending'}
        if cmd['do'] == 'goal':
            mission['planner'] = cmd.get('planner', 'GridBased')
            mission['waypoints'] = [[float(cmd['x']), float(cmd['y']), float(cmd.get('heading_deg', 0.0))]]
        else:
            mission['planner'] = cmd.get('planner', 'Through')
            mission['waypoints'] = [[float(a), float(b), float(c)] for a, b, c in cmd['waypoints']]
        self.active = mission
        self._send(mission, mission['waypoints'])

    def _send(self, mission, waypoints):
        self.planner_pub.publish(String(data=mission['planner']))
        stamp = self.get_clock().now().to_msg()
        poses = [pose_msg(x, y, math.radians(h), stamp) for x, y, h in waypoints]
        if mission['do'] == 'goal':
            client, goal = self.to_pose, NavigateToPose.Goal()
            goal.pose = poses[0]
        else:
            client, goal = self.through, NavigateThroughPoses.Goal()
            goal.poses = poses
        if not client.wait_for_server(timeout_sec=3.0):
            self._finish(mission, 'REFUSED', 'Nav2 is not there (no action server)')
            return
        mission['phase'] = 'sending'
        mission['sent'] = waypoints
        fut = client.send_goal_async(goal, feedback_callback=lambda f: self._on_feedback(mission, f))
        fut.add_done_callback(lambda f: self._on_accepted(mission, f))

    def _on_accepted(self, mission, fut):
        if self.active is not mission:
            return
        handle = fut.result()
        if handle is None or not handle.accepted:
            self._finish(mission, 'REJECTED', 'Nav2 rejected the goal')
            return
        mission['handle'] = handle
        mission['goal_uuid'] = bytes(handle.goal_id.uuid).hex()[:8]
        mission['phase'] = 'driving'
        mission['result_future'] = handle.get_result_async()
        mission['result_future'].add_done_callback(lambda f: self._on_result(mission, f))
        self.get_logger().info(f'{mission["do"]} {mission["id"]}: Nav2 goal {mission["goal_uuid"]} accepted, '
                               f'{len(mission["sent"])} pose(s)')
        self._publish_status()

    def _on_feedback(self, mission, f):
        if self.active is not mission:
            return
        fb = f.feedback
        p = fb.current_pose.pose
        mission['feedback'] = {'dist_m': round(fb.distance_remaining, 2), 'x': round(p.position.x, 2),
                               'y': round(p.position.y, 2), 'h_deg': round(math.degrees(yaw(p.orientation))),
                               'left': getattr(fb, 'number_of_poses_remaining', 1)}

    def _on_result(self, mission, fut):
        if self.active is not mission:
            return
        status = fut.result().status
        if status == GoalStatus.STATUS_CANCELED and self.held is not None and self.held['mission'] is mission:
            mission['phase'] = 'held'            # our own cancel, for the gate: the mission waits
            self.get_logger().info(f'{mission["do"]} {mission["id"]}: Nav2 confirmed the cancel; held ({self.held["why"]})')
            self._publish_status()
            return
        self._finish(mission, STATUS_NAMES.get(status, str(status)), '')

    def _finish(self, mission, outcome, note):
        fb = mission.get('feedback') or {}
        line = (f'result {outcome} after {time.monotonic() - mission["t0"]:.1f} s'
                + (f'; {note}' if note else '')
                + (f'; ended {fb["dist_m"]:.2f} m from the goal' if 'dist_m' in fb else ''))
        self.get_logger().info(f'{mission["do"]} {mission["id"]}: {line}')
        self.result_pub.publish(String(data=json.dumps({'id': mission['id'], 'do': mission['do'], 'outcome': outcome,
                                                        'note': note, 'line': line, 'feedback': fb})))
        if self.active is mission:
            self.active = None
        if self.held and self.held['mission'] is mission:
            self.held = None
        self._publish_status()

    def _cancel(self, why):
        mission = self.active
        if mission is None or mission.get('handle') is None:
            if mission is not None:
                self._finish(mission, 'CANCELED', why)
            return
        mission['phase'] = 'cancelling'
        self.get_logger().warning(f'{mission["do"]} {mission["id"]}: cancelling Nav2 goal {mission["goal_uuid"]}: {why}')
        mission['handle'].cancel_goal_async()          # Nav2's answer arrives as the goal's result (CANCELED)

    def _resume(self, why):
        if self.held is None:
            self.get_logger().info('nothing held: nothing to resume')
            return
        mission = self.held['mission']
        permit = (self.gate or {}).get('permit', False)
        if not permit:
            self.get_logger().warning('cannot resume: the safety gate still does not permit')
            return
        left = remaining_route(mission['waypoints'], (mission.get('feedback') or {}).get('left'))
        self.get_logger().warning(f'{mission["do"]} {mission["id"]}: resuming {why} after {time.monotonic() - self.held["since"]:.0f} s '
                                  f'held, {len(left)} pose(s) to go')
        self.held = None
        self.active = mission
        mission['feedback'] = {}
        self._send(mission, left)

    # ------------------------------------------------------------------ gate --

    def _tick(self):
        now = time.monotonic()
        mission = self.active
        gate = self.gate or {}
        permit = gate.get('permit', False)
        if mission is not None and mission['phase'] == 'driving' and not permit:
            why = '; '.join(gate.get('reasons') or [gate.get('state', 'not permitted')])
            self.held = {'since': now, 'why': why, 'mission': mission}
            self._cancel(f'the safety gate: {why}')
        elif self.held is not None and permit and self.active is not None and self.active['phase'] == 'held':
            held_s = now - self.held['since']
            if resume_policy(held_s, float(self.get_parameter('resume_within_s').value)) == 'resume':
                self._resume('on its own')
            # else: stays held, the status says "resume to continue"
        if mission is not None and mission['phase'] in ('driving', 'sending') and now - mission['t0'] > mission['timeout_s']:
            self._cancel(f'timeout after {mission["timeout_s"]:.0f} s')
            mission['timed_out'] = True
        if now - getattr(self, '_status_at', 0.0) > 0.5:
            self._publish_status()

    def _publish_status(self):
        self._status_at = time.monotonic()
        m = self.active
        held = None
        if self.held is not None:
            held_s = round(time.monotonic() - self.held['since'], 1)
            policy = resume_policy(held_s, float(self.get_parameter('resume_within_s').value))
            held = {'since_s': held_s, 'why': self.held['why'],
                    'next': 'resumes on its own when permitted' if policy == 'resume' else 'held: send resume to continue'}
        status = {'mission': m['do'] if m else 'idle', 'phase': m['phase'] if m else 'idle',
                  'id': m['id'] if m else None, 'goal': m.get('goal_uuid') if m else None,
                  'since_s': round(time.monotonic() - m['t0'], 1) if m else None,
                  'feedback': (m.get('feedback') or {}) if m else {}, 'held': held,
                  'gate': (self.gate or {}).get('state', 'unknown')}
        self.status_pub.publish(String(data=json.dumps(status)))


def main(args=None):
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=args)
    node = Mission()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except (Exception, KeyboardInterrupt):  # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
