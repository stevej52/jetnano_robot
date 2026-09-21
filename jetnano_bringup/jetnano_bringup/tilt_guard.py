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

"""
Back the robot out when it tilts too far, as if it had hit an obstacle.

The EKF runs in ``two_d_mode``, which pins roll and pitch to zero in the fused
pose, so nothing downstream of it knows the robot is on its side. The BNO055
still publishes full orientation on ``imu/data`` at 50 Hz, though, so this
node reads the IMU directly and needs no change to the EKF at all.

It publishes on ``cmd_vel_tilt``, which belongs in twist_mux ABOVE teleop:

    tilt_recovery:
      topic: cmd_vel_tilt
      timeout: 0.5
      priority: 150

Above teleop on purpose. If the robot is going over, the operator's stick
should not be able to fight the recovery.

While the guard is not recovering it publishes NOTHING, so twist_mux times
out that input after 0.5 s and control falls back to navigation or teleop on
its own. There is no "release" message that can go missing.

The decision itself lives in ``tilt.py`` and has no ROS in it, so it can be
tested against a list of angles rather than against a robot on a slope.
"""

from geometry_msgs.msg import Twist
from jetnano_bringup.tilt import roll_pitch_from_quaternion, State, TiltGuard, TiltLimits
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Imu
from std_msgs.msg import String


class TiltGuardNode(Node):
    """Watch the IMU and drive backwards out of a tilt."""

    def __init__(self) -> None:
        """Declare parameters, build the guard, subscribe to the IMU."""
        super().__init__('tilt_guard')

        self.declare_parameter('roll_trigger_deg', 25.0)
        self.declare_parameter('pitch_trigger_deg', 35.0)
        self.declare_parameter('roll_release_deg', 15.0)
        self.declare_parameter('pitch_release_deg', 20.0)
        self.declare_parameter('debounce_s', 0.2)
        self.declare_parameter('min_recovery_s', 1.5)
        self.declare_parameter('max_recovery_s', 5.0)
        self.declare_parameter('reverse_speed', 0.15)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('imu_timeout_s', 1.0)
        self.declare_parameter('stop_on_imu_timeout', True)

        limits = TiltLimits(
            roll_trigger_deg=float(self.get_parameter('roll_trigger_deg').value),
            pitch_trigger_deg=float(self.get_parameter('pitch_trigger_deg').value),
            roll_release_deg=float(self.get_parameter('roll_release_deg').value),
            pitch_release_deg=float(self.get_parameter('pitch_release_deg').value),
            debounce_s=float(self.get_parameter('debounce_s').value),
            min_recovery_s=float(self.get_parameter('min_recovery_s').value),
            max_recovery_s=float(self.get_parameter('max_recovery_s').value))
        try:
            limits.validate()
        except ValueError as error:
            # Bad thresholds make the guard useless in a way that is invisible
            # at run time. Refuse to start rather than pretend to protect.
            self.get_logger().fatal(str(error))
            raise
        self._guard = TiltGuard(limits)

        self._reverse_speed = abs(float(self.get_parameter('reverse_speed').value))
        self._imu_timeout = float(self.get_parameter('imu_timeout_s').value)
        self._stop_on_timeout = bool(self.get_parameter('stop_on_imu_timeout').value)

        self._last_imu = None
        self._timed_out = False
        self._recoveries = 0

        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel_tilt', 10)
        self._status_pub = self.create_publisher(String, '~/status', 10)
        self.create_subscription(
            Imu, 'imu/data', self._on_imu, QoSPresetProfiles.SENSOR_DATA.value)

        rate = float(self.get_parameter('publish_rate').value)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f'tilt guard active: roll {limits.roll_trigger_deg:g} deg, '
            f'pitch {limits.pitch_trigger_deg:g} deg, reversing at '
            f'{self._reverse_speed:g} m/s on cmd_vel_tilt')

    def _now(self) -> float:
        """Seconds from the node clock, as the guard wants them."""
        return self.get_clock().now().nanoseconds / 1e9

    def _on_imu(self, message: Imu) -> None:
        """Feed one orientation sample into the guard."""
        self._last_imu = self._now()
        if self._timed_out:
            self.get_logger().info('imu/data is publishing again, tilt guard restored')
            self._timed_out = False

        orientation = message.orientation
        roll, pitch = roll_pitch_from_quaternion(
            orientation.x, orientation.y, orientation.z, orientation.w)
        before = self._guard.state
        after = self._guard.update(roll, pitch, self._last_imu)
        if after is not before:
            self._announce(before, after)

    def _announce(self, before: State, after: State) -> None:
        """Log and publish every state change, so the robot's reasons are visible."""
        if after is State.RECOVERING:
            self._recoveries += 1
            self.get_logger().warning(
                f'tilt limit exceeded ({self._guard.last_reason}), reversing '
                f'[recovery #{self._recoveries}]')
        elif before is State.RECOVERING and self._guard.gave_up:
            self.get_logger().error(
                'still tilted after max_recovery_s of reversing; releasing control. '
                'The robot may be stuck, or on a slope it cannot back off.')
        elif before is State.RECOVERING:
            self.get_logger().info('tilt recovered, releasing control')

        status = String()
        status.data = f'{before.value} -> {after.value}: {self._guard.last_reason}'
        self._status_pub.publish(status)

    def _tick(self) -> None:
        """Publish a recovery command, or nothing at all."""
        if self._imu_is_stale():
            if self._stop_on_timeout:
                self._cmd_pub.publish(Twist())   # all zeros: hold still
            return

        if self._guard.reversing():
            command = Twist()
            command.linear.x = -self._reverse_speed
            self._cmd_pub.publish(command)
        # Otherwise publish nothing, and let twist_mux time this input out.

    def _imu_is_stale(self) -> bool:
        """True when the IMU has gone quiet for longer than the timeout."""
        if self._last_imu is None:
            return False        # nothing has arrived yet: startup, not a fault
        stale = (self._now() - self._last_imu) > self._imu_timeout
        if stale and not self._timed_out:
            self._timed_out = True
            if self._stop_on_timeout:
                self.get_logger().error(
                    'imu/data has stopped; the tilt guard is blind and is HOLDING THE '
                    'ROBOT STILL. Set stop_on_imu_timeout:=false to drive unguarded.')
            else:
                self.get_logger().error(
                    'imu/data has stopped; the tilt guard is blind and is NOT '
                    'protecting the robot (stop_on_imu_timeout is false).')
        return stale


def main(args=None) -> None:
    """Entry point for ``ros2 run jetnano_bringup tilt_guard``."""
    rclpy.init(args=args)
    node = TiltGuardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
