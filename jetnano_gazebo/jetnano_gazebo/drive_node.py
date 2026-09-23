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
Drive the simulated robot: cmd_vel in, eight joint commands out.

This is the simulator's counterpart to ros2_pca9685 on the real machine. That
node turns cmd_vel into three PWM channels; this one turns the same cmd_vel
into four steering angles and four wheel speeds for ros2_control. Both read
the same chassis geometry, so a turn commanded in simulation and the same turn
commanded on the carpet mean the same thing.

    cmd_vel  ->  steer_controller/commands   (4 positions, rad)
             ->  wheel_controller/commands   (4 velocities, rad/s)

JOINT ORDER is the trap here. Both controllers take a bare
Float64MultiArray with no joint names in it, so the order of the four values
is the only thing that says which wheel they belong to. Get it wrong and the
robot still drives, just wrongly - front-left steering appears on the
rear-right wheel. ORDER below is the single definition, and it must match the
joints lists in config/controllers.yaml.

The kinematics are in steering.py, with no ROS in them, tested against
closed-form geometry.
"""

from geometry_msgs.msg import Twist
from jetnano_gazebo.steering import Chassis, commands_from_twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# Must match the `joints:` lists in config/controllers.yaml, in this order.
ORDER = ['front_left', 'front_right', 'rear_left', 'rear_right']


class DriveNode(Node):
    """Turn Twist commands into ros2_control joint commands."""

    def __init__(self) -> None:
        """Read the chassis geometry from parameters and start listening."""
        super().__init__('sim_drive')

        self.declare_parameter('wheelbase', 0.330)
        self.declare_parameter('track_width', 0.230)
        self.declare_parameter('wheel_radius', 0.065)
        self.declare_parameter('steer_limit', 0.5236)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('publish_rate', 50.0)

        self._chassis = Chassis(
            wheelbase=float(self.get_parameter('wheelbase').value),
            track_width=float(self.get_parameter('track_width').value),
            wheel_radius=float(self.get_parameter('wheel_radius').value),
            steer_limit=float(self.get_parameter('steer_limit').value))
        self._timeout = float(self.get_parameter('cmd_timeout').value)

        self._linear = 0.0
        self._angular = 0.0
        self._last_command = None
        self._stopped = True

        self._steer_pub = self.create_publisher(
            Float64MultiArray, 'steer_controller/commands', 10)
        self._wheel_pub = self.create_publisher(
            Float64MultiArray, 'wheel_controller/commands', 10)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)

        rate = float(self.get_parameter('publish_rate').value)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f'sim drive ready: wheelbase {self._chassis.wheelbase:g} m, '
            f'track {self._chassis.track_width:g} m, tightest turn '
            f'{self._chassis.min_turning_radius():.3f} m')

    def _now(self) -> float:
        """Seconds from the node clock, which is Gazebo's under use_sim_time."""
        return self.get_clock().now().nanoseconds / 1e9

    def _on_cmd_vel(self, message: Twist) -> None:
        """Remember the latest command; the timer does the work."""
        self._linear = message.linear.x
        self._angular = message.angular.z
        self._last_command = self._now()
        self._stopped = False

    def _tick(self) -> None:
        """
        Publish joint commands at a steady rate.

        Commands go out on a timer rather than straight from the callback so
        the controllers see a continuous stream even when cmd_vel is sparse,
        and so a publisher that stops does not leave the wheels spinning.
        """
        if self._last_command is not None and \
                self._now() - self._last_command > self._timeout:
            if not self._stopped:
                self.get_logger().warning(
                    f'no cmd_vel for {self._timeout:g}s, stopping the wheels')
                self._linear = 0.0
                self._angular = 0.0
                self._stopped = True

        commands = commands_from_twist(self._chassis, self._linear, self._angular)

        steer = Float64MultiArray()
        steer.data = commands.steer_list(ORDER)
        self._steer_pub.publish(steer)

        wheel = Float64MultiArray()
        wheel.data = commands.speed_list(ORDER)
        self._wheel_pub.publish(wheel)


def main(args=None) -> None:
    """Entry point for ``ros2 run jetnano_gazebo sim_drive``."""
    rclpy.init(args=args)
    node = DriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
