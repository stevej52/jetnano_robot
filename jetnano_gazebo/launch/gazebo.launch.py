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
Put the robot in Gazebo, wire up its sensors, and let cmd_vel drive it.

    ros2 launch jetnano_gazebo gazebo.launch.py
    ros2 launch jetnano_gazebo gazebo.launch.py headless:=true

Layers 1 to 3 of 4. Spawns the robot in the textured testbed, publishes its
transforms, bridges the lidar, RGB-D camera and IMU onto the topic names the
real drivers use, and brings up ros2_control so the robot can be driven from
cmd_vel. Layer 4 is the full stack on top: Nav2, SLAM and the tilt guard.

use_sim_time is true throughout. Everything downstream must agree, or the TF
tree will be timestamped from two different clocks and nothing will line up.

Note for anyone adding to this: the tilt guard has a bug class that only
appears here. Simulated clocks start at 0.0, and 0.0 is falsy in Python, so
``self._timestamp or now`` silently evaluates to ``now``. It cost an
afternoon; see jetnano_bringup/jetnano_bringup/tilt.py.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gazebo_pkg = get_package_share_directory('jetnano_gazebo')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    default_world = os.path.join(gazebo_pkg, 'worlds', 'testbed.sdf')
    default_model = os.path.join(gazebo_pkg, 'urdf', 'jetnano_sim.urdf.xacro')

    world = LaunchConfiguration('world')
    model = LaunchConfiguration('model')
    headless = LaunchConfiguration('headless')

    # -r runs physics immediately; -s is server only, for a machine with no
    # display or for a test that should not open a window.
    gz_args = PythonExpression([
        "'-r -s ' + '", world, "' if '", headless, "' == 'true' else '-r ' + '", world, "'"])

    robot_description = ParameterValue(Command(['xacro ', model]), value_type=str)

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_jetnano',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'jetnano',
            '-x', '0.0', '-y', '0.0', '-z', '0.10',
        ],
        parameters=[{'use_sim_time': True}],
    )

    def spawner(controller):
        """A controller_manager spawner for one controller."""
        return Node(
            package='controller_manager',
            executable='spawner',
            name=f'spawn_{controller}',
            output='screen',
            arguments=[controller, '--controller-manager', '/controller_manager'],
            parameters=[{'use_sim_time': True}],
        )

    joint_state_broadcaster = spawner('joint_state_broadcaster')
    steer_controller = spawner('steer_controller')
    wheel_controller = spawner('wheel_controller')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('model', default_value=default_model),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run the server with no GUI window'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args}.items(),
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }],
        ),

        # Spawns whatever robot_state_publisher is advertising, so the
        # simulator and the TF tree cannot describe different robots.
        spawn,

        # The controller_manager only exists once the robot is in the world,
        # because it runs inside the Gazebo plugin attached to the model. A
        # spawner started any earlier fails to find it and gives up.
        RegisterEventHandler(OnProcessExit(
            target_action=spawn,
            on_exit=[joint_state_broadcaster, steer_controller, wheel_controller],
        )),

        # The simulator's counterpart to ros2_pca9685: cmd_vel in, eight joint
        # commands out, using the same chassis geometry as the real robot.
        Node(
            package='jetnano_gazebo',
            executable='sim_drive',
            name='sim_drive',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        # Without the clock bridge every use_sim_time node waits forever for a
        # /clock that never arrives, and the whole graph sits there silent.
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            output='screen',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            parameters=[{'use_sim_time': True}],
        ),

        # Renames Gazebo's sensor topics to the ones the real drivers publish,
        # so odometry, SLAM and Nav2 cannot tell they are in a simulator.
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='sensor_bridge',
            output='screen',
            parameters=[{
                'config_file': os.path.join(gazebo_pkg, 'config', 'ros_gz_bridge.yaml'),
                'use_sim_time': True,
            }],
        ),
    ])
