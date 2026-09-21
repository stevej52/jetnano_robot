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
The whole robot, in simulation.

    ros2 launch jetnano_gazebo full_stack.launch.py
    ros2 launch jetnano_gazebo full_stack.launch.py headless:=true slam:=false

Layer 4. On top of the simulator this adds the robot's own stack, unmodified:
visual odometry, the EKF, twist_mux, the tilt guard and slam_toolbox. They are
the same launch files the real robot uses, given use_sim_time:=true.

WHY drive.launch.py RUNS HERE WITH simulate:=true

It brings up twist_mux and the tilt guard, which the robot needs in simulation
exactly as much as on carpet. It also brings up ros2_pca9685, which has no I2C
bus to talk to here - so it runs in its simulate mode, where it logs what it
would have written instead of writing it. Keeping it in the graph means the
topic wiring in simulation is the same wiring as on the robot, rather than a
second arrangement that has to be kept in step.

The wheels are turned by jetnano_gazebo's sim_drive, which reads the same
/cmd_vel that twist_mux publishes.

EVERYTHING gets use_sim_time:=true. A node left on the wall clock timestamps
its output from a different clock than the images and transforms it is
reasoning about, and the failure is not loud: tf lookups start failing at the
edges and poses drift in ways that look like a tuning problem.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

SIM_TIME = {'use_sim_time': 'true'}


def generate_launch_description():
    gazebo_pkg = get_package_share_directory('jetnano_gazebo')
    bringup_pkg = get_package_share_directory('jetnano_bringup')
    navigation_pkg = get_package_share_directory('jetnano_navigation')

    def include(package_dir, name, condition=None, arguments=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(package_dir, 'launch', name)),
            condition=condition,
            launch_arguments=(arguments or SIM_TIME).items())

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument(
            'odometry', default_value='true',
            description='rtabmap visual odometry and the EKF'),
        DeclareLaunchArgument(
            'slam', default_value='true',
            description='slam_toolbox in mapping mode'),
        DeclareLaunchArgument(
            'tilt_guard', default_value='true',
            description='Back out of a roll or pitch past its limit'),

        # Layers 1 to 3: the simulator, its sensors and the drive chain.
        include(gazebo_pkg, 'gazebo.launch.py', arguments={
            'headless': LaunchConfiguration('headless'),
        }),

        # twist_mux and the tilt guard, plus ros2_pca9685 in simulate mode so
        # the topic graph matches the real robot's.
        include(bringup_pkg, 'drive.launch.py', arguments={
            'simulate': 'true',
            'use_sim_time': 'true',
            'use_tilt_guard': LaunchConfiguration('tilt_guard'),
        }),

        # Given 20 s to let the controllers come up and the camera produce a
        # few frames first. rtabmap started against a silent camera spends its
        # first seconds complaining rather than initialising.
        TimerAction(period=20.0, actions=[
            include(bringup_pkg, 'odometry.launch.py',
                    condition=IfCondition(LaunchConfiguration('odometry'))),
        ]),

        # SLAM last: it needs /scan and a tf tree that already reaches
        # base_footprint, both of which exist by now.
        TimerAction(period=25.0, actions=[
            include(navigation_pkg, 'slam.launch.py',
                    condition=IfCondition(LaunchConfiguration('slam')),
                    arguments={'use_sim_time': 'true', 'mode': 'mapping'}),
        ]),
    ])
