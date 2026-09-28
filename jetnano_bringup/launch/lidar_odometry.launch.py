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

"""Lidar odometry: MOLA on the 2D lidar, a second source of motion for the EKF.

    ros2 launch jetnano_bringup lidar_odometry.launch.py
    (normally through robot.launch.py / odometry.launch.py lidar_odom:=true)

Needs  sudo apt install ros-jazzy-mola-lidar-odometry  (without it this logs
why and starts nothing).

MOLA (mola_lidar_odometry) matches each filtered /scan against a local grid map
of recent keyframes (its "lidar2d" pipeline: point-to-gridmap 2D ICP), which
drifts far less than matching scan to scan. It runs here as an ODOMETRY source
only:

* its frame is renamed lidar_odom and it publishes NO /tf, so it cannot fight
  slam_toolbox (map -> odom) or the EKF (odom -> base_footprint);
* lidar_odom_relay turns its pose (lidar_odometry/pose) into forward, sideways
  and turning speeds on lidar_odom, which the EKF fuses next to the camera's
  (ekf.yaml odom2) - speeds, so a restart of either cannot yank the pose;
* no GUI, no RViz, no base_footprint -> base_link of its own (the URDF has it).

MOLA's scan matcher (mp2p_icp) runs on Intel TBB, which starts one thread per
core and keeps them all busy for a 2D scan: on H2-Host (32 threads) 177 % of a
core while driving. Pinned to ONE core it did the same work at the same rate
(7.7 Hz) for 17 % (2026-09-27, synthetic 720-beam scans at 8 Hz), so it runs
under taskset on the last core (cpus:=). TBB sizes itself from that mask.

MOLA's own launch file shuts down every launch that includes it when MOLA
exits. Included in robot.launch.py, one crash would take the whole robot down,
so it runs here as a separate `ros2 launch` process that is respawned instead.

Simulator, 2026-09-27 (H2-Host, a 12 m double loop): worst position error 4.9 cm
against ground truth, ~10 Hz. Why a second source: the camera's odometry goes
blind or silent with the camera (USB glitches, 2026-09-24 and 27; a firmware
that left the projector on, 2026-09-26). The lidar does not care about light,
USB 3 or projectors, and the camera does not care about glass or long corridors.
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _mola(context):
    try:
        get_package_share_directory('mola_lidar_odometry')
    except PackageNotFoundError:
        return [LogInfo(msg='lidar odometry: mola_lidar_odometry is not installed '
                            '(sudo apt install ros-jazzy-mola-lidar-odometry) - not started')]
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    scan_topic = LaunchConfiguration('scan_topic').perform(context)
    cpus = LaunchConfiguration('cpus').perform(context).strip() or str(os.cpu_count() - 1)
    pin = [] if cpus == 'all' else ['taskset', '-c', cpus]
    return [
        LogInfo(msg=f'lidar odometry: MOLA on CPU {cpus}' if pin else 'lidar odometry: MOLA on all CPUs'),
        ExecuteProcess(
            name='mola_lidar_odometry',
            cmd=pin + [
                'ros2', 'launch', 'mola_lidar_odometry', 'ros2-lidar-odometry.launch.py',
                f'lidar_topic_name:={scan_topic}',
                'lidar_topic_type:=LaserScan',
                'mola_lo_pipeline:=../pipelines/lidar2d.yaml',
                'use_rviz:=False',
                'use_mola_gui:=False',
                'use_state_estimator:=False',
                'publish_localization_following_rep105:=False',
                'mola_lo_reference_frame:=lidar_odom',
                'mola_state_estimator_reference_frame:=lidar_odom',
                f'use_sim_time:={use_sim_time}'],
            # not launch arguments in MOLA's launch file, only environment variables
            additional_env={
                'MOLA_LOCALIZATION_PUBLISH_TF': 'false',
                'MOLA_LOCALIZATION_PUBLISH_ODOM_MSGS': 'true',
                'MOLA_TF_FOOTPRINT_LINK': '',
            },
            output='screen',
            respawn=True,
            respawn_delay=5.0,
        ),
        Node(
            package='jetnano_bringup',
            executable='lidar_odom_relay',
            name='lidar_odom_relay',
            respawn=True,
            respawn_delay=3.0,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time == 'true'}],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('scan_topic', default_value='/scan',
                              description='the filtered scan (the robot itself removed)'),
        DeclareLaunchArgument('cpus', default_value='',
                              description="taskset CPU list for MOLA; empty = the last core, 'all' = no pinning"),
        OpaqueFunction(function=_mola),
    ])
