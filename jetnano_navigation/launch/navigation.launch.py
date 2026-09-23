"""Nav2, wired so it cannot go round the safety layer.

Three ways to run, matching the three mapping modes:

    # 1. Unknown space: map it as you go, and navigate in it at the same time
    ros2 launch jetnano_navigation navigation.launch.py mode:=mapping

    # 2. Known space: load the map and keep adding to it as you find new rooms
    ros2 launch jetnano_navigation navigation.launch.py \\
        mode:=continue map:=~/maps/home

    # 3. Known space, finished map: localise only, never modify it
    ros2 launch jetnano_navigation navigation.launch.py \\
        mode:=localization map:=~/maps/home

All three use slam_toolbox, so map -> odom always comes from the same place and
behaves the same way. AMCL is not used; it cannot extend a map, which is the
whole point of mode 2.

THE IMPORTANT BIT
~~~~~~~~~~~~~~~~~
Nav2's controller_server publishes to /cmd_vel, and /cmd_vel is exactly the
topic ros2_pca9685 listens to. Left alone, Nav2 would drive the servos
DIRECTLY - straight past twist_mux, which means past the joystick's priority
and past the e-stop lock. An autonomous robot you cannot override is not a
feature.

So Nav2's outputs are remapped away from /cmd_vel, and the chain inside Nav2 is

    controller  --cmd_vel_nav_raw-->  velocity_smoother  --cmd_vel_nav_smoothed-->
    collision_monitor  --cmd_vel_nav-->  twist_mux

The collision monitor (Jazzy's bringup always starts one) is last, so what
twist_mux sees has already been slowed or stopped for anything in the lidar
scan on a collision course. Its topics are parameters in nav2.yaml, not
remaps: with the stock names its output would have landed on cmd_vel_nav_raw
(a loop into the smoother) and twist_mux would have read the smoother's
unfiltered output. twist_mux then treats Nav2 as the low-priority input it
should be: teleop outranks it, and the e_stop lock stops it dead.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString


def generate_launch_description():
    nav_pkg = get_package_share_directory('jetnano_navigation')
    nav2_params = os.path.join(nav_pkg, 'config', 'nav2.yaml')

    nav2_bringup = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'])
    slam_launch = PathJoinSubstitution(
        [FindPackageShare('jetnano_navigation'), 'launch', 'slam.launch.py'])

    # The behaviour trees without Spin live in this package; nav2.yaml names
    # them with a placeholder for the install path.
    bt_params = ReplaceString(
        source_file=LaunchConfiguration('params_file'),
        replacements={'__BT_DIR__': os.path.join(nav_pkg, 'behavior_trees')})

    # nvblox:=true adds the camera's 3D obstacle layer to the local costmap by
    # rewriting the plugin list in a copy of the params file. The layer's own
    # parameters are always in nav2.yaml; only the list decides whether Nav2
    # loads the plugin, so the simulator (no nvblox there) is untouched.
    params_file = ReplaceString(
        source_file=bt_params,
        replacements={
            'plugins: ["obstacle_layer", "inflation_layer"]':
            'plugins: ["obstacle_layer", "nvblox_layer", "inflation_layer"]',
        },
        condition=IfCondition(LaunchConfiguration('nvblox')))

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='mapping',
            description='mapping | continue | localization'),
        DeclareLaunchArgument(
            'map', default_value='',
            description='Serialised pose-graph, no extension (continue/localization)'),
        DeclareLaunchArgument('params_file', default_value=nav2_params),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'nvblox', default_value='false',
            description='Add the nvblox 3D-map layer to the local costmap (Jetson only: '
                        'needs ros-jazzy-nvblox-nav2 and the nvblox node running)'),

        GroupAction([
            # Keep Nav2 off the driver's topic. See the docstring above.
            SetRemap(src='/cmd_vel', dst='/cmd_vel_nav_raw'),
            SetRemap(src='/cmd_vel_smoothed', dst='/cmd_vel_nav_smoothed'),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_bringup),
                launch_arguments={
                    'use_sim_time': 'false',
                    'params_file': params_file,
                    'autostart': LaunchConfiguration('autostart'),
                }.items(),
            ),
        ]),

        # slam_toolbox supplies map -> odom in every mode.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={
                'mode': LaunchConfiguration('mode'),
                'map': LaunchConfiguration('map'),
            }.items(),
        ),
    ])
