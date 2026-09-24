"""RViz, pointed at this robot.

    ros2 launch jetnano_bringup rviz.launch.py                # view:=nav
    ros2 launch jetnano_bringup rviz.launch.py view:=drive

Run it wherever there is a screen. Running it on the HOST PC rather than the
Jetson is usually the better idea: RViz is the heaviest thing in the stack and
the Jetson has better uses for its GPU. ROS_DOMAIN_ID must match (7).

Two saved layouts, both in jetnano_description/rviz:

    nav     robot, TF, laser scan, the camera, nvblox's obstacle grid, the map
            and the planned path, with the Nav2 panel for setting goals. Fixed
            frame `map`: if nothing appears, nothing is publishing map -> odom
            yet - start navigation.launch.py, or use the drive view.
    drive   for driving by hand, no Nav2 needed: fixed frame `odom`, the camera
            large, laser scan, nvblox grid, the EKF's trail, and a view that
            follows the robot.

The camera panel reads the JPEG stream (/camera/color/image_raw/compressed):
the raw stream is ~220 Mbit/s and the robot's Wi-Fi uplink carries ~36. The
costmap layers in the nav view are present but switched off - turn them on
when you are tuning Nav2, and off again afterwards, because two translucent
costmaps on top of a map is unreadable.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    rviz_dir = os.path.join(get_package_share_directory('jetnano_description'), 'rviz')
    view = LaunchConfiguration('view')
    rviz_config = LaunchConfiguration('rviz_config')

    # An explicit file wins; otherwise the view picks one of the saved layouts.
    config = PythonExpression([
        "'", rviz_config, "' or ('", rviz_dir, "/' + ('drive.rviz' if '", view,
        "' == 'drive' else 'robot.rviz'))"])

    return LaunchDescription([
        DeclareLaunchArgument('view', default_value='nav', choices=['nav', 'drive'],
                              description='saved layout: nav (map frame, Nav2 panel) or drive (odom frame, camera large)'),
        DeclareLaunchArgument('rviz_config', default_value='',
                              description='an explicit .rviz file instead of a view'),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', config],
        ),
    ])
