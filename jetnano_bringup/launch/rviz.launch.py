"""RViz, pointed at this robot.

    ros2 launch jetnano_bringup rviz.launch.py

Run it wherever there is a screen. Running it on the HOST PC rather than the
Jetson is usually the better idea: RViz is the heaviest thing in the stack and
the Jetson has better uses for its GPU. ROS_DOMAIN_ID must match (7).

The saved layout shows the robot, TF, the laser scan, the map and the planned
path, with the Nav2 panel for setting goals. The costmap layers are present
but switched off - turn them on when you are tuning Nav2, and off again
afterwards, because two translucent costmaps on top of a map is unreadable.

Fixed frame is `map`. If nothing appears, it is almost always because nothing
is publishing map -> odom yet: start SLAM, or switch the fixed frame to `odom`.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_pkg = get_package_share_directory('jetnano_description')
    default_rviz = os.path.join(description_pkg, 'rviz', 'robot.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
        ),
    ])
