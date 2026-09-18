"""Joystick teleoperation.

One node, either controller. It works out which is attached by looking at what
is actually plugged in, so nothing here needs to change when you swap between
the Thrustmaster set and the Xbox pad.

Run this on whichever machine the controller is plugged into - the host PC or
the Jetson. ROS_DOMAIN_ID must match on both (7 for this robot).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    teleop_pkg = get_package_share_directory('jetnano_teleop')
    default_profiles = os.path.join(teleop_pkg, 'config', 'joysticks.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('profiles_file', default_value=default_profiles),
        DeclareLaunchArgument(
            'turbo_scale', default_value='2.0',
            description='Multiplier on max_linear while the turbo button is held'),

        Node(
            package='jetnano_teleop',
            executable='teleop_node',
            name='jetnano_teleop',
            output='screen',
            parameters=[{
                'profiles_file': LaunchConfiguration('profiles_file'),
                'turbo_scale': LaunchConfiguration('turbo_scale'),
            }],
        ),
    ])
