"""Robot description and TF.

Publishes the robot's own rigid-body transforms from the URDF. It deliberately
does NOT publish map or odom - the EKF publishes odom->base_link and SLAM
publishes map->odom.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_pkg = get_package_share_directory('jetnano_description')
    default_model = os.path.join(description_pkg, 'urdf', 'jetnano.urdf.xacro')

    model = LaunchConfiguration('model')
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description = ParameterValue(Command(['xacro ', model]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=default_model),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            respawn=True,
            respawn_delay=3.0,
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),

        # The steering servos have no position feedback, so nothing measures
        # the steering joints. This publishes zeros so the TF tree is complete
        # and RViz can draw the robot. Replace it if encoders ever appear.
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            respawn=True,
            respawn_delay=3.0,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
