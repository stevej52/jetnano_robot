"""Show the robot model on its own, with sliders for the steering and wheels.

This launches no hardware and no odometry. It is the quick way to check that
the URDF is well formed and that the sensor mounts are in the right places:

    ros2 launch jetnano_description display.launch.py

The TF tree it produces is rooted at base_footprint. There is deliberately no
map or odom frame here - those come from SLAM and odometry at runtime, not
from the robot description.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('jetnano_description')
    default_model = os.path.join(pkg, 'urdf', 'jetnano.urdf.xacro')
    default_rviz = os.path.join(pkg, 'rviz', 'view_robot.rviz')

    model_arg = DeclareLaunchArgument(
        'model', default_value=default_model,
        description='Absolute path to the robot xacro file')
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Use joint_state_publisher_gui (sliders) instead of the plain publisher')
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Also start RViz')
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value=default_rviz,
        description='RViz configuration file')

    # ParameterValue(..., value_type=str) matters: without it the expanded XML
    # is passed as a plain string and robot_state_publisher rejects it.
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]), value_type=str)

    return LaunchDescription([
        model_arg, gui_arg, rviz_arg, rviz_config_arg,

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(LaunchConfiguration('gui')),
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=UnlessCondition(LaunchConfiguration('gui')),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
