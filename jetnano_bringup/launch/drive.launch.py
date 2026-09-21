"""Everything that makes the wheels turn: the PCA9685 driver and twist_mux.

twist_mux is the only thing that publishes /cmd_vel, and the driver is the only
thing that touches the I2C bus. Teleop and Nav2 publish to their own topics and
twist_mux decides who wins.

    ros2 launch jetnano_bringup drive.launch.py simulate:=true

runs the whole chain with no hardware attached; every PWM write is logged
instead of sent.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory('jetnano_bringup')
    pca_config = os.path.join(bringup_pkg, 'config', 'pca9685.yaml')
    mux_config = os.path.join(bringup_pkg, 'config', 'twist_mux.yaml')

    simulate = LaunchConfiguration('simulate')
    i2c_bus = LaunchConfiguration('i2c_bus')

    return LaunchDescription([
        DeclareLaunchArgument(
            'simulate', default_value='false',
            description='Log PWM writes instead of driving the board'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Take time from /clock. Must be true under Gazebo.'),
        DeclareLaunchArgument(
            'use_tilt_guard', default_value='true',
            description='Back out of a roll or pitch past its limit. Needs imu/data, '
                        'so it does nothing until sensors.launch.py is up.'),
        DeclareLaunchArgument(
            'i2c_bus', default_value='7',
            description='I2C bus the PCA9685 is on. Was 1 on the Jetson Nano; '
                        'check with i2cdetect -l and i2cdetect -y -r <bus>'),

        Node(
            package='ros2_pca9685',
            executable='pca9685_node',
            name='pca9685',
            output='screen',
            parameters=[pca_config, {
                'simulate': simulate,
                'i2c_bus': i2c_bus,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),

        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[mux_config, {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            # twist_mux publishes cmd_vel_out; the driver listens on cmd_vel.
            remappings=[('cmd_vel_out', 'cmd_vel')],
        ),

        # Reads imu/data directly rather than the EKF, because the EKF runs in
        # two_d_mode and pins roll and pitch to zero. Publishes cmd_vel_tilt,
        # which twist_mux ranks above teleop.
        Node(
            package='jetnano_bringup',
            executable='tilt_guard',
            name='tilt_guard',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            condition=IfCondition(LaunchConfiguration('use_tilt_guard')),
        ),
    ])
