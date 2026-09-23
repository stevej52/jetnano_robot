"""Where the robot thinks it is: visual odometry fused with the IMU.

This chassis has no wheel encoders, so there is no wheel odometry to publish.
rtabmap's RGB-D odometry estimates motion from the RealSense instead, and
robot_localization fuses that with the BNO055 to publish odom -> base_footprint.

Only ONE node may publish odom -> base_footprint. rgbd_odometry therefore runs with
publish_tf:=false and hands its estimate to the EKF on /vo as a measurement.
The EKF owns the transform.
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
    ekf_config = os.path.join(bringup_pkg, 'config', 'ekf.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_visual_odometry', default_value='true'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Take time from /clock. MUST be true under Gazebo, or '
                        'rtabmap stamps its output from the wall clock while the '
                        'images carry sim time and nothing in tf lines up.'),

        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='visual_odometry',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_visual_odometry')),
            parameters=[{
                'use_sim_time': use_sim_time,
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'publish_tf': False,
                'approx_sync': True,
                'wait_imu_to_init': False,
                'subscribe_rgbd': False,
                # Frame-to-frame with optical flow and fewer features: 19 Hz
                # instead of the defaults' 10 (docs/vo-sweep-2026-09-23.md).
                # The node has one worker thread and drops every frame that
                # arrives while it is busy, so only a cheaper estimate raises
                # the rate. Drift under motion with these values is unmeasured.
                # This is the CPU fallback; the GPU path (cuVSLAM, see
                # ros2_gpu_robot/cuvslam_d435) publishes /vo at 89 Hz instead.
                'Odom/Strategy': '1',
                'Vis/CorType': '1',
                'Odom/KeyFrameThr': '0.6',
                'Vis/MaxFeatures': '500',
            }],
            remappings=[
                ('rgb/image', '/camera/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/camera/color/camera_info'),
                ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
                ('odom', '/vo'),
            ],
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': use_sim_time}],
        ),
    ])
