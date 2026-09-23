"""The three sensors: RPLidar A1M8, RealSense D435, BNO055 IMU.

Each driver publishes into the frame the URDF defines for it, so the frame_id
values below MUST match jetnano.urdf.xacro: lidar_link, camera_link, imu_link.
Getting one of these wrong is the classic cause of a map that slowly shears
away from reality.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('use_imu', default_value='true'),
        DeclareLaunchArgument(
            'lidar_port', default_value='/dev/rplidar',
            description='RPLidar serial port. udev/99-robot-sensors.rules gives it this stable name.'),
        DeclareLaunchArgument(
            'imu_i2c_bus', default_value='7',
            description='I2C bus the BNO055 is on (same header as the PCA9685)'),

        GroupAction(
            condition=IfCondition(LaunchConfiguration('use_lidar')),
            actions=[Node(
                package='rplidar_ros',
                executable='rplidar_composition',
                name='rplidar',
                output='screen',
                parameters=[{
                    'serial_port': LaunchConfiguration('lidar_port'),
                    'serial_baudrate': 115200,
                    'frame_id': 'lidar_link',
                    'angle_compensate': True,
                    # A1M8 (fw 1.27) offers Standard/Express/Boost/Stability;
                    # Boost = 8K samples/s = 720 points per turn at ~7.6 Hz.
                    'scan_mode': 'Boost',
                }],
            )],
        ),

        GroupAction(
            condition=IfCondition(LaunchConfiguration('use_camera')),
            actions=[Node(
                package='realsense2_camera',
                executable='realsense2_camera_node',
                name='camera',
                namespace='camera',
                output='screen',
                parameters=[{
                    # The D435 has no IMU. That is the BNO055 job.
                    'enable_gyro': False,
                    'enable_accel': False,
                    'enable_color': True,
                    'enable_depth': True,
                    # The stock Jetson kernel's UVC driver does not know the
                    # IR streams' Y8 format; leaving them on stalls depth.
                    'enable_infra1': False,
                    'enable_infra2': False,
                    # Visual odometry needs depth registered to the colour frame.
                    'align_depth.enable': True,
                    'pointcloud.enable': False,
                    # The driver prefixes this with the camera name, so 'link'
                    # yields camera_link (the URDF frame); 'camera_link' would
                    # give camera_camera_link and cut the TF chain to the images.
                    'base_frame_id': 'link',
                    'rgb_camera.color_profile': '640,480,30',
                    'depth_module.depth_profile': '640,480,30',
                }],
            )],
        ),

        GroupAction(
            condition=IfCondition(LaunchConfiguration('use_imu')),
            actions=[Node(
                package='bno055',
                executable='bno055',
                name='bno055',
                output='screen',
                parameters=[{
                    'connection_type': 'i2c',
                    'i2c_bus': LaunchConfiguration('imu_i2c_bus'),
                    'i2c_addr': 0x28,
                    'frame_id': 'imu_link',
                    'data_query_frequency': 50,
                    'ros_topic_prefix': 'imu/',
                }],
                # The driver names its fused output imu/imu; everything
                # downstream (EKF, tilt guard, the sim bridge) uses imu/data.
                remappings=[('imu/imu', 'imu/data')],
            )],
        ),
    ])
