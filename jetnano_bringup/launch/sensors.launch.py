"""The three sensors: RPLidar A1M8, RealSense D435, BNO055 IMU.

Each driver publishes into the frame the URDF defines for it, so the frame_id
values below MUST match jetnano.urdf.xacro: lidar_link, camera_link, imu_link.
Getting one of these wrong is the classic cause of a map that slowly shears
away from reality.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from jetnano_bringup.launch_lock import only_one


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('jetnano_bringup'), 'config')
    # The BNO055's saved calibration; see the file for where it came from.
    imu_config = os.path.join(config_dir, 'bno055.yaml')
    # The box around the robot's own parts that the lidar must not report.
    scan_filter_config = os.path.join(config_dir, 'scan_filter.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('use_imu', default_value='true'),
        DeclareLaunchArgument(
            'lidar_port', default_value='/dev/rplidar',
            description='RPLidar serial port. udev/99-robot-sensors.rules gives it this stable name.'),
        DeclareLaunchArgument(
            'imu_i2c_bus', default_value='7',
            description='I2C bus the BNO055 is on (same header as the PCA9685, the INA219 and the cliff mux)'),
        DeclareLaunchArgument('use_battery', default_value='true',
                              description='INA219 battery monitor (battery topic, e_stop when flat)'),
        DeclareLaunchArgument('battery_voltage_only', default_value='true',
                              description='nothing flows through the shunt: report the pack voltage only'),
        DeclareLaunchArgument('battery_simulate', default_value='false'),
        DeclareLaunchArgument('use_cliff', default_value='false',
                              description='VL53L0X cliff sensors behind a TCA9548A (fit them first)'),
        DeclareLaunchArgument('cliff_python', default_value='/home/jeston/venv-sensors/bin/python3',
                              description='the venv python with the Adafruit VL53L0X driver (system site-packages on)'),
        DeclareLaunchArgument('cliff_simulate', default_value='false'),
        DeclareLaunchArgument('cliff_simulated_drops', default_value="['']",
                              description="corners to pretend are drops in simulate mode, e.g. ['front_left']"),

        # One driver per sensor: a second copy of this launch stops itself
        # (two rplidar drivers on one serial port both die).
        only_one('jetnano_sensors', [

        GroupAction(
            condition=IfCondition(LaunchConfiguration('use_lidar')),
            actions=[
                Node(
                    package='rplidar_ros',
                    executable='rplidar_composition',
                    name='rplidar',
                    output='screen',
                    # Plugged in late, or a port hiccup: try again rather than
                    # driving blind until someone notices.
                    respawn=True,
                    respawn_delay=5.0,
                    parameters=[{
                        'serial_port': LaunchConfiguration('lidar_port'),
                        'serial_baudrate': 115200,
                        'frame_id': 'lidar_link',
                        'angle_compensate': True,
                        # A1M8 (fw 1.27) offers Standard/Express/Boost/Stability;
                        # Boost = 8K samples/s = 720 points per turn at ~7.6 Hz.
                        'scan_mode': 'Boost',
                    }],
                    # The raw scan includes the robot's own antenna; the filter
                    # below publishes the scan everything else uses.
                    remappings=[('scan', 'scan_raw')],
                ),
                Node(
                    package='laser_filters',
                    executable='scan_to_scan_filter_chain',
                    name='scan_filter',
                    output='screen',
                    parameters=[scan_filter_config],
                    remappings=[('scan', 'scan_raw'), ('scan_filtered', 'scan')],
                ),
            ],
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
                    # Off because nothing on the CPU path uses them, not because
                    # they fail: the stock JetPack 7.2 kernel streams the Y8
                    # infrared pair fine (verified 2026-09-23 in every
                    # combination, up to 848x480x60 and all four streams at
                    # once). The 2026-09-22 stall was a wedged camera, which
                    # initial_reset below now clears. GPU visual odometry
                    # (ros2_gpu_robot/cuvslam_d435) runs its own driver on
                    # the IR pair inside a container while this node is stopped.
                    'enable_infra1': False,
                    'enable_infra2': False,
                    # Visual odometry needs depth registered to the colour frame,
                    # and colour and depth stamped together: without enable_sync
                    # the aligned depth trails the colour by a frame and rtabmap
                    # drops pairs ('time difference ... is high').
                    'align_depth.enable': True,
                    'enable_sync': True,
                    'pointcloud.enable': False,
                    # The driver prefixes this with the camera name, so 'link'
                    # yields camera_link (the URDF frame); 'camera_link' would
                    # give camera_camera_link and cut the TF chain to the images.
                    'base_frame_id': 'link',
                    # Hardware-reset the camera before streaming. After a driver
                    # crash the D435 can keep colour running while depth never
                    # starts ('Frames didn't arrive'); the reset clears it.
                    'initial_reset': True,
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
                # The driver exits on an I2C error at start (a loose ground on
                # 2026-09-24 left the IMU dead until a full restart); try again.
                respawn=True,
                respawn_delay=5.0,
                parameters=[imu_config, {
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

        # The INA219 on the same I2C header (A0 bridged: 0x41). Without the
        # board it logs once every 30 s and keeps trying; a pack under 6 V
        # (the wall supply) is reported as absent and never stops anything.
        GroupAction(
            condition=IfCondition(LaunchConfiguration('use_battery')),
            actions=[Node(
                package='jetnano_bringup',
                executable='battery_monitor',
                name='battery_monitor',
                output='screen',
                respawn=True,
                respawn_delay=10.0,
                parameters=[{
                    'i2c_bus': LaunchConfiguration('imu_i2c_bus'),
                    'address': 0x41,
                    'voltage_only': LaunchConfiguration('battery_voltage_only'),
                    'simulate': LaunchConfiguration('battery_simulate'),
                }],
            )],
        ),

        # Four VL53L0X behind a TCA9548A, looking down at the corners. Its
        # driver lives in a venv (see cliff_guard.py), so the node runs under
        # that venv's python. Off until the sensors are fitted; when on,
        # drive.launch.py cliff:=true makes the collision guard read them.
        GroupAction(
            condition=IfCondition(LaunchConfiguration('use_cliff')),
            actions=[Node(
                package='jetnano_bringup',
                executable='cliff_guard',
                name='cliff_guard',
                output='screen',
                respawn=True,
                respawn_delay=10.0,
                prefix=[LaunchConfiguration('cliff_python'), ' '],
                parameters=[{
                    'i2c_bus': LaunchConfiguration('imu_i2c_bus'),
                    'simulate': LaunchConfiguration('cliff_simulate'),
                    'simulated_drops': LaunchConfiguration('cliff_simulated_drops'),
                }],
            )],
        ),

        ]),
    ])
