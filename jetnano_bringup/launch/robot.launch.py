"""Bring the whole robot up.

    ros2 launch jetnano_bringup robot.launch.py

Everything that runs on the robot itself: description and TF, the PCA9685
driver behind twist_mux, the three sensors, visual odometry and the EKF.

Useful variations:

    simulate:=true          no I2C writes; everything else real
    use_camera:=false       skip the RealSense (also disables visual odometry)
    use_teleop:=true        run the joystick node HERE instead of on the PC

Teleop is off by default because the controller is normally plugged into the
host PC. ROS_DOMAIN_ID must match on both machines; it is 7 for this robot.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include(name, condition=None, arguments=None):
    source = PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare('jetnano_bringup'), 'launch', name]))
    return IncludeLaunchDescription(
        source, condition=condition, launch_arguments=arguments or {})


def generate_launch_description():
    simulate = LaunchConfiguration('simulate')
    i2c_bus = LaunchConfiguration('i2c_bus')
    use_camera = LaunchConfiguration('use_camera')

    return LaunchDescription([
        DeclareLaunchArgument('simulate', default_value='false'),
        DeclareLaunchArgument('i2c_bus', default_value='7'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('use_imu', default_value='true'),
        DeclareLaunchArgument('use_odometry', default_value='true'),
        DeclareLaunchArgument('use_teleop', default_value='false'),

        _include('description.launch.py'),

        _include('drive.launch.py', arguments={
            'simulate': simulate,
            'i2c_bus': i2c_bus,
        }.items()),

        _include('sensors.launch.py', arguments={
            'use_lidar': LaunchConfiguration('use_lidar'),
            'use_camera': use_camera,
            'use_imu': LaunchConfiguration('use_imu'),
        }.items()),

        _include('odometry.launch.py',
                 condition=IfCondition(LaunchConfiguration('use_odometry'))),

        _include('teleop.launch.py',
                 condition=IfCondition(LaunchConfiguration('use_teleop'))),
    ])
