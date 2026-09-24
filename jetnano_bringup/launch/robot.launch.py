"""Bring the whole robot up.

    ros2 launch jetnano_bringup robot.launch.py

Everything that runs on the robot itself: description and TF, the PCA9685
driver behind twist_mux, the three sensors, visual odometry and the EKF.

Useful variations:

    simulate:=true          no I2C writes; everything else real
    vo:=rtabmap             visual odometry on the CPU instead of cuVSLAM on the
                            GPU (the default here; see odometry.launch.py). With
                            cuvslam the container's driver owns the camera, so the
                            host RealSense node is not started.
    nvblox:=true            with cuvslam: also build a 3D map of what the camera
                            sees, for Nav2's local costmap (navigation.launch.py
                            nvblox:=true) and for the collision guard. Odometry
                            drops from 89 to ~43 Hz.
    use_camera:=false       skip the RealSense (and any visual odometry)
    use_teleop:=true        run the joystick node HERE instead of on the PC
    use_web_teleop:=false   no driving web page (http://<robot>:8081/)

Joystick teleop is off by default because the controller is normally plugged
into the host PC; the web page is on by default because it lives on the robot.
ROS_DOMAIN_ID must match on both machines; it is 7 for this robot.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare

from jetnano_bringup.launch_lock import only_one


def _include(name, condition=None, arguments=None):
    source = PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare('jetnano_bringup'), 'launch', name]))
    include = IncludeLaunchDescription(
        source, condition=condition, launch_arguments=arguments or {})
    # Scoped, because an include's arguments otherwise leak into this launch:
    # sensors.launch.py's use_camera:=False was read by the odometry include
    # a few lines later and turned vo into 'none' (2026-09-23).
    return GroupAction([include], scoped=True, forwarding=True)


def generate_launch_description():
    simulate = LaunchConfiguration('simulate')
    i2c_bus = LaunchConfiguration('i2c_bus')
    use_camera = LaunchConfiguration('use_camera')
    use_sim_time = LaunchConfiguration('use_sim_time')
    vo = LaunchConfiguration('vo')

    # cuVSLAM brings its own RealSense driver (in the container); the host
    # driver must stay off or the two fight over the camera.
    host_camera = PythonExpression(
        ["'", use_camera, "' == 'true' and '", vo, "' != 'cuvslam'"])
    # No camera at all means no visual odometry either.
    vo_source = PythonExpression(
        ["'", vo, "' if '", use_camera, "' == 'true' else 'none'"])

    return LaunchDescription([
        DeclareLaunchArgument('simulate', default_value='false'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Take time from /clock. Must be true under Gazebo.'),
        DeclareLaunchArgument('i2c_bus', default_value='7'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('use_imu', default_value='true'),
        DeclareLaunchArgument('use_odometry', default_value='true'),
        DeclareLaunchArgument(
            'vo', default_value='cuvslam', choices=['cuvslam', 'rtabmap', 'none'],
            description='visual odometry: cuvslam (GPU, in the isaac_vo container) or rtabmap (CPU)'),
        DeclareLaunchArgument(
            'nvblox', default_value='false',
            description='with vo:=cuvslam, also run nvblox 3D mapping for the Nav2 costmap'),
        DeclareLaunchArgument('use_teleop', default_value='false'),
        DeclareLaunchArgument(
            'use_web_teleop', default_value='true',
            description='serve the driving web page (camera + arrows) on port 8081'),

        # One robot per machine: a second copy of this launch stops itself.
        only_one('jetnano_robot', [

        LogInfo(msg=['robot.launch: vo=', vo_source, ', host camera=', host_camera]),

        _include('description.launch.py', arguments={
            'use_sim_time': use_sim_time,
        }.items()),

        _include('drive.launch.py', arguments={
            'simulate': simulate,
            'i2c_bus': i2c_bus,
            'use_sim_time': use_sim_time,
            # The collision guard also reads the camera's map when nvblox runs.
            'nvblox': PythonExpression(["'", vo_source, "' == 'cuvslam' and '",
                                        LaunchConfiguration('nvblox'), "' == 'true'"]),
        }.items()),

        _include('sensors.launch.py', arguments={
            'use_lidar': LaunchConfiguration('use_lidar'),
            'use_camera': host_camera,
            'use_imu': LaunchConfiguration('use_imu'),
        }.items()),

        _include('odometry.launch.py',
                 condition=IfCondition(LaunchConfiguration('use_odometry')),
                 arguments={'use_sim_time': use_sim_time, 'vo': vo_source,
                            'nvblox': LaunchConfiguration('nvblox')}.items()),

        _include('teleop.launch.py',
                 condition=IfCondition(LaunchConfiguration('use_teleop'))),

        _include('web_teleop.launch.py',
                 condition=IfCondition(LaunchConfiguration('use_web_teleop'))),

        ]),
    ])
