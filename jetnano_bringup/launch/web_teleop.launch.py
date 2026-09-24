"""The driving web page: camera feed on top, a virtual joystick underneath.

    ros2 launch jetnano_bringup web_teleop.launch.py
    http://<robot>:8081/

Runs on the robot (robot.launch.py includes it), for a phone or a laptop on the
robot's Wi-Fi. The video comes from web_video_server in the Isaac container
(port 8080); the knob publishes cmd_vel_web, which twist_mux ranks between the
joystick and Nav2. Details in jetnano_teleop/web_teleop.py.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8081'),
        DeclareLaunchArgument(
            'max_linear', default_value='0.5',
            description='throttle at 100 % on the page, as a fraction of full (the ESC crawls from ~0.12)'),
        DeclareLaunchArgument(
            'max_angular', default_value='2.4',
            description='rad/s while a left/right arrow is held; 2.4 is full steering lock'),
        DeclareLaunchArgument(
            'video_url', default_value='',
            description="the MJPEG stream; empty = http://<host the page came from>:8080/stream?topic=/camera/color/image_raw"),

        Node(
            package='jetnano_teleop',
            executable='web_teleop',
            name='web_teleop',
            output='screen',
            respawn=True,
            respawn_delay=5.0,
            parameters=[{
                'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
                'max_linear': ParameterValue(LaunchConfiguration('max_linear'), value_type=float),
                'max_angular': ParameterValue(LaunchConfiguration('max_angular'), value_type=float),
                'video_url': LaunchConfiguration('video_url'),
            }],
        ),
    ])
