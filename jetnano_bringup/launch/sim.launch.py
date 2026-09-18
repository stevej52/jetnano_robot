"""Exercise the whole stack on a desk, with no robot attached.

    ros2 launch jetnano_bringup sim.launch.py

Starts the robot description, a fake lidar in a fake room, a fake odom frame,
and the PCA9685 driver in simulate mode behind twist_mux. Add SLAM in another
terminal:

    ros2 launch jetnano_navigation slam.launch.py mode:=mapping
    ros2 run jetnano_navigation save_map ~/maps/desk

This exists because the robot is in storage. It is not a physics simulator and
it will not tell you whether the servo limits are right - it tells you whether
the graph is wired up correctly, which is the part that used to be wrong.

The fake sensors publish odom -> base_footprint themselves, so the EKF is NOT
started here. Running both would mean two publishers of one transform.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('jetnano_bringup')

    return LaunchDescription([
        DeclareLaunchArgument('drive', default_value='true',
                              description='Move the fake robot so SLAM sees motion'),
        DeclareLaunchArgument('room_width', default_value='6.0'),
        DeclareLaunchArgument('room_height', default_value='4.0'),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            PathJoinSubstitution([share, 'launch', 'description.launch.py']))),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([share, 'launch', 'drive.launch.py'])),
            launch_arguments={'simulate': 'true'}.items()),

        Node(
            package='jetnano_bringup',
            executable='fake_sensors',
            name='fake_sensors',
            output='screen',
            parameters=[{
                'drive': LaunchConfiguration('drive'),
                'room_width': LaunchConfiguration('room_width'),
                'room_height': LaunchConfiguration('room_height'),
            }],
        ),
    ])
