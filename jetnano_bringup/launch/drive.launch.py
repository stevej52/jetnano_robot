"""Everything that makes the wheels turn: twist_mux, the collision guard, the PCA9685 driver.

twist_mux decides who is driving; the collision guard is the last thing before
the wheels, whoever that is; the driver is the only thing that touches the I2C
bus. Teleop, the web page and Nav2 publish to their own topics:

    cmd_vel_teleop / cmd_vel_web / cmd_vel_nav / cmd_vel_tilt
        --> twist_mux --cmd_vel_mux--> collision_guard --cmd_vel--> pca9685

The guard (nav2_collision_monitor, config/collision_guard.yaml) stops the robot
for anything the lidar - and with nvblox:=true the camera's 3D map - puts
within 30 cm in the direction of travel, and slows it to 30 % within 80 cm.
guard:=false wires twist_mux straight to the driver.

    ros2 launch jetnano_bringup drive.launch.py simulate:=true

runs the whole chain with no hardware attached; every PWM write is logged
instead of sent.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString

from jetnano_bringup.launch_lock import only_one


def generate_launch_description():
    bringup_pkg = get_package_share_directory('jetnano_bringup')
    pca_config = os.path.join(bringup_pkg, 'config', 'pca9685.yaml')
    mux_config = os.path.join(bringup_pkg, 'config', 'twist_mux.yaml')
    guard_config = os.path.join(bringup_pkg, 'config', 'collision_guard.yaml')

    simulate = LaunchConfiguration('simulate')
    i2c_bus = LaunchConfiguration('i2c_bus')
    use_sim_time = LaunchConfiguration('use_sim_time')
    guard = LaunchConfiguration('guard')
    nvblox = LaunchConfiguration('nvblox')

    # With nvblox, the camera's map is a second source for the guard. It is
    # not in the file by default: a source that never publishes stops the
    # robot for good (source_timeout), which is right for a dead lidar and
    # wrong for a camera that was never started.
    cliff = LaunchConfiguration('cliff')
    sources = PythonExpression([
        "'[\"scan\"' + (', \"nvblox\"' if '", nvblox, "' == 'true' else '') + "
        "(', \"cliff\"' if '", cliff, "' == 'true' else '') + ']'"])
    guard_params = ReplaceString(
        source_file=guard_config,
        replacements={'observation_sources: ["scan"]': ['observation_sources: ', sources]})

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
            'guard', default_value='true',
            description='the collision guard between twist_mux and the driver (needs /scan)'),
        DeclareLaunchArgument(
            'nvblox', default_value='false',
            description="also feed the guard the camera's 3D map (robot.launch.py nvblox:=true)"),
        DeclareLaunchArgument(
            'cliff', default_value='false',
            description='also feed the guard the cliff sensors (sensors.launch.py use_cliff:=true)'),
        DeclareLaunchArgument(
            'i2c_bus', default_value='7',
            description='I2C bus the PCA9685 is on. Was 1 on the Jetson Nano; '
                        'check with i2cdetect -l and i2cdetect -y -r <bus>'),

        # One driver on the I2C bus, one twist_mux: a second copy stops itself.
        only_one('jetnano_drive', [

        Node(
            package='ros2_pca9685',
            executable='pca9685_node',
            name='pca9685',
            output='screen',
            parameters=[pca_config, {
                'simulate': simulate,
                'i2c_bus': i2c_bus,
                'use_sim_time': use_sim_time,
            }],
        ),

        # twist_mux publishes cmd_vel_out; with the guard in the chain that is
        # cmd_vel_mux, which the guard turns into cmd_vel. Without it, the
        # driver listens to twist_mux directly.
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[mux_config, {'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel_out', 'cmd_vel_mux')],
            condition=IfCondition(guard),
        ),
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[mux_config, {'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel_out', 'cmd_vel')],
            condition=UnlessCondition(guard),
        ),

        GroupAction(condition=IfCondition(guard), actions=[
            # A lifecycle node: the manager below configures and activates it
            # and restarts it if it dies. Named collision_guard so it never
            # collides with the collision_monitor Nav2's own bringup starts.
            Node(
                package='nav2_collision_monitor',
                executable='collision_monitor',
                name='collision_guard',
                output='screen',
                parameters=[guard_params, {'use_sim_time': use_sim_time}],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_guard',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': True,
                    'node_names': ['collision_guard'],
                }],
            ),
            # The camera's map as points, for the guard's second source.
            Node(
                package='jetnano_bringup',
                executable='grid_to_points',
                name='grid_to_points',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(nvblox),
            ),
        ]),

        # Reads imu/data directly rather than the EKF, because the EKF runs in
        # two_d_mode and pins roll and pitch to zero. Publishes cmd_vel_tilt,
        # which twist_mux ranks above teleop.
        Node(
            package='jetnano_bringup',
            executable='tilt_guard',
            name='tilt_guard',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('use_tilt_guard')),
        ),

        ]),
    ])
