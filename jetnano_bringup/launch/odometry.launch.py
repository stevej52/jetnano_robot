"""Where the robot thinks it is: visual odometry fused with the IMU.

This chassis has no wheel encoders, so there is no wheel odometry to publish.
Visual odometry estimates motion from the RealSense instead, and
robot_localization fuses that with the BNO055 to publish odom -> base_footprint.

Two visual odometry sources, chosen with vo:=

    cuvslam   NVIDIA cuVSLAM (Isaac ROS 4.6) on the GPU, stereo on the D435's
              infrared pair, 89 Hz. Runs inside the isaac_vo container on the
              Jetson (ros2_gpu_robot/cuvslam_d435) through scripts/cuvslam_vo.sh.
              The container's own RealSense driver owns the camera, so the host
              camera must be off (robot.launch.py takes care of that).
    rtabmap   rgbd_odometry on the CPU from colour + aligned depth, ~19 Hz.
              The default here, because the simulator on the host PC has no
              container; robot.launch.py defaults to cuvslam.
    none      the EKF alone (IMU only - useful for a few seconds at most).

lidar_odom:=true adds a second, independent source: MOLA's lidar odometry on
/scan (lidar_odometry.launch.py), fused as speeds next to the camera's. It keeps
the pose moving when the camera's odometry drops out. Off by default until it
has been measured on the robot.

Whichever runs, only ONE node may publish odom -> base_footprint. Both sources
run with their transform broadcasters off and hand their estimate to the EKF on
/vo as a measurement. The EKF owns the transform. Numbers behind the choice:
docs/vo-sweep-2026-09-23.md.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix

from jetnano_bringup.launch_lock import only_one


def generate_launch_description():
    bringup_pkg = get_package_share_directory('jetnano_bringup')
    ekf_config = os.path.join(bringup_pkg, 'config', 'ekf.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    vo = LaunchConfiguration('vo')
    use_vo = LaunchConfiguration('use_visual_odometry')

    def wants(source):
        return IfCondition(PythonExpression(
            ["'", use_vo, "' == 'true' and '", vo, "' == '", source, "'"]))

    # robot.launch.py runs the VO watchdog inside the C++ safety_monitor and turns
    # this Python one off; a bare odometry.launch.py still gets it.
    py_vo_watchdog = IfCondition(PythonExpression(
        ["'", use_vo, "' == 'true' and '", vo, "' == 'cuvslam' and '",
         LaunchConfiguration('vo_watchdog'), "' == 'true'"]))

    cuvslam_script = PathJoinSubstitution(
        [FindPackagePrefix('jetnano_bringup'), 'lib', 'jetnano_bringup', 'cuvslam_vo.sh'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'vo_watchdog', default_value='true',
            description='the Python VO watchdog here (robot.launch.py: false, the C++ '
                        'safety_monitor does it)'),
        DeclareLaunchArgument(
            'vo', default_value='rtabmap', choices=['cuvslam', 'rtabmap', 'none'],
            description='visual odometry source: cuvslam (GPU, Jetson only), rtabmap (CPU), none'),
        DeclareLaunchArgument(
            'use_visual_odometry', default_value='true',
            description='false = EKF alone, whatever vo says'),
        DeclareLaunchArgument(
            'cuvslam_infra_profile', default_value='640,360,90',
            description="D435 infrared profile for cuvslam, 'W,H,FPS'; 640,360,90 measured best"),
        DeclareLaunchArgument(
            'cuvslam_jitter_ms', default_value='30.0',
            description='cuvslam image_jitter_threshold_ms (a warning threshold only; the wrapper uses 100 with nvblox)'),
        DeclareLaunchArgument(
            'color_mesh', default_value='false',
            description='nvblox only: paint the colour camera onto the 3D map (RViz, saved .ply)'),
        DeclareLaunchArgument(
            'lidar_odom', default_value='false',
            description='also MOLA lidar odometry on /scan, fused by the EKF as a second '
                        'source (needs ros-jazzy-mola-lidar-odometry)'),
        DeclareLaunchArgument(
            'nvblox', default_value='false',
            description='cuvslam only: also run nvblox 3D mapping from the same camera '
                        '(projector alternates, odometry drops to ~43 Hz, Nav2 gets a '
                        'costmap layer of what the camera sees)'),

        # One odometry source and one EKF: a second copy stops itself.
        only_one('jetnano_odometry', [

        LogInfo(msg=['visual odometry source: ', vo, ' (use_visual_odometry=', use_vo, ')']),

        # GPU: cuVSLAM in the container. The wrapper starts the container if it
        # is stopped, refuses if the host camera driver is running, and stops
        # the launch inside the container when this launch shuts down. The
        # launch inside the container shuts itself down when one of its nodes
        # dies, so the wrapper exits and is respawned here: the odometry heals
        # itself instead of staying dead until someone notices.
        ExecuteProcess(
            name='cuvslam_vo',
            cmd=['bash', cuvslam_script,
                 LaunchConfiguration('cuvslam_infra_profile'),
                 LaunchConfiguration('cuvslam_jitter_ms'),
                 'base_link',
                 LaunchConfiguration('nvblox'), LaunchConfiguration('color_mesh')],
            output='screen',
            respawn=True,
            respawn_delay=10.0,
            # The wrapper needs up to ~15 s to stop the launch in the container
            # cleanly (or kill it); the defaults (5 + 5) SIGKILLed it half-way
            # on 2026-09-23 and left an orphaned launch blocking every respawn.
            # 20 + 10 still fits inside jetnano-robot.service's TimeoutStopSec=40.
            sigterm_timeout='20',
            sigkill_timeout='10',
            condition=wants('cuvslam'),
        ),

        # When /vo goes quiet without anything dying (a camera USB glitch
        # starved cuVSLAM on 2026-09-24): hold the EKF still, then restart
        # the container launch so the wrapper above respawns it.
        Node(
            package='jetnano_bringup',
            executable='vo_watchdog',
            name='vo_watchdog',
            respawn=True,
            respawn_delay=3.0,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=py_vo_watchdog,
        ),

        # CPU: rtabmap.
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Take time from /clock. MUST be true under Gazebo, or '
                        'rtabmap stamps its output from the wall clock while the '
                        'images carry sim time and nothing in tf lines up.'),

        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='visual_odometry',
            respawn=True,
            respawn_delay=3.0,
            output='screen',
            condition=wants('rtabmap'),
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

        # Lidar: MOLA and its relay (ekf.yaml odom2).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_pkg, 'launch', 'lidar_odometry.launch.py')),
            condition=IfCondition(LaunchConfiguration('lidar_odom')),
            launch_arguments={'use_sim_time': use_sim_time}.items()),

        # The EKF's heading comes from imu/base (ekf.yaml imu0). On the robot bno055_lean
        # publishes it; Gazebo's IMU publishes only imu/data, so turn it here.
        Node(
            package='jetnano_bringup',
            executable='imu_base_relay',
            name='imu_base_relay',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(PythonExpression(["'", use_sim_time, "' == 'true'"])),
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            # ahead of everything else when process start-ups pile up (Nav2, the
            # camera pipeline): see LimitNICE in systemd/jetnano-robot.service. Where
            # that is not allowed (a manual launch, the simulator), nice says so and
            # runs it at normal priority.
            prefix='nice -n -10',
            respawn=True,
            respawn_delay=3.0,
            output='screen',
            parameters=[ekf_config, {'use_sim_time': use_sim_time}],
        ),

        ]),
    ])
