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

Whichever runs, only ONE node may publish odom -> base_footprint. Both sources
run with their transform broadcasters off and hand their estimate to the EKF on
/vo as a measurement. The EKF owns the transform. Numbers behind the choice:
docs/vo-sweep-2026-09-23.md.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix


def generate_launch_description():
    bringup_pkg = get_package_share_directory('jetnano_bringup')
    ekf_config = os.path.join(bringup_pkg, 'config', 'ekf.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    vo = LaunchConfiguration('vo')
    use_vo = LaunchConfiguration('use_visual_odometry')

    def wants(source):
        return IfCondition(PythonExpression(
            ["'", use_vo, "' == 'true' and '", vo, "' == '", source, "'"]))

    cuvslam_script = PathJoinSubstitution(
        [FindPackagePrefix('jetnano_bringup'), 'lib', 'jetnano_bringup', 'cuvslam_vo.sh'])

    return LaunchDescription([
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
            'cuvslam_jitter_ms', default_value='12.0',
            description='cuvslam image_jitter_threshold_ms: 12 for 90 fps, 19 for 60'),

        # GPU: cuVSLAM in the container. The wrapper starts the container if it
        # is stopped, refuses if the host camera driver is running, and stops
        # the launch inside the container when this launch shuts down.
        ExecuteProcess(
            name='cuvslam_vo',
            cmd=['bash', cuvslam_script,
                 LaunchConfiguration('cuvslam_infra_profile'),
                 LaunchConfiguration('cuvslam_jitter_ms'),
                 'base_link'],
            output='screen',
            condition=wants('cuvslam'),
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

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': use_sim_time}],
        ),
    ])
