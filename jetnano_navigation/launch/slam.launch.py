"""Mapping and map-based localisation, in three modes.

slam_toolbox can serialise its pose-graph and pick it up again later. That is
worth more than the usual "map once, then localise forever" workflow, because
the third mode below does not exist with AMCL and a static image: you can walk
into a room the robot has never seen and have it added to the map it already
had.

    mode:=mapping                                  (default)
        Start from nothing and build a map. What you did before.

    mode:=continue  map:=~/maps/home
        Load an existing map AND keep extending it. Known rooms are
        remembered; new ones get added. Loop closure still runs, so revisiting
        an old area corrects drift across the whole graph.

    mode:=localization  map:=~/maps/home
        Load an existing map and only localise in it. The map is never
        modified. This is the safe choice for a finished, trusted map.

`map` is the serialised pose-graph WITHOUT an extension. slam_toolbox writes
two files, <name>.posegraph and <name>.data; give it <name>.

To save what you have mapped:

    ros2 run jetnano_navigation save_map ~/maps/home

which serialises the graph AND writes home.pgm / home.yaml so you can look at
it, hand it to other tools, or use it with AMCL if you ever want to.

Whatever the mode, slam_toolbox publishes map -> odom and nothing else. The
EKF keeps odom -> base_footprint.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def _slam_node(context, *args, **kwargs):
    nav_pkg = get_package_share_directory('jetnano_navigation')
    default_params = os.path.join(nav_pkg, 'config', 'slam_toolbox.yaml')

    mode = context.launch_configurations.get('mode', 'mapping')
    map_path = context.launch_configurations.get('map', '')
    params_file = context.launch_configurations.get('params_file', default_params)
    use_sim_time = context.launch_configurations.get('use_sim_time', 'false')

    if mode not in ('mapping', 'continue', 'localization'):
        raise RuntimeError(
            f"mode must be mapping, continue or localization - got '{mode}'")

    if mode in ('continue', 'localization') and not map_path:
        raise RuntimeError(f"mode:={mode} needs map:=<serialised map, no extension>")

    if map_path:
        map_path = os.path.expanduser(map_path)
        # Catch the easy mistake early rather than at runtime.
        if map_path.endswith(('.posegraph', '.data', '.yaml', '.pgm')):
            map_path = os.path.splitext(map_path)[0]
        if not os.path.exists(map_path + '.posegraph'):
            raise RuntimeError(
                f'no serialised map at {map_path}.posegraph - '
                'run mode:=mapping first, then save it with save_map')

    overrides = {'use_sim_time': use_sim_time == 'true'}

    if mode == 'localization':
        executable = 'localization_slam_toolbox_node'
        overrides['mode'] = 'localization'
        overrides['map_file_name'] = map_path
        # Start where the map's origin is; override with map_start_pose if the
        # robot is put down somewhere else.
        overrides['map_start_at_dock'] = True
    else:
        executable = 'async_slam_toolbox_node'
        overrides['mode'] = 'mapping'
        if mode == 'continue':
            overrides['map_file_name'] = map_path
            overrides['map_start_at_dock'] = True

    return [
        Node(
            package='slam_toolbox',
            executable=executable,
            name='slam_toolbox',
            output='screen',
            parameters=[params_file, overrides],
        ),
        # slam_toolbox is a LIFECYCLE node on Jazzy. Launched on its own it
        # comes up "unconfigured": no subscriptions, no /map, no error - it
        # simply sits there waiting to be told to start. This manager drives
        # it through configure -> activate. Without it, nothing happens and
        # nothing says why.
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['slam_toolbox'],
                'bond_timeout': 0.0,
            }],
        ),
    ]


def generate_launch_description():
    nav_pkg = get_package_share_directory('jetnano_navigation')
    default_params = os.path.join(nav_pkg, 'config', 'slam_toolbox.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='mapping',
            description='mapping | continue | localization'),
        DeclareLaunchArgument(
            'map', default_value='',
            description='Serialised pose-graph, no extension (continue/localization)'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        OpaqueFunction(function=_slam_node),
    ])
