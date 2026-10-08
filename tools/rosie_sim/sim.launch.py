"""Rosie's Nav2 (her own nav2.yaml and behaviour trees) on the simulator, sim time."""
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

D = os.path.dirname(os.path.realpath(__file__))
P = f'{D}/nav2_sim.yaml'
SIM = {'use_sim_time': True}
NAMES = ['map_server', 'controller_server', 'planner_server', 'behavior_server',
         'velocity_smoother', 'collision_monitor', 'bt_navigator']


def generate_launch_description():
    def nav(pkg, exe):
        return Node(package=pkg, executable=exe, name=exe, parameters=[P, SIM], output='screen')
    return LaunchDescription([
        ExecuteProcess(cmd=['python3', f'{D}/rosie_sim.py', '--ros-args', '-p', 'speed:=' + os.environ.get('SIM_SPEED', '3.0'), '-p', 'map:=' + os.environ.get('PHYS_MAP', f'{D}/map/home.yaml'),
                                    '-p', 'low_map:=' + os.environ.get('LOW_MAP', 'none'),
                                    '-p', 'range_noise:=' + os.environ.get('RANGE_NOISE', '0.0')], output='screen'),
    ] + ([ExecuteProcess(cmd=['python3', f'{D}/getout.py', '--ros-args', '-p', 'use_sim_time:=true'], output='screen')]
         if os.environ.get('GETOUT') == '1' else []) + (
        [ExecuteProcess(cmd=['python3', '-c', 'from jetnano_navigation.nav_helper import main; main()',
                             '--ros-args', '-p', 'cmd_topic:=cmd_vel', '-p', 'sweep:=false'],
                        additional_env={'PYTHONPATH': f'{D}/pylib:' + os.environ.get('PYTHONPATH', '')},
                        output='screen')]
        if os.environ.get('RESCUE') == '1' else []) + [
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=[SIM, {'yaml_filename': os.environ.get('PLAN_MAP', f'{D}/map/home.yaml')}]),
        nav('nav2_controller', 'controller_server'),
        nav('nav2_planner', 'planner_server'),
        nav('nav2_behaviors', 'behavior_server'),
        nav('nav2_velocity_smoother', 'velocity_smoother'),
        nav('nav2_collision_monitor', 'collision_monitor'),
        nav('nav2_bt_navigator', 'bt_navigator'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
             output='screen', parameters=[SIM, {'autostart': True, 'node_names': NAMES, 'bond_timeout': 0.0}]),
    ])
