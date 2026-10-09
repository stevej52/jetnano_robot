# Copyright 2026 stevej52
# Licensed under the Apache License, Version 2.0. See LICENSE.
"""Her voice and hearing, as a stack of its own: the microphone (ears), the servo-buzz settler
(it listens through the ears), her sounds (the speaker), her English voice (speak), the
words (listen) and, since it exists for meeting people, the people detector. OFF and
unloaded unless asked for.

    ros2 launch jetnano_bringup voice.launch.py [mic_channel:=0 board_profile:=clean ...]
    ros2 run jetnano_bringup voice on|off|status      (jetnano-voice.service, see voice_switch.py)

Steve, 2026-10-01 21:40, after the reSpeaker's USB audio stream panicked the kernel mid-lap
and the servos cooked: "a universal off switch for the entire voice and listening system...
when I'm driving it around, all that is off and unloaded. When I want to show it off to
somebody, then it'll get loaded." Until then these nodes lived in robot.launch.py.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    listen_python = LaunchConfiguration('listen_python')
    venv_ok = PythonExpression(["__import__('os').path.exists('", listen_python, "')"])
    return LaunchDescription([
        DeclareLaunchArgument('use_sounds', default_value='true', description='her beeps and sounds (the speaker)'),
        DeclareLaunchArgument('use_people', default_value='true', description='the people detector (YOLOv8-pose on the GPU, 750 MB)'),
        DeclareLaunchArgument('use_ears', default_value='true', description='the reSpeaker microphone'),
        DeclareLaunchArgument('use_listen', default_value='true', description='words and replies (needs listen_python)'),
        DeclareLaunchArgument('listen_python', default_value='/home/jeston/venv-voice/bin/python3'),
        DeclareLaunchArgument('location', default_value=''),
        DeclareLaunchArgument('asr', default_value='parakeet'),
        DeclareLaunchArgument('mic_channel', default_value='1'),
        DeclareLaunchArgument('board_profile', default_value=''),
        DeclareLaunchArgument('trim_own', default_value='false'),
        DeclareLaunchArgument('tts_url', default_value='http://192.168.1.238:8092'),
        DeclareLaunchArgument('tts_voice', default_value='nicole'),
        DeclareLaunchArgument('use_settle', default_value='false'),
        DeclareLaunchArgument('lead_s', default_value='0.08'),
        DeclareLaunchArgument('pause_s', default_value='0.2'),

        # Who is in front of her (people.py -> /people), for meet. Steve, 2026-10-01: "the people
        # detector definitely should load only for meet, and if the voice and hearing are turned
        # off then the people detector needs to be turned off as well."
        Node(package='jetnano_bringup', executable='people', name='people', output='screen',
             respawn=True, respawn_delay=15.0,
             condition=IfCondition(LaunchConfiguration('use_people'))),

        Node(package='jetnano_bringup', executable='sounds', name='sounds', output='screen',
             respawn=True, respawn_delay=10.0,
             condition=IfCondition(LaunchConfiguration('use_sounds'))),

        Node(package='jetnano_bringup', executable='ears', name='ears', output='screen',
             respawn=True, respawn_delay=10.0,
             parameters=[{'channel': ParameterValue(LaunchConfiguration('mic_channel'), value_type=int),
                          'board_profile': ParameterValue(LaunchConfiguration('board_profile'), value_type=str)}],
             condition=IfCondition(LaunchConfiguration('use_ears'))),

        # Settled buzzing steering servos after a stop by ear (Steve, 2026-09-27). Off since
        # 2026-10-09: its releases did not quiet the servo (buzz_source), and stop_settle in the
        # robot stack now does what did, at every stop, with no microphone. use_settle:=true for
        # the old one (the two would fight over cmd_vel_settle).
        Node(package='jetnano_bringup', executable='settle', name='settle', output='screen',
             respawn=True, respawn_delay=10.0,
             condition=IfCondition(PythonExpression(["'", LaunchConfiguration('use_ears'), "' == 'true' and '",
                                                     LaunchConfiguration('use_settle'), "' == 'true'"]))),

        # Her English voice (text on /speak). Same venv as listen.
        Node(package='jetnano_bringup', executable='speak', name='speak', output='screen',
             respawn=True, respawn_delay=10.0,
             parameters=[{'tts_url': ParameterValue(LaunchConfiguration('tts_url'), value_type=str),
                          'tts_voice': ParameterValue(LaunchConfiguration('tts_voice'), value_type=str),
                          'lead_s': ParameterValue(LaunchConfiguration('lead_s'), value_type=float),
                          'pause_s': ParameterValue(LaunchConfiguration('pause_s'), value_type=float)}],
             prefix=[listen_python, ' '],
             condition=IfCondition(PythonExpression(["'", LaunchConfiguration('use_listen'), "' == 'true' and ", venv_ok]))),

        # Words, from the ears' audio stream; needs the ears.
        Node(package='jetnano_bringup', executable='listen', name='listen', output='screen',
             respawn=True, respawn_delay=10.0,
             parameters=[{'location': LaunchConfiguration('location'), 'asr': LaunchConfiguration('asr'),
                          'trim_own': ParameterValue(LaunchConfiguration('trim_own'), value_type=bool)}],
             prefix=[listen_python, ' '],
             condition=IfCondition(PythonExpression(["'", LaunchConfiguration('use_listen'), "' == 'true' and '",
                                                     LaunchConfiguration('use_ears'), "' == 'true' and ", venv_ok]))),
    ])
