#!/usr/bin/env python3
#
# Copyright 2026 AutoRace Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Section test: the race with the listed missions, started by a key.

Runs race.launch.py with missions:=<mission> (the other signs are ignored)
and no auto start, with the standby timeout off so the robot stands until
told. One window (start_button) shows the cameras with the detections drawn
on them and the manager's state; SPACE in that window starts the run, p (or
1..5) asks for a listed mission without its sign, s stops the robot, q closes
the window.

  ros2 launch autorace_bringup parking_test.launch.py
  ros2 launch autorace_bringup parking_test.launch.py mission:=construction,parking

Every run is recorded (record:=false not to): a bag in <bag_dir>/<mission>_<time>
with the odometry, laser, lane lines and target, mission state, signs, both
cameras' JPEG streams and /rosout (the nodes' log lines, so what the mission
saw can be read back), and the nodes' output in ~/.ros/log/<run>/launch.log.

The robot runs ~/start_all.sh on the Pi first (and /reset for the gyro).
"""

import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


RECORDED_TOPICS = [
    '/rosout', '/odom', '/scan', '/cmd_vel', '/imu', '/battery_state',
    '/detect/lane_lines/yellow', '/detect/lane_lines/white', '/detect/lane_target',
    '/detect/lane_state', '/detect/lane_follow', '/control/max_vel',
    '/autorace/mission_state', '/autorace/sign', '/autorace/trigger', '/autorace/manual_start',
    '/camera_lane/image_raw/compressed', '/camera/image_raw/compressed',
]


def recorder(context):
    if LaunchConfiguration('record').perform(context).lower() != 'true':
        return []
    mission = LaunchConfiguration('mission').perform(context)
    bag_dir = LaunchConfiguration('bag_dir').perform(context) or os.path.join(os.getcwd(), 'bags')
    bag = os.path.join(bag_dir, f"{mission.replace(',', '+')}_{time.strftime('%H%M%S')}")
    missions = [f'/autorace/mission/{m}/active' for m in mission.split(',')]
    return [ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', bag, '--topics'] + RECORDED_TOPICS + missions,
        output='screen', name='bag_record',
    )]


def generate_launch_description():
    pkg_bringup = get_package_share_directory('autorace_bringup')
    mission = LaunchConfiguration('mission')

    race = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'race.launch.py')),
        launch_arguments={
            'profile': LaunchConfiguration('profile'),
            'auto_start': 'false',
            'missions': mission,
            # Never start on its own: the key does it.
            'standby_timeout': '1e9',
            'compressed': LaunchConfiguration('compressed'),
            'sign_camera_topic': LaunchConfiguration('sign_camera_topic'),
        }.items()
    )

    button = Node(
        package='autorace_core', executable='start_button', name='start_button',
        output='screen',
        parameters=[{'mission': mission, 'image_topic': '/detect/image_sign'}],
    )

    return LaunchDescription([
        # The nodes' output goes to ~/.ros/log/<run>/launch.log as well as the screen.
        SetEnvironmentVariable('OVERRIDE_LAUNCH_PROCESS_OUTPUT', 'both'),
        DeclareLaunchArgument('profile', default_value='real'),
        DeclareLaunchArgument('record', default_value='true'),
        # Empty: bags/ under the directory the launch was started from.
        DeclareLaunchArgument('bag_dir', default_value=''),
        # The one mission enabled, and the one the p key asks for.
        DeclareLaunchArgument('mission', default_value='parking'),
        DeclareLaunchArgument('compressed', default_value='true'),
        DeclareLaunchArgument('sign_camera_topic', default_value=''),
        race,
        button,
        OpaqueFunction(function=recorder),
    ])
