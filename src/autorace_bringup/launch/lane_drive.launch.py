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

"""Lane following only: camera pipeline, lane detection, PD control, mux.

This is the baseline every mission falls back to, so it is also the launch
file to use when tuning the controller.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('autorace_bringup')
    pkg_detect = get_package_share_directory('turtlebot3_autorace_detect')

    use_sim_time = LaunchConfiguration('use_sim_time')
    max_vel = LaunchConfiguration('max_vel')

    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('calibration_mode', default_value='False'),
        DeclareLaunchArgument('max_vel', default_value='0.12'),
    ]

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'camera.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'calibration_mode': LaunchConfiguration('calibration_mode'),
        }.items(),
    )

    detect_lane = Node(
        package='turtlebot3_autorace_detect',
        executable='detect_lane',
        name='detect_lane',
        output='screen',
        parameters=[
            os.path.join(pkg_detect, 'param', 'lane', 'lane.yaml'),
            {'is_detection_calibration_mode': LaunchConfiguration('calibration_mode'),
             'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('/detect/image_input', '/camera/image_projected'),
            ('/detect/image_input/compressed', '/camera/image_projected/compressed'),
            ('/detect/image_output', '/detect/image_lane'),
            ('/detect/image_output/compressed', '/detect/image_lane/compressed'),
            ('/detect/image_output_sub1', '/detect/image_white_lane_marker'),
            ('/detect/image_output_sub1/compressed',
             '/detect/image_white_lane_marker/compressed'),
            ('/detect/image_output_sub2', '/detect/image_yellow_lane_marker'),
            ('/detect/image_output_sub2/compressed',
             '/detect/image_yellow_lane_marker/compressed'),
        ],
    )

    control_lane = Node(
        package='turtlebot3_autorace_mission',
        executable='control_lane',
        name='control_lane',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/control/lane', '/detect/lane'),
            # The mux owns /cmd_vel; lane following is its lowest priority input.
            ('/control/cmd_vel', '/cmd_vel/lane'),
        ],
    )

    cmd_vel_mux = Node(
        package='autorace_core',
        executable='cmd_vel_mux',
        name='cmd_vel_mux',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # control_lane defaults to 0.1 m/s and only learns the cruise speed from a
    # topic, so keep publishing it until the mission manager takes over.
    set_max_vel = TimerAction(
        period=3.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '-r', '1', '/control/max_vel',
                 'std_msgs/msg/Float64', ['{data: ', max_vel, '}']],
            output='log',
        )],
    )

    return LaunchDescription(
        declare_args + [camera, detect_lane, control_lane, cmd_vel_mux, set_max_vel]
    )
