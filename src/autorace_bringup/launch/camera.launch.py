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

"""Brightness compensated camera image for the sign and light detectors.

The lane pipeline works on the bird's eye view and is launched separately; this
is the forward view that the sign, traffic light and level crossing detectors
read.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    camera_topic = LaunchConfiguration('camera_topic')

    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('camera_topic', default_value='/camera/image_raw'),
    ]

    image_compensation = Node(
        package='turtlebot3_autorace_camera',
        executable='image_compensation',
        namespace='camera',
        name='image_compensation',
        output='screen',
        parameters=[
            {'camera.extrinsic_camera_calibration.clip_hist_percent': 1.0,
             'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('/camera/image_input', camera_topic),
            ('/camera/image_input/compressed', [camera_topic, '/compressed']),
            ('/camera/image_output', '/camera/image_compensated'),
            ('/camera/image_output/compressed', '/camera/image_compensated/compressed'),
        ],
    )

    return LaunchDescription(declare_args + [image_compensation])
