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

"""Lane following: bird's eye projection, lane detection, control, mux.

This is the baseline every mission falls back to, so it is also the launch
file to use when tuning the controller.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('autorace_bringup')
    params = os.path.join(pkg_bringup, 'param', 'perception_sim.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    camera_topic = LaunchConfiguration('camera_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')

    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # The downward looking lane camera; the forward one is for signs.
        DeclareLaunchArgument('camera_topic', default_value='/camera_lane/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera_lane/camera_info'),
    ]

    sim_time = {'use_sim_time': use_sim_time}

    bev_projector = Node(
        package='autorace_perception',
        executable='bev_projector',
        name='bev_projector',
        output='screen',
        parameters=[params, sim_time],
        remappings=[
            ('/camera/image_input', camera_topic),
            ('/camera/camera_info', camera_info_topic),
            ('/camera/image_output', '/camera/image_projected'),
        ],
    )

    detect_lane = Node(
        package='autorace_perception',
        executable='detect_lane',
        name='detect_lane',
        output='screen',
        parameters=[params, sim_time],
        remappings=[
            ('/detect/image_input', '/camera/image_projected'),
            ('/detect/image_output', '/detect/image_lane'),
        ],
    )

    lane_controller = Node(
        package='autorace_core',
        executable='lane_controller',
        name='lane_controller',
        output='screen',
        parameters=[params, sim_time],
    )

    cmd_vel_mux = Node(
        package='autorace_core',
        executable='cmd_vel_mux',
        name='cmd_vel_mux',
        output='screen',
        parameters=[sim_time],
    )

    return LaunchDescription(
        declare_args + [bev_projector, detect_lane, lane_controller, cmd_vel_mux]
    )
