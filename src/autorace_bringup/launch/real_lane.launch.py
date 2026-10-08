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

"""Lane camera, bird's eye view and lane following on the real robot.

Runs on the robot.  drive:=false (default) only looks: camera, bird's eye
view, lane detection and the web view, nothing that moves the wheels.
drive:=true adds the controller and the cmd_vel mux; the robot still stands
still until /autorace/run_active is true, and stops again when it is false.
The motors need turtlebot3_bringup robot.launch.py running alongside.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('autorace_bringup'), 'param', 'perception_real.yaml')
    drive = LaunchConfiguration('drive')
    real_time = {'use_sim_time': False}

    declare_args = [
        DeclareLaunchArgument('drive', default_value='false'),
    ]

    usb_camera = Node(
        package='autorace_perception',
        executable='usb_camera',
        name='usb_camera',
        output='screen',
        parameters=[params, real_time],
        remappings=[
            ('image_raw', '/camera_lane/image_raw'),
            ('camera_info', '/camera_lane/camera_info'),
        ],
    )

    bev_projector = Node(
        package='autorace_perception',
        executable='bev_projector',
        name='bev_projector',
        output='screen',
        parameters=[params, real_time],
        remappings=[
            ('/camera/image_input', '/camera_lane/image_raw'),
            ('/camera/camera_info', '/camera_lane/camera_info'),
            ('/camera/image_output', '/camera/image_projected'),
        ],
    )

    detect_lane = Node(
        package='autorace_perception',
        executable='detect_lane',
        name='detect_lane',
        output='screen',
        parameters=[params, real_time],
        remappings=[
            ('/detect/image_input', '/camera/image_projected'),
            ('/detect/image_output', '/detect/image_lane'),
        ],
    )

    web_view = Node(
        package='autorace_perception',
        executable='web_view',
        name='web_view',
        output='screen',
        parameters=[params, real_time],
    )

    lane_controller = Node(
        package='autorace_core',
        executable='lane_controller',
        name='lane_controller',
        output='screen',
        parameters=[params, real_time],
        condition=IfCondition(drive),
    )

    cmd_vel_mux = Node(
        package='autorace_core',
        executable='cmd_vel_mux',
        name='cmd_vel_mux',
        output='screen',
        parameters=[params, real_time],
        condition=IfCondition(drive),
    )

    return LaunchDescription(declare_args + [
        usb_camera, bev_projector, detect_lane, web_view, lane_controller, cmd_vel_mux,
    ])
