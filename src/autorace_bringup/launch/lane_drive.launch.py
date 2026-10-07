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

profile:=real (default) reads param/perception_real.yaml and wall time;
profile:=sim reads perception_sim.yaml (pass use_sim_time:=true as well).
compressed:=true takes the camera as <camera_topic>/compressed and decodes it
here, on the PC, so only JPEG frames cross the Wi-Fi from the robot.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from autorace_bringup.launch_helpers import decoded_topic, republish


def launch_setup(context):
    pkg_bringup = get_package_share_directory('autorace_bringup')
    profile = LaunchConfiguration('profile').perform(context)
    params = os.path.join(pkg_bringup, 'param', f'perception_{profile}.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    sim_time = {'use_sim_time': use_sim_time}
    compressed = LaunchConfiguration('compressed').perform(context).lower() == 'true'
    camera_topic = LaunchConfiguration('camera_topic').perform(context)
    camera_info_topic = LaunchConfiguration('camera_info_topic').perform(context)

    nodes = []
    if compressed:
        nodes.append(republish(camera_topic, use_sim_time, 'republish_lane'))
        camera_topic = decoded_topic(camera_topic)

    nodes.append(Node(
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
    ))

    nodes.append(Node(
        package='autorace_perception',
        executable='detect_lane',
        name='detect_lane',
        output='screen',
        parameters=[params, sim_time],
        remappings=[
            ('/detect/image_input', '/camera/image_projected'),
            ('/detect/image_output', '/detect/image_lane'),
        ],
    ))

    nodes.append(Node(
        package='autorace_core',
        executable='lane_controller',
        name='lane_controller',
        output='screen',
        parameters=[params, sim_time],
    ))

    nodes.append(Node(
        package='autorace_core',
        executable='cmd_vel_mux',
        name='cmd_vel_mux',
        output='screen',
        parameters=[params, sim_time],
    ))
    return nodes


def generate_launch_description():
    declare_args = [
        # real: wall clock and param/perception_real.yaml. sim: perception_sim.yaml.
        DeclareLaunchArgument('profile', default_value='real'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # The downward looking lane camera; the forward one is for signs.
        DeclareLaunchArgument('camera_topic', default_value='/camera_lane/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera_lane/camera_info'),
        # true: subscribe to <camera_topic>/compressed and decode on this machine.
        DeclareLaunchArgument('compressed', default_value='true'),
    ]
    return LaunchDescription(declare_args + [OpaqueFunction(function=launch_setup)])
