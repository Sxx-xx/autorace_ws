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

"""The race: lane following, sign detection, the mission manager and missions.

This runs on the PC. The robot runs robot.launch.py (drivers and cameras).

profile:=real (default) reads param/mission_real.yaml and
perception_real.yaml with wall time; profile:=sim reads the *_sim.yaml files
(pass use_sim_time:=true as well). compressed:=true decodes the cameras'
/compressed streams here so only JPEG frames cross the Wi-Fi.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from autorace_bringup.launch_helpers import decoded_topic, republish


def launch_setup(context):
    pkg_bringup = get_package_share_directory('autorace_bringup')
    profile = LaunchConfiguration('profile').perform(context)
    params = os.path.join(pkg_bringup, 'param', f'mission_{profile}.yaml')
    perception_params = os.path.join(pkg_bringup, 'param', f'perception_{profile}.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    sim_time = {'use_sim_time': use_sim_time}
    auto_start = LaunchConfiguration('auto_start').perform(context).lower() == 'true'
    missions = LaunchConfiguration('missions').perform(context).replace(' ', '')
    compressed = LaunchConfiguration('compressed').perform(context).lower() == 'true'
    sign_camera = LaunchConfiguration('sign_camera_topic').perform(context)
    lane_camera = LaunchConfiguration('lane_camera_topic').perform(context)
    if not sign_camera:
        # The real robot reads the signs, the traffic light and the level
        # crossing with its one C920 (15.5 cm up, 7.3 deg down: the picture
        # reaches 14 deg above the horizon). The sim has a forward camera.
        sign_camera = lane_camera if profile == 'real' else '/camera/image_raw'

    nodes = []

    nodes.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'lane_drive.launch.py')
        ),
        launch_arguments={
            'profile': profile,
            'use_sim_time': use_sim_time,
            'compressed': LaunchConfiguration('compressed'),
            'camera_topic': LaunchConfiguration('lane_camera_topic'),
            'camera_info_topic': LaunchConfiguration('lane_camera_info_topic'),
        }.items()
    ))

    if compressed:
        if sign_camera != lane_camera:
            nodes.append(republish(sign_camera, use_sim_time, 'republish_forward'))
        # One camera for both: lane_drive's decoder already decodes it.
        sign_camera = decoded_topic(sign_camera)
    sign_input = [('/detect/image_input', sign_camera)]

    def perception(executable, remappings, param_file=params):
        return Node(
            package='autorace_perception', executable=executable, name=executable,
            output='screen', parameters=[param_file, sim_time], remappings=remappings,
        )

    def mission(executable, extra=None):
        return Node(
            package='autorace_core', executable=executable, name=executable,
            output='screen', parameters=[params, sim_time] + ([extra] if extra else []),
        )

    nodes.append(perception('detect_sign', sign_input))
    nodes.append(perception('detect_traffic_light', sign_input))
    # The stop line is on the road, in the lane camera's bird's eye view.
    nodes.append(perception('detect_stop_line',
                            [('/detect/image_input', '/camera/image_projected')],
                            perception_params))
    nodes.append(perception('detect_level_crossing', sign_input))

    # missions:='' leaves the yaml's enabled_missions in force; a value
    # overrides it (section tests).
    manager_overrides = {'auto_start': auto_start}
    if missions:
        manager_overrides['enabled_missions'] = missions
    nodes.append(mission('mission_manager', manager_overrides))

    for name in ('traffic_light_mission', 'intersection_mission', 'construction_mission',
                 'parking_mission', 'level_crossing_mission', 'tunnel_mission'):
        nodes.append(mission(name))
    return nodes


def generate_launch_description():
    declare_args = [
        # real: wall clock and param/*_real.yaml. sim: *_sim.yaml.
        DeclareLaunchArgument('profile', default_value='real'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # true starts the run at once instead of waiting at the stop line
        # for the green light.
        DeclareLaunchArgument('auto_start', default_value='false'),
        # Comma-separated missions to run; empty keeps the yaml's list.
        DeclareLaunchArgument('missions', default_value=''),
        # The camera for the signs, light and bar, and the lane camera (road).
        # Empty: the lane camera on the real robot, /camera/image_raw in the sim.
        DeclareLaunchArgument('sign_camera_topic', default_value=''),
        DeclareLaunchArgument('lane_camera_topic', default_value='/camera_lane/image_raw'),
        DeclareLaunchArgument('lane_camera_info_topic', default_value='/camera_lane/camera_info'),
        # true: subscribe to the cameras' /compressed streams and decode here.
        DeclareLaunchArgument('compressed', default_value='true'),
    ]
    return LaunchDescription(declare_args + [OpaqueFunction(function=launch_setup)])
