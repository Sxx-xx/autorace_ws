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

Missions are added here as they are written; so far there are the traffic
light, the intersection, the construction zone, parking, the level
crossing and the tunnel.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_bringup = get_package_share_directory('autorace_bringup')
    params = os.path.join(pkg_bringup, 'param', 'mission_sim.yaml')
    perception_params = os.path.join(pkg_bringup, 'param', 'perception_sim.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    auto_start = LaunchConfiguration('auto_start')

    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # true starts the run at once instead of waiting at the stop line
        # for the green light.
        DeclareLaunchArgument('auto_start', default_value='false'),
        # Comma-separated missions to run; empty for all. For section tests.
        DeclareLaunchArgument('missions', default_value=''),
        # The forward camera; the lane camera looks at the road.
        DeclareLaunchArgument('sign_camera_topic', default_value='/camera/image_raw'),
    ]

    sim_time = {'use_sim_time': use_sim_time}

    lane_drive = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'lane_drive.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    detect_sign = Node(
        package='autorace_perception',
        executable='detect_sign',
        name='detect_sign',
        output='screen',
        parameters=[params, sim_time],
        remappings=[('/detect/image_input', LaunchConfiguration('sign_camera_topic'))],
    )

    detect_traffic_light = Node(
        package='autorace_perception',
        executable='detect_traffic_light',
        name='detect_traffic_light',
        output='screen',
        parameters=[params, sim_time],
        remappings=[('/detect/image_input', LaunchConfiguration('sign_camera_topic'))],
    )

    # The stop line is on the road, in the lane camera's bird's eye view.
    detect_stop_line = Node(
        package='autorace_perception',
        executable='detect_stop_line',
        name='detect_stop_line',
        output='screen',
        parameters=[perception_params, sim_time],
        remappings=[('/detect/image_input', '/camera/image_projected')],
    )

    traffic_light_mission = Node(
        package='autorace_core',
        executable='traffic_light_mission',
        name='traffic_light_mission',
        output='screen',
        parameters=[params, sim_time],
    )

    mission_manager = Node(
        package='autorace_core',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
        parameters=[
            params, sim_time,
            {'auto_start': ParameterValue(auto_start, value_type=bool),
             'enabled_missions': ParameterValue(LaunchConfiguration('missions'), value_type=str)},
        ],
    )

    intersection_mission = Node(
        package='autorace_core',
        executable='intersection_mission',
        name='intersection_mission',
        output='screen',
        parameters=[params, sim_time],
    )

    construction_mission = Node(
        package='autorace_core',
        executable='construction_mission',
        name='construction_mission',
        output='screen',
        parameters=[params, sim_time],
    )

    parking_mission = Node(
        package='autorace_core',
        executable='parking_mission',
        name='parking_mission',
        output='screen',
        parameters=[params, sim_time],
    )

    detect_level_crossing = Node(
        package='autorace_perception',
        executable='detect_level_crossing',
        name='detect_level_crossing',
        output='screen',
        parameters=[params, sim_time],
        remappings=[('/detect/image_input', LaunchConfiguration('sign_camera_topic'))],
    )

    level_crossing_mission = Node(
        package='autorace_core',
        executable='level_crossing_mission',
        name='level_crossing_mission',
        output='screen',
        parameters=[params, sim_time],
    )

    tunnel_mission = Node(
        package='autorace_core',
        executable='tunnel_mission',
        name='tunnel_mission',
        output='screen',
        parameters=[params, sim_time],
    )

    return LaunchDescription(
        declare_args + [
            lane_drive,
            detect_sign,
            detect_traffic_light,
            detect_stop_line,
            mission_manager,
            traffic_light_mission,
            intersection_mission,
            construction_mission,
            parking_mission,
            detect_level_crossing,
            level_crossing_mission,
            tunnel_mission,
        ]
    )
