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

"""Show a robot camera on the PC: decode <camera_topic>/compressed and open rqt_image_view.

The robot's cameras send best effort, which a viewer's default reliable
subscription does not match; republish (launch_helpers) reads them best effort
and hands the decoded image to the viewer on this machine.

  ros2 launch autorace_bringup view_camera.launch.py                      # /camera (CSI)
  ros2 launch autorace_bringup view_camera.launch.py camera_topic:=/camera_lane/image_raw
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from autorace_bringup.launch_helpers import decoded_topic, republish


def nodes(context):
    camera_topic = LaunchConfiguration('camera_topic').perform(context)
    name = camera_topic.strip('/').split('/')[0]
    return [
        republish(camera_topic, False, f'republish_view_{name}'),
        Node(package='rqt_image_view', executable='rqt_image_view', name=f'view_{name}',
             arguments=[decoded_topic(camera_topic)], output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_topic', default_value='/camera/image_raw'),
        OpaqueFunction(function=nodes),
    ])
