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

"""The robot's cameras through csi_camera, on the Pi (what tools/pi/start_camera*.sh run).

  /camera       the CSI camera (forward: signs, light, bar), found by its V4L2 name
                'camera0' (legacy stack: start_x=1, bcm2835-v4l2)
  /camera_lane  the C920 (down at the lane), its own MJPEG frames passed through;
                started only when it is plugged in (lane:=auto, the default)

Both publish <ns>/image_raw/compressed and <ns>/camera_info, best effort. On
the PC, race.launch.py decodes them. fps:=15 halves the Wi-Fi load on a
crowded 2.4 GHz band (the detectors were tuned at 30).
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

C920 = '/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920-video-index0'


def cameras(context):
    fps = ParameterValue(LaunchConfiguration('fps'), value_type=int)
    nodes = []
    if LaunchConfiguration('forward').perform(context).lower() == 'true':
        nodes.append(Node(
            package='autorace_bringup', executable='csi_camera', name='csi_camera',
            namespace='camera', output='screen',
            parameters=[{'v4l2_name': 'camera0', 'width': 320, 'height': 240, 'fps': fps,
                         'frame_id': 'camera_link'}]))
    lane = LaunchConfiguration('lane').perform(context).lower()
    if lane == 'true' or (lane == 'auto' and os.path.exists(C920)):
        nodes.append(Node(
            package='autorace_bringup', executable='csi_camera', name='csi_camera',
            namespace='camera_lane', output='screen',
            parameters=[{'v4l2_device': C920, 'width': 432, 'height': 240, 'fps': fps,
                         'passthrough': True, 'fov_deg': 70.4,
                         'frame_id': 'camera_lane_link'}]))
    elif lane == 'auto':
        nodes.append(LogInfo(msg=f'lane camera not started: no {C920}'))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('forward', default_value='true',
                              description='start the CSI camera on /camera'),
        DeclareLaunchArgument('lane', default_value='auto',
                              description='C920 on /camera_lane: true, false, or auto (if plugged in)'),
        DeclareLaunchArgument('fps', default_value='30'),
        OpaqueFunction(function=cameras),
    ])
