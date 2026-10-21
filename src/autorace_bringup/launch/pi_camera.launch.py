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

The CSI camera fast (csi_camera's docstring): 90 fps needs a 16:9 size and
the GPU's own JPEG,
  forward_width:=384 forward_height:=216 forward_fps:=90 passthrough:=true quality:=60
with exposure_us:=8000 to fix the exposure (<= 1e6 / fps), stamp:=capture for
the time each frame was taken, stats:=true for a line a second. The defaults
are the camera as before (320x240 at fps, re-encoded at quality 80).
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

    def integer(name):
        return ParameterValue(LaunchConfiguration(name), value_type=int)

    common = {'stamp': LaunchConfiguration('stamp'),
              'stats': ParameterValue(LaunchConfiguration('stats'), value_type=bool)}
    nodes = []
    if LaunchConfiguration('forward').perform(context).lower() == 'true':
        nodes.append(Node(
            package='autorace_bringup', executable='csi_camera', name='csi_camera',
            namespace='camera', output='screen',
            parameters=[{'v4l2_name': 'camera0', 'frame_id': 'camera_link',
                         'width': integer('forward_width'), 'height': integer('forward_height'),
                         'fps': integer('forward_fps' if LaunchConfiguration('forward_fps')
                                        .perform(context) else 'fps'),
                         'passthrough': ParameterValue(LaunchConfiguration('passthrough'),
                                                       value_type=bool),
                         'jpeg_quality': integer('quality'),
                         'exposure_us': integer('exposure_us'),
                         'controls': LaunchConfiguration('controls'),
                         **common}]))
    lane = LaunchConfiguration('lane').perform(context).lower()
    if lane == 'true' or (lane == 'auto' and os.path.exists(C920)):
        nodes.append(Node(
            package='autorace_bringup', executable='csi_camera', name='csi_camera',
            namespace='camera_lane', output='screen',
            parameters=[{'v4l2_device': C920, 'width': 432, 'height': 240, 'fps': fps,
                         'passthrough': True, 'fov_deg': 70.4,
                         'frame_id': 'camera_lane_link', **common}]))
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
        # The CSI camera (see above); forward_fps empty: fps.
        DeclareLaunchArgument('forward_width', default_value='320'),
        DeclareLaunchArgument('forward_height', default_value='240'),
        DeclareLaunchArgument('forward_fps', default_value=''),
        DeclareLaunchArgument('passthrough', default_value='false',
                              description="send the GPU's MJPEG as it is"),
        DeclareLaunchArgument('quality', default_value='80', description='JPEG quality'),
        DeclareLaunchArgument('exposure_us', default_value='0', description='0: automatic'),
        DeclareLaunchArgument('controls', default_value='',
                              description="other V4L2 controls, 'name=value,...'"),
        # Both cameras.
        DeclareLaunchArgument('stamp', default_value='publish',
                              description="'capture': the time the frame was taken"),
        DeclareLaunchArgument('stats', default_value='false',
                              description='log rate, size, Mbps and delay once a second'),
        OpaqueFunction(function=cameras),
    ])
