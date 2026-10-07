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

"""Everything that runs on the robot's Raspberry Pi: base, laser, cameras.

Includes turtlebot3_bringup's robot.launch.py (OpenCR and the LDS laser;
needs TURTLEBOT3_MODEL=burger and LDS_MODEL in the environment) and two
usb_cam nodes:

  /camera       forward, for the signs, the light and the bar
  /camera_lane  looking down at the road, for the lane and the stop line

Both publish image_raw, image_raw/compressed and camera_info in their
namespace; race.launch.py on the PC decodes the compressed streams. The
calibration files in param/ are approximate (a 62 deg lens at 320x240):
replace them with camera_calibration's output.

A CSI camera driven by camera_ros instead of usb_cam: launch it by hand in
the same namespace and pass cameras:=false here.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def camera(namespace, device, frame_id, calibration):
    return Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='camera',
        namespace=namespace,
        output='screen',
        condition=IfCondition(LaunchConfiguration('cameras')),
        parameters=[{
            'video_device': device,
            'image_width': ParameterValue(LaunchConfiguration('width'), value_type=int),
            'image_height': ParameterValue(LaunchConfiguration('height'), value_type=int),
            'framerate': ParameterValue(LaunchConfiguration('fps'), value_type=float),
            'pixel_format': LaunchConfiguration('pixel_format'),
            'io_method': 'mmap',
            'frame_id': frame_id,
            'camera_name': namespace,
            'camera_info_url': 'file://' + calibration,
            # Fixed exposure and white balance: the detectors' colour
            # thresholds are tuned once, under the venue's lights, and must
            # not drift as the camera auto-adjusts. Set exposure with
            # v4l2-ctl once the picture looks right, then write it here.
            'autoexposure': ParameterValue(LaunchConfiguration('autoexposure'), value_type=bool),
            'exposure': ParameterValue(LaunchConfiguration('exposure'), value_type=int),
            'auto_white_balance': ParameterValue(LaunchConfiguration('auto_white_balance'), value_type=bool),
            'white_balance': 4000,
        }],
    )


def generate_launch_description():
    pkg_bringup = get_package_share_directory('autorace_bringup')
    pkg_tb3 = get_package_share_directory('turtlebot3_bringup')

    declare_args = [
        DeclareLaunchArgument('base', default_value='true',
                              description='start turtlebot3_bringup (OpenCR, laser)'),
        DeclareLaunchArgument('usb_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('cameras', default_value='true',
                              description='start the two usb_cam nodes'),
        DeclareLaunchArgument('forward_device', default_value='/dev/video0'),
        DeclareLaunchArgument('lane_device', default_value='/dev/video2'),
        # 320x240 is what every pixel threshold was tuned at (sim camera).
        DeclareLaunchArgument('width', default_value='320'),
        DeclareLaunchArgument('height', default_value='240'),
        DeclareLaunchArgument('fps', default_value='30.0'),
        # yuyv is decoded on the Pi without ffmpeg; mjpeg2rgb if the camera
        # only offers MJPEG at this size (v4l2-ctl --list-formats-ext).
        DeclareLaunchArgument('pixel_format', default_value='yuyv'),
        DeclareLaunchArgument('autoexposure', default_value='true'),
        DeclareLaunchArgument('exposure', default_value='100'),
        DeclareLaunchArgument('auto_white_balance', default_value='true'),
    ]

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_tb3, 'launch', 'robot.launch.py')),
        condition=IfCondition(LaunchConfiguration('base')),
        launch_arguments={'usb_port': LaunchConfiguration('usb_port')}.items(),
    )

    forward = camera('camera', LaunchConfiguration('forward_device'), 'camera_forward',
                     os.path.join(pkg_bringup, 'param', 'camera_forward.yaml'))
    lane = camera('camera_lane', LaunchConfiguration('lane_device'), 'camera_lane',
                  os.path.join(pkg_bringup, 'param', 'camera_lane.yaml'))

    return LaunchDescription(declare_args + [base, forward, lane])
