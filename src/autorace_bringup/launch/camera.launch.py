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

"""Camera pipeline: bird's eye projection and brightness compensation.

In simulation the camera is distortion free, so the raw image goes straight
into the pipeline without rectification.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('autorace_bringup')

    calibration_mode = LaunchConfiguration('calibration_mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    source_topic = LaunchConfiguration('source_topic')

    declare_args = [
        DeclareLaunchArgument('calibration_mode', default_value='False',
                              description='Publish the projection guide overlay and accept '
                                          'live parameter changes.'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('source_topic', default_value='/camera/image_raw',
                              description='Rectified colour image to feed the pipeline.'),
    ]

    projection_param = os.path.join(pkg_bringup, 'param', 'projection_sim.yaml')

    image_projection = Node(
        package='turtlebot3_autorace_camera',
        executable='image_projection',
        namespace='camera',
        name='image_projection',
        output='screen',
        parameters=[
            projection_param,
            {'is_extrinsic_camera_calibration_mode': calibration_mode,
             'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('/camera/image_input', source_topic),
            ('/camera/image_input/compressed', [source_topic, '/compressed']),
            ('/camera/image_output', '/camera/image_projected'),
            ('/camera/image_output/compressed', '/camera/image_projected/compressed'),
            ('/camera/image_calib', '/camera/image_extrinsic_calib'),
            ('/camera/image_calib/compressed', '/camera/image_extrinsic_calib/compressed'),
        ],
    )

    image_compensation = Node(
        package='turtlebot3_autorace_camera',
        executable='image_compensation',
        namespace='camera',
        name='image_compensation',
        output='screen',
        parameters=[projection_param, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/camera/image_input', source_topic),
            ('/camera/image_input/compressed', [source_topic, '/compressed']),
            ('/camera/image_output', '/camera/image_compensated'),
            ('/camera/image_output/compressed', '/camera/image_compensated/compressed'),
        ],
    )

    return LaunchDescription(declare_args + [image_projection, image_compensation])
