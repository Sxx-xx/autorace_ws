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

"""Everything that runs on the robot's Raspberry Pi, and nothing more: the hardware.

  base     the OpenCR (turtlebot3_node, /cmd_vel as plain Twist) and the robot's
           transforms (turtlebot3_state_publisher)
  lidar    the LDS-02 (ld_lidar, /scan)
  cameras  pi_camera.launch.py: JPEG frames on /camera and /camera_lane, best effort
  watchdog cmd_vel_watchdog: zero velocity when the PC's /cmd_vel stops coming

No image is decoded or processed here: the PC runs all of that
(laptop_compute.launch.py) and sends back /cmd_vel. Each part can be left out
(lidar:=false ...). tools/pi/start_all.sh starts the same parts as separate
processes instead, so that one can be restarted without the others.

turtlebot3_bringup's own robot.launch.py is not used: it also starts the laser
driver LDS_MODEL names, and none of its drivers is the LDS-02's ld_lidar.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('autorace_bringup')
    pkg_tb3 = get_package_share_directory('turtlebot3_bringup')
    base = IfCondition(LaunchConfiguration('base'))

    declare_args = [
        DeclareLaunchArgument('base', default_value='true',
                              description='start the OpenCR base and the robot transforms'),
        DeclareLaunchArgument('lidar', default_value='true', description='start ld_lidar'),
        DeclareLaunchArgument('watchdog', default_value='true',
                              description='stop the wheels when /cmd_vel stops coming'),
        DeclareLaunchArgument('cameras', default_value='true',
                              description='start pi_camera.launch.py'),
        DeclareLaunchArgument('usb_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('lane', default_value='auto',
                              description='C920 on /camera_lane: true, false, or auto'),
        # 15 halves the Wi-Fi load on a crowded band (the detectors were tuned at 30). The hotspot
        # is 5 GHz now (iPhone: Maximize Compatibility off); 2.4 GHz stalled for up to 0.85 s.
        DeclareLaunchArgument('fps', default_value='30'),
    ]

    return LaunchDescription(declare_args + [
        # turtlebot3_state_publisher reads the model from the environment.
        SetEnvironmentVariable('TURTLEBOT3_MODEL', os.environ.get('TURTLEBOT3_MODEL', 'burger')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_tb3, 'launch', 'turtlebot3_state_publisher.launch.py')),
            condition=base,
            launch_arguments={'use_sim_time': 'false', 'namespace': ''}.items(),
        ),
        Node(
            package='turtlebot3_node', executable='turtlebot3_ros', output='screen',
            condition=base,
            parameters=[os.path.join(pkg_bringup, 'param', 'tb3_burger.yaml'), {'namespace': ''}],
            arguments=['-i', LaunchConfiguration('usb_port')],
        ),
        Node(
            package='autorace_bringup', executable='cmd_vel_watchdog',
            name='cmd_vel_watchdog', output='screen',
            condition=IfCondition(LaunchConfiguration('watchdog')),
        ),
        Node(
            package='autorace_bringup', executable='ld_lidar', name='ld_lidar', output='screen',
            condition=IfCondition(LaunchConfiguration('lidar')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'pi_camera.launch.py')),
            condition=IfCondition(LaunchConfiguration('cameras')),
            launch_arguments={'lane': LaunchConfiguration('lane'),
                              'fps': LaunchConfiguration('fps')}.items(),
        ),
    ])
