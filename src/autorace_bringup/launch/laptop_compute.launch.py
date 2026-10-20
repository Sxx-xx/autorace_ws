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

"""Everything that runs on the PC: the other half of pi_sensors.launch.py.

It is race.launch.py, under the name that says where it runs; every argument
of race.launch.py works here as well (missions:=tunnel, auto_start:=true, ...).

  in    /camera/image_raw/compressed, /camera_lane/image_raw/compressed (JPEG,
        best effort), /scan, /odom from the Pi
  here  decoding, the bird's eye view, lane detection, the sign, light and bar
        detectors, the mission manager and missions, the lane controller and
        the cmd_vel mux
  out   /cmd_vel to the Pi (reliable, depth 1)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    race = os.path.join(
        get_package_share_directory('autorace_bringup'), 'launch', 'race.launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(race)),
    ])
