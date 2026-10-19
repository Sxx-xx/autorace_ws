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

"""Pieces shared by the launch files."""

from launch_ros.actions import Node


def decoded_topic(image_topic):
    """The raw topic a compressed stream is decoded onto: /x/image_raw -> /x/image_decoded."""
    base = image_topic.rsplit('/', 1)[0]
    return base + '/image_decoded'


def republish(image_topic, use_sim_time, name, out_topic=None):
    """image_transport republish: <image_topic>/compressed -> decoded raw image.

    out_topic defaults to decoded_topic(image_topic), what the detectors read;
    a viewer passes its own so that it never doubles their frames.

    Run on the PC: only the JPEG frames cross the Wi-Fi, and the robot's raw
    topic goes unsubscribed (DDS sends nothing nobody listens to). The input is
    read best effort, depth 1: republish's own default (reliable, 10) does not
    match the robot's best effort camera, and over Wi-Fi a resent frame is a
    stale one.
    """
    return Node(
        package='image_transport',
        executable='republish',
        name=name,
        output='screen',
        # Jazzy's republish reads the transports from parameters; the positional
        # arguments of older versions are ignored there (it then waits for a
        # raw input that never comes).
        parameters=[{'use_sim_time': use_sim_time,
                     'in_transport': 'compressed', 'out_transport': 'raw',
                     f'qos_overrides.{image_topic}/compressed.subscription.reliability': 'best_effort',
                     f'qos_overrides.{image_topic}/compressed.subscription.depth': 1}],
        remappings=[
            ('in/compressed', image_topic + '/compressed'),
            ('out', out_topic or decoded_topic(image_topic)),
        ],
    )
