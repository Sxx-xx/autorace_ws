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

"""Bring up the AutoRace course in Gazebo Harmonic with a TurtleBot3 on it."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

# The start pose of the 2020 course: on the start line, facing the lights.
START_X = '0.8'
START_Y = '-1.747'
START_YAW = '0.0'

ROBOT_MODEL = 'waffle_pi'
# Our own copy of the waffle_pi: same chassis, AutoRace camera mast.
ROBOT_SDF_MODEL = 'autorace_waffle_pi'


def generate_launch_description():
    # turtlebot3_gazebo reads this at launch-description build time.
    os.environ.setdefault('TURTLEBOT3_MODEL', ROBOT_MODEL)

    pkg_sim = get_package_share_directory('autorace_sim')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    world = PathJoinSubstitution([pkg_sim, 'worlds', 'autorace_2023.sdf'])

    robot_sdf = os.path.join(pkg_sim, 'models', ROBOT_SDF_MODEL, 'model.sdf')
    bridge_config = os.path.join(pkg_sim, 'params', 'autorace_bridge.yaml')

    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Run the Gazebo GUI as well as the server.'),
        DeclareLaunchArgument('x_pose', default_value=START_X),
        DeclareLaunchArgument('y_pose', default_value=START_Y),
        DeclareLaunchArgument('yaw', default_value=START_YAW),
    ]

    set_model_env = SetEnvironmentVariable('TURTLEBOT3_MODEL', ROBOT_MODEL)
    set_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(pkg_sim, 'models')
    )
    set_tb3_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(pkg_tb3_gazebo, 'models')
    )

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v2 ', world],
                          'on_exit_shutdown': 'true'}.items()
    )

    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        condition=IfCondition(gui),
        launch_arguments={'gz_args': '-g -v2 '}.items()
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', ROBOT_SDF_MODEL,
            '-file', robot_sdf,
            '-x', LaunchConfiguration('x_pose'),
            '-y', LaunchConfiguration('y_pose'),
            '-z', '0.02',
            '-Y', LaunchConfiguration('yaw'),
        ],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='autorace_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
        output='screen',
    )

    # Services cannot be listed in the bridge config file, so they get their
    # own bridge process.
    service_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='autorace_service_bridge',
        arguments=['/world/autorace/set_pose@ros_gz_interfaces/srv/SetEntityPose'],
        output='screen',
    )

    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='autorace_image_bridge',
        arguments=['/camera/image_raw'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    traffic_light = Node(
        package='autorace_sim',
        executable='sim_traffic_light',
        name='sim_traffic_light',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    level_crossing = Node(
        package='autorace_sim',
        executable='sim_level_crossing',
        name='sim_level_crossing',
        parameters=[{
            'use_sim_time': use_sim_time,
            'start_x': float(START_X),
            'start_y': float(START_Y),
        }],
        output='screen',
    )

    return LaunchDescription(
        declare_args + [
            set_model_env,
            set_resources,
            set_tb3_resources,
            gz_server,
            gz_client,
            robot_state_publisher,
            spawn_robot,
            bridge,
            service_bridge,
            image_bridge,
            traffic_light,
            level_crossing,
        ]
    )
