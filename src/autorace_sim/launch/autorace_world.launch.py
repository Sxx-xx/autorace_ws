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
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

ROBOT_MODEL = 'burger'
# TurtleBot3 Burger with the AutoRace camera.
ROBOT_SDF_MODEL = 'autorace_burger'


def generate_launch_description():
    # turtlebot3_gazebo reads this at launch-description build time.
    os.environ['TURTLEBOT3_MODEL'] = ROBOT_MODEL

    pkg_sim = get_package_share_directory('autorace_sim')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Generated together with the world by scripts/gen_course_world.py.
    course_params = os.path.join(pkg_sim, 'params', 'course_sim_nodes.yaml')
    with open(os.path.join(pkg_sim, 'params', 'course.yaml')) as f:
        start = yaml.safe_load(f)['course']['start']

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    world = os.path.join(pkg_sim, 'worlds', 'autorace_course.sdf')

    robot_sdf = os.path.join(pkg_sim, 'models', ROBOT_SDF_MODEL, 'model.sdf')
    bridge_config = os.path.join(pkg_sim, 'params', 'autorace_bridge.yaml')

    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Run the Gazebo GUI as well as the server.'),
        # Faster than real time, for tests: the nodes run on simulated time,
        # but their work still takes real time, so too high a factor leaves
        # the cameras' pictures unprocessed. 0 is as fast as the machine can.
        DeclareLaunchArgument('rtf', default_value='1.0',
                              description='Real time factor of the simulation.'),
        # On the start line, facing the lights.
        DeclareLaunchArgument('x_pose', default_value=str(start['x'])),
        DeclareLaunchArgument('y_pose', default_value=str(start['y'])),
        DeclareLaunchArgument('yaw', default_value=str(start['yaw'])),
    ]

    set_model_env = SetEnvironmentVariable('TURTLEBOT3_MODEL', ROBOT_MODEL)
    set_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(pkg_sim, 'models')
    )
    set_tb3_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(pkg_tb3_gazebo, 'models')
    )

    def gz_server(context):
        # The factor lives in the world file; a copy carries the one asked for.
        rtf = float(LaunchConfiguration('rtf').perform(context))
        world_file = world
        if rtf != 1.0:
            with open(world) as f:
                sdf = f.read()
            world_file = os.path.join('/tmp', f'autorace_course_rtf{rtf:g}.sdf')
            with open(world_file, 'w') as f:
                f.write(sdf.replace('<real_time_factor>1.0</real_time_factor>',
                                    f'<real_time_factor>{rtf}</real_time_factor>'))
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': f'-r -s -v2 {world_file}',
                              'on_exit_shutdown': 'true'}.items()
        )]

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
        # Forward camera (signs, traffic light) and lane camera.
        arguments=['/camera/image_raw', '/camera_lane/image_raw'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    traffic_light = Node(
        package='autorace_sim',
        executable='sim_traffic_light',
        name='sim_traffic_light',
        parameters=[course_params, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    level_crossing = Node(
        package='autorace_sim',
        executable='sim_level_crossing',
        name='sim_level_crossing',
        parameters=[course_params, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    firmware = Node(
        package='autorace_sim',
        executable='sim_firmware',
        name='sim_firmware',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # On the wall clock: it is the one making the simulated clock.
    clock = Node(
        package='autorace_sim',
        executable='sim_clock',
        name='sim_clock',
        parameters=[{'use_sim_time': False}],
        output='screen',
    )

    return LaunchDescription(
        declare_args + [
            set_model_env,
            set_resources,
            set_tb3_resources,
            OpaqueFunction(function=gz_server),
            gz_client,
            robot_state_publisher,
            spawn_robot,
            bridge,
            service_bridge,
            image_bridge,
            traffic_light,
            level_crossing,
            firmware,
            clock,
        ]
    )
