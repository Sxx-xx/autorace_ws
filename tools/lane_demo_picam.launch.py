"""The lane pipeline on the Pi camera (/camera), for while the C920 is unplugged (2026-10-21).

Run on the PC after the Pi's camera is up, with tools/lane_view.py --camera /camera:
  ros2 launch tools/lane_demo_picam.launch.py [pipeline:=nodes] [detector:=sliding_window]
pipeline:=integrated (default) is lane_pipeline, decode + BEV + detection in one process, as
lane_drive.launch.py runs it; nodes is republish, bev_projector and the detector apart.

Nothing in the workspace is changed: the Pi camera's geometry is passed here as overrides.
No lane_controller / cmd_vel_mux: nothing is sent to the wheels.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from autorace_bringup.launch_helpers import republish

# The forward (Pi) camera as docs/REAL_ROBOT.md has it: 4.5 cm ahead of the axle, 12 cm up,
# 0.10 rad down. Its 62 deg lens (csi_camera fov_deg) sees the floor from ~0.25 m on.
PICAM = {'camera.height': 0.12, 'camera.pitch': 0.10, 'camera.forward_offset': 0.045,
         'bev.near': 0.26}


def nodes(context):
    params = os.path.join(get_package_share_directory('autorace_bringup'), 'param',
                          'perception_real.yaml')
    detector = LaunchConfiguration('detector').perform(context)
    view = {'bev.near': PICAM['bev.near'], 'process_every_n': 1,
            'debug_rate': float(LaunchConfiguration('view_rate').perform(context))}
    if LaunchConfiguration('pipeline').perform(context) == 'integrated':
        return [Node(
            package='autorace_perception', executable='lane_pipeline', output='screen',
            arguments=['--detector', detector],
            parameters=[params, PICAM, view, {'stats': True}],
            remappings=[('/camera/image_compressed', '/camera/image_raw/compressed'),
                        ('/camera/camera_info', '/camera/camera_info'),
                        ('/camera/image_output', '/camera/image_projected'),
                        ('/camera/image_decoded', '/camera/image_decoded'),
                        ('/camera/image_input', '/lane_pipeline/unused_raw'),
                        ('/detect/image_input', '/lane_pipeline/unused_bev'),
                        ('/detect/image_output', '/detect/image_lane')])]
    executable = {'detect_lane': 'detect_lane', 'sliding_window': 'detect_lane_sw'}[detector]
    return [
        republish('/camera/image_raw', False, 'republish_forward'),
        Node(package='autorace_perception', executable='bev_projector', name='bev_projector',
             output='screen', parameters=[params, PICAM],
             remappings=[('/camera/image_input', '/camera/image_decoded'),
                         ('/camera/camera_info', '/camera/camera_info'),
                         ('/camera/image_output', '/camera/image_projected')]),
        Node(package='autorace_perception', executable=executable, name='detect_lane',
             output='screen', parameters=[params, view],
             remappings=[('/detect/image_input', '/camera/image_projected'),
                         ('/detect/image_output', '/detect/image_lane')]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pipeline', default_value='integrated'),
        DeclareLaunchArgument('detector', default_value='detect_lane'),
        # The detector's picture (what lane_view shows) at most this often, Hz.
        DeclareLaunchArgument('view_rate', default_value='30'),
        OpaqueFunction(function=nodes),
    ])
