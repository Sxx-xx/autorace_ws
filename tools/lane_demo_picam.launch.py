"""The lane pipeline on the Pi camera (/camera), for while the C920 is unplugged (2026-10-21).

Run on the PC after ~/start_all.sh cameras on the Pi, with tools/lane_view.py --camera /camera:
  ros2 launch tools/lane_demo_picam.launch.py

Nothing in the workspace is changed: the Pi camera's geometry is passed here as overrides.
No lane_controller / cmd_vel_mux: nothing is sent to the wheels.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from autorace_bringup.launch_helpers import republish

# The forward (Pi) camera as docs/REAL_ROBOT.md has it: 4.5 cm ahead of the axle, 12 cm up,
# 0.10 rad down. Its 62 deg lens (csi_camera fov_deg) sees the floor from ~0.25 m on.
PICAM = {'camera.height': 0.12, 'camera.pitch': 0.10, 'camera.forward_offset': 0.045,
         'bev.near': 0.26}


def generate_launch_description():
    params = os.path.join(get_package_share_directory('autorace_bringup'), 'param',
                          'perception_real.yaml')
    return LaunchDescription([
        republish('/camera/image_raw', False, 'republish_forward'),
        Node(package='autorace_perception', executable='bev_projector', name='bev_projector',
             output='screen', parameters=[params, PICAM],
             remappings=[('/camera/image_input', '/camera/image_decoded'),
                         ('/camera/camera_info', '/camera/camera_info'),
                         ('/camera/image_output', '/camera/image_projected')]),
        Node(package='autorace_perception', executable='detect_lane_sw', name='detect_lane',
             output='screen', parameters=[params, {'bev.near': PICAM['bev.near'],
                                                   'process_every_n': 1}],
             remappings=[('/detect/image_input', '/camera/image_projected'),
                         ('/detect/image_output', '/detect/image_lane')]),
    ])
