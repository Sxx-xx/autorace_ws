#!/usr/bin/env python3
"""lane_center_check.py : is the lane camera's centre line the robot's?

Put the robot by hand exactly in the middle of a straight stretch of lane
(tape measure from each line to the wheels), pointing along it, with
lane_drive or the race launch running. This reads the lane detector's line
points for a few seconds and prints where the two lines lie, left of the
robot's axle, and the middle between them. On the robot's centre line the
middle is 0.000. A middle of +0.02 means the camera sees the lane 2 cm to
the left of where it is, i.e. the camera sits (or looks) 2 cm to the right
of the robot's centre line: put -0.02 in perception_real.yaml as
bev_projector camera.lateral_offset (metres, + for a camera left of the
centre line) and check again.

Run:  python3 tools/lane_center_check.py   (ROS_DOMAIN_ID=30)
"""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class Check(Node):

    def __init__(self):
        super().__init__('lane_center_check')
        self.lines = {'yellow': [], 'white': []}
        for colour in self.lines:
            self.create_subscription(
                PointCloud2, f'/detect/lane_lines/{colour}',
                lambda msg, c=colour: self.callback(msg, c), 10)

    def callback(self, msg, colour):
        points = point_cloud2.read_points_numpy(msg, field_names=('x', 'y'))
        points = points[(points[:, 0] > 0.18) & (points[:, 0] < 0.40)]
        if len(points) >= 30:
            self.lines[colour].append(float(np.median(points[:, 1])))


def main():
    rclpy.init()
    node = Check()
    print('reading the lane lines for 4 s; keep the robot still ...')
    end = time.time() + 4.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    ys = {c: np.array(v) for c, v in node.lines.items()}
    for colour in ('yellow', 'white'):
        v = ys[colour]
        print(f'  {colour:6s}: ' + (f'{np.median(v):+.3f} m left of the axle ({len(v)} pictures, spread {np.std(v):.3f})'
                                   if len(v) else 'not seen'))
    if len(ys['yellow']) and len(ys['white']):
        middle = (np.median(ys['yellow']) + np.median(ys['white'])) / 2.0
        width = np.median(ys['yellow']) - np.median(ys['white'])
        print(f'  lane middle {middle:+.3f} m left of the axle, lane {width:.3f} m wide between line centres')
        print(f'  -> if the robot really is in the middle, set camera.lateral_offset: {-middle:+.3f}'
              f' (current value plus this if one is set already)')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
