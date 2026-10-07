#!/usr/bin/env python3
"""lane_overlay.py [camera_topic] [perception.yaml] : the camera picture with the detected lines drawn on it.

Subscribes to <camera_topic>/compressed, <camera>/camera_info, /detect/lane_lines/yellow and
/white (points in metres from the axle) and /detect/lane_target, projects them back into the
camera picture with the same camera geometry the BEV projector uses (camera.height / pitch /
forward_offset from the perception yaml) and publishes <camera>/image_overlay. Yellow and white
dots are the line points of the current frame, the red dot is the control target, the green
band is the strip of road the BEV covers (bev.near .. far).

Run next to lane_view.sh or lane_drive.launch.py; look at it with
    ros2 run rqt_image_view rqt_image_view <camera>/image_overlay
"""
import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import PointStamped
import yaml


class LaneOverlay(Node):

    def __init__(self, camera, params):
        super().__init__('lane_overlay')
        cfg = yaml.safe_load(open(params))['bev_projector']['ros__parameters']
        cam, bev = cfg['camera'], cfg['bev']
        self.height, self.pitch, self.offset = cam['height'], cam['pitch'], cam['forward_offset']
        self.near = bev['near']
        self.far = bev['near'] + bev['height'] / bev['pixels_per_meter']
        self.half_width = bev['width'] / bev['pixels_per_meter'] / 2.0
        self.k = None
        self.lines = {'yellow': None, 'white': None}
        self.target = None
        ns = camera.rsplit('/', 1)[0]
        self.create_subscription(CameraInfo, ns + '/camera_info', self.callback_info, 1)
        self.create_subscription(CompressedImage, camera + '/compressed', self.callback_image, 1)
        for colour in self.lines:
            self.create_subscription(PointCloud2, f'/detect/lane_lines/{colour}',
                                     lambda m, c=colour: self.lines.__setitem__(c, m), 1)
        self.create_subscription(PointStamped, '/detect/lane_target',
                                 lambda m: setattr(self, 'target', (m.point.x, m.point.y)), 1)
        self.pub = self.create_publisher(Image, ns + '/image_overlay', 1)
        self.get_logger().info(f'{camera} with lines from {params}; publishing {ns}/image_overlay')

    def callback_info(self, msg):
        self.k = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def project(self, ahead_axle, lateral):
        """Pixel of a ground point (metres ahead of the axle, metres to the left)."""
        fx, fy, cx, cy = self.k
        ahead = np.asarray(ahead_axle, dtype=float) - self.offset
        cos_p, sin_p = np.cos(self.pitch), np.sin(self.pitch)
        x_c = ahead * cos_p + self.height * sin_p
        z_c = ahead * sin_p - self.height * cos_p
        return cx - fx * np.asarray(lateral, dtype=float) / x_c, cy - fy * z_c / x_c

    def callback_image(self, msg):
        if self.k is None:
            return
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        h, w = img.shape[:2]
        # The strip of road the BEV covers.
        band = np.array([self.project(d, s) for d, s in
                         [(self.far, self.half_width), (self.far, -self.half_width),
                          (self.near, -self.half_width), (self.near, self.half_width)]])
        cv2.polylines(img, [band.astype(np.int32)], True, (0, 200, 0), 1)
        for colour, bgr in (('yellow', (0, 220, 255)), ('white', (255, 255, 255))):
            cloud = self.lines[colour]
            if cloud is None:
                continue
            pts = np.array([(p[0], p[1]) for p in point_cloud2.read_points(cloud, ('x', 'y'), skip_nans=True)])
            if len(pts) == 0:
                continue
            u, v = self.project(pts[:, 0], pts[:, 1])
            inside = (u >= 0) & (u < w) & (v >= 0) & (v < h)
            for x, y in zip(u[inside].astype(int), v[inside].astype(int)):
                cv2.circle(img, (x, y), 2, bgr, -1)
        if self.target is not None:
            u, v = self.project(*self.target)
            cv2.circle(img, (int(u), int(v)), 6, (0, 0, 255), 2)
        out = Image()
        out.header = msg.header
        out.height, out.width, out.encoding, out.step = h, w, 'bgr8', 3 * w
        out.data = img.tobytes()
        self.pub.publish(out)


def main():
    camera = sys.argv[1] if len(sys.argv) > 1 else '/camera_lane/image_raw'
    import os
    default = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..',
                           'src', 'autorace_bringup', 'param', 'perception_real.yaml')
    params = sys.argv[2] if len(sys.argv) > 2 else default
    rclpy.init()
    node = LaneOverlay(camera, params)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
