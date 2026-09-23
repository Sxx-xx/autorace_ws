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

"""Metric bird's eye projection of the road in front of the robot.

Unlike turtlebot3_autorace_camera's image_projection, which is calibrated with
four pixel coordinates that only mean something for one particular camera, this
node is told where the camera *is* -- height above the road and downward pitch
-- and reads the intrinsics from camera_info.  The homography then follows from
the ground plane, and the output has a known, uniform scale in pixels per
metre, which is what the lane detector needs to reason in metres.

Output frame: x to the right, y forward-to-back, the robot's centreline at the
middle column and `near` metres ahead at the bottom row.
"""

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image


class BevProjector(Node):

    def __init__(self):
        super().__init__('bev_projector')

        self.declare_parameter('camera.height', 0.20)
        self.declare_parameter('camera.pitch', 0.30)
        self.declare_parameter('camera.forward_offset', 0.073)
        self.declare_parameter('bev.pixels_per_meter', 1200.0)
        self.declare_parameter('bev.width', 1000)
        self.declare_parameter('bev.height', 600)
        self.declare_parameter('bev.near', 0.24)
        self.declare_parameter('blur_kernel', 5)
        # Fallback intrinsics, used until camera_info arrives.
        self.declare_parameter('camera.fx', 0.0)
        self.declare_parameter('camera.fy', 0.0)
        self.declare_parameter('camera.cx', 0.0)
        self.declare_parameter('camera.cy', 0.0)

        self.height = self.get_parameter('camera.height').value
        self.pitch = self.get_parameter('camera.pitch').value
        self.forward_offset = self.get_parameter('camera.forward_offset').value
        self.ppm = self.get_parameter('bev.pixels_per_meter').value
        self.out_w = self.get_parameter('bev.width').value
        self.out_h = self.get_parameter('bev.height').value
        self.near = self.get_parameter('bev.near').value
        self.blur = self.get_parameter('blur_kernel').value

        self.fx = self.get_parameter('camera.fx').value
        self.fy = self.get_parameter('camera.fy').value or self.fx
        self.cx = self.get_parameter('camera.cx').value
        self.cy = self.get_parameter('camera.cy').value

        self.bridge = CvBridge()
        self.homography = None

        self.create_subscription(CameraInfo, '/camera/camera_info', self.callback_info, 1)
        self.create_subscription(Image, '/camera/image_input', self.callback_image, 1)
        self.pub_bev = self.create_publisher(Image, '/camera/image_output', 1)

        self.far = self.near + self.out_h / self.ppm
        self.half_width = 0.5 * self.out_w / self.ppm
        self.get_logger().info(
            f'Bird\'s eye view: {self.near:.2f}..{self.far:.2f} m ahead, '
            f'+/-{self.half_width:.2f} m across, {self.ppm:.0f} px/m'
        )

    def callback_info(self, msg):
        fx, fy, cx, cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]
        if (fx, fy, cx, cy) == (self.fx, self.fy, self.cx, self.cy):
            return
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
        self.homography = None
        self.get_logger().info(f'Intrinsics: fx={fx:.2f} fy={fy:.2f} cx={cx:.1f} cy={cy:.1f}')

    def project(self, distance, lateral):
        """Image pixel of a ground point `distance` ahead of the robot centre."""
        # Distances are given from the robot centre; the camera sits ahead of it.
        ahead = distance - self.forward_offset
        cos_p, sin_p = np.cos(self.pitch), np.sin(self.pitch)
        # Camera frame: x forward along the optical axis, y left, z up.
        x_c = ahead * cos_p + self.height * sin_p
        z_c = ahead * sin_p - self.height * cos_p
        u = self.cx - self.fx * lateral / x_c
        v = self.cy - self.fy * z_c / x_c
        return u, v

    def build_homography(self):
        """Map the ground rectangle the output covers onto the source image."""
        left = self.half_width
        source = np.array([
            self.project(self.far, left),     # output top left
            self.project(self.far, -left),    # output top right
            self.project(self.near, -left),   # output bottom right
            self.project(self.near, left),    # output bottom left
        ], dtype=np.float32)
        destination = np.array([
            [0, 0],
            [self.out_w - 1, 0],
            [self.out_w - 1, self.out_h - 1],
            [0, self.out_h - 1],
        ], dtype=np.float32)
        self.homography = cv2.getPerspectiveTransform(source, destination)
        self.get_logger().info(
            'Source quad (u,v): ' + ', '.join(f'({u:.0f},{v:.0f})' for u, v in source)
        )

    def callback_image(self, msg):
        if not self.fx:
            self.get_logger().warn('Waiting for camera_info', throttle_duration_sec=5.0)
            return
        if self.homography is None:
            self.build_homography()

        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        if self.blur > 1:
            image = cv2.GaussianBlur(image, (self.blur, self.blur), 0)

        bev = cv2.warpPerspective(
            image, self.homography, (self.out_w, self.out_h),
            flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0)
        )

        out = self.bridge.cv2_to_imgmsg(bev, 'bgr8')
        out.header = msg.header
        self.pub_bev.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = BevProjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
