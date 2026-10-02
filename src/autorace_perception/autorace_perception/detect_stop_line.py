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
"""Stop line from the bird's eye view: a red band across the lane.

Publishes /detect/stop_line while the line is in view: the distance in metres
from the axle to its near edge.
"""

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64


class DetectStopLine(Node):

    def __init__(self):
        super().__init__('detect_stop_line')

        # Geometry of the bird's eye view this node is fed.
        self.declare_parameter('bev.pixels_per_meter', 1200.0)
        self.declare_parameter('bev.near', 0.07)
        self.declare_parameter('red.hue_margin', 10)
        self.declare_parameter('red.saturation_l', 120)
        self.declare_parameter('red.value_l', 80)
        # The line runs from one side of the lane to the other; red that is
        # narrower than this is something else.
        self.declare_parameter('min_width_m', 0.10)

        self.ppm = self.get_parameter('bev.pixels_per_meter').value
        self.near = self.get_parameter('bev.near').value
        self.red_margin = self.get_parameter('red.hue_margin').value
        self.saturation_l = self.get_parameter('red.saturation_l').value
        self.value_l = self.get_parameter('red.value_l').value
        self.min_width_px = self.get_parameter('min_width_m').value * self.ppm

        self.bridge = CvBridge()
        self.create_subscription(Image, '/detect/image_input', self.callback_image, 1)
        self.pub_distance = self.create_publisher(Float64, '/detect/stop_line', 1)

    def callback_image(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hue, saturation, value = cv2.split(cv2.cvtColor(image, cv2.COLOR_BGR2HSV))
        red = (((hue < self.red_margin) | (hue > 180 - self.red_margin))
               & (saturation >= self.saturation_l) & (value >= self.value_l))

        rows = np.flatnonzero(np.count_nonzero(red, axis=1) >= self.min_width_px)
        if len(rows) == 0:
            return
        # The bottom row of the view is `near` ahead of the axle.
        nearest = image.shape[0] - 1 - rows.max()
        self.pub_distance.publish(Float64(data=self.near + nearest / self.ppm))


def main(args=None):
    rclpy.init(args=args)
    node = DetectStopLine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
