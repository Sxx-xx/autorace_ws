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
"""Traffic light colour from the forward camera.

A lit lamp is a small, round, bright patch of red, yellow or green set in a
dark housing. Each of those on its own turns up elsewhere on the course -- the
yellow lane line is bright and yellow, a warning sign has a red border -- so
all of them are asked for: the colour and the brightness, a round outline, and
dark to either side.

Publishes /detect/traffic_light on every frame: 0 for no light, 1 red,
2 yellow, 3 green.
"""

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8


LIGHT_NONE = 0
LIGHT_RED = 1
LIGHT_YELLOW = 2
LIGHT_GREEN = 3

NAMES = {LIGHT_RED: 'red', LIGHT_YELLOW: 'yellow', LIGHT_GREEN: 'green'}


class DetectTrafficLight(Node):

    def __init__(self):
        super().__init__('detect_traffic_light')

        # A lit lamp is brighter and more saturated than anything painted.
        self.declare_parameter('lamp.saturation_l', 100)
        self.declare_parameter('lamp.value_l', 200)
        self.declare_parameter('red.hue_margin', 10)
        self.declare_parameter('yellow.hue_l', 20)
        self.declare_parameter('yellow.hue_h', 38)
        self.declare_parameter('green.hue_l', 45)
        self.declare_parameter('green.hue_h', 95)
        self.declare_parameter('min_area', 30)
        # The housing either side of the lamp is darker than this.
        self.declare_parameter('housing.value_h', 110)
        self.declare_parameter('publish_debug_image', True)

        def value(name):
            return self.get_parameter(name).value

        self.saturation_l = value('lamp.saturation_l')
        self.value_l = value('lamp.value_l')
        self.red_margin = value('red.hue_margin')
        self.hues = {
            LIGHT_YELLOW: (value('yellow.hue_l'), value('yellow.hue_h')),
            LIGHT_GREEN: (value('green.hue_l'), value('green.hue_h')),
        }
        self.min_area = value('min_area')
        self.housing_value = value('housing.value_h')
        self.publish_debug = value('publish_debug_image')

        self.bridge = CvBridge()
        self.create_subscription(Image, '/detect/image_input', self.callback_image, 1)
        self.pub_light = self.create_publisher(UInt8, '/detect/traffic_light', 1)
        self.pub_image = self.create_publisher(Image, '/detect/image_traffic_light', 1)

    def lamps(self, hsv):
        """Lit lamps in the picture: (area, colour, box) for each."""
        hue, saturation, value = cv2.split(hsv)
        lit = (saturation >= self.saturation_l) & (value >= self.value_l)
        height, width = value.shape
        for colour in NAMES:
            if colour == LIGHT_RED:
                in_hue = (hue < self.red_margin) | (hue > 180 - self.red_margin)
            else:
                in_hue = (hue >= self.hues[colour][0]) & (hue <= self.hues[colour][1])
            mask = (lit & in_hue).astype(np.uint8)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < self.min_area:
                    continue
                x, y, w, h = cv2.boundingRect(contour)
                side = max(2, w // 3)
                if x - side < 0 or y <= 0 or x + w + side > width or y + h >= height:
                    continue
                # Round: as wide as tall, and filling its box as a disc does.
                if not 0.6 < w / h < 1.6 or not 0.7 < area / (np.pi / 4 * w * h) < 1.2:
                    continue
                # In a housing: dark to the left and to the right.
                beside = np.concatenate([value[y:y + h, x - side:x].ravel(),
                                         value[y:y + h, x + w:x + w + side].ravel()])
                if np.median(beside) > self.housing_value:
                    continue
                yield area, colour, (x, y, w, h)

    def callback_image(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # More than one: the nearest, which is the largest.
        lamps = sorted(self.lamps(hsv), reverse=True)
        colour = lamps[0][1] if lamps else LIGHT_NONE
        self.pub_light.publish(UInt8(data=colour))

        if self.publish_debug:
            if lamps:
                _, _, (x, y, w, h) = lamps[0]
                cv2.rectangle(image, (x, y), (x + w, y + h), (255, 255, 255), 2)
                cv2.putText(image, NAMES[colour], (x, max(12, y - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            out = self.bridge.cv2_to_imgmsg(image, 'bgr8')
            out.header = msg.header
            self.pub_image.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DetectTrafficLight()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
