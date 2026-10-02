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

"""Level crossing bar detection: whether the bar is across the road or raised.

The bar is striped red and white, and that is what it is known by: several
solid red patches of a size, in a row. Nothing else on the course looks like
that. The border of a warning sign is a red ring, not a solid patch; the stop
line and the red lamp are one patch each.

Which way the row runs tells the state of the bar. Standing up beside the road
it is open; lying across, or anywhere on the way down, it is closed. Only a
bar near enough to matter is reported, judged by how thick its stripes look.

/detect/level_crossing: 0 no bar in sight, 1 open, 2 closed.
"""

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8


BAR_NONE = 0
BAR_OPEN = 1
BAR_CLOSED = 2


class DetectLevelCrossing(Node):

    def __init__(self):
        super().__init__('detect_level_crossing')

        self.declare_parameter('red.hue_margin', 10)
        self.declare_parameter('red.saturation_l', 100)
        self.declare_parameter('red.value_l', 80)
        # A stripe: at least this many pixels, and this much of its box.
        self.declare_parameter('stripe.min_area', 40)
        self.declare_parameter('stripe.min_fill', 0.6)
        # How thick the stripes must look for the bar to be near enough.
        self.declare_parameter('stripe.min_thickness', 9)
        # The bar counts as open when it stands within this of upright.
        self.declare_parameter('open_angle', 0.35)
        self.declare_parameter('publish_debug_image', True)

        def value(name):
            return self.get_parameter(name).value

        self.red_margin = value('red.hue_margin')
        self.red_floor = (value('red.saturation_l'), value('red.value_l'))
        self.min_area = value('stripe.min_area')
        self.min_fill = value('stripe.min_fill')
        self.min_thickness = value('stripe.min_thickness')
        self.open_angle = value('open_angle')
        self.publish_debug = value('publish_debug_image')

        self.bridge = CvBridge()

        self.create_subscription(Image, '/detect/image_input', self.callback_image, 1)
        self.pub_state = self.create_publisher(UInt8, '/detect/level_crossing', 1)
        self.pub_image = self.create_publisher(Image, '/detect/image_level_crossing', 1)

    def stripes(self, hsv):
        """Solid red patches: (centre x, centre y, thickness, box) of each."""
        hue, saturation, value = cv2.split(hsv)
        red = (((hue < self.red_margin) | (hue > 180 - self.red_margin))
               & (saturation > self.red_floor[0]) & (value > self.red_floor[1]))
        contours, _ = cv2.findContours(
            red.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        found = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area:
                continue
            # The box that fits the patch whichever way the bar is turned.
            (x, y), (w, h), _ = cv2.minAreaRect(contour)
            if w * h <= 0.0 or area / (w * h) < self.min_fill:
                continue
            found.append((x, y, min(w, h), cv2.boundingRect(contour)))
        return found

    def bar(self, stripes):
        """Return the state of the bar the stripes make up, and its stripes."""
        if len(stripes) < 2:
            return BAR_NONE, []
        # Stripes of one bar are alike in thickness; keep those like the
        # thickest, which is the nearest bar if there should be two things.
        thickest = max(stripe[2] for stripe in stripes)
        alike = [stripe for stripe in stripes if stripe[2] > 0.6 * thickest]
        if len(alike) < 2 or thickest < self.min_thickness:
            return BAR_NONE, []
        centres = np.array([stripe[:2] for stripe in alike])
        offsets = centres - centres.mean(axis=0)
        _, vectors = np.linalg.eigh(offsets.T @ offsets)
        along = vectors[:, 1]
        # They lie on one line, no further off it than they are thick.
        if np.abs(offsets @ np.array([-along[1], along[0]])).max() > 0.6 * thickest:
            return BAR_NONE, []
        # Two patches are not much to go by: sign borders far apart are two
        # patches as well. Stripes of a bar are no further apart than a few
        # times their thickness.
        reach = np.sort(offsets @ along)
        if len(alike) < 3 and np.diff(reach).max() > 4.0 * thickest:
            return BAR_NONE, []
        from_upright = np.arctan2(abs(along[0]), abs(along[1]))
        return (BAR_OPEN if from_upright < self.open_angle else BAR_CLOSED), alike

    def callback_image(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        state, stripes = self.bar(self.stripes(hsv))
        self.pub_state.publish(UInt8(data=state))

        if self.publish_debug:
            colour = (0, 255, 0) if state == BAR_OPEN else (0, 0, 255)
            for _, _, _, (x, y, w, h) in stripes:
                cv2.rectangle(image, (x, y), (x + w, y + h), colour, 2)
            cv2.putText(image, ('no bar', 'open', 'closed')[state], (5, 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, 1)
            out = self.bridge.cv2_to_imgmsg(image, 'bgr8')
            out.header = msg.header
            self.pub_image.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DetectLevelCrossing()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
