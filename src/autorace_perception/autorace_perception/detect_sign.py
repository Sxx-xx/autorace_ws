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

"""Traffic sign detection: colour and shape, no features.

Each sign is told apart by what it is made of rather than by matching a
picture of it:

* the intersection warning: a yellow triangle in a red border with a black T,
* turn left / turn right: a blue disc with a bent arrow, whose head lies to
  one side of its stem,
* parking: a blue square with a white P.

Left and right are mirror images, so the direction is read from the arrow
itself: the head is in the upper half of the disc, the stem in the lower half,
and the head sits on the side the arrow points to.

Only a sign that is entirely inside the picture is reported; one cut off by
the edge cannot be read reliably.
"""

from autorace_msgs.msg import SignDetection
import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class DetectSign(Node):

    def __init__(self):
        super().__init__('detect_sign')

        self.declare_parameter('blue.hue_l', 95)
        self.declare_parameter('blue.hue_h', 125)
        self.declare_parameter('blue.saturation_l', 120)
        self.declare_parameter('blue.value_l', 50)
        self.declare_parameter('yellow.hue_l', 18)
        self.declare_parameter('yellow.hue_h', 35)
        self.declare_parameter('yellow.saturation_l', 120)
        self.declare_parameter('yellow.value_l', 100)
        self.declare_parameter('red.hue_margin', 10)
        self.declare_parameter('red.saturation_l', 100)
        self.declare_parameter('red.value_l', 80)
        self.declare_parameter('black.value_h', 70)
        self.declare_parameter('white.saturation_h', 80)
        self.declare_parameter('white.value_l', 120)
        # Smallest sign worth reading, as the area of its coloured face.
        self.declare_parameter('min_area', 250)
        # How far the arrow head must sit from the stem, as a fraction of the
        # disc's width, before the sign counts as pointing that way.
        self.declare_parameter('arrow.min_shift', 0.08)
        self.declare_parameter('publish_debug_image', True)

        def value(name):
            return self.get_parameter(name).value

        self.blue = (np.array([value('blue.hue_l'), value('blue.saturation_l'),
                               value('blue.value_l')]),
                     np.array([value('blue.hue_h'), 255, 255]))
        self.yellow = (np.array([value('yellow.hue_l'), value('yellow.saturation_l'),
                                 value('yellow.value_l')]),
                       np.array([value('yellow.hue_h'), 255, 255]))
        self.red_margin = value('red.hue_margin')
        self.red_floor = (value('red.saturation_l'), value('red.value_l'))
        self.black_value = value('black.value_h')
        self.white_saturation = value('white.saturation_h')
        self.white_value = value('white.value_l')
        self.min_area = value('min_area')
        self.arrow_min_shift = value('arrow.min_shift')
        self.publish_debug = value('publish_debug_image')

        self.bridge = CvBridge()
        self.streaks = {}

        self.create_subscription(Image, '/detect/image_input', self.callback_image, 1)
        self.pub_sign = self.create_publisher(SignDetection, '/autorace/sign', 5)
        self.pub_image = self.create_publisher(Image, '/detect/image_sign', 1)

    # --- shapes ----------------------------------------------------------

    def blobs(self, mask):
        """Outer contours big enough to read that do not touch the image edge."""
        height, width = mask.shape
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv2.contourArea(contour) < self.min_area:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if x <= 0 or y <= 0 or x + w >= width or y + h >= height:
                continue
            yield contour, (x, y, w, h)

    def filled(self, contour, box):
        """Return the contour filled in, holes and all, cropped to its box."""
        x, y, w, h = box
        mask = np.zeros((h, w), np.uint8)
        cv2.drawContours(mask, [contour], -1, 255, -1, offset=(-x, -y))
        return mask

    def direction_signs(self, hsv):
        """Blue discs with an arrow: (name, box) for each one that can be read."""
        blue = cv2.inRange(hsv, *self.blue)
        for contour, box in self.blobs(blue):
            x, y, w, h = box
            # A disc seen from the side is an ellipse, narrower than tall; the
            # square parking sign fills its box, an ellipse only pi/4 of it.
            if not 0.5 < w / h < 1.25:
                continue
            if not 0.85 < cv2.contourArea(contour) / (np.pi / 4 * w * h) < 1.12:
                continue
            disc = self.filled(contour, box)
            arrow = cv2.bitwise_and(disc, cv2.bitwise_not(blue[y:y + h, x:x + w]))
            if not 0.08 < np.count_nonzero(arrow) / np.count_nonzero(disc) < 0.5:
                continue
            head = arrow[:h // 2].nonzero()[1]
            stem = arrow[h // 2:].nonzero()[1]
            if len(head) == 0 or len(stem) == 0:
                continue
            shift = (head.mean() - stem.mean()) / w
            if abs(shift) < self.arrow_min_shift:
                continue
            yield ('left' if shift < 0 else 'right'), box

    def parking_signs(self, hsv):
        """Blue squares with white lettering: ('parking', box) for each one."""
        blue = cv2.inRange(hsv, *self.blue)
        for contour, box in self.blobs(blue):
            x, y, w, h = box
            # Seen from the side the square is narrower than tall.
            if not 0.5 < w / h < 1.25:
                continue
            # A square fills its box where a disc leaves the corners empty.
            if cv2.contourArea(contour) < 0.88 * w * h:
                continue
            face = self.filled(contour, box)
            patch = hsv[y:y + h, x:x + w]
            white = ((patch[..., 1] < self.white_saturation) & (patch[..., 2] > self.white_value)
                     & (face > 0) & (blue[y:y + h, x:x + w] == 0))
            if 0.08 < np.count_nonzero(white) / np.count_nonzero(face) < 0.4:
                yield 'parking', box

    def is_tee(self, pictogram):
        """Whether a black pictogram is the T of the intersection warning.

        A bar across the full width at the top, and below it a stem in the
        middle with nothing either side. The digging worker and the tunnel
        mouth of the other warning triangles have neither.
        """
        rows, cols = pictogram.nonzero()
        if len(rows) == 0:
            return False
        tee = pictogram[rows.min():rows.max() + 1, cols.min():cols.max() + 1] > 0
        h, w = tee.shape
        if w < 6 or h < 6 or not 0.6 < w / h < 1.6:
            return False
        bar = tee[:max(1, int(0.3 * h))]
        below = tee[h // 2:]
        side = max(1, int(0.25 * w))
        return (bar.mean() > 0.6
                and below[:, side:w - side].mean() > 0.6
                and below[:, :side].mean() < 0.3
                and below[:, w - side:].mean() < 0.3)

    def warning_signs(self, hsv):
        """Yellow triangles in a red border: (name, box) for each one read."""
        yellow = cv2.inRange(hsv, *self.yellow)
        hue, saturation, value = cv2.split(hsv)
        red = (((hue < self.red_margin) | (hue > 180 - self.red_margin))
               & (saturation > self.red_floor[0]) & (value > self.red_floor[1]))
        black = value < self.black_value
        height, width = yellow.shape
        for contour, box in self.blobs(yellow):
            x, y, w, h = box
            area = cv2.contourArea(contour)
            # A triangle covers half of its box.
            if not 0.35 < area / (w * h) < 0.7:
                continue
            # The lane's yellow line has no red border round it.
            pad = max(2, w // 4)
            x0, y0 = max(0, x - pad), max(0, y - pad)
            x1, y1 = min(width, x + w + pad), min(height, y + h + pad)
            if np.count_nonzero(red[y0:y1, x0:x1]) < 0.3 * area:
                continue
            face = self.filled(contour, box)
            pictogram = cv2.bitwise_and(face, black[y:y + h, x:x + w].astype(np.uint8) * 255)
            if self.is_tee(pictogram):
                yield 'intersection', box

    # --- main ------------------------------------------------------------

    def callback_image(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        seen = {}
        for name, box in (list(self.direction_signs(hsv)) + list(self.warning_signs(hsv))
                          + list(self.parking_signs(hsv))):
            # The same kind twice in one frame: keep the nearer, larger one.
            if name not in seen or box[2] * box[3] > seen[name][2] * seen[name][3]:
                seen[name] = box

        # A streak is consecutive frames; one frame without the sign ends it.
        self.streaks = {name: self.streaks.get(name, 0) + 1 for name in seen}
        for name, streak in self.streaks.items():
            detection = SignDetection()
            detection.header = msg.header
            detection.name = name
            detection.streak = min(streak, 65535)
            self.pub_sign.publish(detection)

        if self.publish_debug:
            for name, (x, y, w, h) in seen.items():
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(image, f'{name} {self.streaks[name]}', (x, max(12, y - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
            out = self.bridge.cv2_to_imgmsg(image, 'bgr8')
            out.header = msg.header
            self.pub_image.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DetectSign()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
