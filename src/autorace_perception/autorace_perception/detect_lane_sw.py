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

"""Lane detection on the bird's eye view by sliding windows: the textbook way.

An alternative to detect_lane, in its place (lane_drive.launch.py
lane_detector:=sliding_window): it is run under the name detect_lane, reads
the same parameters from perception_*.yaml, and publishes what detect_lane
publishes, so lane_controller and the missions do not know which one runs.

  1. bird's eye view  done before this node, by bev_projector
                      (cv2.warpPerspective from the camera's height, pitch and
                      camera_info); here only its scale is needed: bev.near
                      (m from the axle to the bottom row), bev.pixels_per_meter
  2. binarise         binarise(): HSV thresholds for the yellow and the white
                      line (detect.lane.*), and for white also a Sobel x edge
                      nearby (sliding.sobel_min; 0 leaves it out)
  3. sliding windows  find_line(): from the column histogram's peak at the
                      bottom, windows up the picture, each recentred on the
                      line pixels in it; a parabola left(ahead) through all
                      of them, in metres
  4. lane centre      find_target(): at sliding.target_ahead in front of the
                      axle, midway between the two lines, or half a lane from
                      the one there is (yellow on the left, white on the right)

Out: /detect/lane_target (ahead, left of the axle, m) for lane_controller,
/detect/lane_state, the line points on /detect/lane_lines/yellow and /white,
and the picture with the windows on /detect/image_output.

What detect_lane does and this does not: remembering the lines along the
odometry (in a tight bend the inner line is out of view), and choosing a way
at a fork by the two lane centres there; /detect/lane_follow is taken as
"steer by that line only".
"""

import math

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float64
from std_msgs.msg import Header
from std_msgs.msg import String
from std_msgs.msg import UInt8


LANE_NONE = 0
LANE_LEFT_ONLY = 1
LANE_BOTH = 2
LANE_RIGHT_ONLY = 3


class DetectLaneSlidingWindow(Node):

    def __init__(self):
        super().__init__('detect_lane')

        # Shared with detect_lane (perception_*.yaml, section detect_lane).
        for colour, bounds in (('white', (0, 179, 0, 70, 105, 255)),
                               ('yellow', (10, 127, 70, 255, 95, 255))):
            for name, default in zip(('hue_l', 'hue_h', 'saturation_l', 'saturation_h',
                                      'lightness_l', 'lightness_h'), bounds):
                self.declare_parameter(f'detect.lane.{colour}.{name}', default)
        self.declare_parameter('bev.pixels_per_meter', 1200.0)
        self.declare_parameter('bev.near', 0.24)
        self.declare_parameter('lane.width_m', 0.25)
        self.declare_parameter('lane.control_lookahead', 0.16)
        self.declare_parameter('lane.max_target_angle', 1.1)
        self.declare_parameter('process_every_n', 2)
        self.declare_parameter('hold_last_center_sec', 0.4)
        self.declare_parameter('follow_timeout', 0.5)
        self.declare_parameter('publish_debug_image', True)
        # This node's own.
        self.declare_parameter('sliding.windows', 10)
        self.declare_parameter('sliding.margin_m', 0.05)        # half a window's width
        self.declare_parameter('sliding.recentre_pixels', 40)   # to move a window on
        self.declare_parameter('sliding.min_pixels', 300)       # in all windows, for a line
        self.declare_parameter('sliding.sobel_min', 0)          # 0: colour only
        self.declare_parameter('sliding.target_ahead', 0.0)     # 0: the nearer of the view's edge
        # A pair this far from a lane apart is not one lane: steer by the
        # stronger line alone.
        self.declare_parameter('sliding.width_tolerance', 0.4)

        def value(name):
            return self.get_parameter(name).value

        self.bounds = {
            colour: (np.array([value(f'detect.lane.{colour}.{n}_l')
                               for n in ('hue', 'saturation', 'lightness')], np.uint8),
                     np.array([value(f'detect.lane.{colour}.{n}_h')
                               for n in ('hue', 'saturation', 'lightness')], np.uint8))
            for colour in ('white', 'yellow')}
        self.ppm = value('bev.pixels_per_meter')
        self.near = value('bev.near')
        self.half_lane = value('lane.width_m') / 2.0
        self.max_target_angle = value('lane.max_target_angle')
        self.process_every_n = max(1, value('process_every_n'))
        self.hold_last_center = value('hold_last_center_sec')
        self.follow_timeout = value('follow_timeout')
        self.publish_debug = value('publish_debug_image')
        self.windows = value('sliding.windows')
        self.margin = int(value('sliding.margin_m') * self.ppm)
        self.recentre_pixels = value('sliding.recentre_pixels')
        self.min_pixels = value('sliding.min_pixels')
        self.sobel_min = value('sliding.sobel_min')
        self.target_ahead = value('sliding.target_ahead') or max(
            value('lane.control_lookahead'), self.near)
        self.width_tolerance = value('sliding.width_tolerance')
        self.line_kernel = np.ones((1, max(3, int(0.03 * self.ppm))), np.uint8)
        self.speck_kernel = np.ones((3, 3), np.uint8)

        self.bridge = CvBridge()
        self.counter = 0
        self.last_target = None
        self.last_target_time = 0.0
        self.follow = None
        self.follow_time = 0.0

        self.create_subscription(Image, '/detect/image_input', self.callback_image,
                                 qos_profile_sensor_data)
        self.create_subscription(String, '/detect/lane_follow', self.callback_follow, 1)
        self.pub_target = self.create_publisher(PointStamped, '/detect/lane_target', 1)
        self.pub_state = self.create_publisher(UInt8, '/detect/lane_state', 1)
        self.pub_offset = self.create_publisher(Float64, '/detect/lane_offset', 1)
        self.pub_reliability = {
            colour: self.create_publisher(UInt8, f'/detect/{colour}_line_reliability', 1)
            for colour in ('white', 'yellow')}
        self.pub_lines = {
            colour: self.create_publisher(PointCloud2, f'/detect/lane_lines/{colour}', 1)
            for colour in ('white', 'yellow')}
        self.pub_image = self.create_publisher(Image, '/detect/image_output', 1)

        self.get_logger().info(
            f'Sliding windows: {self.windows} of +-{self.margin} px, lane centre '
            f'{self.target_ahead:.2f} m ahead, Sobel {self.sobel_min or "off"}')

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def callback_follow(self, msg):
        self.follow = msg.data if msg.data in ('left', 'right') else None
        self.follow_time = self.now()

    # --- 2. binarise -----------------------------------------------------

    def binarise(self, image):
        """Masks of the yellow and the white line in the bird's eye view."""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        masks = {colour: cv2.inRange(hsv, *self.bounds[colour]) for colour in self.bounds}
        if self.sobel_min > 0:
            # A white line has edges across it; a bright patch of floor does
            # not. Keep the white within a line's width of a strong x edge.
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            edge = np.abs(cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3))
            edge = cv2.dilate((edge >= self.sobel_min).astype(np.uint8) * 255, self.line_kernel)
            masks['white'] &= edge
        for colour in masks:
            masks[colour] = cv2.morphologyEx(masks[colour], cv2.MORPH_OPEN, self.speck_kernel)
        return masks

    # --- 3. sliding windows ----------------------------------------------

    def to_robot(self, x, y, shape):
        """Metres ahead and to the left of the axle of bird's eye pixels."""
        return (self.near + (shape[0] - 1 - y) / self.ppm, (shape[1] / 2.0 - x) / self.ppm)

    def find_line(self, mask, boxes):
        """Follow one line up the picture: its parabola left(ahead) and points, or None."""
        rows, cols = mask.shape
        histogram = np.count_nonzero(mask[rows // 2:], axis=0)
        if histogram.max() < 5:
            return None
        x = int(np.argmax(histogram))
        height = rows // self.windows
        ys, xs = mask.nonzero()
        keep = np.zeros(len(xs), bool)
        for i in range(self.windows):
            top, bottom = rows - (i + 1) * height, rows - i * height
            left, right = x - self.margin, x + self.margin
            boxes.append((left, top, right, bottom))
            inside = (ys >= top) & (ys < bottom) & (xs >= left) & (xs < right)
            keep |= inside
            if np.count_nonzero(inside) >= self.recentre_pixels:
                x = int(np.mean(xs[inside]))
            if x < 0 or x >= cols:
                break
        if np.count_nonzero(keep) < self.min_pixels:
            return None
        ahead, left = self.to_robot(xs[keep], ys[keep], mask.shape)
        degree = 2 if np.ptp(ahead) > 0.08 else 1
        return np.polyfit(ahead, left, degree), np.stack([ahead, left], axis=1)

    # --- 4. lane centre --------------------------------------------------

    def find_target(self, lines):
        """The lane centre at target_ahead: ((ahead, left), state), or (None, LANE_NONE)."""
        ahead = self.target_ahead
        at = {colour: float(np.polyval(line[0], ahead))
              for colour, line in lines.items() if line is not None}
        if self.follow is not None and self.now() - self.follow_time < self.follow_timeout:
            at.pop('white' if self.follow == 'left' else 'yellow', None)
        if 'yellow' in at and 'white' in at:
            width = at['yellow'] - at['white']
            if abs(width - 2 * self.half_lane) < self.width_tolerance * 2 * self.half_lane:
                return (ahead, (at['yellow'] + at['white']) / 2.0), LANE_BOTH
            # Not one lane: keep the line with more of itself in view.
            weaker = min(('yellow', 'white'), key=lambda c: len(lines[c][1]))
            at.pop(weaker)
        if 'yellow' in at:
            return (ahead, at['yellow'] - self.half_lane), LANE_LEFT_ONLY
        if 'white' in at:
            return (ahead, at['white'] + self.half_lane), LANE_RIGHT_ONLY
        return None, LANE_NONE

    # --- main ------------------------------------------------------------

    def callback_image(self, msg):
        self.counter += 1
        if self.counter % self.process_every_n != 0:
            return
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        masks = self.binarise(image)
        boxes = []
        lines = {colour: self.find_line(masks[colour], boxes) for colour in masks}
        for colour, line in lines.items():
            self.pub_reliability[colour].publish(UInt8(data=100 if line is not None else 0))
            points = line[1][::6] if line is not None else np.zeros((0, 2))
            cloud = np.zeros((len(points), 3), np.float32)
            cloud[:, :2] = points
            header = Header(stamp=msg.header.stamp, frame_id='base_footprint')
            self.pub_lines[colour].publish(point_cloud2.create_cloud_xyz32(header, cloud))

        target, state = self.find_target(lines)
        now = self.now()
        if target is not None:
            # Not further aside than the controller can turn for.
            angle = max(-self.max_target_angle,
                        min(self.max_target_angle, math.atan2(target[1], target[0])))
            distance = math.hypot(*target)
            target = (distance * math.cos(angle), distance * math.sin(angle))
            self.last_target, self.last_target_time = target, now
        elif self.last_target is not None and now - self.last_target_time < self.hold_last_center:
            target = self.last_target
        self.pub_state.publish(UInt8(data=state))
        if target is not None:
            point = PointStamped()
            point.header.stamp = msg.header.stamp
            point.header.frame_id = 'base_footprint'
            point.point.x, point.point.y = target
            self.pub_target.publish(point)
            self.pub_offset.publish(Float64(data=-target[1]))
        if self.publish_debug:
            self.publish_debug_image(image.shape, masks, boxes, lines, target, state, msg.header)

    def publish_debug_image(self, shape, masks, boxes, lines, target, state, header):
        """The binarised view with the windows, the fitted lines and the target."""
        overlay = np.full(shape[:2] + (3,), 40, np.uint8)
        overlay[masks['yellow'] > 0] = (0, 200, 255)
        overlay[masks['white'] > 0] = (255, 255, 255)
        for left, top, right, bottom in boxes:
            cv2.rectangle(overlay, (left, top), (right, bottom), (0, 255, 0), 1)

        def pixel(ahead, left):
            return (int(round(shape[1] / 2.0 - left * self.ppm)),
                    int(round(shape[0] - 1 - (ahead - self.near) * self.ppm)))

        ahead = self.near + np.linspace(0.0, (shape[0] - 1) / self.ppm, 30)
        for colour, line in lines.items():
            if line is not None:
                curve = np.array([pixel(a, l) for a, l in zip(ahead, np.polyval(line[0], ahead))])
                cv2.polylines(overlay, [curve], False, (255, 0, 255), 2)
        if target is not None:
            cv2.circle(overlay, pixel(*target), 7, (0, 0, 255), -1)
        cv2.putText(overlay, f'sliding window  state {state}', (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        out = self.bridge.cv2_to_imgmsg(overlay, 'bgr8')
        out.header = header
        self.pub_image.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DetectLaneSlidingWindow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
