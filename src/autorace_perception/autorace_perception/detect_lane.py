#!/usr/bin/env python3
#
# Copyright 2026 AutoRace Team
# Based on turtlebot3_autorace_detect/detect_lane.py
#   Copyright 2018 ROBOTIS CO., LTD.
#   Authors: Leon Jung, Gilbert, Ashe Kim, Hyungyu Kim, ChanHyeong Lee
#   Special thanks: Roger Sacchelli
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

"""Lane detection on the metric bird's eye view.

Same approach as the ROBOTIS original -- HSV masks for the white and yellow
line, a sliding window to seed a second order fit, then a fit around the
previous curve -- but everything the original hard coded for one particular
bird's eye calibration is a parameter here:

* half a lane is `lane.width_m / 2 * bev.pixels_per_meter` pixels instead of a
  fixed 280, so the projection can cover more road; the wide view is what keeps
  a line inside the frame through the tight bends of the AutoRace course,
* the control point is a lookahead distance in metres rather than row 350,
* a line stays trustworthy while it covers enough rows, and how much is
  "enough" is tunable instead of being 500 of 600 rows.

It also fixes two things that stop the robot on the course: the original
publishes no centre at all when a single frame drops both lines, and it can
reference an unset curve on the first frames.
"""

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_msgs.msg import UInt8


# lane_state values, kept compatible with turtlebot3_autorace.
LANE_NONE = 0
LANE_LEFT_ONLY = 1
LANE_BOTH = 2
LANE_RIGHT_ONLY = 3


class DetectLane(Node):

    def __init__(self):
        super().__init__('detect_lane')

        self.declare_parameter('detect.lane.white.hue_l', 0)
        self.declare_parameter('detect.lane.white.hue_h', 179)
        self.declare_parameter('detect.lane.white.saturation_l', 0)
        self.declare_parameter('detect.lane.white.saturation_h', 70)
        self.declare_parameter('detect.lane.white.lightness_l', 105)
        self.declare_parameter('detect.lane.white.lightness_h', 255)
        self.declare_parameter('detect.lane.yellow.hue_l', 10)
        self.declare_parameter('detect.lane.yellow.hue_h', 127)
        self.declare_parameter('detect.lane.yellow.saturation_l', 70)
        self.declare_parameter('detect.lane.yellow.saturation_h', 255)
        self.declare_parameter('detect.lane.yellow.lightness_l', 95)
        self.declare_parameter('detect.lane.yellow.lightness_h', 255)

        # Geometry of the bird's eye view this node is fed.
        self.declare_parameter('bev.pixels_per_meter', 1200.0)
        self.declare_parameter('bev.near', 0.24)
        self.declare_parameter('lane.width_m', 0.25)
        self.declare_parameter('lane.control_lookahead', 0.45)

        self.declare_parameter('detect.min_pixels', 3000)
        self.declare_parameter('detect.max_line_width_m', 0.08)
        self.declare_parameter('detect.auto_threshold', True)
        self.declare_parameter('detect.auto_threshold_low', 5000)
        self.declare_parameter('detect.auto_threshold_high', 35000)
        self.declare_parameter('reliability.max_missing_fraction', 0.45)
        self.declare_parameter('reliability.step', 5)
        self.declare_parameter('reliability.threshold', 50)
        self.declare_parameter('process_every_n', 2)
        self.declare_parameter('hold_last_center_sec', 0.4)
        self.declare_parameter('publish_debug_image', True)

        self.ppm = self.get_parameter('bev.pixels_per_meter').value
        self.near = self.get_parameter('bev.near').value
        self.lane_width_px = self.get_parameter('lane.width_m').value * self.ppm
        self.lookahead = self.get_parameter('lane.control_lookahead').value

        self.min_pixels = self.get_parameter('detect.min_pixels').value
        blob_px = int(self.get_parameter('detect.max_line_width_m').value * self.ppm)
        self.blob_kernel_h = np.ones((1, max(3, blob_px)), np.uint8)
        self.blob_kernel_v = np.ones((max(3, blob_px), 1), np.uint8)
        self.auto_threshold = self.get_parameter('detect.auto_threshold').value
        self.auto_low = self.get_parameter('detect.auto_threshold_low').value
        self.auto_high = self.get_parameter('detect.auto_threshold_high').value
        self.max_missing_fraction = self.get_parameter('reliability.max_missing_fraction').value
        self.reliability_step = self.get_parameter('reliability.step').value
        self.reliability_threshold = self.get_parameter('reliability.threshold').value
        self.process_every_n = max(1, self.get_parameter('process_every_n').value)
        self.hold_last_center = self.get_parameter('hold_last_center_sec').value
        self.publish_debug = self.get_parameter('publish_debug_image').value

        self.white = self.hsv_bounds('white')
        self.yellow = self.hsv_bounds('yellow')

        self.bridge = CvBridge()
        self.counter = 0

        self.reliability_white = 0
        self.reliability_yellow = 0
        self.left_fit = None      # yellow line, to the robot's left
        self.right_fit = None     # white line, to the robot's right
        self.left_fitx = None
        self.right_fitx = None
        self.last_center = None
        self.last_center_time = 0.0

        self.create_subscription(Image, '/detect/image_input', self.callback_image, 1)
        self.pub_lane = self.create_publisher(Float64, '/detect/lane', 1)
        self.pub_offset = self.create_publisher(Float64, '/detect/lane_offset', 1)
        self.pub_state = self.create_publisher(UInt8, '/detect/lane_state', 1)
        self.pub_reliability_white = self.create_publisher(
            UInt8, '/detect/white_line_reliability', 1)
        self.pub_reliability_yellow = self.create_publisher(
            UInt8, '/detect/yellow_line_reliability', 1)
        self.pub_image = self.create_publisher(Image, '/detect/image_output', 1)

        self.get_logger().info(
            f'Lane width {self.lane_width_px:.0f} px at {self.ppm:.0f} px/m, '
            f'control point {self.lookahead:.2f} m ahead'
        )

    def hsv_bounds(self, color):
        def value(name):
            return self.get_parameter(f'detect.lane.{color}.{name}').value
        return {
            'lower': np.array([value('hue_l'), value('saturation_l'), value('lightness_l')]),
            'upper': np.array([value('hue_h'), value('saturation_h'), value('lightness_h')]),
        }

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    # --- masking ---------------------------------------------------------

    def drop_blobs(self, mask):
        """Remove everything too wide to be a painted line.

        A line is thin in at least one direction, whatever its heading in the
        bird's eye view. Anything that is wide both horizontally and
        vertically -- the floor beyond the edge of the course, a tunnel wall,
        a patch of glare -- is not a lane marking, and following it drives the
        robot off the track.
        """
        wide_h = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.blob_kernel_h)
        wide_v = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.blob_kernel_v)
        return cv2.subtract(mask, cv2.bitwise_and(wide_h, wide_v))

    def mask_line(self, hsv, bounds, reliability):
        mask = self.drop_blobs(cv2.inRange(hsv, bounds['lower'], bounds['upper']))
        pixels = int(np.count_nonzero(mask))

        if self.auto_threshold:
            # Follow the scene brightness: too few pixels and the lightness
            # floor drops, too many and it rises.
            if pixels > self.auto_high and bounds['lower'][2] < 250:
                bounds['lower'][2] += 5
            elif pixels < self.auto_low and bounds['lower'][2] > 50:
                bounds['lower'][2] -= 5

        rows_with_line = int(np.count_nonzero(mask.any(axis=1)))
        missing = 1.0 - rows_with_line / mask.shape[0]
        if missing > self.max_missing_fraction:
            reliability = max(0, reliability - self.reliability_step)
        else:
            reliability = min(100, reliability + self.reliability_step)

        return pixels, mask, reliability

    # --- curve fitting ---------------------------------------------------

    def fit_around(self, fit, mask, margin=100):
        nonzero_y, nonzero_x = mask.nonzero()
        if len(nonzero_x) == 0:
            return None
        centers = fit[0] * nonzero_y ** 2 + fit[1] * nonzero_y + fit[2]
        inside = (nonzero_x > centers - margin) & (nonzero_x < centers + margin)
        if np.count_nonzero(inside) < 50:
            return None
        return np.polyfit(nonzero_y[inside], nonzero_x[inside], 2)

    def fit_sliding_window(self, mask, side, windows=20, margin=50, min_pixels=50):
        height, width = mask.shape
        histogram = np.sum(mask[height // 2:, :], axis=0)
        midpoint = width // 2
        if side == 'left':
            base = int(np.argmax(histogram[:midpoint]))
        else:
            base = int(np.argmax(histogram[midpoint:])) + midpoint

        nonzero_y, nonzero_x = mask.nonzero()
        window_height = height // windows
        current = base
        collected = []

        for window in range(windows):
            y_low = height - (window + 1) * window_height
            y_high = height - window * window_height
            inside = (
                (nonzero_y >= y_low) & (nonzero_y < y_high) &
                (nonzero_x >= current - margin) & (nonzero_x < current + margin)
            ).nonzero()[0]
            collected.append(inside)
            if len(inside) > min_pixels:
                current = int(np.mean(nonzero_x[inside]))

        collected = np.concatenate(collected)
        if len(collected) < 50:
            return None
        return np.polyfit(nonzero_y[collected], nonzero_x[collected], 2)

    def update_line(self, fit, mask, side):
        """Track the line from the previous fit, falling back to a fresh search."""
        new_fit = self.fit_around(fit, mask) if fit is not None else None
        if new_fit is None:
            new_fit = self.fit_sliding_window(mask, side)
        return new_fit

    # --- main ------------------------------------------------------------

    def callback_image(self, msg):
        self.counter += 1
        if self.counter % self.process_every_n != 0:
            return

        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        height = image.shape[0]

        yellow_pixels, yellow_mask, self.reliability_yellow = self.mask_line(
            hsv, self.yellow, self.reliability_yellow)
        white_pixels, white_mask, self.reliability_white = self.mask_line(
            hsv, self.white, self.reliability_white)

        self.pub_reliability_yellow.publish(UInt8(data=self.reliability_yellow))
        self.pub_reliability_white.publish(UInt8(data=self.reliability_white))

        plot_y = np.linspace(0, height - 1, height)

        if yellow_pixels > self.min_pixels:
            fit = self.update_line(self.left_fit, yellow_mask, 'left')
            if fit is not None:
                self.left_fit = fit
                self.left_fitx = fit[0] * plot_y ** 2 + fit[1] * plot_y + fit[2]

        if white_pixels > self.min_pixels:
            fit = self.update_line(self.right_fit, white_mask, 'right')
            if fit is not None:
                self.right_fit = fit
                self.right_fitx = fit[0] * plot_y ** 2 + fit[1] * plot_y + fit[2]

        self.publish_lane(image, plot_y, yellow_pixels, white_pixels, msg.header)

    def usable(self, side):
        """A line counts when it is reliable enough and has been fitted."""
        if side == 'left':
            return self.reliability_yellow > self.reliability_threshold \
                and self.left_fitx is not None
        return self.reliability_white > self.reliability_threshold \
            and self.right_fitx is not None

    def publish_lane(self, image, plot_y, yellow_pixels, white_pixels, header):
        height, width = image.shape[:2]
        fresh_yellow = yellow_pixels > self.min_pixels and self.usable('left')
        fresh_white = white_pixels > self.min_pixels and self.usable('right')

        center_x = None
        state = LANE_NONE

        if fresh_yellow and fresh_white:
            center_x = np.mean([self.left_fitx, self.right_fitx], axis=0)
            state = LANE_BOTH
        elif fresh_yellow:
            center_x = self.left_fitx + self.lane_width_px / 2.0
            state = LANE_LEFT_ONLY
        elif fresh_white:
            center_x = self.right_fitx - self.lane_width_px / 2.0
            state = LANE_RIGHT_ONLY
        elif self.usable('left'):
            # Reliable but not seen in this frame: keep steering on the last curve.
            center_x = self.left_fitx + self.lane_width_px / 2.0
            state = LANE_LEFT_ONLY
        elif self.usable('right'):
            center_x = self.right_fitx - self.lane_width_px / 2.0
            state = LANE_RIGHT_ONLY

        row = self.control_row(height)
        now = self.now()

        if center_x is not None:
            control_x = float(center_x[row])
            self.last_center = control_x
            self.last_center_time = now
        elif self.last_center is not None and now - self.last_center_time < self.hold_last_center:
            # Bridge a dropped frame or two rather than handing back a gap in
            # the command stream; standing still is what ends a run.
            control_x = self.last_center
        else:
            control_x = None

        self.pub_state.publish(UInt8(data=state))

        if control_x is not None:
            self.pub_lane.publish(Float64(data=control_x))
            self.pub_offset.publish(Float64(data=(control_x - width / 2.0) / self.ppm))

        if self.publish_debug:
            self.publish_debug_image(image, plot_y, center_x, row, state, header)

    def control_row(self, height):
        """Bird's eye row that sits `control_lookahead` metres ahead."""
        row = height - 1 - int((self.lookahead - self.near) * self.ppm)
        return int(np.clip(row, 0, height - 1))

    def publish_debug_image(self, image, plot_y, center_x, row, state, header):
        overlay = image.copy()
        for curve, color in ((self.left_fitx, (0, 200, 255)), (self.right_fitx, (255, 200, 0))):
            if curve is None:
                continue
            points = np.int_(np.transpose(np.vstack([curve, plot_y])))
            cv2.polylines(overlay, [points], False, color, 8)
        if center_x is not None:
            points = np.int_(np.transpose(np.vstack([center_x, plot_y])))
            cv2.polylines(overlay, [points], False, (0, 255, 0), 6)
            cv2.circle(overlay, (int(center_x[row]), row), 14, (0, 0, 255), -1)
        cv2.putText(
            overlay,
            f'state {state}  y{self.reliability_yellow} w{self.reliability_white}',
            (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2
        )
        out = self.bridge.cv2_to_imgmsg(overlay, 'bgr8')
        out.header = header
        self.pub_image.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DetectLane()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
