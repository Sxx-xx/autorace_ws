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

The lines are found by colour, as in the ROBOTIS original: HSV masks for the
white and the yellow line. What is done with them is different. The original
fits a curve x(y) to each line and takes the lane centre half a lane sideways
from it, which holds on a gentle bend seen from far off. On the AutoRace
course the bends are as tight as the lane is wide and the camera looks at the
30 cm in front of the robot: the inner line of a bend is out of the picture
altogether and the outer one runs across it, where no x(y) curve can follow
it and "sideways" is no longer square to the line.

So the lane centre is taken from distances instead. The point to steer for
lies on an arc `lane.control_lookahead` ahead of the axle, and it is the point
of that arc which is

* as far from the yellow line as from the white one, when both bound the lane
  there, or
* half a lane from the one line there is,

with the yellow line on the left of the way there and the white line on the
right. Distance to a line is the same whichever way the line runs, so the
robot keeps its place in the lane round a bend as on a straight.

The camera sees a strip barely wider than the lane, and in a tight bend the
inner line is outside it the whole way round. The lines are therefore
remembered: what was seen over the last stretch of road is carried along with
the odometry and laid next to what is seen now, so a line that has slid out
of the picture still bounds the lane.

Where the road forks there are two lane centres on the arc. The one on the
`lane.fork_side` is taken, or the one /detect/lane_follow asks for.

The lines seen in each picture are also published as points, in metres from
the axle (/detect/lane_lines/yellow and /white), for the missions that have to
know where the road ends rather than where its middle is.
"""

from collections import deque

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import Header
from std_msgs.msg import String
from std_msgs.msg import UInt8


# lane_state values, kept compatible with turtlebot3_autorace.
LANE_NONE = 0
LANE_LEFT_ONLY = 1
LANE_BOTH = 2
LANE_RIGHT_ONLY = 3
# Colour-blind mode: at most this many lines, each at least this many map pixels.
LINE_COMPONENTS_MAX = 8
LINE_COMPONENT_MIN_PX = 30
# Lane curvature: a line needs this many points over this much road to be fitted.
CURVATURE_MIN_POINTS = 40
CURVATURE_MIN_SPAN = 0.12


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
        # Between the centres of the two lines, and how wide a line is.
        # Distances are measured to the near edge of a line.
        self.declare_parameter('lane.width_m', 0.25)
        self.declare_parameter('lane.line_width_m', 0.03)
        self.declare_parameter('lane.control_lookahead', 0.16)
        # Speed-adaptive look-ahead: control_lookahead + lookahead_gain * speed
        # (odometry), kept within [lookahead_min, lookahead_max]. A gain of 0
        # keeps it fixed at control_lookahead.
        self.declare_parameter('lane.lookahead_gain', 0.0)
        self.declare_parameter('lane.lookahead_min', 0.0)
        self.declare_parameter('lane.lookahead_max', 10.0)
        # Two lines this much further apart than a lane are not one lane: the
        # road is forking, or doubling in width, and each line has a lane
        # centre of its own beside it.
        self.declare_parameter('lane.max_width_ratio', 1.4)
        # Which way to go where there are two, unless /detect/lane_follow
        # says otherwise.
        self.declare_parameter('lane.fork_side', 'right')
        # True: the yellow line must lie on the left of the way and the white
        # on the right (the AutoRace convention). False: colour does not
        # matter, the robot keeps between whatever lines lie either side.
        self.declare_parameter('lane.yellow_left', True)
        # A point of the arc this close to the middle counts as lane centre.
        self.declare_parameter('lane.centre_tolerance', 0.025)
        # Lines are remembered over this much road behind the robot.
        self.declare_parameter('lane.memory_distance', 0.4)
        # A line further than this from a point says nothing about the lane
        # there.
        self.declare_parameter('lane.max_line_distance', 0.30)
        # How far to either side of straight ahead the target may lie.
        self.declare_parameter('lane.max_target_angle', 1.1)

        self.declare_parameter('detect.min_pixels', 3000)
        self.declare_parameter('detect.max_line_width_m', 0.08)
        # How much brighter than its surroundings a white line must be; 0 turns
        # the check off.
        self.declare_parameter('detect.white_contrast_min', 20)
        # White pixels this close to yellow ones are dropped: glare along the
        # edge of a glossy yellow tape reads as white. 0 turns it off.
        self.declare_parameter('detect.white_yellow_gap_m', 0.0)
        self.declare_parameter('detect.auto_threshold', True)
        self.declare_parameter('detect.auto_threshold_low', 5000)
        self.declare_parameter('detect.auto_threshold_high', 35000)
        self.declare_parameter('reliability.step', 5)
        self.declare_parameter('process_every_n', 2)
        self.declare_parameter('hold_last_center_sec', 0.4)
        # A request to follow one line only lapses when it stops being repeated.
        self.declare_parameter('follow_timeout', 0.5)
        self.declare_parameter('publish_debug_image', True)

        self.ppm = self.get_parameter('bev.pixels_per_meter').value
        self.near = self.get_parameter('bev.near').value
        self.lane_width = (self.get_parameter('lane.width_m').value
                           - self.get_parameter('lane.line_width_m').value)
        self.base_lookahead = self.get_parameter('lane.control_lookahead').value
        self.lookahead_gain = self.get_parameter('lane.lookahead_gain').value
        self.lookahead_min = self.get_parameter('lane.lookahead_min').value
        self.lookahead_max = self.get_parameter('lane.lookahead_max').value
        self.lookahead = self.lookahead_for(0.0)
        self.max_width = self.get_parameter('lane.max_width_ratio').value * self.lane_width
        self.fork_side = self.get_parameter('lane.fork_side').value
        self.yellow_left = self.get_parameter('lane.yellow_left').value
        self.centre_tolerance = self.get_parameter('lane.centre_tolerance').value
        self.memory_distance = self.get_parameter('lane.memory_distance').value
        self.max_line_distance = self.get_parameter('lane.max_line_distance').value
        self.max_target_angle = self.get_parameter('lane.max_target_angle').value

        self.min_pixels = self.get_parameter('detect.min_pixels').value
        blob_px = int(self.get_parameter('detect.max_line_width_m').value * self.ppm)
        self.blob_kernel_h = np.ones((1, max(3, blob_px)), np.uint8)
        self.blob_kernel_v = np.ones((max(3, blob_px), 1), np.uint8)
        self.contrast_kernel = np.ones((max(3, blob_px), max(3, blob_px)), np.uint8)
        self.edge_kernel = np.ones((7, 7), np.uint8)
        self.white_contrast_min = self.get_parameter('detect.white_contrast_min').value
        gap_px = int(round(self.get_parameter('detect.white_yellow_gap_m').value * self.ppm))
        # A square, not a disc: OpenCV splits a rectangular kernel into a row
        # and a column pass, ~10x faster at this size (a 49 px disc cost the
        # Pi ~35 ms a frame).
        self.yellow_gap_kernel = (
            cv2.getStructuringElement(cv2.MORPH_RECT, (2 * gap_px + 1, 2 * gap_px + 1))
            if gap_px > 0 else None)
        self.auto_threshold = self.get_parameter('detect.auto_threshold').value
        self.auto_low = self.get_parameter('detect.auto_threshold_low').value
        self.auto_high = self.get_parameter('detect.auto_threshold_high').value
        self.reliability_step = self.get_parameter('reliability.step').value
        self.process_every_n = max(1, self.get_parameter('process_every_n').value)
        self.hold_last_center = self.get_parameter('hold_last_center_sec').value
        self.follow_timeout = self.get_parameter('follow_timeout').value
        self.publish_debug = self.get_parameter('publish_debug_image').value

        self.white = self.hsv_bounds('white')
        self.yellow = self.hsv_bounds('yellow')

        self.bridge = CvBridge()
        self.counter = 0

        self.reliability_white = 0
        self.reliability_yellow = 0
        # The lines are worked on in a map round the robot, at half the bird's
        # eye resolution: wider than the camera's view, and reaching back
        # past the axle, to hold the lines that are remembered.
        self.map_ppm = self.ppm / 2.0
        self.map_half_width = 0.45
        self.map_behind = 0.12
        self.map_shape = None       # set from the first image
        self.map_far = 0.0
        self.arc = None             # candidate targets
        # Odometry of the last few seconds, to find where the robot was when
        # a picture was taken: (time, x, y, yaw, distance travelled).
        self.odometry = deque(maxlen=400)
        self.pose = None            # x, y, yaw when the current picture was taken
        self.travelled = 0.0
        self.memory = []            # (travelled, pose, yellow points, white points)
        self.last_target = None     # (ahead, left) of the axle, metres
        self.last_target_time = 0.0
        self.follow = None
        self.follow_time = 0.0
        self.speed = 0.0            # forward speed from odometry, m/s
        self.curve = False          # the controller's straight/curve call, for display

        self.create_subscription(Image, '/detect/image_input', self.callback_image, 1)
        self.create_subscription(String, '/detect/lane_follow', self.callback_follow, 1)
        self.create_subscription(Odometry, '/odom', self.callback_odom, 1)
        self.create_subscription(Bool, '/control/curve', self.callback_curve, 1)
        self.pub_lookahead = self.create_publisher(Float64, '/detect/lookahead', 1)
        self.pub_curvature = self.create_publisher(Float64, '/detect/lane_curvature', 1)
        self.lane_curvature = float('nan')
        self.pub_lane = self.create_publisher(Float64, '/detect/lane', 1)
        self.pub_offset = self.create_publisher(Float64, '/detect/lane_offset', 1)
        self.pub_target = self.create_publisher(PointStamped, '/detect/lane_target', 1)
        self.pub_state = self.create_publisher(UInt8, '/detect/lane_state', 1)
        self.pub_reliability_white = self.create_publisher(
            UInt8, '/detect/white_line_reliability', 1)
        self.pub_reliability_yellow = self.create_publisher(
            UInt8, '/detect/yellow_line_reliability', 1)
        self.pub_lines = {
            'yellow': self.create_publisher(PointCloud2, '/detect/lane_lines/yellow', 1),
            'white': self.create_publisher(PointCloud2, '/detect/lane_lines/white', 1),
        }
        self.pub_image = self.create_publisher(Image, '/detect/image_output', 1)
        self.pub_bev_overlay = self.create_publisher(Image, '/detect/image_lane_bev', 1)

        self.get_logger().info(
            f'Lane {self.lane_width:.2f} m between the lines at {self.ppm:.0f} px/m, '
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

    def lookahead_for(self, speed):
        lookahead = self.base_lookahead + self.lookahead_gain * max(0.0, speed)
        return min(self.lookahead_max, max(self.lookahead_min, lookahead))

    def callback_curve(self, msg):
        self.curve = msg.data

    def callback_odom(self, msg):
        self.speed = msg.twist.twist.linear.x
        position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        travelled = 0.0
        if self.odometry:
            _, x, y, _, travelled = self.odometry[-1]
            travelled += float(np.hypot(position.x - x, position.y - y))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.odometry.append((stamp, position.x, position.y, float(yaw), travelled))

    def locate(self, stamp):
        """Set the pose to where the robot was at `stamp`.

        A picture is a tenth of a second old or more by the time it gets here,
        and in a bend the robot has turned several degrees since. Lines laid
        down with the pose of the moment land that far out of place.
        """
        if not self.odometry:
            return
        when = stamp.sec + stamp.nanosec * 1e-9
        _, x, y, yaw, travelled = min(self.odometry, key=lambda entry: abs(entry[0] - when))
        self.pose = (x, y, yaw)
        self.travelled = travelled

    def callback_follow(self, msg):
        """Take the 'left' or the 'right' branch where the road forks."""
        self.follow = msg.data if msg.data in ('left', 'right') else None
        self.follow_time = self.now()

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

    def thin_bright(self, hsv):
        """Pixels brighter than their surroundings in a strip thinner than a line.

        Colour alone cannot tell a white line from a light grey floor next to
        it: in simulation the two differ by only about 40 in lightness, and on
        the real course the floor is whatever the venue has. A top-hat keeps
        what stands out from its neighbourhood at the scale of a line, so the
        floor drops out whatever its brightness, while the line survives next
        to it as well as next to the black road.
        """
        # Outside the camera's view the bird's eye image is pure black, which
        # would make the floor next to it look like a bright strip. Count it,
        # and the few interpolated pixels along its edge, as bright instead.
        outside = cv2.dilate((hsv.max(axis=2) == 0).astype(np.uint8), self.edge_kernel)
        value = hsv[..., 2].copy()
        value[outside > 0] = 255
        tophat = cv2.morphologyEx(value, cv2.MORPH_TOPHAT, self.contrast_kernel)
        bright = (tophat >= self.white_contrast_min).astype(np.uint8) * 255
        # Where the edge of the view runs to a point, the strip counted as
        # bright is itself thin; nothing along the edge is a line.
        bright[outside > 0] = 0
        return bright

    def mask_line(self, hsv, bounds, reliability, only=None):
        mask = cv2.inRange(hsv, bounds['lower'], bounds['upper'])
        if only is not None:
            mask = cv2.bitwise_and(mask, only)
        mask = self.drop_blobs(mask)
        pixels = int(np.count_nonzero(mask))

        if self.auto_threshold:
            # Follow the scene brightness: too few pixels and the lightness
            # floor drops, too many and it rises.
            if pixels > self.auto_high and bounds['lower'][2] < 250:
                bounds['lower'][2] += 5
            elif pixels < self.auto_low and bounds['lower'][2] > 50:
                bounds['lower'][2] -= 5

        # A scatter of stray pixels is not a line.
        if pixels < self.min_pixels:
            reliability = max(0, reliability - self.reliability_step)
        else:
            reliability = min(100, reliability + self.reliability_step)

        return pixels, mask, reliability

    # --- geometry --------------------------------------------------------

    def to_pixel(self, ahead, left):
        """Map pixel (x, y) of a point given in metres from the axle."""
        return (self.map_shape[1] / 2.0 - left * self.map_ppm,
                (self.map_far - ahead) * self.map_ppm)

    def to_robot(self, x, y):
        """Metres ahead and to the left of the axle of a map pixel."""
        return (self.map_far - y / self.map_ppm,
                (self.map_shape[1] / 2.0 - x) / self.map_ppm)

    def build_map(self, image_shape):
        """Lay out the map and the candidate targets for this image size."""
        self.map_far = self.near + image_shape[0] / self.ppm
        self.view_half_width = image_shape[1] / self.ppm / 2.0
        self.map_shape = (int((self.map_far + self.map_behind) * self.map_ppm),
                          int(2 * self.map_half_width * self.map_ppm))
        self.build_arc()

    def build_arc(self):
        """Candidate targets: the points `lookahead` from the axle."""
        angle = np.linspace(-self.max_target_angle, self.max_target_angle, 221)
        ahead = self.lookahead * np.cos(angle)
        left = self.lookahead * np.sin(angle)
        x, y = self.to_pixel(ahead, left)
        self.arc = {
            'angle': angle, 'ahead': ahead, 'left': left,
            'x': np.int_(np.round(x)), 'y': np.int_(np.round(y)),
        }

    def line_points(self, mask):
        """Where a bird's eye mask has line, in metres from the axle (N x 2)."""
        small = cv2.resize(mask, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
        y, x = (small >= 128).nonzero()
        ahead = self.near + (small.shape[0] - 1 - y) / self.map_ppm
        left = (small.shape[1] / 2.0 - x) / self.map_ppm
        return np.stack([ahead, left], axis=1)

    def draw(self, points):
        """Map mask of points given in metres from the axle."""
        mask = np.zeros(self.map_shape, np.uint8)
        x, y = self.to_pixel(points[:, 0], points[:, 1])
        x, y = np.int_(np.round(x)), np.int_(np.round(y))
        inside = (x >= 0) & (x < mask.shape[1]) & (y >= 0) & (y < mask.shape[0])
        mask[y[inside], x[inside]] = 255
        return mask

    def remembered(self, index):
        """Lines seen earlier, moved to where they are from the robot now."""
        if self.pose is None or not self.memory:
            return np.zeros(self.map_shape, np.uint8)
        x_now, y_now, yaw_now = self.pose
        moved = []
        for entry in self.memory:
            points = entry[index]
            x_then, y_then, yaw_then = entry[1]
            turn = yaw_then - yaw_now
            cos_t, sin_t = np.cos(turn), np.sin(turn)
            # Where the robot was then, seen from where it is now.
            dx, dy = x_then - x_now, y_then - y_now
            ahead_0 = np.cos(yaw_now) * dx + np.sin(yaw_now) * dy
            left_0 = -np.sin(yaw_now) * dx + np.cos(yaw_now) * dy
            moved.append(np.stack([
                ahead_0 + cos_t * points[:, 0] - sin_t * points[:, 1],
                left_0 + sin_t * points[:, 0] + cos_t * points[:, 1],
            ], axis=1))
        # The remembered points are thinned out; close the gaps between them.
        return cv2.dilate(self.draw(np.concatenate(moved)), np.ones((3, 3), np.uint8))

    def remember(self, yellow_points, white_points):
        """Keep this frame's lines, and drop what is too far behind."""
        if self.pose is None:
            return
        if self.memory:
            _, (x, y, yaw), _, _ = self.memory[-1]
            turned = abs(np.arctan2(np.sin(self.pose[2] - yaw), np.cos(self.pose[2] - yaw)))
            if np.hypot(self.pose[0] - x, self.pose[1] - y) < 0.015 and turned < 0.05:
                return
        self.memory.append((self.travelled, self.pose, yellow_points[::3], white_points[::3]))
        self.memory = [entry for entry in self.memory
                       if self.travelled - entry[0] < self.memory_distance][-40:]

    def line_along_arc(self, mask, side):
        """Distance from each candidate to a line, and whether it bounds the lane there.

        Returns (distance in metres, usable) per candidate. A line bounds the
        lane at a candidate when it is near enough to say anything, and lies
        on its own side of the way from the robot to the candidate: yellow on
        the left, white on the right. That is what tells the robot which way
        to turn at a line running across in front of it, and what keeps it
        from taking the far side of a line for the lane.
        """
        metres, sine = self.line_geometry(mask)
        on_its_side = sine > 0.15 if side == 'left' else sine < -0.15
        return metres, on_its_side & (metres < self.max_line_distance)

    def line_geometry(self, mask):
        """Per candidate: distance to the nearest line pixel (m), and the sine of
        the angle from the way to the candidate round to it (positive: left)."""
        arc = self.arc
        if not mask.any():
            return np.zeros(len(arc['x'])), np.zeros(len(arc['x']))
        distance, labels = cv2.distanceTransformWithLabels(
            cv2.bitwise_not(mask), cv2.DIST_L2, cv2.DIST_MASK_5,
            labelType=cv2.DIST_LABEL_PIXEL)
        # Label of a line pixel -> where that pixel is.
        line_y, line_x = mask.nonzero()
        where = np.zeros((labels.max() + 1, 2), np.float32)
        where[labels[line_y, line_x]] = np.stack([line_x, line_y], axis=1)

        nearest = where[labels[arc['y'], arc['x']]]
        metres = distance[arc['y'], arc['x']] / self.map_ppm
        near_ahead, near_left = self.to_robot(nearest[:, 0], nearest[:, 1])
        # Sine of the angle from the way to the candidate round to the line:
        # positive with the line on the left.
        to_line_ahead = near_ahead - arc['ahead']
        to_line_left = near_left - arc['left']
        sine = ((arc['ahead'] * to_line_left - arc['left'] * to_line_ahead)
                / (self.lookahead * np.maximum(metres, 1e-3)))
        return metres, sine

    def clear_way(self, lines):
        """Candidates that can be reached without driving across a line."""
        arc = self.arc
        clear = np.ones(len(arc['x']), bool)
        origin_x, origin_y = self.to_pixel(0.0, 0.0)
        for fraction in np.linspace(0.3, 0.9, 7):
            x = np.int_(origin_x + fraction * (arc['x'] - origin_x))
            y = np.int_(origin_y + fraction * (arc['y'] - origin_y))
            clear &= lines[y, x] == 0
        return clear

    def fit_curvature(self, mask):
        """Curvature of the road ahead (1/m, positive bending left), or NaN.

        Each line in view is fitted with left = a + b*ahead + c*ahead^2; the
        curvature is 2c. Where the robot sits and which way it points move a
        and b, not c, so a robot weaving down a straight does not read as a
        bend, which the steering curvature cannot tell apart.
        """
        small = cv2.resize(mask, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)
        count, labels = cv2.connectedComponents((small >= 128).astype(np.uint8))
        scale = self.ppm / 4.0
        values, weights = [], []
        for i in range(1, count):
            y, x = (labels == i).nonzero()
            if len(y) < CURVATURE_MIN_POINTS:
                continue
            ahead = self.near + (small.shape[0] - 1 - y) / scale
            if ahead.max() - ahead.min() < CURVATURE_MIN_SPAN:
                continue
            left = (small.shape[1] / 2.0 - x) / scale
            values.append(2.0 * np.polyfit(ahead, left, 2)[0])
            weights.append(len(y))
        return float(np.average(values, weights=weights)) if values else float('nan')

    def lines_either_side(self, mask):
        """Nearest line on the left and on the right of the way to each candidate.

        Every line (connected patch of line pixels, whatever its colour) is
        judged on its own, so a line on the left cannot hide one on the right.
        Returns (left distance, left usable, right distance, right usable).
        """
        n = len(self.arc['x'])
        left, right = np.full(n, np.inf), np.full(n, np.inf)
        count, labels, stats, _ = cv2.connectedComponentsWithStats(
            cv2.dilate(mask, np.ones((3, 3), np.uint8)))
        biggest = sorted(range(1, count), key=lambda i: -stats[i, cv2.CC_STAT_AREA])
        for i in biggest[:LINE_COMPONENTS_MAX]:
            if stats[i, cv2.CC_STAT_AREA] < LINE_COMPONENT_MIN_PX:
                break
            line = np.where((labels == i) & (mask > 0), 255, 0).astype(np.uint8)
            distance, sine = self.line_geometry(line)
            near = distance < self.max_line_distance
            for usable, best in ((near & (sine > 0.15), left), (near & (sine < -0.15), right)):
                closer = usable & (distance < best)
                best[closer] = distance[closer]
        left_ok, right_ok = np.isfinite(left), np.isfinite(right)
        return np.where(left_ok, left, 0.0), left_ok, np.where(right_ok, right, 0.0), right_ok

    def find_target(self, yellow_mask, white_mask):
        """Pick the candidate on the lane centre: (index, state), or (None, LANE_NONE)."""
        half = self.lane_width / 2.0
        if self.yellow_left:
            yellow, yellow_ok = self.line_along_arc(yellow_mask, 'left')
            white, white_ok = self.line_along_arc(white_mask, 'right')
        else:
            # Colour-blind: 'yellow' is the left line, 'white' the right one.
            yellow, yellow_ok, white, white_ok = self.lines_either_side(yellow_mask | white_mask)
        clear = self.clear_way(yellow_mask | white_mask)
        yellow_ok &= clear
        white_ok &= clear

        # How far each candidate is from the middle of the lane: between the
        # two lines where they are a lane apart, else beside whichever is
        # there.
        both = yellow_ok & white_ok & (yellow + white < self.max_width)
        beside_yellow = np.where(yellow_ok, np.abs(yellow - half), np.inf)
        beside_white = np.where(white_ok, np.abs(white - half), np.inf)
        cost = np.where(both, np.abs(yellow - white) / 2.0,
                        np.minimum(beside_yellow, beside_white))
        if not np.isfinite(cost).any():
            return None, LANE_NONE

        centred = np.flatnonzero(cost < self.centre_tolerance)
        if len(centred) > 0:
            # Each run of candidates is a way on; at a fork there are two.
            # The arc runs from right to left.
            gaps = np.flatnonzero(np.diff(centred) > 1)
            first = np.concatenate([centred[:1], centred[gaps + 1]])
            last = np.concatenate([centred[gaps], centred[-1:]])
            way = -1 if (self.follow or self.fork_side) == 'left' else 0
            index = int(first[way] + np.argmin(cost[first[way]:last[way] + 1]))
        else:
            index = int(np.argmin(cost))

        if both[index]:
            return index, LANE_BOTH
        if beside_yellow[index] <= beside_white[index]:
            return index, LANE_LEFT_ONLY
        return index, LANE_RIGHT_ONLY

    # --- main ------------------------------------------------------------

    def callback_image(self, msg):
        self.counter += 1
        if self.counter % self.process_every_n != 0:
            return

        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        if self.map_shape is None:
            self.build_map(image.shape[:2])
        # The candidate arc must stay on the map (indices) and in front of the axle.
        lookahead = min(self.lookahead_for(self.speed), self.map_far - 0.02)
        if abs(lookahead - self.lookahead) > 0.005:
            self.lookahead = lookahead
            self.build_arc()
        self.pub_lookahead.publish(Float64(data=self.lookahead))

        yellow_pixels, yellow_mask, self.reliability_yellow = self.mask_line(
            hsv, self.yellow, self.reliability_yellow)
        only = self.thin_bright(hsv) if self.white_contrast_min > 0 else None
        if self.yellow_gap_kernel is not None:
            yellow_any = cv2.inRange(hsv, self.yellow['lower'], self.yellow['upper'])
            away = cv2.bitwise_not(cv2.dilate(yellow_any, self.yellow_gap_kernel))
            only = away if only is None else cv2.bitwise_and(only, away)
        white_pixels, white_mask, self.reliability_white = self.mask_line(
            hsv, self.white, self.reliability_white, only=only)

        self.pub_reliability_yellow.publish(UInt8(data=self.reliability_yellow))
        self.pub_reliability_white.publish(UInt8(data=self.reliability_white))

        now = self.now()
        if now - self.follow_time > self.follow_timeout:
            self.follow = None

        self.locate(msg.header.stamp)
        self.lane_curvature = self.fit_curvature(yellow_mask | white_mask)
        self.pub_curvature.publish(Float64(data=self.lane_curvature))
        # A scatter of stray pixels is not a line.
        none = np.zeros((0, 2))
        yellow_points = self.line_points(yellow_mask) if yellow_pixels > self.min_pixels else none
        white_points = self.line_points(white_mask) if white_pixels > self.min_pixels else none
        yellow_map = self.draw(yellow_points) | self.remembered(2)
        white_map = self.draw(white_points) | self.remembered(3)
        self.remember(yellow_points, white_points)
        self.publish_lines(yellow_points, white_points, msg.header.stamp)

        index, state = self.find_target(yellow_map, white_map)
        if index is not None:
            target = (float(self.arc['ahead'][index]), float(self.arc['left'][index]))
            self.last_target = target
            self.last_target_time = now
        elif self.last_target is not None and now - self.last_target_time < self.hold_last_center:
            # Bridge a dropped frame or two rather than handing back a gap in
            # the command stream; standing still is what ends a run.
            target = self.last_target
        else:
            target = None

        self.pub_state.publish(UInt8(data=state))

        if target is not None:
            ahead, left = target
            self.pub_lane.publish(Float64(data=image.shape[1] / 2.0 - left * self.ppm))
            self.pub_offset.publish(Float64(data=-left))
            point = PointStamped()
            point.header.stamp = msg.header.stamp
            point.header.frame_id = 'base_footprint'
            point.point.x = ahead
            point.point.y = left
            self.pub_target.publish(point)

        if self.publish_debug:
            self.publish_debug_image(yellow_map, white_map, target, state, msg.header)
            self.publish_bev_overlay(
                image,
                yellow_mask if yellow_pixels > self.min_pixels else None,
                white_mask if white_pixels > self.min_pixels else None,
                target, msg.header)

    def draw_status(self, image):
        """Straight/curve call, speed and look-ahead, top left."""
        text = (f'{"CURVE" if self.curve else "STRAIGHT"}  v {self.speed:.2f}  '
                f'LAD {self.lookahead:.2f}  k {self.lane_curvature:+.1f}')
        colour = (0, 0, 255) if self.curve else (0, 200, 0)
        cv2.rectangle(image, (0, 0), (min(image.shape[1] - 1, 420), 30), (0, 0, 0), -1)
        cv2.putText(image, text, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 2)

    def publish_bev_overlay(self, image, yellow_mask, white_mask, target, header):
        """The bird's eye view with only the line pixels the lane is steered by painted on.

        Pixels that matched a colour but were thrown out (blobs, too few to be
        a line) are left as they are, so what is painted is what counts.
        """
        if not self.pub_bev_overlay.get_subscription_count():
            return
        out = image.copy()
        for mask, colour in ((yellow_mask, (0, 220, 255)), (white_mask, (255, 255, 0))):
            if mask is not None:
                out[mask > 0] = colour
        h, w = out.shape[:2]

        def to_bev(ahead, left):
            return (np.int_(w / 2.0 - left * self.ppm),
                    np.int_(h - 1 - (ahead - self.near) * self.ppm))

        # The candidate targets, `lookahead` from the axle.
        for x, y in zip(*to_bev(self.arc['ahead'][::4], self.arc['left'][::4])):
            cv2.circle(out, (int(x), int(y)), 1, (0, 255, 0), -1)
        if target is not None:
            x, y = to_bev(*target)
            cv2.circle(out, (int(x), int(y)), 7, (0, 0, 255), -1)
        self.draw_status(out)
        msg = self.bridge.cv2_to_imgmsg(out, 'bgr8')
        msg.header = header
        self.pub_bev_overlay.publish(msg)

    def publish_lines(self, yellow_points, white_points, stamp):
        """Publish this picture's lines as points (x ahead, y left of the axle)."""
        for colour, points in (('yellow', yellow_points), ('white', white_points)):
            points = points[::3]
            cloud = np.zeros((len(points), 3), np.float32)
            cloud[:, :2] = points
            header = Header(stamp=stamp, frame_id='base_footprint')
            self.pub_lines[colour].publish(point_cloud2.create_cloud_xyz32(header, cloud))

    def publish_debug_image(self, yellow_map, white_map, target, state, header):
        """Publish the map round the robot: lines, the arc of candidates, the target."""
        overlay = np.full(self.map_shape + (3,), 40, np.uint8)
        # What the camera sees now (the rest is lines remembered from earlier
        # frames), and a tick every 10 cm ahead of the axle.
        x0, y0 = self.to_pixel(self.near, self.view_half_width)
        x1, y1 = self.to_pixel(self.map_far, -self.view_half_width)
        cv2.rectangle(overlay, (int(x0), int(y1)), (int(x1), int(y0)), (90, 90, 90), 1)
        cv2.putText(overlay, 'camera view', (int(x0) + 4, int(y0) - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (130, 130, 130), 1)
        for ahead in np.arange(0.0, self.map_far, 0.1):
            _, y = self.to_pixel(ahead, 0.0)
            cv2.line(overlay, (0, int(y)), (8, int(y)), (130, 130, 130), 1)
            cv2.putText(overlay, f'{ahead:.1f}', (10, int(y) + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (130, 130, 130), 1)
        overlay[yellow_map > 0] = (0, 200, 255)
        overlay[white_map > 0] = (255, 255, 255)
        for x, y in zip(self.arc['x'][::4], self.arc['y'][::4]):
            cv2.circle(overlay, (int(x), int(y)), 1, (0, 255, 0), -1)
        x, y = self.to_pixel(0.0, 0.0)
        cv2.circle(overlay, (int(x), int(y)), 4, (255, 120, 0), -1)
        if target is not None:
            x, y = self.to_pixel(target[0], target[1])
            cv2.circle(overlay, (int(x), int(y)), 7, (0, 0, 255), -1)
        cv2.putText(
            overlay,
            f'state {state}' + (f'  follow {self.follow}' if self.follow else ''),
            (60, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1
        )
        self.draw_status(overlay)
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
