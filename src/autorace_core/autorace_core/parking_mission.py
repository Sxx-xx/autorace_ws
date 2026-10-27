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

"""Parking mission: into the empty bay of the parking lot, and out again.

The lot lies off the road, up a short lane that leaves it to the left. From
the road nothing tells that lane from the road going straight on, so the
parking sign is what starts the mission, and from then on the lane follower
is asked to keep to the left line, which turns up the lane.

The lane runs between two yellow lines. Where it reaches the lot the lines
become white and broken: an aisle with a bay on either side, and another robot
parked in one of them. No white line is in sight all the way up the lane, so
the first white seen there is the lot, and the lane camera gives where it
begins and where the middle of the aisle is. The laser, meanwhile, has been
counting what stands to the left and to the right of the way ahead: the side
with something in it is the bay that is taken.

From there the robot is driven by odometry, over a metre or so. Turning on
the spot costs the odometry a few centimetres each time, so in the aisle the
broken lines either side are used to stay in the middle of it:

    turn_in   the lane follower takes the left line off the road and up the lane
    aisle     straight on to just short of the bays
    swing_in  round a quarter circle towards the empty bay
    into_bay  drive in
    parked    stand for a moment
    back_out  reverse out, not quite to the middle of the aisle
    face_out  turn on the spot, the same way round, to face down the lane
    leave     drive out of the aisle and a little way down the lane
    exit      the lane follower takes the left line again, down the lane and
              round on to the road in the direction the robot was going

The line on the left going down the lane is the one that carries on along the
road, so keeping to it brings the robot out the right way.

The robot's tail sticks out 10 cm behind the axle and swings towards the
parked robot whenever the robot turns to or from the empty bay, and in the
middle of the aisle there are only 3 cm to spare for it. Both turns are
therefore made with the axle on the empty bay's side of the middle: the one in
on an arc rather than on the spot, the one out before the robot is all the way
back.
"""

from collections import deque
import math

import cv2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import String


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class ParkingMission(Node):

    def __init__(self):
        super().__init__('parking_mission')

        # The lot is where white line shows after this much road without any.
        self.declare_parameter('white_free_distance', 0.3)
        self.declare_parameter('white_min_points', 150)
        # Up the lane to the lot (after the turn in) the lot begins where the
        # lane's yellow lines end: no yellow nearer than near_yellow_distance
        # for near_yellow_missing pictures, the lot lot_ahead_at_end ahead.
        # Failing that, lane_length_max up the lane. The lot's white cannot
        # say it: the lane is short, so it is in view from the turn, and the
        # shiny mat throws the window's light back white (2026-10-27).
        self.declare_parameter('near_yellow_distance', 0.30)
        self.declare_parameter('near_yellow_missing', 3)
        self.declare_parameter('lot_ahead_at_end', 0.15)
        self.declare_parameter('lane_length_max', 0.60)
        # From where the white begins to level with the middle of the bays,
        # and how far from the middle of the aisle the axle goes into a bay:
        # past the middle of the bay, as most of the robot is behind its axle.
        self.declare_parameter('bay_station', 0.24)
        # From a line of the aisle, or of the lane up to it, to its middle.
        self.declare_parameter('aisle_half_width', 0.12)
        self.declare_parameter('bay_depth', 0.29)
        # The arc into the bay, and how far short of the middle of the aisle
        # the robot stops backing out to turn.
        self.declare_parameter('swing_radius', 0.10)
        self.declare_parameter('turn_offset', 0.06)
        # Where the lane follower takes over again: this far before the white,
        # far enough down the lane for the lines of the lot to be behind it.
        self.declare_parameter('leave_station', -0.30)
        # Where the laser looks for the parked robot, either side of the way
        # ahead: this far ahead, and between these distances to the side.
        self.declare_parameter('bay_look_ahead', 0.9)
        self.declare_parameter('bay_look_near', 0.12)
        self.declare_parameter('bay_look_far', 0.45)
        # The bay taken when the laser has seen nothing in either.
        self.declare_parameter('default_bay', 'left')
        self.declare_parameter('lidar_offset', -0.032)
        self.declare_parameter('speed', 0.08)
        # The lane follower is held to this on the way up to the lot and
        # back down to the road: it arrives steadily, and the reckoning
        # inside starts from a robot that is not still pitching from a
        # sudden slowing.
        self.declare_parameter('approach_speed', 0.12)
        self.declare_parameter('turn_rate', 0.8)
        self.declare_parameter('park_time', 1.0)
        # The mission is over when the robot is heading along the road again,
        # or has driven this far from the lot without getting there.
        self.declare_parameter('exit_limit', 1.6)
        # The aisle's direction from the yellow lines of this much lane before
        # the lot, each side at least this long in view, and no more than
        # this far off the robot's heading (else the robot's heading is used).
        # The entrance to the lot on the real course: the left yellow line
        # turns up the lane to the lot while the road, and its yellow line,
        # carry straight on beyond it, so keeping to the left line drives past
        # (2026-10-10). The robot turns in on an arc of this radius, centred
        # on the lane to the lot, which lies half a lane past where the
        # yellow line runs up it; it looks for the entrance over at most
        # entrance_limit of road after the sign, then falls back to keeping
        # left.
        self.declare_parameter('find_entrance', True)
        self.declare_parameter('entrance_half_lane', 0.165)
        # The entrance is the gap in the left yellow line: empty for at least
        # entrance_gap_min past where the line ended, the white line still on
        # the right, the robot not turning; the middle of the lane to the lot
        # is entrance_gap_half past the end of the line.
        self.declare_parameter('entrance_gap_min', 0.06)
        # The left yellow line has to have run this far before the gap (a
        # glimpse of yellow leaving the construction zone is not the line).
        # 0.30 was a hair too much with the robot set down 0.3 m before
        # the line (2026-10-27).
        self.declare_parameter('entrance_line_min', 0.20)
        self.declare_parameter('entrance_gap_half', 0.15)
        self.declare_parameter('entrance_radius', 0.17)
        self.declare_parameter('entrance_speed', 0.08)
        self.declare_parameter('entrance_limit', 2.5)
        # After the turn in, up the lane to the lot between its two yellow
        # lines this node drives itself: the lane follower takes no yellow line
        # for the right edge of a lane, and steered out of it (2026-10-10).
        self.declare_parameter('up_lane_speed', 0.08)
        # Steering up the lane: by the robot's place in the lane (m to the
        # left of the middle) and its heading against the lane's direction,
        # the direction being the median of the lines' angles over the last
        # up_lane_direction_memory of road. Angles beyond up_lane_max_angle
        # are not the lane's lines.
        self.declare_parameter('up_lane_middle_gain', 3.0)
        self.declare_parameter('up_lane_heading_gain', 0.7)
        self.declare_parameter('up_lane_direction_memory', 0.25)
        # Pictures the direction has to rest on before it is believed: the
        # first second after the turn in sees the bend of the line coming off
        # the road, and the median sat 30 deg off until 30 pictures (2026-10-27).
        self.declare_parameter('up_lane_direction_min', 30)
        self.declare_parameter('up_lane_max_angle', 0.6)
        self.declare_parameter('aisle_line_memory', 0.30)
        self.declare_parameter('aisle_line_length', 0.10)
        self.declare_parameter('aisle_max_angle', 1.0)
        # The manager repeats the activation; without it the mission is over.
        self.declare_parameter('active_timeout', 1.0)

        def value(name):
            return self.get_parameter(name).value

        self.white_free_distance = value('white_free_distance')
        self.white_min_points = value('white_min_points')
        self.near_yellow_distance = value('near_yellow_distance')
        self.near_yellow_missing = value('near_yellow_missing')
        self.lot_ahead_at_end = value('lot_ahead_at_end')
        self.lane_length_max = value('lane_length_max')
        self.bay_station = value('bay_station')
        self.aisle_half_width = value('aisle_half_width')
        self.bay_depth = value('bay_depth')
        self.swing_radius = value('swing_radius')
        self.turn_offset = value('turn_offset')
        self.leave_station = value('leave_station')
        self.bay_look_ahead = value('bay_look_ahead')
        self.bay_look_near = value('bay_look_near')
        self.bay_look_far = value('bay_look_far')
        self.default_bay = value('default_bay')
        self.lidar_offset = value('lidar_offset')
        self.speed = value('speed')
        self.approach_speed = value('approach_speed')
        self.turn_rate = value('turn_rate')
        self.park_time = value('park_time')
        self.exit_limit = value('exit_limit')
        self.find_entrance = value('find_entrance')
        self.entrance_half_lane = value('entrance_half_lane')
        self.entrance_gap_min = value('entrance_gap_min')
        self.entrance_line_min = value('entrance_line_min')
        self.entrance_gap_half = value('entrance_gap_half')
        self.entrance_radius = value('entrance_radius')
        self.entrance_speed = value('entrance_speed')
        self.entrance_limit = value('entrance_limit')
        self.up_lane_speed = value('up_lane_speed')
        self.up_lane_middle_gain = value('up_lane_middle_gain')
        self.up_lane_heading_gain = value('up_lane_heading_gain')
        self.up_lane_direction_memory = value('up_lane_direction_memory')
        self.up_lane_direction_min = value('up_lane_direction_min')
        self.up_lane_max_angle = value('up_lane_max_angle')
        self.aisle_line_memory = value('aisle_line_memory')
        self.aisle_line_length = value('aisle_line_length')
        self.aisle_max_angle = value('aisle_max_angle')
        self.active_timeout = value('active_timeout')

        self.phase = None           # None while the mission is not running
        self.active_time = 0.0
        # Odometry of the last few seconds: (time, x, y, yaw, distance travelled).
        self.odometry = deque(maxlen=400)
        self.white_seen_at = None   # distance travelled when white was last in sight
        self.seen = {'left': 0, 'right': 0}     # laser returns to either side
        # The lot: where the white begins on the middle of the aisle, and the
        # direction of the aisle, in the odometry frame.
        self.lot = None
        self.side = 0.0             # +1 for the bay on the left, -1 on the right
        # Left of the middle of the aisle by the lane camera, and when; and
        # how much that is more than odometry makes it.
        self.aisle_seen = (0.0, 0.0)
        self.aisle_correction = 0.0
        self.yellow = None          # (stamp, points) of the latest picture
        # On the lane to the lot: (distance travelled, yellow points in the
        # odometry frame) of the last stretch, for the direction of the aisle.
        self.yellow_memory = deque()
        # The entrance: where the last picture put it (along the robot's way),
        # and the turn in: (x, y, yaw at the start, distance to drive before
        # the arc).
        self.entrance_seen = None
        self.entrance = None
        self.arc_yaw = None         # heading the turn in started on
        self.yellow_end = None      # distance travelled to where the left yellow line ends
        self.yellow_start = None    # and to where it was first seen in this stretch
        self.empty_pictures = 0     # pictures in a row with no left yellow line
        self.yellow_side = None     # how far left of the robot that line runs
        self.side_misses = 0        # pictures in a row with yellow on the left but not there
        self.up_lane = False        # turned in: this node drives up the lane
        self.up_lane_yaw = 0.0      # heading at the end of the turn in
        self.lane_dirs = deque()    # (distance travelled, world direction of a lane line in a picture)
        self.lane_sides = {'left': None, 'right': None}   # (left of the robot, when) each line was last seen
        self.near_yellow_gone = 0   # pictures in a row without yellow close ahead
        self.lane_middles = deque() # (distance travelled, the lane's middle left of the robot)
        self.phase_start = 0.0      # time and distance travelled
        self.phase_travelled = 0.0

        self.create_subscription(
            Bool, '/autorace/mission/parking/active', self.callback_active, 1)
        self.create_subscription(Odometry, '/odom', self.callback_odom, 1)
        self.create_subscription(LaserScan, '/scan', self.callback_scan, 1)
        self.create_subscription(
            PointCloud2, '/detect/lane_lines/yellow', self.callback_yellow, 1)
        self.create_subscription(
            PointCloud2, '/detect/lane_lines/white', self.callback_white, 1)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel/mission', 1)
        self.pub_follow = self.create_publisher(String, '/detect/lane_follow', 1)
        self.pub_limit = self.create_publisher(Float64, '/control/max_vel', 1)
        self.pub_done = self.create_publisher(Bool, '/autorace/mission/parking/done', 1)

        self.create_timer(0.05, self.update)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    # --- callbacks -------------------------------------------------------

    def callback_active(self, msg):
        if msg.data:
            self.active_time = self.now()
            if self.phase is None and self.odometry:
                self.get_logger().info('Parking: keeping left for the lane to the lot.')
                self.lot = None
                self.yellow_memory.clear()
                self.seen = {'left': 0, 'right': 0}
                self.white_seen_at = self.odometry[-1][4]
                self.entrance_seen = None
                self.entrance = None
                # The left yellow line has been watched all along the road
                # (yellow_start, yellow_end, yellow_side): the sign stands
                # only half a metre before the entrance, and from the sign
                # alone the line was not seen long enough (0.26 m of 0.30)
                # to count its end as the entrance (2026-10-27).
                self.up_lane = False
                if self.find_entrance:
                    self.get_logger().info('Parking: looking for the entrance to the lot.')
                    self.enter('to_entrance')
                else:
                    self.enter('turn_in')
        elif self.phase is not None:
            self.get_logger().info(f'Parking: deactivated during {self.phase}.')
            self.phase = None

    def callback_odom(self, msg):
        position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        travelled = 0.0
        if self.odometry:
            _, x, y, _, travelled = self.odometry[-1]
            travelled += math.hypot(position.x - x, position.y - y)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.odometry.append((stamp, position.x, position.y, yaw, travelled))

    def pose_at(self, stamp):
        """Return (x, y, yaw, travelled) of the robot at a message stamp."""
        when = stamp.sec + stamp.nanosec * 1e-9
        return min(self.odometry, key=lambda entry: abs(entry[0] - when))[1:]

    def callback_scan(self, msg):
        """Count what stands in either bay, once the lot is known.

        The bays lie beside the aisle: along it from the lot's beginning to
        bay_look_ahead, from bay_look_near to bay_look_far to either side.
        Returns are taken in the lot's frame, so what stood beside the lane
        on the way up (the lot's outline, a sign) is not counted; the lidar
        does not see the lane's own sides anyway (2026-10-27).
        """
        if self.phase not in ('turn_in', 'aisle') or self.lot is None or not self.odometry:
            return
        _, x, y, yaw, _ = self.odometry[-1]
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        seen = np.isfinite(ranges) & (ranges > msg.range_min + 1e-3)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        ahead = self.lidar_offset + ranges[seen] * np.cos(angles[seen])
        left = ranges[seen] * np.sin(angles[seen])
        # Into the lot's frame: along the aisle from where the lot begins,
        # and to the left of the aisle.
        wx = x + math.cos(yaw) * ahead - math.sin(yaw) * left
        wy = y + math.sin(yaw) * ahead + math.cos(yaw) * left
        x_0, y_0, direction = self.lot
        cos_d, sin_d = math.cos(direction), math.sin(direction)
        along = cos_d * (wx - x_0) + sin_d * (wy - y_0)
        across = -sin_d * (wx - x_0) + cos_d * (wy - y_0)
        in_bays = ((along > -0.05) & (along < self.bay_look_ahead)
                   & (np.abs(across) > self.bay_look_near) & (np.abs(across) < self.bay_look_far))
        self.seen['left'] += int(np.count_nonzero(in_bays & (across > 0.0)))
        self.seen['right'] += int(np.count_nonzero(in_bays & (across < 0.0)))

    def callback_yellow(self, msg):
        """Keep the yellow lines of a picture for when its white ones arrive."""
        if self.phase in (None, 'aisle', 'leave', 'to_entrance', 'turn_in'):
            self.yellow = (msg.header.stamp,
                           point_cloud2.read_points_numpy(msg, field_names=('x', 'y')))
        if self.phase == 'turn_in' and self.odometry:
            points = point_cloud2.read_points_numpy(msg, field_names=('x', 'y'))
            if self.up_lane:
                near = points[(points[:, 0] > 0.12) & (points[:, 0] < self.near_yellow_distance)]
                self.near_yellow_gone = 0 if len(near) >= 10 else self.near_yellow_gone + 1
            if len(points) == 0:
                return
            x, y, yaw, travelled = self.pose_at(msg.header.stamp)
            cos_y, sin_y = math.cos(yaw), math.sin(yaw)
            points = points[::2].astype(np.float64)
            world = np.stack([x + cos_y * points[:, 0] - sin_y * points[:, 1],
                              y + sin_y * points[:, 0] + cos_y * points[:, 1]], axis=1)
            self.yellow_memory.append((travelled, world))
            while travelled - self.yellow_memory[0][0] > self.aisle_line_memory:
                self.yellow_memory.popleft()

    def aisle_direction(self, x, y, yaw):
        """The direction of the yellow lines either side of the lane to the lot.

        They run in line with the aisle. The lane comes round a bend to the
        lot, so the robot's own heading when the white shows can be well off
        the aisle's (0.77 rad on 2026-10-10); the lines of the last stretch
        are not. Each side's points are fitted with a straight line; None
        when neither side is long enough to tell.
        """
        if not self.yellow_memory:
            return None
        world = np.concatenate([entry[1] for entry in self.yellow_memory])
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        dx, dy = world[:, 0] - x, world[:, 1] - y
        ahead, left = cos_y * dx + sin_y * dy, -sin_y * dx + cos_y * dy
        angles, weights = [], []
        for side in (left > 0.0, left < 0.0):
            pts = np.stack([ahead[side], left[side]], axis=1).astype(np.float32)
            if len(pts) < 30 or np.ptp(pts[:, 0]) < self.aisle_line_length:
                continue
            vx, vy, _, _ = cv2.fitLine(pts, cv2.DIST_HUBER, 0, 0.01, 0.01).ravel()
            if vx < 0.0:
                vx, vy = -vx, -vy
            angles.append(math.atan2(vy, vx))
            weights.append(len(pts))
        if not angles:
            return None
        angle = float(np.average(angles, weights=weights))
        if abs(angle) > self.aisle_max_angle:
            return None
        return wrap(yaw + angle)

    def callback_white(self, msg):
        if self.phase not in (None, 'to_entrance', 'turn_in', 'aisle', 'leave') or not self.odometry:
            return
        points = point_cloud2.read_points_numpy(msg, field_names=('x', 'y'))
        x, y, yaw, travelled = self.pose_at(msg.header.stamp)
        if self.phase is None:
            # Idle: keep watching the left yellow line, so that when the
            # sign comes its length is known.
            if self.find_entrance and self.yellow is not None \
                    and self.yellow[0] == msg.header.stamp:
                self.look_for_entrance(self.yellow[1], points, travelled)
            return
        if self.phase == 'to_entrance':
            if self.yellow is not None and self.yellow[0] == msg.header.stamp:
                self.look_for_entrance(self.yellow[1], points, travelled)
            return
        if self.phase != 'turn_in':
            # The aisle runs between white lines and the lane to it between
            # yellow ones, in line with them.
            if self.yellow is not None and self.yellow[0] == msg.header.stamp:
                points = np.concatenate([points, self.yellow[1]])
            self.see_aisle(points, yaw)
            return
        if self.up_lane:
            up = travelled - self.phase_travelled
            if up < 0.10:
                return
            if self.near_yellow_gone >= self.near_yellow_missing:
                begins, why = self.lot_ahead_at_end, 'the yellow lines have ended'
            elif up >= self.lane_length_max:
                begins, why = 0.0, f'{up:.2f} m up the lane'
            else:
                return
            middle = self.lane_middle(travelled)
            self.get_logger().info(f'Parking: the lot, as {why}; lane middle {middle:+.3f} m to the left.')
        else:
            if len(points) < self.white_min_points:
                return
            free = travelled - self.white_seen_at
            self.white_seen_at = travelled
            if free < self.white_free_distance:
                # Still the white line of the road.
                return
            # The white begins at the nearest of it. The aisle runs between
            # the broken lines, one to each side.
            begins = float(np.percentile(points[:, 0], 5))
            left = points[points[:, 1] > 0.0, 1]
            right = points[points[:, 1] < 0.0, 1]
            middle = 0.0
            if len(left) > 20 and len(right) > 20:
                middle = float(np.median(left) + np.median(right)) / 2.0
        # The aisle runs the way of the yellow lines up to it; without them,
        # the way the robot has been going.
        recent = [entry[3] for entry in self.odometry if travelled - entry[4] < 0.15]
        heading = math.atan2(sum(math.sin(a) for a in recent), sum(math.cos(a) for a in recent))
        direction = None
        if self.up_lane:
            direction, pictures = self.lane_direction(travelled)
            if pictures >= self.up_lane_direction_min:
                self.get_logger().info(
                    'Parking: aisle %+.2f rad from the robot by the lines of the last %d pictures.'
                    % (wrap(direction - yaw), pictures))
            else:
                direction = None
        if direction is None:
            direction = self.aisle_direction(x, y, yaw)
        if direction is None:
            self.get_logger().warn('Parking: no yellow lines to set the aisle by; '
                                   'taking the robot\'s heading.')
            direction = heading
        else:
            self.get_logger().info(
                'Parking: aisle %+.2f rad from the robot by the yellow lines '
                '(heading of the last 15 cm %+.2f).'
                % (wrap(direction - yaw), wrap(heading - yaw)))
        self.lot = (x + math.cos(yaw) * begins - math.sin(yaw) * middle,
                    y + math.sin(yaw) * begins + math.cos(yaw) * middle,
                    direction)
        self.get_logger().info(
            f'Parking: lot {begins:.2f} m ahead, aisle {middle:+.3f} m to the left.')
        self.enter('aisle')

    def look_for_entrance(self, yellow, white, travelled_pic):
        """Find the gap in the left yellow line where the lane to the lot leaves.

        While the left yellow line is in view, how far along the road it runs
        is kept (distance travelled to the axle plus how far ahead it is seen).
        The entrance is where it stops: nothing yellow on the left over the
        whole view, at least entrance_gap_min past its end, with the white line
        still on the right and the robot not turning (in a bend the inner line
        can slide out of view too). A bend in the line, taken for the entrance
        before, was the road itself bending left (2026-10-10). The middle of
        the lane to the lot is then entrance_gap_half past the end of the
        line: straight on to an entrance_radius short of it, then a quarter
        circle to the left.
        """
        _, _, _, yaw_now, travelled = self.odometry[-1]
        recent = [entry[3] for entry in self.odometry if travelled - entry[4] < 0.15]
        turning = bool(recent) and abs(wrap(yaw_now - recent[0])) > 0.15
        left = yellow[(yellow[:, 1] > 0.03) & (yellow[:, 1] < 0.35)
                      & (yellow[:, 0] > 0.15) & (yellow[:, 0] < 0.45)]
        if self.yellow_side is not None:
            # Only the line the robot has been running along: the far side of
            # the gap shows up off to the left of it, and ended the gap
            # before it could be seen whole (2026-10-10).
            beside = left[np.abs(left[:, 1] - self.yellow_side) < 0.06]
            if len(beside) < 10 and len(left) >= 10:
                # Yellow on the left, but not where the line was: after a
                # while that is the line, somewhere else (a bend, or the
                # line watched before was another), and it is taken afresh.
                self.side_misses += 1
                if self.side_misses >= 10:
                    self.yellow_side = None
                    self.yellow_end = None
                    beside = left
            else:
                self.side_misses = 0
            left = beside
        right = white[(white[:, 1] < -0.03) & (white[:, 0] > 0.15) & (white[:, 0] < 0.45)]
        if len(left) >= 10:
            end = travelled_pic + float(np.percentile(left[:, 0], 95))
            if self.yellow_end is None or end - self.yellow_end > 0.25:
                # A new stretch of line (or the first).
                self.yellow_start = travelled_pic + float(np.percentile(left[:, 0], 5))
            self.yellow_end = end if self.yellow_end is None else max(self.yellow_end, end)
            side = float(np.median(left[:, 1]))
            self.yellow_side = side if self.yellow_side is None else (
                0.8 * self.yellow_side + 0.2 * side)
            self.empty_pictures = 0
            return
        self.empty_pictures += 1
        # The line has to have run along the road for a while first (not a
        # glimpse leaving the construction zone), and be gone in three
        # pictures in a row.
        if (self.yellow_end is None or turning or len(right) < 10
                or self.yellow_end - self.yellow_start < self.entrance_line_min
                or self.empty_pictures < 3):
            return
        empty = travelled_pic + 0.45 - self.yellow_end
        if empty < self.entrance_gap_min:
            return
        if self.phase != 'to_entrance':
            # A gap seen while idle is none of this node's business (yet).
            return
        centre = self.yellow_end + self.entrance_gap_half
        straight_on = max(0.0, centre - self.entrance_radius - travelled)
        _, x_now, y_now, _, _ = self.odometry[-1]
        self.entrance = (x_now, y_now, yaw_now, straight_on)
        self.arc_yaw = None
        self.get_logger().info(
            'Parking: gap in the left yellow line %.2f m long, lane to the lot %.2f m ahead; '
            'straight on %.2f m, then round to the left.'
            % (empty, centre - travelled, straight_on))
        self.enter('entrance')

    def see_aisle(self, points, yaw):
        """How far left of the middle of the aisle the camera shows the robot to be.

        Between the lines on both sides when both are in the picture; a
        robot that has drifted to one side has only that side's line in
        view, and the middle is then half an aisle from it. A line along the
        aisle is narrow across the picture; the lot's outline runs across
        the way out, wide, and is not a line to steer between.
        """
        def along_the_aisle(side):
            return side if len(side) >= 20 and np.std(side[:, 1]) < 0.04 else side[:0]
        left = along_the_aisle(points[points[:, 1] > 0.0])
        right = along_the_aisle(points[points[:, 1] < 0.0])
        if len(left) >= 20 and len(right) >= 20:
            middle = float(np.median(left[:, 1]) + np.median(right[:, 1])) / 2.0
        elif len(left) >= 20:
            middle = float(np.median(left[:, 1])) - self.aisle_half_width
        elif len(right) >= 20:
            middle = float(np.median(right[:, 1])) + self.aisle_half_width
        else:
            return
        # The lines are seen some way ahead; a robot not square to the aisle
        # sees them off to one side by that much.
        course = 0.0 if self.phase == 'aisle' else math.pi
        askew = wrap(yaw - self.lot[2] - course)
        ahead = float(np.median(points[:, 0]))
        self.aisle_seen = (-middle - ahead * math.tan(askew), self.now())
        self.get_logger().info(
            'Parking: aisle middle %+.3f m to the left by %d/%d points, askew %.2f.'
            % (middle, len(left), len(right), askew), throttle_duration_sec=1.0)

    # --- driving ---------------------------------------------------------

    def enter(self, phase):
        self.phase = phase
        self.aisle_correction = 0.0
        self.phase_start = self.now()
        self.phase_travelled = self.odometry[-1][4]

    def in_lot(self):
        """Along the aisle from where the white begins, to its left, heading."""
        _, x, y, yaw, _ = self.odometry[-1]
        x_0, y_0, direction = self.lot
        cos_d, sin_d = math.cos(direction), math.sin(direction)
        dx, dy = x - x_0, y - y_0
        return cos_d * dx + sin_d * dy, -sin_d * dx + cos_d * dy, wrap(yaw - direction)

    def turn_to(self, heading, target):
        """Turn on the spot: (twist, whether it is there)."""
        error = wrap(target - heading)
        twist = Twist()
        if abs(error) < 0.03:
            return twist, True
        rate = max(0.15, min(self.turn_rate, 2.0 * abs(error)))
        twist.angular.z = math.copysign(rate, error)
        return twist, False

    def in_aisle(self, off_line):
        """Left of the middle of the aisle: odometry, set right by the camera."""
        seen, when = self.aisle_seen
        if self.now() - when < 0.25:
            self.aisle_correction = seen - off_line
        return off_line + self.aisle_correction

    def drive(self, heading, course, off_line, to_go, reverse=False):
        """Drive along a line on a course, `off_line` metres to the left of it."""
        # Steer back on to the line; going backwards the steering is mirrored.
        correction = max(-0.5, min(0.5, 4.0 * off_line))
        target = course + correction if reverse else course - correction
        # Slow down into the end of the line rather than run past it.
        speed = min(self.speed, max(0.02, 1.5 * to_go))
        twist = Twist()
        twist.linear.x = -speed if reverse else speed
        twist.angular.z = max(-1.0, min(1.0, 2.5 * wrap(target - heading)))
        return twist

    def up_lane_twist(self):
        """Up the middle of the lane to the lot, between its yellow lines.

        Each picture gives where the lines are beside the robot (reliable)
        and which way they run (not: a 27 cm piece of tape with a bent end
        swings 20 deg from one picture to the next, 2026-10-27). So the
        robot is steered by its place in the lane, and by its heading
        against the lane's direction as the median of the last stretch of
        pictures, never by one picture's angle. Weaving up the lane put the
        aisle 29 deg off and the parked robot in the wrong bay.

        The lines are told apart by following them from picture to picture,
        not by which side of the robot they lie: a robot that has drifted
        on to the right line sees it under itself, and cutting the picture
        at the robot's axis then made that line a "left" one and steered
        further right (2026-10-27, 15:25), besides tilting the fit of the
        piece that was cut off.
        """
        _, _, _, yaw, travelled = self.odometry[-1]
        twist = Twist()
        twist.linear.x = self.up_lane_speed
        middle = None
        if self.yellow is not None:
            points = self.yellow[1]
            points = points[(points[:, 0] > 0.12) & (points[:, 0] < 0.45)].astype(np.float64)
            # Lines as groups of points, split where a gap across the lane
            # is wider than a line.
            blobs = []
            if len(points) >= 15:
                order = np.argsort(points[:, 1])
                ys = points[order, 1]
                cuts = np.where(np.diff(ys) > 0.06)[0] + 1
                for group in np.split(order, cuts):
                    pts = points[group]
                    if len(pts) < 15 or np.ptp(pts[:, 0]) < 0.08:
                        continue
                    angle = self.line_angle(pts)
                    # A line across the way is not an edge of this lane:
                    # right after the turn in, the road's yellow line lay
                    # across the view and was steered along, back on to the
                    # road (2026-10-10).
                    if abs(angle) > 1.0:
                        continue
                    blobs.append((float(np.median(pts[:, 1])), angle, len(pts)))
            # Which line is which: the one nearest where that line was last
            # seen; a line never seen yet by its side of the robot.
            for side in ('left', 'right'):
                seen = self.lane_sides.get(side)
                if seen is not None and travelled - seen[1] > 0.30:
                    self.lane_sides[side] = None
            sides = {}
            unplaced = []
            for y, angle, n in sorted(blobs, key=lambda b: -b[2]):
                best = None
                for side in ('left', 'right'):
                    seen = self.lane_sides.get(side)
                    if seen is not None and side not in sides and abs(y - seen[0]) < 0.10:
                        if best is None or abs(y - seen[0]) < abs(y - self.lane_sides[best][0]):
                            best = side
                if best is None:
                    unplaced.append((y, angle))
                else:
                    sides[best] = (y, angle)
            for y, angle in unplaced:
                if 'left' in sides and 'right' not in sides and y < sides['left'][0] - 0.15:
                    side = 'right'
                elif 'right' in sides and 'left' not in sides and y > sides['right'][0] + 0.15:
                    side = 'left'
                elif not sides and not any(self.lane_sides.values()):
                    side = 'left' if y > 0.0 else 'right'
                else:
                    continue
                sides[side] = (y, angle)
            for side, (y, angle) in sides.items():
                self.lane_sides[side] = (y, travelled)
                # Where the line is counts whatever its angle; the angle
                # only when it is believable.
                if abs(angle) <= self.up_lane_max_angle:
                    self.lane_dirs.append((travelled, wrap(yaw + angle)))
            if 'left' in sides and 'right' in sides:
                middle = (sides['left'][0] + sides['right'][0]) / 2.0
            elif 'left' in sides:
                middle = sides['left'][0] - self.entrance_half_lane
            elif 'right' in sides:
                middle = sides['right'][0] + self.entrance_half_lane
        if middle is not None:
            self.lane_middles.append((travelled, middle))
        direction, _ = self.lane_direction(travelled)
        error = wrap(direction - yaw)
        if middle is None:
            # No line to go by: along the lane's direction, gently.
            twist.angular.z = max(-0.2, min(0.2, self.up_lane_heading_gain * error))
            return twist
        steer = self.up_lane_middle_gain * middle + self.up_lane_heading_gain * error
        twist.angular.z = max(-0.4, min(0.4, steer))
        return twist

    @staticmethod
    def line_angle(pts):
        """The angle of a line's points to the robot's heading.

        From the middle of its near half to the middle of its far half, over
        0.18..0.40 m: a least squares fit over the whole piece took the kink
        or the bent end of the tape with it and came out 20 deg off while
        the lateral position of the line did not move (2026-10-27).
        """
        near = pts[(pts[:, 0] < 0.29)]
        far = pts[(pts[:, 0] >= 0.29) & (pts[:, 0] < 0.40)]
        if len(near) >= 10 and len(far) >= 10:
            dx = float(np.median(far[:, 0]) - np.median(near[:, 0]))
            dy = float(np.median(far[:, 1]) - np.median(near[:, 1]))
            if dx > 0.05:
                return math.atan2(dy, dx)
        vx, vy, _, _ = cv2.fitLine(pts.astype(np.float32), cv2.DIST_HUBER, 0, 0.01, 0.01).ravel()
        if vx < 0.0:
            vx, vy = -vx, -vy
        return math.atan2(vy, vx)

    def lane_direction(self, travelled):
        """The lane's direction (world) by the lines of the last stretch, and
        how many pictures it rests on; the turn-in heading until there are
        up_lane_direction_min. The median of the offsets from that heading, so an
        occasional wild angle does not count."""
        recent = [wrap(d - self.up_lane_yaw) for t, d in self.lane_dirs
                  if travelled - t < self.up_lane_direction_memory]
        if len(recent) < self.up_lane_direction_min:
            # The first pictures after the turn in see the bend of the
            # line coming off the road (137 deg for a 90 deg lane, 2026-10-27).
            return self.up_lane_yaw, len(recent)
        # The lane cannot be far off where the turn in ended; a median that
        # says so is the bend again.
        offset = max(-0.35, min(0.35, float(np.median(recent))))
        return wrap(self.up_lane_yaw + offset), len(recent)

    def lane_middle(self, travelled):
        """The lane's middle left of the robot, by the lines of the last stretch.

        Not the very last pictures: at the lot the lines bend or break off,
        and the one picture before the lot put the middle 5 cm off from the
        stretch before (2026-10-27). The median of 0.05 to 0.30 m back.
        """
        recent = [m for d, m in self.lane_middles if 0.05 < travelled - d < 0.30]
        if len(recent) < 3:
            recent = [m for _, m in self.lane_middles][-5:]
        return float(np.median(recent)) if recent else 0.0

    def turn_in_twist(self):
        """Up to the entrance on the lane follower, then round to the left.

        Straight on the lane follower keeps the road (slowly), not a held
        heading: held, a heading a few degrees off at the gap took the robot
        off the lane by the time it turned (2026-10-27). None while the lane
        follower drives; the arc is counted from the heading it starts on.
        """
        x_0, y_0, yaw_0, straight_on = self.entrance
        _, x, y, yaw, _ = self.odometry[-1]
        along = math.cos(yaw_0) * (x - x_0) + math.sin(yaw_0) * (y - y_0)
        if self.arc_yaw is None:
            if along < straight_on:
                self.pub_limit.publish(Float64(data=self.entrance_speed))
                return None
            self.arc_yaw = yaw
            self.get_logger().info('Parking: at the entrance, round to the left.')
        turned = wrap(yaw - self.arc_yaw)
        twist = Twist()
        twist.linear.x = self.entrance_speed
        if turned >= math.pi / 2.0 - 0.05:
            self.get_logger().info('Parking: turned in, up the lane to the lot between its lines.')
            self.white_seen_at = self.odometry[-1][4]
            self.yellow_memory.clear()
            self.up_lane = True
            self.up_lane_yaw = yaw
            self.near_yellow_gone = 0
            self.lane_middles.clear()
            self.lane_dirs.clear()
            self.lane_sides = {'left': None, 'right': None}
            self.enter('turn_in')
            return Twist()
        twist.angular.z = self.entrance_speed / self.entrance_radius
        return twist

    def update(self):
        if self.phase is None or not self.odometry:
            return
        now = self.now()
        if now - self.active_time > self.active_timeout:
            self.get_logger().info(f'Parking: activation lapsed during {self.phase}.')
            self.phase = None
            return
        if self.phase == 'done':
            # Repeated until the manager takes the activation away.
            self.pub_done.publish(Bool(data=True))
            return
        if self.phase == 'to_entrance':
            # The lane follower keeps to the road; this node only watches.
            self.pub_limit.publish(Float64(data=self.approach_speed))
            if self.odometry[-1][4] - self.phase_travelled > self.entrance_limit:
                self.get_logger().warn('Parking: no entrance found; keeping left for the lot.')
                self.white_seen_at = self.odometry[-1][4]
                self.enter('turn_in')
            return
        if self.phase == 'entrance':
            twist = self.turn_in_twist()
            if twist is not None:
                self.pub_cmd_vel.publish(twist)
            return
        if self.phase == 'turn_in':
            if self.up_lane:
                self.pub_cmd_vel.publish(self.up_lane_twist())
                return
            self.pub_follow.publish(String(data='left'))
            self.pub_limit.publish(Float64(data=self.approach_speed))
            return

        along, across, heading = self.in_lot()
        travelled = self.odometry[-1][4] - self.phase_travelled
        twist = Twist()

        if self.phase == 'aisle':
            swing_at = self.bay_station - self.swing_radius
            twist = self.drive(heading, 0.0, self.in_aisle(across), 1.0)
            if along >= swing_at:
                left, right = self.seen['left'], self.seen['right']
                if left == right:
                    self.get_logger().warn(
                        f'Parking: nothing made out in either bay, taking the {self.default_bay}.')
                    self.side = 1.0 if self.default_bay == 'left' else -1.0
                else:
                    self.side = -1.0 if left > right else 1.0
                self.get_logger().info(
                    'Parking: laser returns left %d, right %d; into the %s bay.'
                    % (left, right, 'left' if self.side > 0.0 else 'right'))
                self.enter('swing_in')

        elif self.phase == 'swing_in':
            twist.linear.x = self.speed
            twist.angular.z = self.side * self.speed / self.swing_radius
            if self.side * heading >= math.pi / 2.0 - 0.05:
                self.enter('into_bay')

        elif self.phase == 'into_bay':
            # Along the line across the aisle at the bay station; to the left
            # of that line is back down the aisle for the left bay.
            off_line = -self.side * (along - self.bay_station)
            twist = self.drive(heading, self.side * math.pi / 2.0, off_line,
                               self.bay_depth - self.side * across)
            if self.side * across >= self.bay_depth:
                self.get_logger().info('Parking: in the bay.')
                self.enter('parked')

        elif self.phase == 'parked':
            if now - self.phase_start > self.park_time:
                self.enter('back_out')

        elif self.phase == 'back_out':
            off_line = -self.side * (along - self.bay_station)
            twist = self.drive(heading, self.side * math.pi / 2.0, off_line,
                               self.side * across - self.turn_offset, reverse=True)
            if self.side * across <= self.turn_offset:
                self.enter('face_out')

        elif self.phase == 'face_out':
            twist, there = self.turn_to(heading, math.pi)
            if there:
                self.enter('leave')

        elif self.phase == 'leave':
            off_line = self.in_aisle(-across)
            self.get_logger().info(
                'Parking: leaving, %+.3f m left of the middle (odometry %+.3f).'
                % (off_line, -across), throttle_duration_sec=1.0)
            twist = self.drive(heading, math.pi, off_line, 1.0)
            if along <= self.leave_station:
                self.get_logger().info('Parking: out of the lot, keeping left for the road.')
                self.enter('exit')

        if self.phase == 'exit':
            self.pub_follow.publish(String(data='left'))
            self.pub_limit.publish(Float64(data=self.approach_speed))
            # The road runs a quarter turn to the right of the lane up to
            # the lot.
            on_the_road = abs(wrap(heading + math.pi / 2.0)) < 0.35 and travelled > 0.4
            if on_the_road or travelled > self.exit_limit:
                self.get_logger().info('Parking: back on the road, done.')
                self.enter('done')
            return

        self.pub_cmd_vel.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ParkingMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
