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

"""Construction mission: drive round the obstacles standing in the road.

In the construction zone the road is two lanes wide and blocks stand in it,
each one closing a lane, on alternating sides. The lane follower knows nothing
of them: it keeps to one line and drives into the first block that stands on
it. Between a block and the line opposite there is a gap some 7 cm wider than
the robot, so the way through has to be found, not felt for.

The laser sees the blocks and the lane camera sees the lines, and neither
sees enough alone: the lines are no obstacle to the laser, and the camera
looks at the 30 cm of road in front of the robot. Both are therefore gathered
into one picture of the road as the robot moves, by odometry:

* laser returns are counted in a grid, all the time, so that the obstacles
  are already known when the mission begins. A cell hit again and again is
  an obstacle, one hit now and then is the noise of a return next to it. The
  counts fade, so that what odometry has since moved from under an obstacle
  does not stand in the road for ever,
* the lines the lane detector publishes are kept for the last stretch of road,
  and a straight edge is laid along each. The road here is straight, so an
  edge seen once still bounds the road further on.

That picture is turned to lie along the road and a path is laid through it,
column by column away from the robot: as far from the obstacles and the edges
as the gap allows, and no more sideways than it has to be. The path is laid
afresh from where the robot is every tenth of a second, and the robot steers
for a point a little way along it.

    approach  the lane follower drives until an obstacle stands in the way
    avoid     this node drives, until the road ahead is clear again; with the
              obstacles behind it steers for the lane detector's point, which
              allows for the road bending away at the end of the zone

Nothing but the obstacles starts the mission: when one stands in the robot's
way while it is lane following, this node asks the mission manager for the
mission, and takes over as soon as it is given it.

The path is laid along a straight road, so the robot has to be on the straight
when it takes over. Where the first obstacle stands right at the end of a
bend, on the line the lane follower keeps to, it is met before the road is
straight and the robot drives into it; the lane follower has to come round
the bend on the other line there (`lane.fork_side` of the lane detector).
"""

from array import array
from collections import deque
import math

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool
from std_msgs.msg import String


# The grid the laser returns are counted in: fixed to the odometry frame, and
# laid out afresh round the robot when it comes this near the edge.
HIT_CELL = 0.01
HIT_SIZE = 800
HIT_MARGIN = 1.5

# The picture the path is laid in: along the road (s) from behind the robot
# to as far ahead as the laser is trusted, and across it (d, to the left).
RES = 0.01
S_MIN, S_MAX = -0.30, 1.00
D_MAX = 0.70
ROWS = int(round(2 * D_MAX / RES)) + 1
COLS = int(round((S_MAX - S_MIN) / RES)) + 1
ROW_0 = ROWS // 2
COL_0 = int(round(-S_MIN / RES))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def fit_direction(points, guess, reach):
    """Which way a line runs, from points on it: (angle, scatter, length) or None.

    The points may have a bend at one end, where the road turned into the
    straight, so the direction is first searched for, within `reach` of the
    guess: the one in which the bulk of the points lie in the narrowest band.
    Those in the band are then fitted.
    """
    best = None
    for angle in guess + np.linspace(-reach, reach, 2 * int(reach / 0.0175) + 1):
        across = points @ np.array([-math.sin(angle), math.cos(angle)])
        low, middle, high = np.percentile(across, (20, 50, 80))
        if best is None or high - low < best[0]:
            best = (high - low, angle, middle)
    _, angle, middle = best
    across = points @ np.array([-math.sin(angle), math.cos(angle)])
    band = points[np.abs(across - middle) < 0.03]
    if len(band) < 40:
        return None
    centred = band - band.mean(axis=0)
    _, vectors = np.linalg.eigh(centred.T @ centred)
    along = vectors[:, 1]
    if along @ np.array([math.cos(angle), math.sin(angle)]) < 0.0:
        along = -along
    fitted = math.atan2(along[1], along[0])
    scatter = float(np.std(centred @ np.array([-along[1], along[0]])))
    reach = centred @ along
    return fitted, scatter, float(np.percentile(reach, 97) - np.percentile(reach, 3))


def line_edge(across, towards_road):
    """Where the edge of a line facing the road is, from its points' offsets.

    `across` is how far to the left each point lies and `towards_road` is +1
    when the road is to the left of the line, -1 when to the right. The line
    is where most of the points are; a bend at the far end smears a few
    points out to the side and they do not move it. None with too few points,
    or with points that are not in a line along the road at all, such as the
    line of a road joining from the side.
    """
    if len(across) < 80:
        return None
    step = 0.005
    edges = np.arange(across.min() - step, across.max() + 2 * step, step)
    counts, _ = np.histogram(across, edges)
    counts = np.convolve(counts, (1, 2, 1), mode='same')
    peak = int(np.argmax(counts))
    middle = (edges[peak] + edges[peak + 1]) / 2.0
    if np.count_nonzero(np.abs(across - middle) < 0.035) < 0.4 * len(across):
        return None
    index = peak
    while 0 <= index + towards_road < len(counts) \
            and counts[index + towards_road] >= 0.3 * counts[peak]:
        index += towards_road
    return float(edges[index + 1] if towards_road > 0 else edges[index])


def lay_path(cost, stride=2, reach=3, sideways_weight=0.3):
    """Cheapest way through `cost` from the robot's cell to the far column.

    Returns the row of the path in every `stride`-th column from COL_0 on.
    Each step moves one such column on and at most `reach` rows sideways.
    """
    columns = cost[:, COL_0::stride]
    total = np.full(ROWS, np.inf)
    total[ROW_0] = 0.0
    steps = []
    for column in range(1, columns.shape[1]):
        best = np.full(ROWS, np.inf)
        came = np.zeros(ROWS, np.int8)
        for step in range(-reach, reach + 1):
            shifted = np.full(ROWS, np.inf)
            if step > 0:
                shifted[step:] = total[:-step]
            elif step < 0:
                shifted[:step] = total[-step:]
            else:
                shifted = total
            candidate = shifted + sideways_weight * step * step
            better = candidate < best
            best[better] = candidate[better]
            came[better] = step
        total = best + columns[:, column]
        steps.append(came)
    row = int(np.argmin(total))
    rows = [row]
    for came in reversed(steps):
        row -= int(came[row])
        rows.append(row)
    return np.array(rows[::-1])


class ConstructionMission(Node):

    def __init__(self):
        super().__init__('construction_mission')

        # An obstacle this near, in the strip the robot is about to drive
        # through, is what the mission takes over for.
        self.declare_parameter('takeover_distance', 0.45)
        self.declare_parameter('takeover_half_width', 0.13)
        # The lane follower steering an arc tighter than this (1/m) is in a
        # bend, and there the mission takes over only this near an obstacle.
        self.declare_parameter('bend_curvature', 2.0)
        self.declare_parameter('bend_takeover_distance', 0.20)
        # Laser returns in the strip, two scans running, that make an
        # obstacle: a block gives 13-20 on the real course, the robot's own
        # front or a wall beside a tight bend 3-6 (2026-10-27).
        self.declare_parameter('takeover_points', 3)
        # Asking for the mission from the laser alone also goes by a wider
        # look: returns arm_min_ahead..arm_max_ahead ahead within
        # arm_half_width of the robot's axis (the road's width), arm_points
        # or more for arm_scans scans. That only arms the mission (the lane
        # follower drives on); once armed, armed_points in the strip take
        # over. With one strip test for both, the first block came in from
        # the inside of the bend with 5-7 returns and the mission took over
        # 0.29 m from it, too late to get round (2026-10-27 18:29).
        self.declare_parameter('arm_min_ahead', 0.20)
        self.declare_parameter('arm_max_ahead', 0.60)
        self.declare_parameter('arm_half_width', 0.30)
        self.declare_parameter('arm_points', 3)
        self.declare_parameter('arm_scans', 3)
        self.declare_parameter('armed_points', 3)
        # Laser returns further off than this are left out: too coarse.
        self.declare_parameter('scan_range', 1.0)
        # What is left of a cell's count of returns one scan later.
        self.declare_parameter('hit_fade', 0.93)
        # The laser sits this far ahead of the axle (it is behind it).
        self.declare_parameter('lidar_offset', -0.032)
        # Room the axle needs from an obstacle or an edge to get by, and the
        # room it takes where there is room to spare.
        self.declare_parameter('robot_radius', 0.095)
        self.declare_parameter('comfort_clearance', 0.20)
        self.declare_parameter('crowding_weight', 10.0)
        # A line may be come this much nearer to than an obstacle.
        self.declare_parameter('line_slack', 0.01)
        # Lines are remembered over this much road.
        self.declare_parameter('line_memory', 1.5)
        # While only one edge of the road has been seen, the other is taken
        # to be no further from it than this.
        self.declare_parameter('max_road_width', 0.60)
        self.declare_parameter('speed', 0.10)
        self.declare_parameter('min_speed', 0.03)
        self.declare_parameter('cornering_rate', 0.8)
        self.declare_parameter('max_angular', 1.2)
        self.declare_parameter('lookahead', 0.11)
        # The road is clear when nothing stands in it this far ahead, and
        # the last obstacle is this far behind the axle.
        self.declare_parameter('clear_distance', 0.60)
        self.declare_parameter('rear_clearance', 0.13)
        # Standing cells in the road count as an obstacle only in a cluster
        # of at least this many (after joining neighbours).
        self.declare_parameter('cluster_cells', 12)
        # Nothing further than this from the middle of the road counts as an
        # obstacle, whatever the lines say: a sign 0.6 m left of the road was
        # taken for the third block and the mission ended with the real one
        # still ahead, which the robot then drove into (2026-10-27, twice).
        self.declare_parameter('max_obstacle_across', 0.35)
        # After the last obstacle, driven at most this far looking for a lane.
        self.declare_parameter('leave_distance', 0.4)
        # Without an obstacle after this much road the mission gives up.
        self.declare_parameter('approach_limit', 4.0)
        # The zone holds this many obstacles: the mission is over only once
        # they are all behind, steering by the lane between them (on the real
        # course the next one stood beyond clear_distance, and the mission
        # handed back after the first). 0 counts none.
        self.declare_parameter('obstacle_count', 3)
        # With fewer behind, the mission still ends after this much clear road.
        self.declare_parameter('next_obstacle_distance', 1.0)
        # The zone ends in a bend: past the last obstacle the outer white line
        # shows across the way, and the straight-road path, or the lane point
        # off that line, drove out of the bend (2026-10-10). A white line this
        # near, at least this far off square to the robot's way, ends the
        # mission once an obstacle has been passed: the lane follower takes
        # the bend.
        # The robot is not turned further off the road than this (rad) while
        # it gets round an obstacle: at the second obstacle on 2026-10-10 it
        # stood square across the road (88 deg) to step aside, then swung its
        # front back into the obstacle turning straight again. At the limit it
        # goes on slantwise.
        self.declare_parameter('max_road_angle', 0.785)
        # The last obstacle counts as passed once its centre is this far behind
        # the axle (0: level with it), and the lane follower takes over at
        # once, while the robot still faces down the road: waiting until it
        # was rear_clearance behind left the robot turned off the road with no
        # line in view (2026-10-10).
        self.declare_parameter('last_obstacle_release', 0.0)
        self.declare_parameter('white_across_reach', 0.45)
        self.declare_parameter('white_across_angle', 1.05)
        # The manager repeats the activation; without it the mission is over.
        self.declare_parameter('active_timeout', 1.0)
        self.declare_parameter('publish_debug_image', True)

        def value(name):
            return self.get_parameter(name).value

        self.takeover_distance = value('takeover_distance')
        self.takeover_half_width = value('takeover_half_width')
        self.bend_curvature = value('bend_curvature')
        self.bend_takeover_distance = value('bend_takeover_distance')
        self.takeover_points = value('takeover_points')
        self.arm_min_ahead = value('arm_min_ahead')
        self.arm_max_ahead = value('arm_max_ahead')
        self.arm_half_width = value('arm_half_width')
        self.arm_points = value('arm_points')
        self.arm_scans = value('arm_scans')
        self.armed_points = value('armed_points')
        self.arm_count = 0
        self.scan_range = value('scan_range')
        self.hit_fade = value('hit_fade')
        self.lidar_offset = value('lidar_offset')
        self.robot_radius = value('robot_radius')
        self.comfort_clearance = value('comfort_clearance')
        self.crowding_weight = value('crowding_weight')
        self.line_slack = value('line_slack')
        self.line_memory = value('line_memory')
        self.max_road_width = value('max_road_width')
        self.speed = value('speed')
        self.min_speed = value('min_speed')
        self.cornering_rate = value('cornering_rate')
        self.max_angular = value('max_angular')
        self.lookahead = value('lookahead')
        self.clear_distance = value('clear_distance')
        self.rear_clearance = value('rear_clearance')
        self.cluster_cells = value('cluster_cells')
        self.max_obstacle_across = value('max_obstacle_across')
        self.leave_distance = value('leave_distance')
        self.approach_limit = value('approach_limit')
        self.obstacle_count = value('obstacle_count')
        self.next_obstacle_distance = value('next_obstacle_distance')
        self.max_road_angle = value('max_road_angle')
        self.last_obstacle_release = value('last_obstacle_release')
        self.white_across_reach = value('white_across_reach')
        self.white_across_angle = value('white_across_angle')
        self.active_timeout = value('active_timeout')
        self.publish_debug = value('publish_debug_image')

        self.phase = None           # None while the mission is not running
        self.active_time = 0.0
        # Odometry of the last few seconds, to find where the robot was when
        # a scan or a picture was taken: (time, x, y, yaw, distance travelled).
        self.odometry = deque(maxlen=400)
        self.odom_frame = 'odom'
        self.hits = np.zeros((HIT_SIZE, HIT_SIZE), np.float32)  # laser returns per cell
        self.hits_origin = None     # odometry coordinates of its corner
        self.shadow = None          # nearest return per bearing, latest scan
        self.in_the_way = 0         # scans in a row with an obstacle ahead
        # Lines seen: (distance travelled, yaw, points in the odometry frame).
        self.lines = {'yellow': deque(), 'white': deque()}
        self.direction = 0.0        # of the road, in the odometry frame
        self.road_found = False     # until then the direction is the robot's heading
        # The edges of the road: how far to the left of the odometry origin,
        # and of the robot.
        self.edges = {'yellow': None, 'white': None}
        self.walls = {'yellow': None, 'white': None}
        self.started_at = 0.0       # distance travelled at activation
        self.takeover_at = 0.0      # and when the mission began to drive
        self.cleared_at = None      # and when the road was first clear
        self.clear_count = 0
        self.lane_target = None     # where the lane follower is steering for
        self.lane_time = 0.0
        self.met = False            # an obstacle has been seen standing in the road
        self.by_the_lane = False    # the obstacles passed, steering for the lane's point
        # Obstacles met in the road: [x, y (odometry), cells, passed].
        self.tracked = []
        self.white_now = (0.0, None)  # (time, points of the latest picture)

        self.bridge = CvBridge()

        self.create_subscription(
            Bool, '/autorace/mission/construction/active', self.callback_active, 1)
        self.create_subscription(Odometry, '/odom', self.callback_odom, 1)
        self.create_subscription(LaserScan, '/scan', self.callback_scan, 1)
        for colour in self.lines:
            self.create_subscription(
                PointCloud2, f'/detect/lane_lines/{colour}',
                lambda msg, key=colour: self.callback_lines(msg, key), 1)
        self.create_subscription(PointStamped, '/detect/lane_target', self.callback_lane, 1)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel/mission', 1)
        self.pub_done = self.create_publisher(
            Bool, '/autorace/mission/construction/done', 1)
        self.pub_trigger = self.create_publisher(String, '/autorace/trigger', 5)
        self.pub_image = self.create_publisher(Image, '/detect/image_construction', 1)
        self.pub_map = self.create_publisher(OccupancyGrid, '/detect/construction_map', 1)

        self.create_timer(0.1, self.update)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    # --- callbacks -------------------------------------------------------

    def callback_active(self, msg):
        if msg.data:
            self.active_time = self.now()
            if self.phase is None and self.odometry:
                self.get_logger().info('Construction: watching for obstacles.')
                self.phase = 'approach'
                self.started_at = self.odometry[-1][4]
                self.cleared_at = None
                self.clear_count = 0
                self.by_the_lane = False
                self.road_found = False
                self.edges = {'yellow': None, 'white': None}
                self.walls = {'yellow': None, 'white': None}
        elif self.phase is not None:
            self.stop('deactivated')

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
        self.odom_frame = msg.header.frame_id

    def pose_at(self, stamp):
        """Return (x, y, yaw, travelled) of the robot at a message stamp."""
        if not self.odometry:
            return None
        when = stamp.sec + stamp.nanosec * 1e-9
        return min(self.odometry, key=lambda entry: abs(entry[0] - when))[1:]

    def callback_lane(self, msg):
        self.lane_target = (msg.point.x, msg.point.y)
        self.lane_time = self.now()

    def callback_scan(self, msg):
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        seen = np.isfinite(ranges) & (ranges > msg.range_min + 1e-3)
        ranges = np.where(seen, ranges, 0.0)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        # What lies beyond a return is hidden from the lane camera too, a
        # little to either side.
        nearest = np.where(seen, ranges, np.inf)
        self.shadow = (
            np.minimum.reduce([np.roll(nearest, shift) for shift in range(-2, 3)]),
            msg.angle_min, msg.angle_increment)

        # A return wanders by a centimetre from scan to scan; its neighbours
        # on the same surface wander on their own, and together they steady it.
        total = np.zeros_like(ranges)
        count = np.zeros_like(ranges)
        for shift in range(-2, 3):
            other = np.roll(ranges, shift)
            same = np.roll(seen, shift) & (np.abs(other - ranges) < 0.04)
            total += np.where(same, other, 0.0)
            count += same
        ranges = total / np.maximum(count, 1.0)

        near = seen & (ranges < self.scan_range)
        points = np.stack([self.lidar_offset + ranges[near] * np.cos(angles[near]),
                           ranges[near] * np.sin(angles[near])], axis=1)

        ahead = self.in_the_way_of(points)
        needed = self.armed_points if self.phase == 'approach' else self.takeover_points
        self.in_the_way = self.in_the_way + 1 if np.count_nonzero(ahead) >= needed else 0

        if self.phase is None:
            wide = ((points[:, 0] > self.arm_min_ahead) & (points[:, 0] < self.arm_max_ahead)
                    & (np.abs(points[:, 1]) < self.arm_half_width))
            self.arm_count = self.arm_count + 1 if np.count_nonzero(wide) >= self.arm_points else 0
            if self.in_the_way >= 2 or self.arm_count >= self.arm_scans:
                # The manager takes this only while lane following, and once.
                self.pub_trigger.publish(String(data='construction'))

        pose = self.pose_at(msg.header.stamp)
        if pose is None:
            return
        size = HIT_SIZE * HIT_CELL
        if self.hits_origin is None or not (
                HIT_MARGIN < pose[0] - self.hits_origin[0] < size - HIT_MARGIN
                and HIT_MARGIN < pose[1] - self.hits_origin[1] < size - HIT_MARGIN):
            self.hits_origin = (pose[0] - size / 2.0, pose[1] - size / 2.0)
            self.hits[:] = 0.0
        world = self.to_odom(points, pose)
        column = np.int_((world[:, 0] - self.hits_origin[0]) / HIT_CELL)
        row = np.int_((world[:, 1] - self.hits_origin[1]) / HIT_CELL)
        inside = (column >= 0) & (column < HIT_SIZE) & (row >= 0) & (row < HIT_SIZE)
        self.hits *= self.hit_fade
        np.add.at(self.hits, (row[inside], column[inside]), 1.0)
        if self.publish_debug:
            self.publish_map(msg.header.stamp)

    def in_the_way_of(self, points):
        """Which laser returns stand where the robot is about to drive.

        That is the strip ahead of it. In a bend a straight strip sweeps across
        whatever stands beyond the bend, to either side of the road, and misses
        an obstacle coming in from the inside of it (met 6 cm off on the real
        course), so while the lane follower is steering round one the strip
        bends with the arc it drives, and reaches less far along it.
        """
        ahead, left = points[:, 0], points[:, 1]
        if self.lane_target is not None and self.now() - self.lane_time < 0.5:
            target_ahead, target_left = self.lane_target
            curvature = 2.0 * target_left / (target_ahead ** 2 + target_left ** 2)
            if abs(curvature) > self.bend_curvature:
                radius = 1.0 / curvature           # + to the left
                off_arc = np.abs(np.hypot(ahead, left - radius) - abs(radius))
                angle = np.arctan2(ahead, np.sign(radius) * (radius - left))
                along_arc = abs(radius) * angle
                return ((angle > 0.0) & (along_arc < self.bend_takeover_distance)
                        & (off_arc < self.takeover_half_width))
        return ((ahead > 0.0) & (ahead < self.takeover_distance)
                & (np.abs(left) < self.takeover_half_width))

    def callback_lines(self, msg, colour):
        points = point_cloud2.read_points_numpy(msg, field_names=('x', 'y'))
        if colour == 'white':
            self.white_now = (self.now(), points.astype(np.float64))
        pose = self.pose_at(msg.header.stamp)
        if pose is None or len(points) == 0:
            return
        x, y, yaw, travelled = pose
        memory = self.lines[colour]
        if memory and travelled - memory[-1][0] < 0.015 and abs(wrap(yaw - memory[-1][1])) < 0.05:
            return
        points = self.in_view(points.astype(np.float64))[::4]
        if len(points) == 0:
            return
        memory.append((travelled, yaw, self.to_odom(points, pose)))
        while travelled - memory[0][0] > self.line_memory:
            memory.popleft()

    # --- the road --------------------------------------------------------

    @staticmethod
    def to_odom(points, pose):
        x, y, yaw = pose[:3]
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        return np.stack([x + cos_y * points[:, 0] - sin_y * points[:, 1],
                         y + sin_y * points[:, 0] + cos_y * points[:, 1]], axis=1)

    def in_view(self, points):
        """Line points the camera can really have seen on the road.

        An obstacle in the picture is laid flat by the bird's eye projection,
        out along the road behind it, and its pale sides can pass for a line
        there. Nothing behind a laser return is road.
        """
        if self.shadow is None:
            return points
        nearest, angle_min, increment = self.shadow
        ahead = points[:, 0] - self.lidar_offset
        bearing = np.arctan2(points[:, 1], ahead)
        index = np.int_(np.round((bearing - angle_min) / increment)) % len(nearest)
        return points[np.hypot(ahead, points[:, 1]) < nearest[index] - 0.03]

    def road_coordinates(self, points, x, y):
        """Along the road from the robot, and across it to the left."""
        cos_d, sin_d = math.cos(self.direction), math.sin(self.direction)
        dx, dy = points[:, 0] - x, points[:, 1] - y
        return cos_d * dx + sin_d * dy, -sin_d * dx + cos_d * dy

    def find_road(self, x, y):
        """Bring the direction of the road and its two edges up to date."""
        lines = {colour: np.concatenate([entry[2] for entry in memory])
                 for colour, memory in self.lines.items() if memory}

        # The direction from the longer of the lines near the robot. Until
        # one has been found there is only the robot's heading to go by, and
        # the robot may be coming out of a bend: the search is wide then, and
        # what lies ahead of the robot will do if the stretch round it is not
        # straight. Later a line that is not straight is the road bending
        # away at the end of the zone, and is not to be followed.
        best = None
        reach = 0.35 if self.road_found else 0.9
        for points in lines.values():
            along, _ = self.road_coordinates(points, x, y)
            for behind in (-0.25,) if self.road_found else (-0.25, 0.0):
                near = points[(along > behind) & (along < 0.45)]
                if len(near) < 80:
                    continue
                fit = fit_direction(near[::max(1, len(near) // 1500)], self.direction, reach)
                if fit is None:
                    continue
                angle, scatter, length = fit
                if scatter < 0.016 and length > 0.15 - behind / 5.0:
                    if best is None or length > best[1]:
                        best = (angle, length)
                    break
        if best is not None:
            change = wrap(best[0] - self.direction)
            if self.road_found:
                change = max(-0.05, min(0.05, 0.5 * change))
            else:
                self.get_logger().info('Construction: direction of the road found.')
                self.road_found = True
            self.direction = wrap(self.direction + change)
        if not self.road_found:
            # No telling yet which way an edge runs on from where it was seen.
            return

        # The edges are kept as offsets from the odometry origin rather than
        # from the robot, so that one out of sight stays where it was.
        robot = -math.sin(self.direction) * x + math.cos(self.direction) * y
        for colour, towards_road in (('yellow', -1), ('white', 1)):
            if colour not in lines:
                continue
            along, across = self.road_coordinates(lines[colour], x, y)
            edge = line_edge(across[(along > -0.35) & (along < 0.6)], towards_road)
            if edge is None:
                continue
            if self.edges[colour] is None:
                self.get_logger().info(f'Construction: {colour} edge of the road found.')
            self.edges[colour] = robot + edge
        yellow, white = self.edges['yellow'], self.edges['white']
        if yellow is not None and white is not None and yellow - white < 0.2:
            # Not a road between them: one is a stray. Keep the one with the
            # robot on the right side of it.
            if yellow < robot:
                self.edges['yellow'] = None
            else:
                self.edges['white'] = None
        yellow, white = self.edges['yellow'], self.edges['white']
        if yellow is None and white is not None:
            yellow = white + self.max_road_width
        elif white is None and yellow is not None:
            white = yellow - self.max_road_width
        self.walls = {'yellow': None if yellow is None else yellow - robot,
                      'white': None if white is None else white - robot}

    @staticmethod
    def standing(hits):
        """Which cells of the grid of laser returns hold an obstacle.

        Next to a surface hit every scan are cells hit by its strays; they
        are told apart by how seldom.
        """
        peak = cv2.dilate(hits, np.ones((7, 7), np.uint8))
        return (hits >= 2.0) & (hits * 4.0 >= peak)

    def publish_map(self, stamp):
        """Publish the grid of laser returns for rviz: obstacles dark, strays pale."""
        cells = np.where(self.hits >= 0.5, 40, 0).astype(np.int8)
        cells[self.standing(self.hits)] = 100
        grid = OccupancyGrid()
        grid.header.stamp = stamp
        grid.header.frame_id = self.odom_frame
        grid.info.resolution = HIT_CELL
        grid.info.width = HIT_SIZE
        grid.info.height = HIT_SIZE
        grid.info.origin.position.x = self.hits_origin[0]
        grid.info.origin.position.y = self.hits_origin[1]
        grid.info.origin.orientation.w = 1.0
        grid.data = array('b', cells.tobytes())
        self.pub_map.publish(grid)

    def obstacles(self, x, y):
        """Cells round the robot that the laser keeps hitting, in road coordinates."""
        column = int((x - self.hits_origin[0]) / HIT_CELL)
        row = int((y - self.hits_origin[1]) / HIT_CELL)
        reach = int(1.5 / HIT_CELL)
        row_0, column_0 = max(0, row - reach), max(0, column - reach)
        window = self.hits[row_0:row + reach + 1, column_0:column + reach + 1]
        if window.size == 0:
            return np.zeros(0), np.zeros(0)
        rows, columns = self.standing(window).nonzero()
        cells = np.stack([self.hits_origin[0] + (columns + column_0 + 0.5) * HIT_CELL,
                          self.hits_origin[1] + (rows + row_0 + 0.5) * HIT_CELL], axis=1)
        return self.road_coordinates(cells, x, y)

    def road_cost(self, along, across):
        """Return what it costs the axle to be in each cell of the road ahead."""
        blocked = np.zeros((ROWS, COLS), np.uint8)
        column = np.int_(np.round((along - S_MIN) / RES))
        row = np.int_(np.round(across / RES)) + ROW_0
        inside = (column >= 0) & (column < COLS) & (row >= 0) & (row < ROWS)
        blocked[row[inside], column[inside]] = 1
        if blocked.any():
            clearance = cv2.distanceTransform(1 - blocked, cv2.DIST_L2, 5) * RES
        else:
            clearance = np.full((ROWS, COLS), 10.0, np.float32)

        across_axis = (np.arange(ROWS) - ROW_0) * RES
        edge = np.full(ROWS, 10.0)
        if self.walls['yellow'] is not None:
            edge = np.minimum(edge, self.walls['yellow'] - across_axis)
        if self.walls['white'] is not None:
            edge = np.minimum(edge, across_axis - self.walls['white'])
        edge = edge[:, None]

        radius, comfort = self.robot_radius, self.comfort_clearance
        # Keeping its distance from whatever is nearest puts the path in the
        # middle of a gap; a line counts as a little further off than it is,
        # which puts the path that much nearer the line than the obstacle.
        # Closer than the robot is wide costs far more, and an obstacle more
        # than a line: a line can be driven on if it must.
        nearest = np.minimum(clearance, edge + self.line_slack)
        crowding = np.clip((comfort - nearest) / (comfort - radius), 0.0, 1.0)
        cost = self.crowding_weight * crowding ** 2
        cost += np.where(clearance < radius, 100.0 * (1.0 + (radius - clearance) / 0.01), 0.0)
        cost += np.where(edge < radius - self.line_slack,
                         30.0 * (1.0 + (radius - self.line_slack - edge) / 0.01), 0.0)
        return cost, blocked

    def track_obstacles(self, along, across, x, y):
        """Keep each obstacle met in the road; return how many are behind the axle.

        The standing cells in the road are joined into clusters, as in
        road_is_clear, and each cluster's centre is kept in the odometry frame:
        a centre within 0.15 m of a kept one is the same obstacle. One is
        passed once its centre is rear_clearance behind the axle.
        """
        left, right = self.walls['yellow'], self.walls['white']
        if left is None:
            left, right = 0.35, -0.35
        left = min(left, self.max_obstacle_across)
        right = max(right, -self.max_obstacle_across)
        near = ((along > -0.5) & (along < self.clear_distance)
                & (across > right + 0.01) & (across < left - 0.01))
        if np.count_nonzero(near) >= 3:
            picture = np.zeros((int(1.4 / 0.02) + 2, int((self.clear_distance + 0.5) / 0.02) + 2),
                               np.uint8)
            column = np.int_((along[near] + 0.5) / 0.02)
            row = np.int_((across[near] + 0.7) / 0.02)
            inside = ((row >= 0) & (row < picture.shape[0])
                      & (column >= 0) & (column < picture.shape[1]))
            picture[row[inside], column[inside]] = 1
            picture = cv2.dilate(picture, np.ones((3, 3), np.uint8))
            count, _, stats, centres = cv2.connectedComponentsWithStats(picture)
            cos_d, sin_d = math.cos(self.direction), math.sin(self.direction)
            for k in range(1, count):
                if stats[k, cv2.CC_STAT_AREA] < self.cluster_cells:
                    continue
                c_along = centres[k][0] * 0.02 - 0.5
                c_across = centres[k][1] * 0.02 - 0.7
                cx = x + cos_d * c_along - sin_d * c_across
                cy = y + sin_d * c_along + cos_d * c_across
                for obstacle in self.tracked:
                    if math.hypot(obstacle[0] - cx, obstacle[1] - cy) < 0.15:
                        if not obstacle[3]:
                            # Follow the centre as more of the obstacle is seen.
                            obstacle[0], obstacle[1] = cx, cy
                        break
                else:
                    # Only what first shows up ahead of the axle is an obstacle
                    # to pass: at takeover in a bend, cells beside and behind
                    # the robot were counted as passed at once (2026-10-10).
                    if c_along <= 0.0:
                        continue
                    self.tracked.append([cx, cy, int(stats[k, cv2.CC_STAT_AREA]), False])
                    self.get_logger().info(
                        f'Construction: obstacle {len(self.tracked)} in the road, '
                        f'{c_along:.2f} m ahead, {c_across:+.2f} m across.')
        passed = sum(1 for obstacle in self.tracked if obstacle[3])
        for obstacle in self.tracked:
            if not obstacle[3]:
                o_along, _ = self.road_coordinates(
                    np.array([[obstacle[0], obstacle[1]]]), x, y)
                last = self.obstacle_count > 0 and passed >= self.obstacle_count - 1
                release = self.last_obstacle_release if last else self.rear_clearance
                if o_along[0] < -release:
                    passed += 1
                    obstacle[3] = True
                    self.get_logger().info(
                        f'Construction: passed obstacle {self.tracked.index(obstacle) + 1}.')
        return sum(1 for obstacle in self.tracked if obstacle[3])

    def white_across(self, now):
        """Whether the latest picture shows a white line across the way ahead."""
        when, points = self.white_now
        if points is None or now - when > 0.3:
            return False
        near = points[(points[:, 0] > 0.0) & (points[:, 0] < self.white_across_reach)]
        if len(near) < 30 or np.ptp(near[:, 1]) < 0.15:
            return False
        vx, vy, _, _ = cv2.fitLine(near.astype(np.float32), cv2.DIST_HUBER, 0, 0.01, 0.01).ravel()
        # Angle of the line off the robot's way: 0 along it, pi/2 square across.
        off_way = math.atan2(abs(vy), abs(vx))
        return off_way > self.white_across_angle

    def road_is_clear(self, along, across):
        """Whether no obstacle stands in the road beside or ahead of the robot."""
        left, right = self.walls['yellow'], self.walls['white']
        if left is None:
            left, right = 0.35, -0.35
        left = min(left, self.max_obstacle_across)
        right = max(right, -self.max_obstacle_across)
        standing = ((along > -self.rear_clearance) & (along < self.clear_distance)
                    & (across > right + 0.01) & (across < left - 0.01))
        if np.count_nonzero(standing) < 3:
            return True
        # A few cells scattered about are strays; an obstacle is a cluster.
        length = self.clear_distance + self.rear_clearance
        picture = np.zeros((int(1.4 / 0.02) + 2, int(length / 0.02) + 2), np.uint8)
        column = np.int_((along[standing] + self.rear_clearance) / 0.02)
        row = np.int_((across[standing] + 0.7) / 0.02)
        inside = ((row >= 0) & (row < picture.shape[0])
                  & (column >= 0) & (column < picture.shape[1]))
        picture[row[inside], column[inside]] = 1
        picture = cv2.dilate(picture, np.ones((3, 3), np.uint8))
        _, _, stats, _ = cv2.connectedComponentsWithStats(picture)
        return not (stats[1:, cv2.CC_STAT_AREA] >= self.cluster_cells).any()

    # --- driving ---------------------------------------------------------

    def stop(self, reason):
        self.get_logger().info(f'Construction: {reason} during {self.phase}.')
        self.phase = None

    def update(self):
        if self.phase is None or not self.odometry or self.hits_origin is None:
            return
        now = self.now()
        if now - self.active_time > self.active_timeout:
            self.stop('activation lapsed')
            return
        if self.phase == 'done':
            # Repeated until the manager takes the activation away.
            self.pub_done.publish(Bool(data=True))
            return

        _, x, y, yaw, travelled = self.odometry[-1]

        if self.phase == 'approach':
            if self.in_the_way >= 2:
                self.get_logger().info('Construction: obstacle ahead, taking over.')
                self.phase = 'avoid'
                self.tracked = []
                self.direction = yaw
                self.road_found = False
                self.met = False
                self.takeover_at = travelled
            elif travelled - self.started_at > self.approach_limit:
                self.get_logger().warn('Construction: no obstacle met, done.')
                self.phase = 'done'
            if self.phase != 'avoid':
                return

        self.find_road(x, y)
        along, across = self.obstacles(x, y)

        # Clear means the obstacles have been passed, not that none has been
        # made out in the road yet.
        clear = self.road_is_clear(along, across)
        passed = self.track_obstacles(along, across, x, y)
        if self.obstacle_count > 0 and passed >= self.obstacle_count:
            # The last one is beside the robot: hand over now, facing the road.
            self.get_logger().info(
                f'Construction: last of {passed} obstacles beside the robot, done.')
            self.phase = 'done'
            return
        if not clear:
            self.met = True
            self.clear_count = 0
            self.cleared_at = None
        elif self.met and self.in_the_way == 0:
            self.clear_count += 1
        elif travelled - self.takeover_at > self.approach_limit:
            self.get_logger().warn('Construction: no obstacle found in the road, done.')
            self.phase = 'done'
            return
        if self.clear_count >= 3:
            if self.cleared_at is None:
                self.cleared_at = travelled
            if self.obstacle_count > 0 and passed >= self.obstacle_count:
                # All behind: the lane follower takes the bend that ends the zone.
                self.get_logger().info(
                    f'Construction: all {passed} obstacles passed, road clear, done.')
                self.phase = 'done'
                return
            # Not while an obstacle met is still ahead: weaving between them
            # the zone's own edge can show across the way (2026-10-10).
            if 0 < passed == len(self.tracked) and self.white_across(now):
                self.get_logger().info(
                    f'Construction: white line across the way after {passed} obstacles, '
                    'the zone ends in a bend, done.')
                self.phase = 'done'
                return
            if passed < self.obstacle_count:
                # More to come: carry on by the lane until the next one is in
                # the road, or give up after a stretch of clear road.
                if travelled - self.cleared_at > self.next_obstacle_distance:
                    self.get_logger().warn(
                        f'Construction: {passed} of {self.obstacle_count} obstacles '
                        'passed, road clear, done.')
                    self.phase = 'done'
                    return
            # Hand back to the lane follower as soon as it has a lane.
            elif now - self.lane_time < 0.3 or travelled - self.cleared_at > self.leave_distance:
                self.get_logger().info(
                    f'Construction: {passed} obstacles passed, road clear, done.')
                self.phase = 'done'
                return

        # With the obstacles behind, the lane detector's point is the better
        # guide: the road may be bending away at the end of the zone, and the
        # straight-road picture lays the path on into the outer line there.
        if self.met and clear and now - self.lane_time < 0.3:
            if not self.by_the_lane:
                self.get_logger().info('Construction: obstacles passed, steering by the lane.')
                self.by_the_lane = True
            self.pub_cmd_vel.publish(self.arc(*self.lane_target))
            return
        self.by_the_lane = False

        cost, blocked = self.road_cost(along, across)
        rows = lay_path(cost).astype(np.float64)
        # Take the corners off the steps the path is laid in.
        smooth = np.convolve(np.pad(rows, 2, mode='edge'), np.array([1, 2, 3, 2, 1]) / 9.0,
                             mode='valid')
        smooth[0] = rows[0]
        path_along = np.arange(len(rows)) * 2 * RES
        path_across = (smooth - ROW_0) * RES

        heading = wrap(yaw - self.direction)
        self.pub_cmd_vel.publish(self.steer(path_along, path_across, heading))
        if self.publish_debug:
            self.publish_debug_image(cost, blocked, path_along, path_across, heading)

    def steer(self, path_along, path_across, heading):
        """Drive the arc through the point of the path `lookahead` away."""
        distance = np.hypot(path_along, path_across)
        index = min(int(np.searchsorted(distance, self.lookahead)), len(distance) - 1)
        cos_h, sin_h = math.cos(heading), math.sin(heading)
        ahead = cos_h * path_along[index] + sin_h * path_across[index]
        left = -sin_h * path_along[index] + cos_h * path_across[index]

        twist = Twist()
        bearing = math.atan2(left, ahead)
        if abs(bearing) > 1.0:
            # Too far round to drive an arc to: turn on the spot first.
            twist.angular.z = math.copysign(self.cornering_rate, bearing)
        else:
            twist = self.arc(ahead, left)
        return self.within_road_angle(twist, heading)

    def within_road_angle(self, twist, heading):
        """Turn no further off the road than max_road_angle; bring it back past it."""
        over = abs(heading) - self.max_road_angle
        if over < 0.0 or twist.angular.z * heading <= 0.0:
            return twist
        # Turning further off: hold the slant (a little back towards the road
        # if already past it) and keep moving, rather than turn on the spot.
        twist.angular.z = -math.copysign(min(self.max_angular, 2.0 * over), heading)
        twist.linear.x = max(twist.linear.x, self.min_speed)
        return twist

    def arc(self, ahead, left):
        """Drive the arc from the axle, along the heading, through a point."""
        twist = Twist()
        curvature = 2.0 * left / (ahead * ahead + left * left)
        speed = self.speed
        if abs(curvature) * speed > self.cornering_rate:
            speed = max(self.min_speed, self.cornering_rate / abs(curvature))
        twist.linear.x = speed
        twist.angular.z = max(-self.max_angular, min(self.max_angular, speed * curvature))
        return twist

    def publish_debug_image(self, cost, blocked, path_along, path_across, heading):
        """Publish the road ahead, forward up: cost, obstacles, edges, path, robot."""
        scale = 3

        def pixel(along, across):
            return (int((D_MAX - across) / RES * scale), int((S_MAX - along) / RES * scale))

        shade = np.uint8(np.clip(cost * 12.0, 0, 150))[::-1, ::-1].T
        image = cv2.cvtColor(shade, cv2.COLOR_GRAY2BGR)
        image[blocked[::-1, ::-1].T > 0] = (255, 200, 0)
        image = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
        for colour, bgr in (('yellow', (0, 200, 255)), ('white', (255, 255, 255))):
            if self.walls[colour] is not None:
                cv2.line(image, pixel(S_MIN, self.walls[colour]),
                         pixel(S_MAX, self.walls[colour]), bgr, 1)
        points = np.array([pixel(a, d) for a, d in zip(path_along, path_across)], np.int32)
        cv2.polylines(image, [points], False, (0, 255, 0), 1)
        cv2.circle(image, pixel(0.0, 0.0), 4, (255, 120, 0), -1)
        cv2.line(image, pixel(0.0, 0.0),
                 pixel(0.08 * math.cos(heading), 0.08 * math.sin(heading)), (255, 120, 0), 2)
        out = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        out.header.stamp = self.get_clock().now().to_msg()
        self.pub_image.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ConstructionMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
