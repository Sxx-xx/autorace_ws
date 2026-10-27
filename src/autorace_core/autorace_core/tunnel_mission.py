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

"""Tunnel mission: through the dark, walled square from its entrance to its exit.

The tunnel is a square room, 1.74 m inside, roofed, with the entrance in the
wall the robot comes in by, near its right end, and the exit in the left
wall, near its far end: from the entrance, the exit is ahead and to the left
at about 45 degrees. A few obstacles stand inside, wherever the organisers
put them. There are no lines on its floor and no light, so the lane camera is
of no use; the laser is what the robot has, and the walls are what it steers by.

The lane follower drives the robot up to the tunnel. The entry wall, across
the way ahead and running off to the left (the entrance is at its right end,
where the robot comes in), is recognised by the laser from front_trigger
metres: the mission asks the manager for the road (approach), and drives
square to that wall through the middle of the entrance, which is
opening_width / 2 to the right of the wall's end. On the way the laser sees
it pass between the two ends of the entry wall, 15 cm to either side, with walls all
round ahead of it, and the robot's place in the tunnel is taken from that
scan: the wall ends say where it is, the walls which way it is heading (it
may come in askew). From then on the odometry carries that place along. The
mission itself starts when the forward camera goes dark: its picture's
brightness falls to half of the brightest it was over the last seconds. Dark
without the doorway (a shadow, a hand over the camera) starts nothing.

Inside, the reckoning is corrected on every scan against the four walls:
each wall's returns say how far off it is across that wall, and which way
they run says how far off it is in heading. The returns are also counted in a
grid, as the construction mission counts them, and the walls drawn in; and
across that grid a path is searched (A*) from the robot to just inside the
exit, every scan, keeping clear of everything by the robot's radius and
further where there is room. The robot steers for a point a little way along
it.

At the exit the robot turns on the spot until it lies parallel to the wall on
its right as it faces out (the far wall, running on past the exit as the
exit's right side), measured off the laser rather than the reckoning. It then
drives out along that wall, and the mission is done when the camera brightens
again, to twice the darkest it was over the last seconds.

    approach  square to the entry wall, through the entrance's middle, until
              the doorway has been passed and it is dark (approach_max metres
              at most: then the mission is skipped and the lane follower goes on)
    turn      on the spot, to face the exit (45 deg to the left from the entrance)
    drive     through the tunnel to the exit, along the straight line to it,
              round whatever stands on it (A*)
    align     turn parallel to the exit's right wall
    leave     out along that wall, until it is light
"""

from collections import deque
import heapq
import math

import cv2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_msgs.msg import String


RES = 0.02


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def search(cost, start, goal):
    """Search a cost grid, 8-connected (A*): the cells from start to goal, or None.

    `cost` is what entering a cell costs, with infinity for cells that cannot
    be entered; a diagonal step costs its cell's cost times root two.
    """
    rows, cols = cost.shape
    steps = [(dr, dc, math.hypot(dr, dc))
             for dr in (-1, 0, 1) for dc in (-1, 0, 1) if dr or dc]
    best = {start: 0.0}
    came = {}
    queue = [(0.0, start)]
    while queue:
        _, cell = heapq.heappop(queue)
        if cell == goal:
            path = [cell]
            while cell in came:
                cell = came[cell]
                path.append(cell)
            return path[::-1]
        here = best[cell]
        for dr, dc, length in steps:
            r, c = cell[0] + dr, cell[1] + dc
            if not (0 <= r < rows and 0 <= c < cols):
                continue
            total = here + length * cost[r, c]
            if not math.isfinite(total):
                continue
            if total < best.get((r, c), math.inf):
                best[(r, c)] = total
                came[(r, c)] = cell
                guess = total + math.hypot(goal[0] - r, goal[1] - c)
                heapq.heappush(queue, (guess, (r, c)))
    return None


class TunnelMission(Node):

    def __init__(self):
        super().__init__('tunnel_mission')

        # The tunnel, as seen from its entrance: x into it, y to the left.
        # Across the inside from wall to wall; from the middle of the
        # entrance to the wall on the right; along the entry wall from the
        # entrance to the exit, which is in the left wall. With the openings
        # as far from their corners as each other, the exit is at
        # (exit_x, inside - entrance_right): 45 degrees to the left.
        self.declare_parameter('inside', 1.74)
        self.declare_parameter('entrance_right', 0.18)
        self.declare_parameter('exit_x', 1.56)
        self.declare_parameter('opening_width', 0.34)
        # Wall ends this near on both sides, up to this far ahead, with at
        # least this share of the beams within 60 degrees of straight ahead
        # returning from within 2.5 m (and 60 % of the whole front half),
        # mean the entrance. Forgotten again this far on, if it never got dark.
        self.declare_parameter('doorway_half_width', 0.22)
        self.declare_parameter('doorway_ahead', 0.30)
        self.declare_parameter('enclosed_fraction', 0.7)
        self.declare_parameter('doorway_memory', 1.0)
        # The forward camera's brightness: dark below dark_ratio of the
        # brightest over the last light_window seconds, light again above
        # bright_ratio of the darkest, for light_frames frames in a row.
        self.declare_parameter('dark_ratio', 0.5)
        self.declare_parameter('bright_ratio', 2.0)
        self.declare_parameter('light_window', 3.0)
        self.declare_parameter('light_frames', 3)
        # Room to keep from walls and obstacles, and the room it likes.
        self.declare_parameter('robot_radius', 0.13)
        self.declare_parameter('comfort_clearance', 0.25)
        self.declare_parameter('lidar_offset', -0.032)
        self.declare_parameter('hit_fade', 0.93)
        self.declare_parameter('speed', 0.12)
        self.declare_parameter('min_speed', 0.04)
        self.declare_parameter('cornering_rate', 0.8)
        self.declare_parameter('max_angular', 1.2)
        self.declare_parameter('lookahead', 0.15)
        # Square to the exit's right wall within this, or go after this long.
        self.declare_parameter('align_tolerance', 0.05)
        self.declare_parameter('align_timeout', 4.0)
        # Out along that wall: how hard to turn for its angle and for the
        # distance off it, and at most this far if it never gets light.
        self.declare_parameter('leave_speed', 0.10)
        self.declare_parameter('wall_gain', 2.0)
        self.declare_parameter('side_gain', 3.0)
        self.declare_parameter('leave_distance', 1.0)
        self.declare_parameter('active_timeout', 1.0)
        # Once in and dark: turn on the spot to face the exit (45 deg left),
        # within turn_tolerance, at most turn_timeout seconds, then drive.
        self.declare_parameter('turn_rate', 0.8)
        self.declare_parameter('turn_tolerance', 0.05)
        self.declare_parameter('turn_timeout', 5.0)
        # The entry wall seen from outside: a straight run of returns across
        # the way, from ahead-left (the entrance is at its right end), within
        # front_wall_angle of square, at least front_wall_points returns over
        # front_wall_span, inside front_wall_ahead x front_wall_left. Within
        # front_trigger of it the mission asks for the road and drives square
        # to it through the entrance's middle at approach_speed, aiming
        # approach_beyond past the doorway; approach_max metres without the
        # doorway and the dark, and the mission is skipped.
        self.declare_parameter('front_wall_trigger', True)
        self.declare_parameter('front_wall_ahead', 0.90)
        self.declare_parameter('front_wall_left', 1.20)
        self.declare_parameter('front_wall_points', 20)
        self.declare_parameter('front_wall_span', 0.40)
        self.declare_parameter('front_wall_angle', 0.50)
        self.declare_parameter('front_trigger', 0.60)
        self.declare_parameter('entry_scans', 2)
        self.declare_parameter('approach_beyond', 0.30)
        self.declare_parameter('approach_speed', 0.10)
        self.declare_parameter('approach_max', 1.2)
        self.declare_parameter('heading_gain', 2.0)
        self.declare_parameter('stop_distance', 0.12)
        self.declare_parameter('stop_half_width', 0.10)

        def value(name):
            return self.get_parameter(name).value

        self.inside = value('inside')
        self.right = -value('entrance_right')
        self.left = self.inside + self.right
        self.exit_x = value('exit_x')
        self.opening = value('opening_width')
        self.doorway = value('doorway_half_width')
        self.doorway_ahead = value('doorway_ahead')
        self.enclosed_fraction = value('enclosed_fraction')
        self.doorway_memory = value('doorway_memory')
        self.dark_ratio = value('dark_ratio')
        self.bright_ratio = value('bright_ratio')
        self.light_window = value('light_window')
        self.light_frames = value('light_frames')
        self.robot_radius = value('robot_radius')
        self.comfort = value('comfort_clearance')
        self.lidar_offset = value('lidar_offset')
        self.hit_fade = value('hit_fade')
        self.speed = value('speed')
        self.min_speed = value('min_speed')
        self.cornering_rate = value('cornering_rate')
        self.max_angular = value('max_angular')
        self.lookahead = value('lookahead')
        self.align_tolerance = value('align_tolerance')
        self.align_timeout = value('align_timeout')
        self.leave_speed = value('leave_speed')
        self.wall_gain = value('wall_gain')
        self.side_gain = value('side_gain')
        self.leave_distance = value('leave_distance')
        self.active_timeout = value('active_timeout')
        self.turn_rate = value('turn_rate')
        self.turn_tolerance = value('turn_tolerance')
        self.turn_timeout = value('turn_timeout')
        self.turn_start = 0.0
        self.front_wall_trigger = value('front_wall_trigger')
        self.front_wall_ahead = value('front_wall_ahead')
        self.front_wall_left = value('front_wall_left')
        self.front_wall_points = value('front_wall_points')
        self.front_wall_span = value('front_wall_span')
        self.front_wall_angle = value('front_wall_angle')
        self.front_trigger = value('front_trigger')
        self.entry_scans = value('entry_scans')
        self.approach_beyond = value('approach_beyond')
        self.approach_speed = value('approach_speed')
        self.approach_max = value('approach_max')
        self.heading_gain = value('heading_gain')
        self.stop_distance = value('stop_distance')
        self.stop_half_width = value('stop_half_width')

        # The grid covers the inside and a margin beyond the walls.
        self.margin = 0.3
        self.x_0 = -self.margin
        self.y_0 = self.right - self.margin
        self.cols = int(round((self.inside + 2 * self.margin) / RES)) + 1
        self.rows = int(round((self.inside + 2 * self.margin) / RES)) + 1
        self.hits = np.zeros((self.rows, self.cols), np.float32)
        self.walls = self.draw_walls()

        self.phase = None           # None while the mission is not running
        self.active_time = 0.0
        self.doorway_count = 0      # scans in a row between the wall ends
        self.doorway_travelled = 0.0  # self.travelled when last in the doorway
        self.last_points = None     # the latest scan, in the robot frame
        self.pose = None            # x, y, heading in the tunnel, from the doorway on
        self.odom_pose = None       # the odometry pose the above was taken from
        self.path = None
        self.planned_time = 0.0
        self.levels = deque()       # (time, brightness) over the last light_window
        self.dark_count = 0
        self.bright_count = 0
        self.light = False          # bright again, out of the tunnel
        self.announced = False      # the dark entrance logged
        self.front = None           # the entry wall's latest fit (distance, angle, aim)
        self.front_time = 0.0
        self.front_count = 0        # scans in a row with it within front_trigger
        self.approach_from = 0.0    # self.travelled when the approach began
        self.approach_yaw = None    # heading to hold, square to the entry wall (odom)
        self.approach_target = None  # the point past the entrance's middle (odom)
        self.align_start = 0.0
        self.square_count = 0
        self.leave_from = None
        self.travelled = 0.0

        self.create_subscription(
            Bool, '/autorace/mission/tunnel/active', self.callback_active, 1)
        self.create_subscription(Odometry, '/odom', self.callback_odom, 1)
        self.create_subscription(LaserScan, '/scan', self.callback_scan, 1)
        # The forward camera; race.launch.py remaps this to the decoded stream.
        self.create_subscription(
            Image, '/camera/image_raw', self.callback_image, qos_profile_sensor_data)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel/mission', 1)
        self.pub_done = self.create_publisher(Bool, '/autorace/mission/tunnel/done', 1)
        # The approach found no tunnel: the manager goes back to lane following
        # without counting the mission done.
        self.pub_skipped = self.create_publisher(Bool, '/autorace/mission/tunnel/skipped', 1)
        self.pub_trigger = self.create_publisher(String, '/autorace/trigger', 5)
        # True while the robot's place in the tunnel is known from the doorway:
        # the construction mission must not take the tunnel's walls for blocks.
        self.pub_doorway = self.create_publisher(Bool, '/autorace/tunnel/doorway', 1)

        self.create_timer(0.1, self.update)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    # --- the tunnel ------------------------------------------------------

    def cell(self, x, y):
        return (int(round((y - self.y_0) / RES)), int(round((x - self.x_0) / RES)))

    def draw_walls(self):
        """Draw the four walls as occupied cells, with the entrance and exit left out."""
        walls = np.zeros((self.rows, self.cols), np.uint8)
        half = self.opening / 2.0
        for y in np.arange(self.right, self.left + RES / 2, RES):
            r, c = self.cell(0.0, y)
            if abs(y) > half:
                walls[r, c] = 1
            r, c = self.cell(self.inside, y)
            walls[r, c] = 1
        for x in np.arange(0.0, self.inside + RES / 2, RES):
            r, c = self.cell(x, self.right)
            walls[r, c] = 1
            r, c = self.cell(x, self.left)
            if abs(x - self.exit_x) > half:
                walls[r, c] = 1
        return walls

    def in_tunnel(self, points):
        """Points of a scan in the tunnel frame, by the current pose."""
        x, y, heading = self.pose
        cos_h, sin_h = math.cos(heading), math.sin(heading)
        return np.stack([x + cos_h * points[:, 0] - sin_h * points[:, 1],
                         y + sin_h * points[:, 0] + cos_h * points[:, 1]], axis=1)

    def first_heading(self):
        """Find the heading, in the tunnel, that lays the scan best along the walls."""
        best = None
        x_0, y_0, _ = self.pose
        for heading in np.linspace(-1.2, 1.2, 49):
            self.pose = (x_0, y_0, heading)
            x, y = self.in_tunnel(self.last_points).T
            inside = ((x > -0.1) & (x < self.inside + 0.1)
                      & (y > self.right - 0.1) & (y < self.left + 0.1))
            on_wall = inside & ((np.abs(x) < 0.05) | (np.abs(x - self.inside) < 0.05)
                                | (np.abs(y - self.right) < 0.05) | (np.abs(y - self.left) < 0.05))
            count = int(np.count_nonzero(on_wall))
            if best is None or count > best[0]:
                best = (count, heading)
        return best[1]

    def align(self, points):
        """Correct the pose against the walls the scan shows."""
        x, y, heading = self.pose
        half = self.opening / 2.0
        shift_x, shift_y, turn = [], [], []
        for wall_x, side in ((0.0, 'entry'), (self.inside, 'far')):
            near = points[(np.abs(points[:, 0] - wall_x) < 0.08)
                          & (points[:, 1] > self.right + 0.1) & (points[:, 1] < self.left - 0.1)]
            if side == 'entry':
                near = near[np.abs(near[:, 1]) > half + 0.1]
            if len(near) >= 15:
                shift_x.append(float(np.median(near[:, 0])) - wall_x)
                if np.ptp(near[:, 1]) > 0.3:
                    # The wall runs along y. A heading too large by d turns
                    # the scan anticlockwise, and the wall leans dx/dy = -d.
                    slope = np.polyfit(near[:, 1], near[:, 0], 1)[0]
                    turn.append(-math.atan(slope))
        for wall_y, side in ((self.right, 'right'), (self.left, 'left')):
            near = points[(np.abs(points[:, 1] - wall_y) < 0.08)
                          & (points[:, 0] > 0.1) & (points[:, 0] < self.inside - 0.1)]
            if side == 'left':
                near = near[np.abs(near[:, 0] - self.exit_x) > half + 0.1]
            if len(near) >= 15:
                shift_y.append(float(np.median(near[:, 1])) - wall_y)
                if np.ptp(near[:, 0]) > 0.3:
                    slope = np.polyfit(near[:, 0], near[:, 1], 1)[0]
                    turn.append(math.atan(slope))
        if shift_x:
            x -= 0.5 * float(np.mean(shift_x))
        if shift_y:
            y -= 0.5 * float(np.mean(shift_y))
        if turn:
            heading = wrap(heading - 0.5 * float(np.mean(turn)))
        self.pose = (x, y, heading)

    def right_wall(self):
        """The wall on the robot's right, from the laser: its angle and distance, or None.

        The angle is the wall's direction against the robot's heading, positive
        when the wall comes nearer ahead (the robot is turned towards it).
        """
        points = self.last_points
        if points is None:
            return None
        near = points[(points[:, 1] < -0.05) & (points[:, 1] > -0.40)
                      & (points[:, 0] > -0.25) & (points[:, 0] < 0.35)]
        if len(near) < 10 or np.ptp(near[:, 0]) < 0.2:
            return None
        slope, offset = np.polyfit(near[:, 0], near[:, 1], 1)
        angle = math.atan(slope)
        return angle, -offset * math.cos(angle)

    def occupied(self):
        """Cells with an obstacle in them: the walls, and what the laser keeps hitting."""
        peak = cv2.dilate(self.hits, np.ones((7, 7), np.uint8))
        standing = (self.hits >= 2.0) & (self.hits * 4.0 >= peak)
        return (standing | (self.walls > 0)).astype(np.uint8)

    def plan(self):
        """Lay a path from the robot to just inside the exit."""
        blocked = self.occupied()
        clearance = cv2.distanceTransform(1 - blocked, cv2.DIST_L2, 5) * RES
        crowding = np.clip((self.comfort - clearance) / (self.comfort - self.robot_radius),
                           0.0, 1.0)
        cost = 1.0 + 6.0 * crowding ** 2
        cost[clearance < self.robot_radius] = np.inf

        start = self.cell(self.pose[0], self.pose[1])
        if not math.isfinite(cost[start]):
            # Pressed against something, as in the doorway: start from the
            # nearest cell that is clear.
            free = np.argwhere(np.isfinite(cost))
            nearest = np.argmin(np.hypot(free[:, 0] - start[0], free[:, 1] - start[1]))
            start = tuple(int(v) for v in free[nearest])
        goal = self.cell(self.exit_x, self.left - 0.12)
        if not math.isfinite(cost[goal]):
            free = np.argwhere(np.isfinite(cost))
            nearest = np.argmin(np.hypot(free[:, 0] - goal[0], free[:, 1] - goal[1]))
            goal = tuple(int(v) for v in free[nearest])
        cells = search(cost, start, goal)
        if cells is None:
            return None
        return np.array([(self.x_0 + c * RES, self.y_0 + r * RES) for r, c in cells])

    # --- callbacks -------------------------------------------------------

    def callback_active(self, msg):
        if msg.data:
            self.active_time = self.now()
            if self.phase is None:
                if self.pose is None or self.last_points is None:
                    # Not through the doorway yet: up to it, square to the
                    # entry wall if one is in view, straight on if not.
                    self.phase = 'approach'
                    self.approach_from = self.travelled
                    self.approach_yaw = None if self.odom_pose is None else self.odom_pose[2]
                    self.approach_target = None
                    fresh = self.front is not None and self.now() - self.front_time < 1.0
                    if fresh:
                        self.aim(self.front)
                    self.get_logger().info(
                        'Tunnel: approaching %s at %.2f m/s.'
                        % ('the entry wall %.2f m ahead, %+.0f deg off square'
                           % (self.front[0], math.degrees(self.front[1])) if fresh
                           else 'straight on (no entry wall in view)', self.approach_speed))
                    return
                self.begin_drive()
        elif self.phase is not None:
            self.get_logger().info(f'Tunnel: deactivated during {self.phase}.')
            self.phase = None
            self.pose = None
            self.announced = False

    def begin_drive(self):
        """Through the doorway and dark: drive through the tunnel from the pose."""
        for _ in range(3):
            self.align(self.in_tunnel(self.last_points))
        self.get_logger().info(
            'Tunnel: at (%.2f, %.2f), heading %.0f deg off the tunnel; turning to face the exit.'
            % (self.pose[0], self.pose[1], math.degrees(-self.pose[2])))
        self.hits[:] = 0.0
        self.path = None
        self.light = False
        self.phase = 'turn'
        self.turn_start = self.now()

    def callback_odom(self, msg):
        position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        here = (position.x, position.y, yaw)
        if self.odom_pose is not None:
            dx, dy = here[0] - self.odom_pose[0], here[1] - self.odom_pose[1]
            self.travelled += math.hypot(dx, dy)
            if self.pose is not None:
                # Move the tunnel pose on by what the odometry moved.
                ahead = math.cos(self.odom_pose[2]) * dx + math.sin(self.odom_pose[2]) * dy
                left = -math.sin(self.odom_pose[2]) * dx + math.cos(self.odom_pose[2]) * dy
                x, y, heading = self.pose
                self.pose = (x + math.cos(heading) * ahead - math.sin(heading) * left,
                             y + math.sin(heading) * ahead + math.cos(heading) * left,
                             wrap(heading + wrap(here[2] - self.odom_pose[2])))
        self.odom_pose = here

    def callback_image(self, msg):
        data = np.frombuffer(msg.data, np.uint8)
        if not msg.height or data.size != msg.height * msg.step:
            return
        # Every eighth row is plenty for the mean, and costs next to nothing.
        rows = data.reshape(msg.height, msg.step)
        level = float(rows[::8].mean())
        now = self.now()
        self.levels.append((now, level))
        while self.levels and now - self.levels[0][0] > self.light_window:
            self.levels.popleft()

        if self.phase is None or self.phase == 'approach':
            brightest = max(v for _, v in self.levels)
            self.dark_count = self.dark_count + 1 if level < self.dark_ratio * brightest else 0
            if self.dark_count < self.light_frames:
                return
            if self.pose is None:
                self.get_logger().warn(
                    'Tunnel: dark (%.0f of %.0f) but no entrance seen, %s.'
                    % (level, brightest,
                       'driving on' if self.phase == 'approach' else 'not starting'),
                    throttle_duration_sec=2.0)
                return
            if not self.announced:
                self.announced = True
                self.get_logger().info(
                    'Tunnel: dark (%.0f of %.0f) past the entrance, entering.'
                    % (level, brightest))
            if self.phase == 'approach':
                self.begin_drive()
                return
            # The manager takes this only while lane following, and once.
            self.pub_trigger.publish(String(data='tunnel'))
        elif self.phase == 'leave':
            darkest = min(v for _, v in self.levels)
            bright = level > self.bright_ratio * max(darkest, 1.0)
            self.bright_count = self.bright_count + 1 if bright else 0
            if self.bright_count >= self.light_frames and not self.light:
                self.light = True
                self.get_logger().info(
                    'Tunnel: light again (%.0f, from %.0f).' % (level, darkest))
        else:
            self.dark_count = 0
            self.bright_count = 0

    def callback_scan(self, msg):
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        seen = np.isfinite(ranges) & (ranges > msg.range_min + 1e-3) & (ranges < 3.0)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        points = np.stack([self.lidar_offset + ranges[seen] * np.cos(angles[seen]),
                           ranges[seen] * np.sin(angles[seen])], axis=1)
        self.last_points = points

        if self.phase is None or self.phase == 'approach':
            self.watch_doorway(points, ranges, angles, seen)
            front = self.front_wall_fit(points)
            if front is not None:
                self.front, self.front_time = front, self.now()
            if self.phase is None:
                self.pub_doorway.publish(Bool(data=self.pose is not None))
                near = front is not None and front[0] < self.front_trigger
                self.front_count = self.front_count + 1 if near else 0
                if self.front_wall_trigger and self.front_count >= self.entry_scans:
                    if self.front_count == self.entry_scans:
                        self.get_logger().info(
                            'Tunnel: entry wall %.2f m ahead, %+.0f deg off square; asking for the road.'
                            % (front[0], math.degrees(front[1])))
                    # The manager takes this only while lane following, and once.
                    self.pub_trigger.publish(String(data='tunnel'))
            elif front is not None:
                self.aim(front)     # while the wall is in view; then what it last said
            return
        if self.phase not in ('turn', 'drive', 'align'):
            return

        inside = self.in_tunnel(points)
        self.align(inside)
        if self.phase != 'drive':
            return
        inside = self.in_tunnel(points)
        column = np.int_(np.round((inside[:, 0] - self.x_0) / RES))
        row = np.int_(np.round((inside[:, 1] - self.y_0) / RES))
        keep = (column >= 0) & (column < self.cols) & (row >= 0) & (row < self.rows)
        self.hits *= self.hit_fade
        np.add.at(self.hits, (row[keep], column[keep]), 1.0)
        # The search takes a tenth of a second; not for every scan.
        if self.now() - self.planned_time > 0.3:
            self.planned_time = self.now()
            self.path = self.plan()
            if self.path is None:
                self.get_logger().warn('Tunnel: no way to the exit found.',
                                       throttle_duration_sec=2.0)

    def watch_doorway(self, points, ranges, angles, seen):
        """Take the robot's place in the tunnel while it passes between the wall ends."""
        # Wall ends to both sides, level with the robot or a little ahead.
        beside = (points[:, 0] > -0.12) & (points[:, 0] < self.doorway_ahead)
        left = points[beside & (points[:, 1] > 0.08) & (points[:, 1] < self.doorway)]
        right = points[beside & (points[:, 1] < -0.08) & (points[:, 1] > -self.doorway)]
        sector = np.cos(angles) > 0.5
        half = np.cos(angles) > 0.0
        enclosed = np.mean(seen[sector] & (ranges[sector] < 2.5))
        enclosed_half = np.mean(seen[half] & (ranges[half] < 2.5))
        if (len(left) >= 3 and len(right) >= 3 and enclosed >= self.enclosed_fraction
                and enclosed_half >= 0.6):
            self.doorway_count += 1
        else:
            self.doorway_count = 0
        if self.doorway_count >= 2:
            # In the doorway: the entry wall is here, the entrance's middle is
            # between the two wall ends, and the walls say which way the
            # tunnel runs from here. The latest such scan, the deepest in.
            side_left = float(np.median(left[:, 1]))
            side_right = float(-np.median(right[:, 1]))
            distance = float(np.median(np.concatenate([left[:, 0], right[:, 0]])))
            self.pose = (-distance, (side_right - side_left) / 2.0, 0.0)
            self.pose = (self.pose[0], self.pose[1], self.first_heading())
            for _ in range(3):
                self.align(self.in_tunnel(points))
            self.doorway_travelled = self.travelled
        elif (self.pose is not None
              and self.travelled - self.doorway_travelled > self.doorway_memory):
            # Through a doorway that never went dark: not the tunnel.
            self.pose = None
            self.announced = False

    # --- driving ---------------------------------------------------------

    def front_wall_fit(self, points):
        """The entry wall across the way ahead, running off to the left, as
        (distance ahead, angle off square (rad, +: turn left to face it
        square), aim): aim is the point approach_beyond past the middle of the
        entrance at the wall's right end, square to the wall, in the robot's
        frame. None if no such wall is there."""
        p = points
        band = p[(p[:, 0] > 0.05) & (p[:, 0] < self.front_wall_ahead)
                 & (p[:, 1] > 0.0) & (p[:, 1] < self.front_wall_left)]
        if len(band) < self.front_wall_points:
            return None
        vx, vy, x0, y0 = cv2.fitLine(band.astype(np.float32), cv2.DIST_HUBER, 0, 0.01, 0.01).ravel()
        if vy < 0:
            vx, vy = -vx, -vy            # along the wall, to the left
        along = (band[:, 0] - x0) * vx + (band[:, 1] - y0) * vy
        across = np.abs(-(band[:, 0] - x0) * vy + (band[:, 1] - y0) * vx)
        off = math.atan2(-vx, vy)       # the wall's normal (ahead) against the heading
        if (abs(off) > self.front_wall_angle or np.ptp(along) < self.front_wall_span
                or np.median(across) > 0.02):
            return None
        normal = np.array([vy, -vx])     # square to the wall, away from the robot
        distance = float(abs(np.dot(np.array([x0, y0]), normal)))
        # The wall's right end: the fit is from the left half, but the end
        # may lie right of the robot's centre line (the robot a little left
        # of the entrance): every return on the line counts for it.
        on_line = p[(p[:, 0] > 0.05) & (p[:, 0] < self.front_wall_ahead + 0.2) & (p[:, 1] > -0.6)
                    & (np.abs(-(p[:, 0] - x0) * vy + (p[:, 1] - y0) * vx) < 0.03)]
        # Rightwards from the fitted stretch, return by return, up to the
        # first gap: the entrance (the corner beyond it is on the line too).
        positions = np.sort((on_line[:, 0] - x0) * vx + (on_line[:, 1] - y0) * vy)
        first = along.min()
        for position in positions[positions < first][::-1]:
            if first - position > 0.08:
                break
            first = position
        end = np.array([x0, y0]) + np.array([vx, vy]) * first   # the wall's right end
        middle = end - np.array([vx, vy]) * (self.opening / 2.0)
        return distance, float(off), middle + normal * self.approach_beyond

    def aim(self, front):
        """Square to the entry wall, for the entrance's middle (odometry frame)."""
        if self.odom_pose is None:
            return
        _, off, aim = front
        xo, yo, ao = self.odom_pose
        c, s = math.cos(ao), math.sin(ao)
        self.approach_yaw = wrap(ao + off)
        self.approach_target = (xo + c * aim[0] - s * aim[1], yo + s * aim[0] + c * aim[1])

    def something_ahead(self):
        p = self.last_points
        return p is not None and bool(np.any(
            (p[:, 0] > 0.0) & (p[:, 0] < self.stop_distance) & (np.abs(p[:, 1]) < self.stop_half_width)))

    def through(self):
        """For approach_target (the entrance's middle, past it), then on square to its wall."""
        twist = Twist()
        if self.odom_pose is None:
            return twist
        twist.linear.x = 0.0 if self.something_ahead() else self.approach_speed
        xo, yo, ao = self.odom_pose
        heading = ao if self.approach_yaw is None else self.approach_yaw
        if self.approach_target is not None:
            dx, dy = self.approach_target[0] - xo, self.approach_target[1] - yo
            if math.cos(ao) * dx + math.sin(ao) * dy > 0.10:
                heading = math.atan2(dy, dx)      # for the aim; square to the wall past it
        steer = self.heading_gain * wrap(heading - ao)
        twist.angular.z = max(-self.max_angular, min(self.max_angular, steer))
        return twist

    def steer_to(self, target):
        """Drive the arc through a point given in the tunnel frame."""
        x, y, heading = self.pose
        dx, dy = target[0] - x, target[1] - y
        ahead = math.cos(heading) * dx + math.sin(heading) * dy
        left = -math.sin(heading) * dx + math.cos(heading) * dy
        twist = Twist()
        bearing = math.atan2(left, ahead)
        if abs(bearing) > 1.0:
            twist.angular.z = math.copysign(self.cornering_rate, bearing)
            return twist
        curvature = 2.0 * left / (ahead * ahead + left * left)
        speed = self.speed
        if abs(curvature) * speed > self.cornering_rate:
            speed = max(self.min_speed, self.cornering_rate / abs(curvature))
        twist.linear.x = speed
        twist.angular.z = max(-self.max_angular, min(self.max_angular, speed * curvature))
        return twist

    def turn_square(self):
        """Turn on the spot until parallel to the exit's right wall; True once there."""
        twist = Twist()
        # Facing out of the exit is +90 degrees in the tunnel. Far from it,
        # turn by the reckoning; near it, by the wall itself.
        error = wrap(math.pi / 2.0 - self.pose[2])
        wall = self.right_wall() if abs(error) < 0.3 else None
        if wall is None:
            self.square_count = 0
            twist.angular.z = max(-self.cornering_rate, min(self.cornering_rate, 2.0 * error))
            return twist, False
        angle = wall[0]
        if abs(angle) < self.align_tolerance:
            self.square_count += 1
            return twist, self.square_count >= 3
        self.square_count = 0
        # At least enough to move the wheels at all.
        rate = max(0.15, min(0.6, 2.5 * abs(angle)))
        twist.angular.z = math.copysign(rate, angle)
        return twist, False

    def follow_right_wall(self):
        """Drive out along the wall on the right, the exit's half width off it."""
        twist = Twist()
        twist.linear.x = self.leave_speed
        wall = self.right_wall()
        if wall is None:
            # Past its end: straight on, out of the exit, by the reckoning.
            x, y, _ = self.pose
            twist = self.steer_to((x, y + 0.3))
            twist.linear.x = min(twist.linear.x, self.leave_speed)
            return twist
        angle, distance = wall
        turn = self.wall_gain * angle - self.side_gain * (distance - self.opening / 2.0)
        twist.angular.z = max(-self.cornering_rate, min(self.cornering_rate, turn))
        return twist

    def update(self):
        if self.phase is None:
            return
        now = self.now()
        if now - self.active_time > self.active_timeout:
            self.get_logger().info(f'Tunnel: activation lapsed during {self.phase}.')
            self.phase = None
            self.pose = None
            return
        if self.phase == 'done':
            self.pub_done.publish(Bool(data=True))
            return
        if self.phase == 'skipped':
            self.pub_skipped.publish(Bool(data=True))
            return
        if self.phase == 'approach':
            if self.travelled - self.approach_from > self.approach_max:
                self.get_logger().warn(
                    'Tunnel: %.1f m on and no doorway in the dark; not the tunnel, handing back.'
                    % self.approach_max)
                self.phase = 'skipped'
                return
            self.pub_cmd_vel.publish(self.through())
            return
        if self.pose is None:
            return

        x, y, heading = self.pose
        if self.phase != 'leave' and not (-0.3 < x < self.inside + 0.3
                                          and self.right - 0.3 < y < self.left + 0.6):
            # Not in the tunnel after all: better the lane follower than this.
            self.get_logger().error('Tunnel: the reckoning has left the tunnel, giving up.')
            self.phase = 'done'
            return
        twist = Twist()
        if self.phase == 'turn':
            bearing = math.atan2(self.left - 0.12 - y, self.exit_x - x)
            error = wrap(bearing - heading)
            if abs(error) < self.turn_tolerance or now - self.turn_start > self.turn_timeout:
                self.get_logger().info(
                    'Tunnel: facing the exit (%.0f deg, %s), driving through.'
                    % (math.degrees(heading),
                       'in time' if abs(error) < self.turn_tolerance else 'turn timed out'))
                self.phase = 'drive'
            else:
                twist.angular.z = math.copysign(min(self.turn_rate, max(0.15, 2.0 * abs(error))), error)
                self.pub_cmd_vel.publish(twist)
                return
        if self.phase == 'drive':
            if abs(x - self.exit_x) < 0.10 and y > self.left - 0.25:
                self.get_logger().info('Tunnel: at the exit, squaring up to its right wall.')
                self.phase = 'align'
                self.align_start = now
                self.square_count = 0
            elif self.path is not None and len(self.path) > 1:
                distance = np.hypot(self.path[:, 0] - x, self.path[:, 1] - y)
                index = int(np.argmax(distance >= self.lookahead))
                if distance[index] < self.lookahead:
                    index = len(self.path) - 1
                twist = self.steer_to(self.path[index])
        if self.phase == 'align':
            twist, square = self.turn_square()
            late = now - self.align_start > self.align_timeout
            if square or late:
                if late:
                    self.get_logger().warn(
                        'Tunnel: not square to the wall in time, leaving anyway.')
                else:
                    self.get_logger().info('Tunnel: square to the wall, leaving.')
                self.phase = 'leave'
                self.leave_from = self.travelled
                self.bright_count = 0
                self.light = False
                twist = Twist()
        if self.phase == 'leave':
            if self.light:
                self.get_logger().info('Tunnel: out, done.')
                self.phase = 'done'
                return
            if self.travelled - self.leave_from > self.leave_distance:
                self.get_logger().warn('Tunnel: never got light, done after %.1f m.'
                                       % self.leave_distance)
                self.phase = 'done'
                return
            twist = self.follow_right_wall()
        self.pub_cmd_vel.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = TunnelMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
