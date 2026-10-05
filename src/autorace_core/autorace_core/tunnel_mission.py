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

The tunnel is a square room with an opening in two of its walls and a few
obstacles standing inside, wherever the organisers put them. There are no
lines on its floor and no light, so the lane camera is of no use; the laser
is what the robot has, and the walls are what it steers by.

Entering, the robot passes between the two ends of the wall, 15 cm to either
side, with walls all round ahead of it: nearly every beam of the front sector
of the scan comes back from within a couple of metres, where in the open
most are lost. The two together, with the wall ends still a little way
ahead, are what start the mission: early enough that the lane follower, which
loses its lines at the wall, has not yet begun to cast about. The wall ends
alone would also fit passing between two obstacles in the construction
zone. The robot may come through the doorway askew, so which way the tunnel
runs is read off the walls in that first scan, not taken from the heading.
From that moment the robot keeps its own reckoning of where it is in the
tunnel, from the odometry, and corrects it
on every scan against the four walls: each wall's returns say how far off
the reckoning is across that wall, and which way they run says how far off
it is in heading.

The returns are also counted in a grid, as the construction mission counts
them, and the walls drawn in; and across that grid a path is searched (A*)
from the robot to just inside the exit, every scan, keeping clear of
everything by the robot's radius and further where there is room. The robot
steers for a point a little way along it, and once in the exit drives
straight out until the lane follower has a lane again.

    drive   through the tunnel to the exit
    leave   out through the exit
"""

from collections import deque
import heapq
import math

import cv2
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
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
        # entrance to the exit, which is in the left wall.
        self.declare_parameter('inside', 1.86)
        self.declare_parameter('entrance_right', 0.18)
        self.declare_parameter('exit_x', 1.68)
        self.declare_parameter('opening_width', 0.34)
        # Wall ends this near on both sides, up to this far ahead, with at
        # least this share of the beams within 60 degrees of straight ahead
        # returning from within 2.5 m (and 60 % of the whole front half),
        # mean the entrance.
        self.declare_parameter('doorway_half_width', 0.22)
        self.declare_parameter('doorway_ahead', 0.30)
        self.declare_parameter('enclosed_fraction', 0.7)
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
        # Driven straight out of the exit at most, looking for a lane.
        self.declare_parameter('leave_distance', 0.6)
        self.declare_parameter('active_timeout', 1.0)

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
        self.robot_radius = value('robot_radius')
        self.comfort = value('comfort_clearance')
        self.lidar_offset = value('lidar_offset')
        self.hit_fade = value('hit_fade')
        self.speed = value('speed')
        self.min_speed = value('min_speed')
        self.cornering_rate = value('cornering_rate')
        self.max_angular = value('max_angular')
        self.lookahead = value('lookahead')
        self.leave_distance = value('leave_distance')
        self.active_timeout = value('active_timeout')

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
        self.odometry = deque(maxlen=400)
        self.doorway_count = 0      # scans in a row between the wall ends
        self.doorway_sides = (0.0, 0.0)
        self.doorway_distance = 0.0  # from the axle to the wall ends
        self.last_points = None     # the latest scan, in the robot frame
        self.pose = None            # x, y, heading in the tunnel
        self.odom_pose = None       # the odometry pose the above was taken from
        self.path = None
        self.planned_time = 0.0
        self.leave_from = None
        self.lane_time = 0.0
        self.travelled = 0.0

        self.create_subscription(
            Bool, '/autorace/mission/tunnel/active', self.callback_active, 1)
        self.create_subscription(Odometry, '/odom', self.callback_odom, 1)
        self.create_subscription(LaserScan, '/scan', self.callback_scan, 1)
        self.create_subscription(PointStamped, '/detect/lane_target', self.callback_lane, 1)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel/mission', 1)
        self.pub_done = self.create_publisher(Bool, '/autorace/mission/tunnel/done', 1)
        self.pub_trigger = self.create_publisher(String, '/autorace/trigger', 5)

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
            if (self.phase is None and self.odom_pose is not None
                    and self.last_points is not None):
                left, right = self.doorway_sides
                # In the doorway: the entry wall is here, the entrance's
                # middle is between the two wall ends, and the walls say
                # which way the tunnel runs from here.
                self.pose = (-self.doorway_distance, (right - left) / 2.0, 0.0)
                self.pose = (self.pose[0], self.pose[1], self.first_heading())
                for _ in range(3):
                    self.align(self.in_tunnel(self.last_points))
                self.get_logger().info(
                    'Tunnel: heading %.0f deg off the tunnel at the entrance.'
                    % math.degrees(-self.pose[2]))
                self.hits[:] = 0.0
                self.path = None
                self.get_logger().info('Tunnel: in the entrance, driving through.')
                self.phase = 'drive'
        elif self.phase is not None:
            self.get_logger().info(f'Tunnel: deactivated during {self.phase}.')
            self.phase = None

    def callback_odom(self, msg):
        position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        here = (position.x, position.y, yaw)
        if self.pose is not None and self.odom_pose is not None:
            # Move the tunnel pose on by what the odometry moved.
            dx, dy = here[0] - self.odom_pose[0], here[1] - self.odom_pose[1]
            ahead = math.cos(self.odom_pose[2]) * dx + math.sin(self.odom_pose[2]) * dy
            left = -math.sin(self.odom_pose[2]) * dx + math.cos(self.odom_pose[2]) * dy
            x, y, heading = self.pose
            self.pose = (x + math.cos(heading) * ahead - math.sin(heading) * left,
                         y + math.sin(heading) * ahead + math.cos(heading) * left,
                         wrap(heading + wrap(here[2] - self.odom_pose[2])))
            self.travelled += math.hypot(dx, dy)
        self.odom_pose = here

    def callback_lane(self, msg):
        self.lane_time = self.now()

    def callback_scan(self, msg):
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        seen = np.isfinite(ranges) & (ranges > msg.range_min + 1e-3) & (ranges < 3.0)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        points = np.stack([self.lidar_offset + ranges[seen] * np.cos(angles[seen]),
                           ranges[seen] * np.sin(angles[seen])], axis=1)
        self.last_points = points

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
            self.doorway_sides = (float(np.median(left[:, 1])), float(-np.median(right[:, 1])))
            self.doorway_distance = float(np.median(np.concatenate([left[:, 0], right[:, 0]])))
        else:
            self.doorway_count = 0
        if self.phase is None:
            if self.doorway_count >= 2:
                # The manager takes this only while lane following, and once.
                self.pub_trigger.publish(String(data='tunnel'))
            return
        if self.phase != 'drive':
            return

        inside = self.in_tunnel(points)
        self.align(inside)
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

    # --- driving ---------------------------------------------------------

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

    def update(self):
        if self.phase is None or self.pose is None:
            return
        now = self.now()
        if now - self.active_time > self.active_timeout:
            self.get_logger().info(f'Tunnel: activation lapsed during {self.phase}.')
            self.phase = None
            return
        if self.phase == 'done':
            self.pub_done.publish(Bool(data=True))
            return

        x, y, heading = self.pose
        if not (-0.3 < x < self.inside + 0.3 and self.right - 0.3 < y < self.left + 0.6):
            # Not in the tunnel after all: better the lane follower than this.
            self.get_logger().error('Tunnel: the reckoning has left the tunnel, giving up.')
            self.phase = 'done'
            return
        twist = Twist()
        if self.phase == 'drive':
            if abs(x - self.exit_x) < 0.08 and y > self.left - 0.2:
                self.get_logger().info('Tunnel: at the exit, leaving.')
                self.phase = 'leave'
                self.leave_from = self.travelled
            elif self.path is not None and len(self.path) > 1:
                distance = np.hypot(self.path[:, 0] - x, self.path[:, 1] - y)
                index = int(np.argmax(distance >= self.lookahead))
                if distance[index] < self.lookahead:
                    index = len(self.path) - 1
                twist = self.steer_to(self.path[index])
        if self.phase == 'leave':
            # Straight out through the exit, square to the wall.
            twist = self.steer_to((self.exit_x, y + 0.3))
            out = y > self.left + 0.15
            if (out and now - self.lane_time < 0.3) \
                    or self.travelled - self.leave_from > self.leave_distance:
                self.get_logger().info('Tunnel: out, done.')
                self.phase = 'done'
                return
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
