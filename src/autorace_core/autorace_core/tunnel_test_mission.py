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

"""Tunnel test: the laser alone, from inside the entrance to the far diagonal corner.

A test of the tunnel's driving on its own, without the walls' reckoning of
tunnel_mission. /autorace/run_active true starts it. The goal is the far
diagonal corner, the exit, `goal_x` ahead and `goal_y` to the left of the
entrance's middle (1.8, 1.8 m: 45 degrees, 2.5 m). Where the entrance is:

  start_on: run_active  the robot is put with its axle in the doorway, pointing
                        in, and where it stands at the start is the entrance.
  start_on: tunnel      the robot is driven in (approach_speed straight on, or
                        by hand / teleop with approach_speed 0), and it is in
                        the tunnel when the forward camera is dark (mean
                        brightness at most dark_threshold) and a wall runs
                        along its right (the tunnel's entrance is 0.18 m from
                        the right wall, so that wall starts at the doorway), on
                        entry_scans scans running. That wall then also says
                        where the entrance is: its distance how far the robot
                        is off the entrance's middle, its direction how askew
                        the robot came in, its first return how far in it is.

The straight line from the start to the goal is the route. While the line is
clear of everything by the robot's radius the robot drives along it. Where
something stands on it, an A* search across a grid of the laser's returns
finds the shortest way round, from the robot back onto the line just past the
obstacle; its corners are cut wherever the straight line between two of them
is clear too, so the detour is the shortest the grid allows rather than a
staircase of 45 degree steps. Past the obstacle the robot is on the line again.

A* rather than RRT*: the tunnel is small (2 cm cells, some 16000 of them), so
the search is exact and quick, and the same obstacles give the same path scan
after scan; RRT* samples at random, so its path, and the robot's steering,
would change from one plan to the next, and it needs many samples to come near
the shortest path at all.

The returns are counted in the grid in the start's frame, carried by the
odometry (whose heading is the gyro's), and fade so that a return seen once,
or something that has gone, does not block the way for long. The command goes
to cmd_vel_topic: /cmd_vel (no mux, the test on its own) or /cmd_vel/mission
(through cmd_vel_mux, over the lane controller: lane following in and out).
estop.py stops this node with the missions.

    wait    until /autorace/run_active is true
    armed   (start_on: tunnel) on the way in, until dark and a wall on the right
    line    along the route
    detour  round an obstacle on the route, back onto it (`blocked` when no way is
            found: it waits and looks again)
    align   at the goal, in the exit: turn on the spot to face out (exit_heading by
            the odometry), then until the wall on the right, the far wall running
            on past the exit, lies parallel to the heading (measured off the laser)
    leave   straight out along that wall, at the distance it was, until the camera
            is light again (bright_threshold; start_on tunnel) or leave_distance
    done    /tunnel_test/done true, and nothing more on cmd_vel_topic: with
            cmd_vel_topic /cmd_vel/mission and lane_drive.launch.py up, the lane
            controller, which drove the robot in, drives on from here
"""

import math

import cv2
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from autorace_core.tunnel_mission import search
from autorace_core.tunnel_mission import wrap


class TunnelTestMission(Node):

    def __init__(self, **kwargs):
        super().__init__('tunnel_test_mission', **kwargs)

        # The goal from the start: ahead, to the left (the tunnel's exit seen from its entrance).
        self.declare_parameter('goal_x', 1.8)
        self.declare_parameter('goal_y', 1.8)
        self.declare_parameter('goal_tolerance', 0.06)
        # Where the entrance is taken from (see the module's notes): run_active or tunnel.
        self.declare_parameter('start_on', 'run_active')
        self.declare_parameter('camera_topic', '/camera/image_raw/compressed')
        self.declare_parameter('dark_threshold', 40.0)   # mean grey 0..255 (TUNE: logged while armed)
        # A wall along the right: returns between right_wall_min and right_wall_max to the right,
        # from right_wall_behind behind the axle to right_wall_ahead ahead, at least
        # right_wall_points of them over right_wall_span along a straight line.
        self.declare_parameter('right_wall_min', 0.05)
        # 0.45: the entrance's middle is 0.18 m from it, so up to 0.27 m left of the middle
        # still sees it (0.32 until 2026-10-25: 0.14 m).
        self.declare_parameter('right_wall_max', 0.45)
        self.declare_parameter('right_wall_behind', 0.40)
        self.declare_parameter('right_wall_ahead', 0.40)
        # 20 points over 0.30 m: longer than an obstacle (a 0.15 m box is 0.21 m corner to corner),
        # which close by gives as many points as a wall (8 and 0.15 m until 2026-10-25).
        self.declare_parameter('right_wall_points', 20)
        self.declare_parameter('right_wall_span', 0.30)
        self.declare_parameter('right_wall_max_angle', 0.5)  # rad off the heading
        self.declare_parameter('entry_scans', 2)
        self.declare_parameter('entrance_right', 0.18)   # the entrance's middle from the right wall
        self.declare_parameter('approach_speed', 0.0)    # m/s straight on while armed (0: driven in)
        # /cmd_vel on its own; /cmd_vel/mission under cmd_vel_mux with the lane controller.
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        # The exit (see the module's notes): facing out is this heading from the entrance's.
        self.declare_parameter('exit_heading', math.pi / 2)
        self.declare_parameter('align_rate', 0.6)        # rad/s at most
        self.declare_parameter('align_tolerance', 0.035)  # rad (2 deg) off the wall ...
        self.declare_parameter('align_scans', 3)          # ... on this many scans running
        self.declare_parameter('align_timeout', 8.0)      # s: then out on the odometry's heading
        self.declare_parameter('leave_speed', 0.10)
        self.declare_parameter('wall_gain', 2.0)          # rad/s per rad off parallel
        self.declare_parameter('side_gain', 3.0)          # rad/s per m off the wall's distance
        self.declare_parameter('bright_threshold', 80.0)  # mean grey: out of the tunnel
        self.declare_parameter('leave_min', 0.20)         # m out before the light counts
        self.declare_parameter('leave_distance', 0.60)    # m: out, light or not
        # The grid, in the start's frame: x ahead, y to the left.
        self.declare_parameter('resolution', 0.02)
        self.declare_parameter('plan_resolution', 0.04)   # the A* grid: coarser, quicker
        self.declare_parameter('grid_x', [-0.4, 2.4])
        self.declare_parameter('grid_y', [-0.6, 2.4])
        self.declare_parameter('max_range', 2.5)
        self.declare_parameter('lidar_offset', -0.032)   # the laser behind the axle, m
        self.declare_parameter('hit_fade', 0.9)          # per scan
        self.declare_parameter('hit_threshold', 1.5)     # faded count that makes a cell occupied
        # The body as a circle about the axle (as tunnel_mission): no closer than this, and
        # further up to comfort_clearance where there is room.
        self.declare_parameter('robot_radius', 0.13)
        self.declare_parameter('comfort_clearance', 0.22)
        self.declare_parameter('comfort_weight', 3.0)
        # Back onto the route this far past the end of what blocks it.
        self.declare_parameter('rejoin_margin', 0.10)
        self.declare_parameter('replan_period', 0.3)     # s
        self.declare_parameter('speed', 0.12)
        self.declare_parameter('min_speed', 0.04)
        self.declare_parameter('lookahead', 0.18)
        self.declare_parameter('cornering_rate', 0.8)
        self.declare_parameter('max_angular', 1.2)
        # Turn on the spot first when the way lies further off the heading than this.
        self.declare_parameter('turn_in_place_angle', 1.0)
        # Anything this close ahead (from the axle) stops the robot whatever the plan says.
        self.declare_parameter('stop_distance', 0.16)
        self.declare_parameter('stop_half_width', 0.10)

        value = lambda name: self.get_parameter(name).value
        self.goal = np.array([value('goal_x'), value('goal_y')])
        self.goal_tolerance = value('goal_tolerance')
        self.start_on = value('start_on')
        self.dark_threshold = value('dark_threshold')
        self.right_wall = (value('right_wall_min'), value('right_wall_max'),
                           value('right_wall_behind'), value('right_wall_ahead'))
        self.right_wall_points = value('right_wall_points')
        self.right_wall_span = value('right_wall_span')
        self.right_wall_max_angle = value('right_wall_max_angle')
        self.entry_scans = value('entry_scans')
        self.entrance_right = value('entrance_right')
        self.approach_speed = value('approach_speed')
        self.exit_heading = value('exit_heading')
        self.align_rate = value('align_rate')
        self.align_tolerance = value('align_tolerance')
        self.align_scans = value('align_scans')
        self.align_timeout = value('align_timeout')
        self.leave_speed = value('leave_speed')
        self.wall_gain = value('wall_gain')
        self.side_gain = value('side_gain')
        self.bright_threshold = value('bright_threshold')
        self.leave_min = value('leave_min')
        self.leave_distance = value('leave_distance')
        self.res = value('resolution')
        self.plan_factor = max(1, int(round(value('plan_resolution') / self.res)))
        self.x_0, x_1 = value('grid_x')
        self.y_0, y_1 = value('grid_y')
        self.cols = int(round((x_1 - self.x_0) / self.res))
        self.rows = int(round((y_1 - self.y_0) / self.res))
        self.max_range = value('max_range')
        self.lidar_offset = value('lidar_offset')
        self.hit_fade = value('hit_fade')
        self.hit_threshold = value('hit_threshold')
        self.robot_radius = value('robot_radius')
        self.comfort = value('comfort_clearance')
        self.comfort_weight = value('comfort_weight')
        self.rejoin_margin = value('rejoin_margin')
        self.replan_period = value('replan_period')
        self.speed = value('speed')
        self.min_speed = value('min_speed')
        self.lookahead = value('lookahead')
        self.cornering_rate = value('cornering_rate')
        self.max_angular = value('max_angular')
        self.turn_in_place_angle = value('turn_in_place_angle')
        self.stop_distance = value('stop_distance')
        self.stop_half_width = value('stop_half_width')

        self.hits = np.zeros((self.rows, self.cols), np.float32)
        self.clearance = None          # m from the nearest occupied cell, per cell
        self.odom = None               # (x, y, yaw) in the odometry's frame
        self.start = None              # the odometry pose at the start
        self.pose = None               # (x, y, yaw) in the start's frame
        self.points = np.zeros((0, 2))  # the last scan, robot frame
        self.state = 'wait'
        self.path = None               # waypoints (start frame), the robot first
        self.planned_time = -math.inf
        self.brightness = None         # the forward camera's mean grey
        self.scan_time = None          # when the last scan came
        self.phase_since = 0.0         # when align or leave began
        self.aligned_count = 0
        self.leave_from = None         # where leave began (start frame)
        self.leave_wall = None         # the right wall's distance to keep while leaving
        self.entry_count = 0

        self.create_subscription(Odometry, '/odom', self.callback_odom, 10)
        self.create_subscription(LaserScan, '/scan', self.callback_scan, qos_profile_sensor_data)
        self.create_subscription(Bool, '/autorace/run_active', self.callback_run_active, 1)
        if self.start_on == 'tunnel':
            self.create_subscription(CompressedImage, value('camera_topic'), self.callback_image,
                                     qos_profile_sensor_data)
        # Across the Wi-Fi to the robot, as custom_teleop: reliable, only the newest.
        self.pub_cmd_vel = self.create_publisher(Twist, value('cmd_vel_topic'), QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE))
        # For RViz (tools/tunnel_test.rviz), in the odometry's frame: the path driven for, the
        # route, the goal, and the grid (100 seen, 50 within the robot's radius of it).
        self.pub_path = self.create_publisher(Path, '/tunnel_test/path', 1)
        self.pub_route = self.create_publisher(Path, '/tunnel_test/route', 1)
        self.pub_goal = self.create_publisher(PoseStamped, '/tunnel_test/goal', 1)
        self.pub_grid = self.create_publisher(OccupancyGrid, '/tunnel_test/grid', 1)
        self.pub_done = self.create_publisher(Bool, '/tunnel_test/done', 1)
        self.grid_time = -math.inf
        self.create_timer(0.05, self.update)
        self.get_logger().info(
            f'Tunnel test: goal {self.goal[0]:.2f} m ahead, {self.goal[1]:.2f} m left; '
            f'start on {self.start_on}; waiting for /autorace/run_active.')

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    # ---- inputs ----

    def callback_run_active(self, msg):
        if msg.data and self.state == 'wait' and self.odom is not None:
            self.hits[:] = 0.0
            self.start = self.pose = None
            if self.start_on == 'tunnel':
                self.entry_count = 0
                self.state = 'armed'
                self.get_logger().info('Tunnel test: armed, watching for the tunnel.')
            else:
                self.begin(self.odom, 'started here')
        elif not msg.data and self.state not in ('wait', 'done'):
            self.state = 'wait'
            self.pub_cmd_vel.publish(Twist())
            self.get_logger().info('Tunnel test: stopped (run_active false).')

    def callback_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        self.callback_odom_pose()

    def callback_scan(self, msg):
        self.scan_time = self.now()
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        seen = np.isfinite(ranges) & (ranges > msg.range_min + 1e-3) & (ranges < self.max_range)
        self.points = np.stack([self.lidar_offset + ranges[seen] * np.cos(angles[seen]),
                                ranges[seen] * np.sin(angles[seen])], axis=1)
        if self.state == 'armed':
            self.watch_entry()
            return
        if self.pose is None or self.state in ('wait', 'done'):
            return
        x, y, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        world = np.stack([x + c * self.points[:, 0] - s * self.points[:, 1],
                          y + s * self.points[:, 0] + c * self.points[:, 1]], axis=1)
        column = np.int_(np.round((world[:, 0] - self.x_0) / self.res))
        row = np.int_(np.round((world[:, 1] - self.y_0) / self.res))
        keep = (column >= 0) & (column < self.cols) & (row >= 0) & (row < self.rows)
        self.hits *= self.hit_fade
        np.add.at(self.hits, (row[keep], column[keep]), 1.0)
        free = (self.hits < self.hit_threshold).astype(np.uint8)
        self.clearance = cv2.distanceTransform(free, cv2.DIST_L2, 5) * self.res
        if self.now() - self.grid_time > 0.5:
            self.grid_time = self.now()
            self.publish_grid()

    def callback_image(self, msg):
        grey = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_REDUCED_GRAYSCALE_8)
        if grey is not None:
            self.brightness = float(grey.mean())

    def begin(self, start, why):
        """Drive from here on, with `start` (odometry pose) as the entrance's middle."""
        self.start = start
        self.callback_odom_pose()
        self.state = 'line'
        self.planned_time = -math.inf
        self.get_logger().info(f'Tunnel test: {why}; driving for the goal.')
        self.publish_route()

    def right_wall_fit(self):
        """The wall along the right as (distance, angle, first): its distance from the axle, its
        direction against the heading (rad, + to the left) and how far along it, from the axle,
        its first return lies (- behind). None if no straight wall is there."""
        near, far, behind, ahead = self.right_wall
        p = self.points
        band = p[(p[:, 1] < -near) & (p[:, 1] > -far) & (p[:, 0] > -behind) & (p[:, 0] < ahead)]
        if len(band) < self.right_wall_points:
            return None
        vx, vy, x0, y0 = cv2.fitLine(band.astype(np.float32), cv2.DIST_HUBER, 0, 0.01, 0.01).ravel()
        if vx < 0:
            vx, vy = -vx, -vy
        angle = math.atan2(vy, vx)
        along = (band[:, 0] - x0) * vx + (band[:, 1] - y0) * vy
        across = np.abs(-(band[:, 0] - x0) * vy + (band[:, 1] - y0) * vx)
        if (abs(angle) > self.right_wall_max_angle or np.ptp(along) < self.right_wall_span
                or np.median(across) > 0.02):
            return None
        distance = abs(-(0.0 - x0) * vy + (0.0 - y0) * vx)
        first = float(along.min() + (0.0 - x0) * -vx + (0.0 - y0) * -vy)
        return float(distance), float(angle), first

    def watch_entry(self):
        dark = self.brightness is not None and self.brightness <= self.dark_threshold
        wall = self.right_wall_fit()
        self.get_logger().info(
            'Tunnel test: armed: brightness %s (dark at %.0f), right wall %s'
            % ('none' if self.brightness is None else '%.0f' % self.brightness, self.dark_threshold,
               'none' if wall is None else '%.2f m, %+.0f deg' % (wall[0], math.degrees(wall[1]))),
            throttle_duration_sec=1.0)
        self.entry_count = self.entry_count + 1 if dark and wall is not None else 0
        if self.entry_count < self.entry_scans or self.odom is None:
            return
        distance, angle, first = wall
        # The robot in the tunnel's frame (x along the right wall from the doorway, y to the
        # left from the entrance's middle): the wall runs along x at y = -entrance_right.
        xt = float(np.clip(-first, -0.40, 0.40))   # - : the axle still short of the doorway
        yt = distance - self.entrance_right
        at = -angle
        xo, yo, ao = self.odom
        a0 = ao - at
        c, s = math.cos(a0), math.sin(a0)
        start = (xo - (c * xt - s * yt), yo - (s * xt + c * yt), a0)
        self.pub_cmd_vel.publish(Twist())
        self.begin(start, 'in the tunnel (dark, right wall %.2f m, %+.0f deg): %.2f m in, '
                          '%+.2f m off the middle' % (distance, math.degrees(angle), xt, yt))

    def callback_odom_pose(self):
        if self.start is None or self.odom is None:
            return
        x0, y0, a0 = self.start
        dx, dy = self.odom[0] - x0, self.odom[1] - y0
        c, s = math.cos(a0), math.sin(a0)
        self.pose = (c * dx + s * dy, -s * dx + c * dy, wrap(self.odom[2] - a0))

    # ---- the grid ----

    def cell(self, p, res=None):
        res = res or self.res
        return (int(round((p[1] - self.y_0) / res)), int(round((p[0] - self.x_0) / res)))

    def point(self, cell, res=None):
        res = res or self.res
        return np.array([self.x_0 + cell[1] * res, self.y_0 + cell[0] * res])

    def clearance_at(self, points):
        """Clearance at each of an N x 2 array of points; off the grid nothing was seen: inf."""
        points = np.atleast_2d(points)
        row = np.int_(np.round((points[:, 1] - self.y_0) / self.res))
        column = np.int_(np.round((points[:, 0] - self.x_0) / self.res))
        inside = (row >= 0) & (row < self.rows) & (column >= 0) & (column < self.cols)
        out = np.full(len(points), np.inf)
        out[inside] = self.clearance[row[inside], column[inside]]
        return out

    def segment_clear(self, a, b, margin):
        n = max(2, int(np.linalg.norm(b - a) / (self.res / 2)))
        t = np.linspace(0.0, 1.0, n)[:, None]
        return bool(np.all(self.clearance_at(a + (b - a) * t) >= margin))

    def free_near(self, p, margin, radius=0.10):
        """p, or the nearest point within radius that keeps margin from everything; None if none."""
        if self.clearance_at(p)[0] >= margin:
            return p
        steps = np.arange(-radius, radius + 1e-9, self.res)
        dx, dy = np.meshgrid(steps, steps)
        offsets = np.stack([dx.ravel(), dy.ravel()], 1)
        offsets = offsets[np.linalg.norm(offsets, axis=1) <= radius]
        offsets = offsets[np.argsort(np.linalg.norm(offsets, axis=1))]
        ok = self.clearance_at(p + offsets) >= margin
        return p + offsets[np.argmax(ok)] if ok.any() else None

    # ---- planning ----

    def plan(self, here):
        """Waypoints from here: straight along the route, or A* round what blocks it."""
        margin = self.robot_radius
        length = float(np.linalg.norm(self.goal))
        along = self.goal / length
        goal = self.free_near(self.goal, margin)
        if goal is None:
            return None, 'blocked'
        # Where on the route the robot is, and the route's rest from there.
        s_here = float(np.clip(np.dot(here, along), 0.0, length))
        samples = np.arange(s_here, length, self.res)
        clear = self.clearance_at(along * samples[:, None]) >= margin
        if self.segment_clear(here, goal, margin) and clear.all():
            return [here, goal], 'line'
        # The first stretch of the route ahead that is blocked, and where it is clear again.
        target = goal
        blocked = np.nonzero(~clear)[0]
        if len(blocked):
            after = np.nonzero(clear[blocked[0]:])[0]
            if len(after):
                s_free = samples[blocked[0] + after[0]] + self.rejoin_margin
                if s_free < length - self.rejoin_margin:
                    target = self.free_near(along * s_free, margin)
                    target = goal if target is None else target
        # A* on cells of plan_resolution, each as clear as the least clear fine cell in it.
        k = self.plan_factor
        rows, cols = self.rows // k, self.cols // k
        coarse = self.clearance[:rows * k, :cols * k].reshape(rows, k, cols, k).min(axis=(1, 3))
        cost = np.where(coarse < margin, np.inf,
                        1.0 + self.comfort_weight
                        * np.clip((self.comfort - coarse) / (self.comfort - margin), 0.0, 1.0))
        res = self.res * k
        start, end = self.cell(here, res), self.cell(target, res)
        if not (0 <= start[0] < rows and 0 <= start[1] < cols
                and 0 <= end[0] < rows and 0 <= end[1] < cols):
            return None, 'blocked'
        cost[start] = 1.0            # wherever the robot is, it may leave it
        cost[end] = min(cost[end], 1.0 + self.comfort_weight)
        cells = search(cost, start, end)
        if cells is None:
            return None, 'blocked'
        points = [here] + [self.point(c, res) for c in cells[1:-1]] + [target]
        path = self.shortcut(points, margin)
        if target is not goal:
            path.append(goal)
        return path, 'detour'

    def shortcut(self, points, margin):
        """The corners of a grid path cut wherever the straight line between them is clear."""
        out = [points[0]]
        i = 0
        while i < len(points) - 1:
            j = len(points) - 1
            while j > i + 1 and not self.segment_clear(points[i], points[j], margin):
                j -= 1
            out.append(points[j])
            i = j
        return out

    # ---- driving ----

    def carrot(self, here):
        """The point `lookahead` along the path from the robot."""
        left = self.lookahead
        for a, b in zip([here] + self.path[1:-1], self.path[1:]):
            length = float(np.linalg.norm(b - a))
            if length >= left:
                return a + (b - a) * (left / max(length, 1e-6))
            left -= length
        return self.path[-1]

    def update(self):
        if self.state not in ('wait', 'done') and (self.scan_time is None
                                                   or self.now() - self.scan_time > 1.0):
            # Nothing to see by (2026-10-27: a second lidar driver on the port starved ld_lidar,
            # and the test waited silently in the tunnel).
            self.get_logger().warn('Tunnel test: no /scan for over a second.',
                                   throttle_duration_sec=2.0)
        if self.state == 'armed':
            if self.approach_speed > 0.0:
                twist = Twist()
                twist.linear.x = 0.0 if self.something_ahead() else self.approach_speed
                self.pub_cmd_vel.publish(twist)
            return
        if self.state in ('wait', 'done') or self.pose is None or self.clearance is None:
            return
        x, y, yaw = self.pose
        here = np.array([x, y])
        if self.state == 'align':
            self.align(yaw)
            return
        if self.state == 'leave':
            self.leave(here)
            return
        if np.linalg.norm(self.goal - here) < self.goal_tolerance:
            self.state = 'align'
            self.phase_since = self.now()
            self.aligned_count = 0
            self.pub_cmd_vel.publish(Twist())
            self.get_logger().info(f'Tunnel test: at the goal ({x:.2f}, {y:.2f}); turning to face out.')
            return

        now = self.now()
        if now - self.planned_time > self.replan_period:
            self.planned_time = now
            t0 = now
            path, state = self.plan(here)
            if state != self.state:
                self.get_logger().info(
                    f'Tunnel test: {self.state} -> {state} at ({x:.2f}, {y:.2f}), '
                    f'plan {1000 * (self.now() - t0):.0f} ms')
            self.state, self.path = state, path
            self.publish_path()
        if self.path is None:
            self.pub_cmd_vel.publish(Twist())
            self.get_logger().warn('Tunnel test: no way found; waiting.', throttle_duration_sec=2.0)
            return

        target = self.carrot(here)
        dx, dy = target[0] - x, target[1] - y
        ahead = math.cos(yaw) * dx + math.sin(yaw) * dy
        left = -math.sin(yaw) * dx + math.cos(yaw) * dy
        twist = Twist()
        bearing = math.atan2(left, ahead)
        if abs(bearing) > self.turn_in_place_angle:
            twist.angular.z = math.copysign(min(self.max_angular, 1.5 * abs(bearing)), bearing)
        else:
            curvature = 2.0 * left / max(1e-6, ahead * ahead + left * left)
            speed = self.speed
            if abs(curvature) * speed > self.cornering_rate:
                speed = max(self.min_speed, self.cornering_rate / abs(curvature))
            # Slow down onto the goal.
            speed = min(speed, max(self.min_speed, float(np.linalg.norm(self.goal - here))))
            twist.linear.x = speed
            twist.angular.z = max(-self.max_angular, min(self.max_angular, speed * curvature))
        if twist.linear.x > 0.0 and self.something_ahead():
            twist.linear.x = 0.0
            self.get_logger().warn('Tunnel test: something right ahead, holding.',
                                   throttle_duration_sec=1.0)
        self.pub_cmd_vel.publish(twist)

    def align(self, yaw):
        """Face out of the exit, parallel to the wall on the right."""
        twist = Twist()
        off = wrap(self.exit_heading - yaw)
        wall = self.right_wall_fit()
        if abs(off) > 0.3 or wall is None:
            # Round towards the way out by the odometry until the wall is in view.
            twist.angular.z = math.copysign(min(self.align_rate, 1.5 * abs(off)), off)
            self.aligned_count = 0
            if abs(off) < self.align_tolerance and wall is None:
                self.get_logger().warn('Tunnel test: no wall on the right at the exit.',
                                       throttle_duration_sec=1.0)
        else:
            angle = wall[1]           # the wall's direction against the heading, + left
            if abs(angle) < self.align_tolerance:
                self.aligned_count += 1
            else:
                self.aligned_count = 0
                twist.angular.z = math.copysign(min(self.align_rate, max(0.15, 1.5 * abs(angle))),
                                                angle)
        timed_out = self.now() - self.phase_since > self.align_timeout
        if self.aligned_count >= self.align_scans or timed_out:
            self.state = 'leave'
            self.phase_since = self.now()
            self.leave_from = np.array(self.pose[:2])
            self.leave_wall = None if wall is None else wall[0]
            self.pub_cmd_vel.publish(Twist())
            self.get_logger().info(
                'Tunnel test: %s; leaving along the right wall (%s).'
                % ('aligned' if not timed_out else 'align timed out, out on the odometry',
                   'none' if wall is None else '%.2f m, %+.1f deg' % (wall[0], math.degrees(wall[1]))))
            return
        self.pub_cmd_vel.publish(twist)

    def leave(self, here):
        """Straight out along the right wall, until it is light or far enough."""
        out = float(np.linalg.norm(here - self.leave_from))
        light = (self.start_on == 'tunnel' and self.brightness is not None
                 and self.brightness >= self.bright_threshold)
        if out >= self.leave_distance or (out >= self.leave_min and light):
            self.state = 'done'
            self.pub_cmd_vel.publish(Twist())
            self.pub_done.publish(Bool(data=True))
            self.get_logger().info(
                f'Tunnel test: out ({out:.2f} m, {"light again" if light else "by distance"}); done.')
            return
        twist = Twist()
        twist.linear.x = self.leave_speed
        wall = self.right_wall_fit()
        if wall is not None:
            distance, angle, _ = wall
            keep = self.leave_wall if self.leave_wall is not None else distance
            twist.angular.z = self.wall_gain * angle + self.side_gain * (keep - distance)
            twist.angular.z = max(-self.max_angular, min(self.max_angular, twist.angular.z))
        if self.something_ahead():
            twist.linear.x = 0.0
        self.pub_cmd_vel.publish(twist)

    def something_ahead(self):
        p = self.points
        return bool(np.any((p[:, 0] > 0.0) & (p[:, 0] < self.stop_distance)
                           & (np.abs(p[:, 1]) < self.stop_half_width)))

    def to_odom(self, p):
        x0, y0, a0 = self.start
        c, s = math.cos(a0), math.sin(a0)
        return x0 + c * p[0] - s * p[1], y0 + s * p[0] + c * p[1]

    def stamped(self, p, header):
        pose = PoseStamped()
        pose.header = header
        pose.pose.position.x, pose.pose.position.y = self.to_odom(p)
        a0 = self.start[2] + math.atan2(self.goal[1], self.goal[0])
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(a0 / 2), math.cos(a0 / 2)
        return pose

    def header(self):
        msg = Path()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        return msg

    def publish_path(self):
        if self.path is None or self.start is None:
            return
        msg = self.header()
        msg.poses = [self.stamped(p, msg.header) for p in self.path]
        self.pub_path.publish(msg)

    def publish_route(self):
        msg = self.header()
        msg.poses = [self.stamped(p, msg.header) for p in (np.zeros(2), self.goal)]
        self.pub_route.publish(msg)
        self.pub_goal.publish(self.stamped(self.goal, msg.header))

    def publish_grid(self):
        msg = OccupancyGrid()
        msg.header = self.header().header
        msg.info.resolution = float(self.res)
        msg.info.width, msg.info.height = self.cols, self.rows
        msg.info.origin.position.x, msg.info.origin.position.y = self.to_odom((self.x_0, self.y_0))
        a0 = self.start[2]
        msg.info.origin.orientation.z, msg.info.origin.orientation.w = math.sin(a0 / 2), math.cos(a0 / 2)
        grid = np.zeros((self.rows, self.cols), np.int8)
        grid[self.clearance < self.robot_radius] = 50
        grid[self.hits >= self.hit_threshold] = 100
        msg.data = grid.ravel().tolist()
        self.pub_grid.publish(msg)
        self.publish_route()

    def shut_down(self):
        # After Ctrl+C the context may be gone already; the Pi's watchdog stops the wheels then.
        if rclpy.ok():
            self.pub_cmd_vel.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = TunnelTestMission()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shut_down()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
