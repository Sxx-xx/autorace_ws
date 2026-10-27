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

Verified on the robot 2026-10-27 (fc45994), lane following in and out: front wall 0.57 m
ahead -> in; dark (right wall 0.11 m) -> tunnel, origin 7 cm right of the middle; exit seen
at (1.56, 1.74) before the goal (the tunnel is smaller than 1.8 m), re-measured to (1.49,
1.76); there 2.4 s later, faced out, out 0.30 m and light (41 over 40): back to the lane,
31 s from the front wall.

A test of the tunnel's driving on its own, without the walls' reckoning of
tunnel_mission. /autorace/run_active true starts it. The goal is the far
diagonal corner, the exit, `goal_x` ahead and `goal_y` to the left of the
entrance's middle (1.8, 1.8 m: 45 degrees, 2.5 m). Where the entrance is:

  start_on: run_active  the robot is put with its axle in the doorway, pointing
                        in, and where it stands at the start is the entrance.
  start_on: tunnel      the lane follower drives the robot up to the tunnel.
                        The tunnel's front wall, across the way ahead and to
                        the left (the entrance is at its right end, straight on
                        along the lane), within front_trigger ahead hands the
                        driving to this node, which goes in square to that
                        wall through the middle of the entrance (aiming
                        approach_beyond past the doorway's middle, opening_half
                        right of the wall's end) at approach_speed, holding
                        that direction once the wall is out of view; with
                        front_wall_trigger false it goes straight on at once,
                        or with approach_speed 0 too it is driven in by hand. It is in
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
    armed   (start_on: tunnel) the lane follower drives; until the front wall, ahead left
    approach square to it through the entrance, until dark and a wall on the right (or
            `no entry` after approach_max m: it stops there)
    line    along the route
    detour  round an obstacle on the route, back onto it (`blocked` when no way is
            found: it waits and looks again)
            On the way, an opening between two long walls in the tunnel's left wall (the
            exit) seen close by takes the place of the goal: the robot heads (A*) for
            just inside it, and once there faces out square to that wall.
    align   at the goal: turn on the spot exit_turn to the left of the route (45 deg: from
            the 45 deg route to facing out of the exit), by the odometry
    leave   out as in: square to the wall across the way, which is the tunnel's left wall
            now, through the middle of the exit at its right end (approach_speed), until
            it is light again (brightness over dark_threshold, the same as going in) or
            leave_distance
    done    /tunnel_test/done and /control/lane_resume true (the lane controller starts
            searching for the lane afresh), and nothing more on cmd_vel_topic: with
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
        self.declare_parameter('goal_tolerance', 0.08)
        # Near the goal and getting no nearer (by arrive_progress in arrive_stall s within
        # arrive_radius): there, as near as it gets (a goal against a wall). On the robot it
        # went back and forth at the end of the tunnel for a minute and a half (2026-10-27).
        self.declare_parameter('arrive_radius', 0.25)
        self.declare_parameter('arrive_progress', 0.02)
        self.declare_parameter('arrive_stall', 2.5)
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
        # The approach: the front wall, a straight wall across the way (square to the heading
        # within front_wall_angle), between front_wall_left of it and front_wall_ahead ahead,
        # at least front_wall_points returns over front_wall_span, within front_trigger ahead,
        # on entry_scans scans running. The entrance is at its right end, opening_half wide
        # to either side of its middle.
        self.declare_parameter('front_wall_trigger', True)
        self.declare_parameter('front_wall_ahead', 0.90)
        self.declare_parameter('front_wall_left', 1.20)
        self.declare_parameter('front_wall_points', 20)
        self.declare_parameter('front_wall_span', 0.40)
        self.declare_parameter('front_wall_angle', 0.50)   # rad off square
        self.declare_parameter('front_trigger', 0.60)      # m ahead
        self.declare_parameter('opening_half', 0.17)
        self.declare_parameter('approach_beyond', 0.30)    # m past the doorway to aim at
        self.declare_parameter('approach_speed', 0.10)   # m/s straight on into the tunnel
        self.declare_parameter('approach_max', 1.2)      # m: no tunnel by then, stop
        self.declare_parameter('heading_gain', 2.0)      # rad/s per rad off the held direction
        # /cmd_vel on its own; /cmd_vel/mission under cmd_vel_mux with the lane controller.
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        # The exit (see the module's notes): facing out is this heading from the entrance's.
        self.declare_parameter('exit_turn', math.pi / 4)   # rad left of the route at the goal
        # The exit seen before the goal: in the left wall (the row of the grid, left of the
        # robot, with at least exit_wall_min of wall in it), an opening exit_min..exit_max wide
        # with exit_side_min of wall before it and a wall at its far end, its middle within
        # exit_range of the robot, on exit_checks plans running. Then the goal is exit_inside
        # short of the opening's middle. A wall seen far off and askew has gaps between its
        # returns that look like openings: hence the range and the repeats.
        self.declare_parameter('exit_detect', True)
        self.declare_parameter('exit_wall_min', 0.50)
        self.declare_parameter('exit_min', 0.25)
        # 0.45: the exit is 0.34 m; an obstacle hiding a stretch of the wall next to it made a
        # 0.56 m 'exit' whose middle was 0.11 m off (tunnel_sim two, 2026-10-27).
        self.declare_parameter('exit_max', 0.45)
        self.declare_parameter('exit_lock', 0.40)   # m from it: stop measuring it again
        self.declare_parameter('exit_side_min', 0.30)
        self.declare_parameter('exit_range', 1.20)
        self.declare_parameter('exit_checks', 3)
        self.declare_parameter('exit_inside', 0.15)
        self.declare_parameter('align_rate', 0.6)          # rad/s at most
        self.declare_parameter('align_tolerance', 0.05)    # rad (3 deg)
        self.declare_parameter('align_timeout', 6.0)       # s: then out whatever the heading
        # The camera is ahead of the axle and sees the light first: at 0.15 m the axle was still
        # in the doorway when the lane controller took over.
        self.declare_parameter('leave_min', 0.30)          # m out before the light counts
        self.declare_parameter('leave_distance', 1.0)      # m: back to the lane, light or not
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
        # Anything this close ahead (from the axle) stops the robot whatever the plan says. Inside
        # robot_radius: the plan passes things 0.13 m off, and 0.16 x 0.10 stopped the robot on
        # such a pass, it turned for the plan, went on and stopped again (2026-10-27).
        self.declare_parameter('stop_distance', 0.12)
        self.declare_parameter('stop_half_width', 0.08)
        # Held that long by the guard: there if near the goal, else plan again at once.
        self.declare_parameter('hold_timeout', 2.0)

        value = lambda name: self.get_parameter(name).value
        self.goal = np.array([value('goal_x'), value('goal_y')])
        self.goal_tolerance = value('goal_tolerance')
        self.arrive_radius = value('arrive_radius')
        self.arrive_progress = value('arrive_progress')
        self.arrive_stall = value('arrive_stall')
        self.hold_timeout = value('hold_timeout')
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
        self.front_wall_trigger = value('front_wall_trigger')
        self.front_wall_ahead = value('front_wall_ahead')
        self.front_wall_left = value('front_wall_left')
        self.front_wall_points = value('front_wall_points')
        self.front_wall_span = value('front_wall_span')
        self.front_wall_angle = value('front_wall_angle')
        self.front_trigger = value('front_trigger')
        self.opening_half = value('opening_half')
        self.approach_beyond = value('approach_beyond')
        self.approach_max = value('approach_max')
        self.heading_gain = value('heading_gain')
        self.exit_heading = math.atan2(self.goal[1], self.goal[0]) + value('exit_turn')
        self.exit_detect = value('exit_detect')
        self.exit_wall_min = value('exit_wall_min')
        self.exit_size = (value('exit_min'), value('exit_max'))
        self.exit_side_min = value('exit_side_min')
        self.exit_range = value('exit_range')
        self.exit_checks = value('exit_checks')
        self.exit_inside = value('exit_inside')
        self.exit_lock = value('exit_lock')
        self.align_rate = value('align_rate')
        self.align_tolerance = value('align_tolerance')
        self.align_timeout = value('align_timeout')
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
        self.best = (math.inf, 0.0)    # nearest to the goal so far, and when
        self.exit_seen = []            # the exit's middle on the last plans running
        self.exit_found = False
        self.held_since = None         # when the guard began holding
        self.status_time = -math.inf
        self.brightness = None         # the forward camera's mean grey
        self.scan_time = None          # when the last scan came
        self.front_count = 0
        self.approach_from = None      # odometry (x, y) where the approach began
        self.approach_yaw = None       # odometry heading to hold: square to the front wall
        self.approach_target = None    # odometry (x, y) past the doorway's middle
        self.phase_since = 0.0         # when align or leave began
        self.leave_from = None         # where leave began (start frame)
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
        # The lane controller gave up searching for the lane 8 s into the tunnel; this has it
        # search afresh when the driving is handed back.
        self.pub_lane_resume = self.create_publisher(Bool, '/control/lane_resume', 1)
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
            self.exit_seen, self.exit_found = [], False
            self.goal = np.array([self.get_parameter('goal_x').value, self.get_parameter('goal_y').value])
            self.exit_heading = (math.atan2(self.goal[1], self.goal[0])
                                 + self.get_parameter('exit_turn').value)
            if self.start_on == 'tunnel':
                self.entry_count = self.front_count = 0
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
        if self.state in ('armed', 'approach'):
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
        return self.wall_fit(-1.0, self.right_wall, self.right_wall_points, self.right_wall_span)

    def front_wall_fit(self):
        """The front wall across the way ahead, to the left, as (distance ahead, angle off square
        (rad, + : the robot should turn left to face it square), aim): aim is the point
        approach_beyond past the middle of the entrance at the wall's right end, square to the
        wall, in the robot's frame. None if no such wall is there."""
        p = self.points
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
        end = np.array([x0, y0]) + np.array([vx, vy]) * along.min()   # the wall's right end
        middle = end - np.array([vx, vy]) * self.opening_half
        return distance, off, middle + normal * self.approach_beyond

    def wall_fit(self, side, limits, min_points, min_span):
        near, far, behind, ahead = limits
        p = self.points
        y = side * p[:, 1]
        band = p[(y > near) & (y < far) & (p[:, 0] > -behind) & (p[:, 0] < ahead)]
        if len(band) < min_points:
            return None
        vx, vy, x0, y0 = cv2.fitLine(band.astype(np.float32), cv2.DIST_HUBER, 0, 0.01, 0.01).ravel()
        if vx < 0:
            vx, vy = -vx, -vy
        angle = math.atan2(vy, vx)
        along = (band[:, 0] - x0) * vx + (band[:, 1] - y0) * vy
        across = np.abs(-(band[:, 0] - x0) * vy + (band[:, 1] - y0) * vx)
        if (abs(angle) > self.right_wall_max_angle or np.ptp(along) < min_span
                or np.median(across) > 0.02):
            return None
        distance = abs(-(0.0 - x0) * vy + (0.0 - y0) * vx)
        first = float(along.min() + (0.0 - x0) * -vx + (0.0 - y0) * -vy)
        return float(distance), float(angle), first

    def watch_entry(self):
        dark = self.brightness is not None and self.brightness <= self.dark_threshold
        wall = self.right_wall_fit()
        front = self.front_wall_fit()
        show = lambda w: 'none' if w is None else '%.2f m, %+.0f deg' % (w[0], math.degrees(w[1]))
        self.get_logger().info(
            'Tunnel test: %s: brightness %s (dark at %.0f), front wall %s, right wall %s'
            % (self.state, 'none' if self.brightness is None else '%.0f' % self.brightness,
               self.dark_threshold, show(front), show(wall)),
            throttle_duration_sec=1.0)
        if self.state == 'armed' and self.odom is not None:
            near = front is not None and front[0] < self.front_trigger
            self.front_count = self.front_count + 1 if near else 0
            if self.front_count >= self.entry_scans and self.front_wall_trigger:
                self.start_approach(front, 'front wall %.2f m ahead, %+.0f deg off square'
                                    % (front[0], math.degrees(front[1])))
            elif not self.front_wall_trigger and self.approach_speed > 0.0:
                self.start_approach(None, 'straight on')
        elif self.state == 'approach' and front is not None and self.odom is not None:
            self.aim(front)      # while the wall is in view; then what it last said
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

    def start_approach(self, front, why):
        self.state = 'approach'
        self.approach_from = np.array(self.odom[:2])
        self.approach_yaw, self.approach_target = self.odom[2], None
        if front is not None:
            self.aim(front)
        self.get_logger().info(
            f'Tunnel test: {why}; into the tunnel at {self.approach_speed:.2f} m/s.')

    def aim(self, front):
        """Square to the front wall, for the entrance's middle (odometry frame)."""
        _, off, aim = front
        xo, yo, ao = self.odom
        c, s = math.cos(ao), math.sin(ao)
        self.approach_yaw = wrap(ao + off)
        self.approach_target = np.array([xo + c * aim[0] - s * aim[1], yo + s * aim[0] + c * aim[1]])

    def approach(self):
        """Through the entrance's middle, square to the front wall."""
        if self.odom is None:
            return
        gone = float(np.linalg.norm(np.array(self.odom[:2]) - self.approach_from))
        if gone > self.approach_max:
            self.pub_cmd_vel.publish(Twist())
            self.get_logger().warn(
                f'Tunnel test: no entry: {gone:.2f} m on and not dark with a wall on the right '
                '(dark_threshold?); holding.', throttle_duration_sec=2.0)
            return
        self.pub_cmd_vel.publish(self.through())

    def through(self):
        """For approach_target (the opening's middle, past it), then on square to its wall."""
        twist = Twist()
        twist.linear.x = 0.0 if self.something_ahead() else self.approach_speed
        xo, yo, ao = self.odom
        heading = self.approach_yaw
        if self.approach_target is not None:
            dx, dy = self.approach_target[0] - xo, self.approach_target[1] - yo
            if math.cos(ao) * dx + math.sin(ao) * dy > 0.10:
                heading = math.atan2(dy, dx)      # for the aim; square to the wall past it
        steer = self.heading_gain * wrap(heading - ao)
        twist.angular.z = max(-self.max_angular, min(self.max_angular, steer))
        return twist

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
        # A goal against a wall (the tunnel not quite as the goal says): the nearest clear point
        # within arrive_radius, and the robot is there once it gets no nearer.
        goal = self.free_near(self.goal, margin, self.arrive_radius)
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
            return                   # the lane follower (or a hand) drives
        if self.state == 'approach':
            self.approach()
            return
        if self.state in ('wait', 'done') or self.pose is None or self.clearance is None:
            return
        x, y, yaw = self.pose
        here = np.array([x, y])
        if self.state in ('align', 'leave'):
            if self.now() - self.status_time >= 1.0:
                self.status_time = self.now()
                self.get_logger().info(
                    f'Tunnel test: {self.state} at ({x:.2f}, {y:.2f}, {math.degrees(yaw):+.0f} deg),'
                    f' brightness {self.brightness if self.brightness is None else round(self.brightness)}'
                    f' (light over {self.dark_threshold:.0f})')
            if self.state == 'align':
                self.align(yaw)
            else:
                self.leave(here)
            return
        now = self.now()
        to_goal = float(np.linalg.norm(self.goal - here))
        if to_goal < self.best[0] - self.arrive_progress:
            self.best = (to_goal, now)
        stalled = to_goal < self.arrive_radius and now - self.best[1] > self.arrive_stall
        held = self.held_since is not None and now - self.held_since > self.hold_timeout
        if to_goal < self.goal_tolerance or stalled or (held and to_goal < self.arrive_radius):
            self.state = 'align'
            self.phase_since = now
            self.held_since = None
            self.pub_cmd_vel.publish(Twist())
            why = 'at the goal' if to_goal < self.goal_tolerance else (
                'as near the goal as it gets (%s)' % ('held' if held else 'no nearer'))
            self.get_logger().info(f'Tunnel test: {why} ({x:.2f}, {y:.2f}, {to_goal:.2f} m off);'
                                   ' turning to face out.')
            return
        if held:
            self.planned_time = -math.inf      # held away from the goal: plan again now
            self.held_since = None

        if now - self.planned_time > self.replan_period:
            self.planned_time = now
            if self.exit_detect and (not self.exit_found
                                     or np.linalg.norm(self.goal - here) > self.exit_lock):
                self.watch_exit(here)      # and again on the way, as more of the wall shows
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
            self.held_since = self.held_since or now
            self.get_logger().warn('Tunnel test: something right ahead, holding.',
                                   throttle_duration_sec=1.0)
        elif twist.linear.x > 0.0:
            self.held_since = None
        self.pub_cmd_vel.publish(twist)
        if now - self.status_time >= 1.0:
            # Once a second, so a run can be drawn again from the log.
            self.status_time = now
            self.get_logger().info(
                f'Tunnel test: {self.state} at ({x:.2f}, {y:.2f}, {math.degrees(yaw):+.0f} deg),'
                f' goal {to_goal:.2f} m, cmd {twist.linear.x:.2f} m/s {twist.angular.z:+.2f} rad/s,'
                f' path {len(self.path or [])} pts{", HELD" if self.held_since else ""}')

    def watch_exit(self, here):
        """Take the exit for the goal once an opening in the left wall is seen close by."""
        occupied = self.hits >= self.hit_threshold
        rows = np.nonzero(occupied.sum(axis=1) * self.res >= self.exit_wall_min)[0]
        rows = rows[self.y_0 + rows * self.res > here[1] + 0.20]       # left of the robot
        found = None
        if len(rows):
            # The left wall: the fullest of those rows, with the rows either side of it.
            r = rows[np.argmax(occupied[rows].sum(axis=1))]
            band = occupied[max(0, r - 2):r + 3].any(axis=0)
            line_y = self.y_0 + r * self.res
            columns = np.nonzero(band)[0]
            for a, b in zip(columns[:-1], columns[1:]):
                width = (b - a - 1) * self.res
                if not (self.exit_size[0] <= width <= self.exit_size[1]):
                    continue
                before = band[:a + 1][::-1]
                run = np.argmin(before) if not before.all() else len(before)
                # The far end: the wall going on, or a wall coming in square (the far wall).
                after = band[b:b + int(self.exit_side_min / self.res)].sum() * self.res
                square = occupied[max(0, r - int(0.3 / self.res)):r + 1, max(0, b - 1):b + 2].any(axis=1)
                if run * self.res < self.exit_side_min or (after < 0.10 and square.sum() * self.res < 0.15):
                    continue
                middle = np.array([self.x_0 + (a + b) / 2.0 * self.res, line_y])
                if np.linalg.norm(middle - here) <= self.exit_range:
                    found = (middle, width)
                    break
        if found is None:
            self.exit_seen = []
            return
        self.exit_seen.append(found[0])
        if len(self.exit_seen) < self.exit_checks:
            return
        spread = np.ptp(np.array(self.exit_seen[-self.exit_checks:]), axis=0).max()
        if spread > 0.08:
            self.exit_seen = self.exit_seen[-1:]
            return
        middle, width = found
        goal = middle - np.array([0.0, self.exit_inside])          # just inside, below the wall
        if self.exit_found:
            if np.linalg.norm(goal - self.goal) > 0.03:
                self.get_logger().info(
                    f'Tunnel test: exit now at ({middle[0]:.2f}, {middle[1]:.2f}), {width:.2f} m wide.')
                self.goal = goal
                self.publish_route()
            return
        self.exit_found = True
        self.goal = goal
        self.exit_heading = math.pi / 2                             # out, square to the left wall
        self.best = (math.inf, self.now())
        self.get_logger().info(
            f'Tunnel test: exit seen at ({middle[0]:.2f}, {middle[1]:.2f}), {width:.2f} m wide;'
            ' heading for it.')
        self.publish_route()

    def align(self, yaw):
        """Turn on the spot exit_turn left of the route: facing out of the exit."""
        off = wrap(self.exit_heading - yaw)   # yaw is in the start's frame
        timed_out = self.now() - self.phase_since > self.align_timeout
        if abs(off) < self.align_tolerance or timed_out:
            self.state = 'leave'
            self.leave_from = np.array(self.pose[:2])
            self.approach_yaw, self.approach_target = self.odom[2], None
            self.pub_cmd_vel.publish(Twist())
            self.get_logger().info(
                'Tunnel test: %s (%+.0f deg off); out through the exit.'
                % ('facing out' if not timed_out else 'turn timed out', math.degrees(off)))
            return
        twist = Twist()
        twist.angular.z = math.copysign(min(self.align_rate, max(0.2, 1.5 * abs(off))), off)
        self.pub_cmd_vel.publish(twist)

    def leave(self, here):
        """Out through the exit as in through the entrance, until it is light again."""
        out = float(np.linalg.norm(here - self.leave_from))
        light = self.brightness is not None and self.brightness > self.dark_threshold
        if out >= self.leave_distance or (out >= self.leave_min and light):
            self.state = 'done'
            self.pub_cmd_vel.publish(Twist())
            self.pub_done.publish(Bool(data=True))
            self.pub_lane_resume.publish(Bool(data=True))
            why = ('light again (%.0f over %.0f)' % (self.brightness, self.dark_threshold)
                   if light else 'by distance')
            self.get_logger().info(f'Tunnel test: out ({out:.2f} m, {why}); back to the lane.')
            return
        front = self.front_wall_fit()
        if front is not None:
            self.aim(front)      # the tunnel's left wall, across the way now, exit at its right end
        self.pub_cmd_vel.publish(self.through())

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
