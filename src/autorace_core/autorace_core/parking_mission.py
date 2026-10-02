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

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool
from std_msgs.msg import String


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class ParkingMission(Node):

    def __init__(self):
        super().__init__('parking_mission')

        # The lot is where white line shows after this much road without any.
        self.declare_parameter('white_free_distance', 0.3)
        self.declare_parameter('white_min_points', 150)
        # From where the white begins to level with the middle of the bays,
        # and how far from the middle of the aisle the axle goes into a bay:
        # past the middle of the bay, as most of the robot is behind its axle.
        self.declare_parameter('bay_station', 0.24)
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
        self.declare_parameter('turn_rate', 0.8)
        self.declare_parameter('park_time', 1.0)
        # The mission is over when the robot is heading along the road again,
        # or has driven this far from the lot without getting there.
        self.declare_parameter('exit_limit', 1.6)
        # The manager repeats the activation; without it the mission is over.
        self.declare_parameter('active_timeout', 1.0)

        def value(name):
            return self.get_parameter(name).value

        self.white_free_distance = value('white_free_distance')
        self.white_min_points = value('white_min_points')
        self.bay_station = value('bay_station')
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
        self.turn_rate = value('turn_rate')
        self.park_time = value('park_time')
        self.exit_limit = value('exit_limit')
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
                self.seen = {'left': 0, 'right': 0}
                self.white_seen_at = self.odometry[-1][4]
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
        """Count what stands to either side of the way ahead."""
        if self.phase not in ('turn_in', 'aisle') or self.white_seen_at is None:
            return
        travelled = self.odometry[-1][4]
        if self.lot is None and travelled - self.white_seen_at < self.white_free_distance:
            # Not on the lane to the lot yet; what is beside the road here
            # is not in the lot.
            return
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        seen = np.isfinite(ranges) & (ranges > msg.range_min + 1e-3)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        ahead = self.lidar_offset + ranges[seen] * np.cos(angles[seen])
        left = ranges[seen] * np.sin(angles[seen])
        beside = ((ahead > 0.0) & (ahead < self.bay_look_ahead)
                  & (np.abs(left) > self.bay_look_near) & (np.abs(left) < self.bay_look_far))
        self.seen['left'] += int(np.count_nonzero(beside & (left > 0.0)))
        self.seen['right'] += int(np.count_nonzero(beside & (left < 0.0)))

    def callback_yellow(self, msg):
        """Keep the yellow lines of a picture for when its white ones arrive."""
        if self.phase in ('aisle', 'leave'):
            self.yellow = (msg.header.stamp,
                           point_cloud2.read_points_numpy(msg, field_names=('x', 'y')))

    def callback_white(self, msg):
        if self.phase not in ('turn_in', 'aisle', 'leave') or not self.odometry:
            return
        points = point_cloud2.read_points_numpy(msg, field_names=('x', 'y'))
        x, y, yaw, travelled = self.pose_at(msg.header.stamp)
        if self.phase != 'turn_in':
            # The aisle runs between white lines and the lane to it between
            # yellow ones, in line with them.
            if self.yellow is not None and self.yellow[0] == msg.header.stamp:
                points = np.concatenate([points, self.yellow[1]])
            self.see_aisle(points, yaw)
            return
        if len(points) < self.white_min_points:
            return
        free = travelled - self.white_seen_at
        self.white_seen_at = travelled
        if free < self.white_free_distance:
            # Still the white line of the road.
            return

        # The white begins at the nearest of it. The aisle runs between the
        # broken lines, one to each side.
        begins = float(np.percentile(points[:, 0], 5))
        left = points[points[:, 1] > 0.0, 1]
        right = points[points[:, 1] < 0.0, 1]
        middle = 0.0
        if len(left) > 20 and len(right) > 20:
            middle = float(np.median(left) + np.median(right)) / 2.0
        # The lane is straight, and the lane follower has had its length to
        # settle on it: the aisle runs the way the robot has been going.
        recent = [entry[3] for entry in self.odometry if travelled - entry[4] < 0.15]
        direction = math.atan2(sum(math.sin(a) for a in recent), sum(math.cos(a) for a in recent))
        self.lot = (x + math.cos(yaw) * begins - math.sin(yaw) * middle,
                    y + math.sin(yaw) * begins + math.cos(yaw) * middle,
                    direction)
        self.get_logger().info(
            f'Parking: lot {begins:.2f} m ahead, aisle {middle:+.3f} m to the left.')
        self.enter('aisle')

    def see_aisle(self, points, yaw):
        """How far left of the middle of the aisle the camera shows the robot to be."""
        left = points[points[:, 1] > 0.0]
        right = points[points[:, 1] < 0.0]
        if len(left) < 20 or len(right) < 20:
            return
        middle = float(np.median(left[:, 1]) + np.median(right[:, 1])) / 2.0
        # The lines are seen some way ahead; a robot not square to the aisle
        # sees them off to one side by that much.
        course = 0.0 if self.phase == 'aisle' else math.pi
        askew = wrap(yaw - self.lot[2] - course)
        ahead = float(np.median(points[:, 0]))
        self.aisle_seen = (-middle - ahead * math.tan(askew), self.now())

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
        if self.phase == 'turn_in':
            self.pub_follow.publish(String(data='left'))
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
            twist = self.drive(heading, math.pi, self.in_aisle(-across), 1.0)
            if along <= self.leave_station:
                self.get_logger().info('Parking: out of the lot, keeping left for the road.')
                self.enter('exit')

        if self.phase == 'exit':
            self.pub_follow.publish(String(data='left'))
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
