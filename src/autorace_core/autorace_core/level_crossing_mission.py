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

"""Level crossing mission: wait at the bar while it is down.

A sensor somewhere on the approach brings the bar down when the robot passes
it; where that sensor is, is not known. A second sensor stands 6 cm in front
of the bar, and a robot that reaches it while the bar is down has failed. So
there is no telling when the bar will come down, only that the robot must
stop well short of it when it does, and go on when it has gone up again.

The forward camera knows the bar by its stripes and says whether it is down
(/detect/level_crossing). When it is, this node asks the mission manager for
the mission and takes the robot off the lane follower. The camera loses the
bar when the robot is within about 35 cm of it, and a bar that comes down
only then is never seen down; so once the camera has seen the bar at all,
down or up, anything the laser finds across the lane close ahead asks for
the mission too.

It then drives up to the bar and waits with its front `stop_gap` from it:
just short of the second sensor, and no further back, since where the first
sensor is nobody knows. The distance is the laser's, which sees the bar
across the lane ahead. From further off the laser may miss the bar, which it
catches by a centimetre; while the camera has the bar in its picture the bar
is still some way off, and the robot keeps creeping up to it. Close to, the
bar is above what the camera sees, so it is the laser that tells when the bar
has gone up: having seen the bar, it has to find the lane ahead empty for a
second before the robot goes on. The lane was empty before the bar came down
too, so a laser that has not yet seen the bar says nothing; should it never
see it, the mission runs into the manager's timeout, which is slow but is
not driving into the bar.
"""

import math

from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import UInt8


# Bar state published by autorace_perception/detect_level_crossing.
BAR_NONE = 0
BAR_CLOSED = 2


class LevelCrossingMission(Node):

    def __init__(self):
        super().__init__('level_crossing_mission')

        # The bar has to be seen down in this many pictures in a row.
        self.declare_parameter('closed_streak', 2)
        # Having seen the bar within this long, the laser finding something
        # across the lane this near ahead starts the mission as well.
        self.declare_parameter('bar_memory', 20.0)
        self.declare_parameter('blocked_distance', 0.5)
        # Where to wait: the front of the robot this far from the bar. The
        # second sensor is 6 cm from the bar. The front is this far ahead of
        # the axle.
        self.declare_parameter('stop_gap', 0.10)
        self.declare_parameter('robot_front', 0.07)
        self.declare_parameter('hold_tolerance', 0.01)
        self.declare_parameter('hold_gain', 1.0)
        self.declare_parameter('hold_speed', 0.10)
        # Towards a bar only the camera sees. It loses the bar about 0.3 m
        # from it, and the robot stops there if the laser has not found it.
        self.declare_parameter('creep_speed', 0.05)
        # The laser looks for the bar in the strip ahead: this far, this wide.
        self.declare_parameter('look_distance', 1.2)
        self.declare_parameter('look_half_width', 0.10)
        self.declare_parameter('lidar_offset', -0.032)
        # Open means the camera has not seen the bar down for this many
        # pictures and the laser has found the lane empty for this many scans.
        self.declare_parameter('open_pictures', 8)
        self.declare_parameter('open_scans', 5)
        # The manager repeats the activation; without it the mission is over.
        self.declare_parameter('active_timeout', 1.0)

        def value(name):
            return self.get_parameter(name).value

        self.closed_streak = value('closed_streak')
        self.bar_memory = value('bar_memory')
        self.blocked_distance = value('blocked_distance')
        self.hold_distance = value('stop_gap') + value('robot_front')
        self.hold_tolerance = value('hold_tolerance')
        self.hold_gain = value('hold_gain')
        self.hold_speed = value('hold_speed')
        self.creep_speed = value('creep_speed')
        self.look_distance = value('look_distance')
        self.look_half_width = value('look_half_width')
        self.lidar_offset = value('lidar_offset')
        self.open_pictures = value('open_pictures')
        self.open_scans = value('open_scans')
        self.active_timeout = value('active_timeout')

        self.phase = None           # None while the mission is not running
        self.active_time = 0.0
        self.closed_count = 0       # pictures in a row with the bar down
        self.open_count = 0         # and without
        self.picture_time = 0.0
        self.bar_picture_time = -1e9   # when the camera last saw the bar at all
        self.bar_distance = None    # from the axle, by the laser
        self.bar_time = 0.0
        self.bar_travelled = 0.0    # distance travelled when it was measured
        self.position = None
        self.travelled = 0.0        # forwards counted positive, backwards negative
        self.empty_scans = 0        # scans in a row with nothing in the lane
        self.bar_seen = False       # by the laser, since the mission began
        self.lane_target = None     # where the lane follower is steering for
        self.lane_time = 0.0

        self.create_subscription(
            Bool, '/autorace/mission/level_crossing/active', self.callback_active, 1)
        self.create_subscription(UInt8, '/detect/level_crossing', self.callback_bar, 1)
        self.create_subscription(LaserScan, '/scan', self.callback_scan, 1)
        self.create_subscription(Odometry, '/odom', self.callback_odom, 1)
        self.create_subscription(PointStamped, '/detect/lane_target', self.callback_lane, 1)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel/mission', 1)
        self.pub_done = self.create_publisher(
            Bool, '/autorace/mission/level_crossing/done', 1)
        self.pub_trigger = self.create_publisher(String, '/autorace/trigger', 5)

        self.create_timer(0.05, self.update)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    # --- callbacks -------------------------------------------------------

    def callback_active(self, msg):
        if msg.data:
            self.active_time = self.now()
            if self.phase is None:
                self.get_logger().info('Level crossing: the bar is down, going up to it.')
                self.phase = 'wait'
                self.open_count = 0
                # The lane was empty before the bar came down; only once
                # the laser has seen the bar does its going count.
                self.empty_scans = 0
                self.bar_seen = False
        elif self.phase is not None:
            self.get_logger().info(f'Level crossing: deactivated during {self.phase}.')
            self.phase = None

    def callback_bar(self, msg):
        self.picture_time = self.now()
        if msg.data != BAR_NONE:
            self.bar_picture_time = self.picture_time
        if msg.data == BAR_CLOSED:
            self.closed_count += 1
            self.open_count = 0
        else:
            self.closed_count = 0
            self.open_count += 1
        if self.phase is None and self.closed_count >= self.closed_streak:
            # The manager takes this only while lane following, and once.
            self.pub_trigger.publish(String(data='level_crossing'))

    def callback_scan(self, msg):
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        seen = np.isfinite(ranges) & (ranges > msg.range_min + 1e-3)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        ahead = self.lidar_offset + ranges[seen] * np.cos(angles[seen])
        left = ranges[seen] * np.sin(angles[seen])
        in_lane = ahead[(ahead > 0.05) & (ahead < self.look_distance)
                        & (np.abs(left) < self.look_half_width)]
        if len(in_lane) >= 3:
            self.bar_distance = float(np.median(in_lane))
            self.bar_time = self.now()
            self.bar_travelled = self.travelled
            self.empty_scans = 0
            self.bar_seen = True
            if (self.phase is None and self.bar_distance < self.blocked_distance
                    and self.now() - self.bar_picture_time < self.bar_memory):
                # The bar, by the laser, before the camera has it down.
                self.pub_trigger.publish(String(data='level_crossing'))
        else:
            self.empty_scans += 1

    def callback_odom(self, msg):
        """Count the road covered, to know the distance between two scans."""
        position = msg.pose.pose.position
        if self.position is not None:
            step = math.hypot(position.x - self.position[0], position.y - self.position[1])
            self.travelled += math.copysign(step, msg.twist.twist.linear.x)
        self.position = (position.x, position.y)

    def callback_lane(self, msg):
        self.lane_target = (msg.point.x, msg.point.y)
        self.lane_time = self.now()

    # --- driving ---------------------------------------------------------

    def update(self):
        if self.phase is None:
            return
        now = self.now()
        if now - self.active_time > self.active_timeout:
            self.get_logger().info(f'Level crossing: activation lapsed during {self.phase}.')
            self.phase = None
            return
        if self.phase == 'done':
            # Repeated until the manager takes the activation away.
            self.pub_done.publish(Bool(data=True))
            return

        if (self.bar_seen and self.open_count >= self.open_pictures
                and self.empty_scans >= self.open_scans):
            self.get_logger().info('Level crossing: the bar is up, going on.')
            self.phase = 'done'
            return

        # Keep the bar at the holding distance by the laser. Without a fresh
        # measure of it, creep on for as long as the camera still sees the
        # bar; with neither, standing still is the safe thing.
        speed = 0.0
        if self.empty_scans == 0 and now - self.bar_time < 0.5:
            # The laser scans five times a second; in between, the bar is
            # nearer by what the robot has driven since.
            distance = self.bar_distance - (self.travelled - self.bar_travelled)
            error = distance - self.hold_distance
            if abs(error) > self.hold_tolerance:
                speed = max(-self.hold_speed, min(self.hold_speed, self.hold_gain * error))
        elif self.closed_count > 0 and now - self.picture_time < 0.3:
            speed = self.creep_speed
        twist = Twist()
        twist.linear.x = speed
        if speed > 0.0 and self.lane_target is not None and now - self.lane_time < 0.5:
            # Up the lane, on the arc the lane follower would drive.
            ahead, left = self.lane_target
            twist.angular.z = speed * 2.0 * left / (ahead * ahead + left * left)
        self.pub_cmd_vel.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = LevelCrossingMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
