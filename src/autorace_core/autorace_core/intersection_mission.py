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

"""Intersection mission: read the direction sign and take that branch.

At the fork the two lines of the lane part: the yellow one runs down the left
branch, the white one down the right. Staying with one line is all it takes to
choose a branch, so the lane follower does the driving throughout and this
node only tells it which line to follow.

The direction sign stands on the island between the branches, facing the road
in. Plain lane following never points the camera at it: where the lines part
it stays with the right one, and the sign slides out of the left of the
picture. So from the moment the mission starts the robot follows the left
line, which swings it round to face the island, and it changes over to the
right line if that is what the sign says.

    approach  follow the left line until the direction sign has been read
    branch    follow the line on the sign's side until the branches rejoin

With turn_at_line (the real robot, whose camera sees nothing nearer than
0.35 m: following the left line from the sign on swung it off the lane
before the fork) the approach is plain lane following, and once the sign is
read the lane controller is ordered to turn that way at the line across the
way (the end of the island), half a lane short of it:

    approach  lane following; the controller's own corner turns held off
    turn      /control/turn_at_line <side> until it reports the turn done
    branch    follow the line on the sign's side for branch_distance
"""

import math

from autorace_msgs.msg import SignDetection
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String


class IntersectionMission(Node):

    def __init__(self):
        super().__init__('intersection_mission')

        self.declare_parameter('sign_streak', 3)
        # A sign read this long before the mission started still counts: the
        # direction sign itself can be what starts the mission.
        self.declare_parameter('sign_memory', 2.0)
        self.declare_parameter('approach_follow', 'left')
        # Driven on the chosen line after the sign is read; long enough to be
        # past the point where the branches rejoin.
        self.declare_parameter('branch_distance', 2.0)
        self.declare_parameter('turn_at_line', False)
        # No line across the way within this long after the sign: go on as
        # without turn_at_line (follow the sign's side).
        self.declare_parameter('turn_timeout', 15.0)
        # The manager repeats the activation; without it the mission is over.
        self.declare_parameter('active_timeout', 1.0)

        self.sign_streak = self.get_parameter('sign_streak').value
        self.sign_memory = self.get_parameter('sign_memory').value
        self.approach_follow = self.get_parameter('approach_follow').value
        self.branch_distance = self.get_parameter('branch_distance').value
        self.active_timeout = self.get_parameter('active_timeout').value
        self.turn_at_line = self.get_parameter('turn_at_line').value
        self.turn_timeout = self.get_parameter('turn_timeout').value
        self.turn_started = 0.0
        self.turn_done = False

        self.phase = None           # None while the mission is not running
        self.active_time = 0.0
        self.direction = None       # last direction sign read, and when
        self.direction_time = 0.0
        self.branch = None
        self.position = None
        self.travelled = 0.0

        self.create_subscription(
            Bool, '/autorace/mission/intersection/active', self.callback_active, 1)
        self.create_subscription(SignDetection, '/autorace/sign', self.callback_sign, 5)
        self.create_subscription(Odometry, '/odom', self.callback_odom, 1)
        self.pub_follow = self.create_publisher(String, '/detect/lane_follow', 1)
        self.pub_order = self.create_publisher(String, '/control/turn_at_line', 1)
        self.create_subscription(Bool, '/control/turn_at_line/done',
                                 lambda m: setattr(self, 'turn_done', bool(m.data)), 1)
        self.pub_done = self.create_publisher(
            Bool, '/autorace/mission/intersection/done', 1)

        self.create_timer(0.1, self.update)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def callback_active(self, msg):
        if msg.data:
            self.active_time = self.now()
            if self.phase is None:
                if self.turn_at_line:
                    self.get_logger().info('Intersection: lane following to the sign.')
                else:
                    self.get_logger().info(
                        f'Intersection: following the {self.approach_follow} line to the sign.')
                self.phase = 'approach'
        elif self.phase is not None:
            self.stop('deactivated')

    def callback_sign(self, msg):
        if msg.name in ('left', 'right') and msg.streak >= self.sign_streak:
            self.direction = msg.name
            self.direction_time = self.now()

    def callback_odom(self, msg):
        position = msg.pose.pose.position
        if self.position is not None and self.phase == 'branch':
            self.travelled += math.hypot(position.x - self.position[0],
                                         position.y - self.position[1])
        self.position = (position.x, position.y)

    def stop(self, reason):
        self.get_logger().info(f'Intersection: {reason} during {self.phase}.')
        self.phase = None

    def update(self):
        if self.phase is None:
            return
        now = self.now()
        if now - self.active_time > self.active_timeout:
            self.stop('activation lapsed')
            return

        if self.phase == 'approach':
            if self.direction is not None and now - self.direction_time < self.sign_memory:
                self.branch = self.direction
                self.travelled = 0.0
                if self.turn_at_line:
                    self.phase = 'turn'
                    self.turn_started = now
                    self.turn_done = False
                    self.get_logger().info(
                        f'Intersection: sign says {self.branch}; turning {self.branch} '
                        'at the line across the way.')
                else:
                    self.phase = 'branch'
                    self.get_logger().info(f'Intersection: sign says {self.branch}.')
        if self.phase == 'turn':
            if self.turn_done:
                self.get_logger().info(f'Intersection: turned {self.branch}, into the branch.')
                self.phase = 'branch'
                self.travelled = 0.0
            elif now - self.turn_started > self.turn_timeout:
                self.get_logger().warn(
                    'Intersection: no line across the way, following the sign side instead.')
                self.phase = 'branch'
                self.travelled = 0.0
            else:
                self.pub_order.publish(String(data=self.branch))
                return

        if self.phase == 'branch' and self.travelled >= self.branch_distance:
            self.get_logger().info(
                f'Intersection: {self.travelled:.2f} m down the {self.branch} branch, done.')
            self.phase = 'done'

        if self.phase == 'done':
            # Repeated until the manager takes the activation away.
            self.pub_done.publish(Bool(data=True))
            return

        if self.phase == 'approach' and self.turn_at_line:
            self.pub_order.publish(String(data='hold'))
            return
        follow = self.approach_follow if self.phase == 'approach' else self.branch
        self.pub_follow.publish(String(data=follow))


def main(args=None):
    rclpy.init(args=args)
    node = IntersectionMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
