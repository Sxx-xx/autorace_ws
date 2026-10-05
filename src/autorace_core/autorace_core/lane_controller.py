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

"""Lane following controller.

Steering is pure pursuit: the lane detector hands over a point on the lane
centre a short way ahead, and the robot drives the arc that passes through it.
On an arc that is exact, so the robot holds the middle of the lane round a
bend instead of drifting to one side of it, and the turn rate follows from the
speed rather than from a gain tuned for one speed.  The lane leaves the robot
about 2 cm either side, which a fixed gain on the sideways offset could not
keep to.  Speed comes down where the arc is tight.

When the lane goes missing the controller does not simply give up: it keeps
creeping forward while turning the way it was last steering, which is usually
enough to bring the line back into view.  A robot that stands still for 30 s
ends its run, so stopping is the last resort, not the first.
"""

from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import UInt8


class LaneController(Node):

    def __init__(self):
        super().__init__('lane_controller')

        self.declare_parameter('max_speed', 0.16)
        self.declare_parameter('min_speed', 0.05)
        # 1.0 is plain pure pursuit; more turns in harder towards the target.
        self.declare_parameter('pursuit_gain', 1.0)
        self.declare_parameter('max_angular', 2.0)
        # Turn rate above which the robot slows down rather than turn faster.
        self.declare_parameter('cornering_rate', 0.6)
        self.declare_parameter('lane_timeout', 0.5)
        self.declare_parameter('recovery_speed', 0.05)
        self.declare_parameter('recovery_angular', 0.45)
        self.declare_parameter('recovery_timeout', 8.0)
        self.declare_parameter('follow_timeout', 0.5)
        # A speed limit on /control/max_vel holds only while it keeps coming.
        self.declare_parameter('limit_timeout', 0.5)
        self.declare_parameter('publish_rate', 20.0)

        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.pursuit_gain = self.get_parameter('pursuit_gain').value
        self.max_angular = self.get_parameter('max_angular').value
        self.cornering_rate = self.get_parameter('cornering_rate').value
        self.lane_timeout = self.get_parameter('lane_timeout').value
        self.recovery_speed = self.get_parameter('recovery_speed').value
        self.recovery_angular = self.get_parameter('recovery_angular').value
        self.recovery_timeout = self.get_parameter('recovery_timeout').value
        self.follow_timeout = self.get_parameter('follow_timeout').value
        self.limit_timeout = self.get_parameter('limit_timeout').value
        rate = self.get_parameter('publish_rate').value

        self.create_subscription(PointStamped, '/detect/lane_target', self.callback_target, 1)
        self.create_subscription(Float64, '/control/max_vel', self.callback_max_vel, 1)
        self.create_subscription(UInt8, '/detect/lane_state', self.callback_state, 1)
        self.create_subscription(String, '/detect/lane_follow', self.callback_follow, 1)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel/lane', 1)

        self.target = None          # (ahead, left) of the axle, metres
        self.target_time = 0.0
        self.last_angular = 0.0
        self.lane_state = 0
        self.follow = None
        self.follow_time = 0.0
        self.limit = None           # a mission's speed limit, and when it last came
        self.limit_time = 0.0
        self.recovering_since = None

        self.create_timer(1.0 / rate, self.update)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def callback_target(self, msg):
        self.target = (msg.point.x, msg.point.y)
        self.target_time = self.now()

    def callback_max_vel(self, msg):
        """Take a mission's speed limit; it holds for as long as it keeps coming."""
        self.limit = msg.data
        self.limit_time = self.now()

    def callback_state(self, msg):
        self.lane_state = msg.data

    def callback_follow(self, msg):
        self.follow = msg.data if msg.data in ('left', 'right') else None
        self.follow_time = self.now()

    def update(self):
        now = self.now()
        fresh = self.target is not None and now - self.target_time < self.lane_timeout

        twist = Twist()
        if fresh:
            self.recovering_since = None
            ahead, left = self.target
            # Curvature of the arc from the axle, along the heading, through
            # the target.
            curvature = self.pursuit_gain * 2.0 * left / (ahead * ahead + left * left)

            speed = self.max_speed
            if self.limit is not None and now - self.limit_time < self.limit_timeout:
                speed = min(speed, self.limit)
            if abs(curvature) * speed > self.cornering_rate:
                speed = max(self.min_speed, self.cornering_rate / abs(curvature))
            angular = speed * curvature
            angular = max(-self.max_angular, min(self.max_angular, angular))
            self.last_angular = angular

            twist.linear.x = speed
            twist.angular.z = angular
        else:
            if self.recovering_since is None:
                self.recovering_since = now
                self.get_logger().warn('Lane lost, searching.')
            if now - self.recovering_since > self.recovery_timeout:
                # Give up steering blind; the mux watchdog takes it from here.
                self.get_logger().error(
                    'Lane not recovered within %.0f s.' % self.recovery_timeout,
                    throttle_duration_sec=5.0
                )
                return
            twist.linear.x = self.recovery_speed
            if self.follow is not None and now - self.follow_time < self.follow_timeout:
                # Asked to follow one line and it is not in view: it is on
                # its own side of the robot, whichever way the robot was
                # steering before.
                direction = 1.0 if self.follow == 'left' else -1.0
                twist.angular.z = direction * self.recovery_angular
            elif self.last_angular == 0.0:
                # Nothing has been tracked yet, so there is no direction to
                # search in; creeping straight brings the lane into view.
                twist.angular.z = 0.0
            else:
                direction = 1.0 if self.last_angular > 0.0 else -1.0
                twist.angular.z = direction * self.recovery_angular

        self.pub_cmd_vel.publish(twist)

    def shut_down(self):
        self.pub_cmd_vel.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = LaneController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shut_down()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
