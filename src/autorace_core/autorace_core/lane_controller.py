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

Steering is a PD loop on the lateral offset of the lane centre, in metres, so
the gains keep their meaning if the bird's eye view is recalibrated.  Speed
falls off with the offset, which is what gets the robot around the tight bends
of the AutoRace course without cutting them.

When the lane goes missing the controller does not simply give up: it keeps
creeping forward while turning the way it was last steering, which is usually
enough to bring the line back into view.  A robot that stands still for 30 s
ends its run, so stopping is the last resort, not the first.
"""

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import UInt8


class LaneController(Node):

    def __init__(self):
        super().__init__('lane_controller')

        self.declare_parameter('max_speed', 0.16)
        self.declare_parameter('min_speed', 0.05)
        self.declare_parameter('kp', 2.6)
        self.declare_parameter('kd', 6.0)
        self.declare_parameter('max_angular', 2.0)
        self.declare_parameter('slowdown_offset', 0.12)
        self.declare_parameter('lane_timeout', 0.5)
        self.declare_parameter('recovery_speed', 0.05)
        self.declare_parameter('recovery_angular', 0.45)
        self.declare_parameter('recovery_timeout', 8.0)
        self.declare_parameter('publish_rate', 20.0)

        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.max_angular = self.get_parameter('max_angular').value
        self.slowdown_offset = self.get_parameter('slowdown_offset').value
        self.lane_timeout = self.get_parameter('lane_timeout').value
        self.recovery_speed = self.get_parameter('recovery_speed').value
        self.recovery_angular = self.get_parameter('recovery_angular').value
        self.recovery_timeout = self.get_parameter('recovery_timeout').value
        rate = self.get_parameter('publish_rate').value

        self.create_subscription(Float64, '/detect/lane_offset', self.callback_offset, 1)
        self.create_subscription(Float64, '/control/max_vel', self.callback_max_vel, 1)
        self.create_subscription(UInt8, '/detect/lane_state', self.callback_state, 1)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel/lane', 1)

        self.offset = None
        self.offset_time = 0.0
        self.last_offset = 0.0
        self.last_angular = 0.0
        self.lane_state = 0
        self.recovering_since = None

        self.create_timer(1.0 / rate, self.update)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def callback_offset(self, msg):
        self.offset = msg.data
        self.offset_time = self.now()

    def callback_max_vel(self, msg):
        self.max_speed = msg.data

    def callback_state(self, msg):
        self.lane_state = msg.data

    def update(self):
        now = self.now()
        fresh = self.offset is not None and now - self.offset_time < self.lane_timeout

        twist = Twist()
        if fresh:
            self.recovering_since = None
            error = self.offset
            derivative = error - self.last_offset
            self.last_offset = error

            angular = -(self.kp * error + self.kd * derivative)
            angular = max(-self.max_angular, min(self.max_angular, angular))
            self.last_angular = angular

            # Ease off the throttle as the robot sits further from the centre.
            slow = max(0.0, 1.0 - abs(error) / self.slowdown_offset) ** 2
            speed = self.min_speed + (self.max_speed - self.min_speed) * slow

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
            if self.last_angular == 0.0:
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
