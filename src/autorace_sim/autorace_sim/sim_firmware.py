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

"""What the real robot makes of a velocity command.

Gazebo's differential drive does whatever it is told; the TurtleBot3 does
not. Between /cmd_vel and the motors stands the OpenCR firmware, and behind
it two XL430 motors that turn no faster than 61 rpm. This node stands in
for them, so that the course is driven with the limits of the real robot:

- the command reaches the board in hundredths (1 cm/s, 0.01 rad/s);
- the firmware clamps the linear speed to what 61 rpm gives at the wheel
  (0.211 m/s) and the angular speed to that over the turning radius
  (2.64 rad/s), each on its own;
- each wheel is then asked for v -/+ w * separation / 2, and a wheel asked
  for more than the motor can turn simply turns at its most. The robot does
  not slow down to keep the curvature: it goes straighter than it was told.

/cmd_vel in, /sim/cmd_vel out to the simulated drive.
"""

import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class SimFirmware(Node):

    def __init__(self):
        super().__init__('sim_firmware')

        # TurtleBot3 Burger, as in the OpenCR turtlebot3_ros2 firmware.
        self.declare_parameter('wheel_radius', 0.033)
        self.declare_parameter('wheel_separation', 0.160)
        self.declare_parameter('turning_radius', 0.080)
        # The motors' no-load speed at 12 V; under load they do less.
        self.declare_parameter('motor_rpm', 61.0)
        self.declare_parameter('wheel_speed', 0.0)   # overrides motor_rpm if > 0

        def value(name):
            return self.get_parameter(name).value

        self.half_separation = value('wheel_separation') / 2.0
        self.max_linear = value('wheel_radius') * 2.0 * math.pi * value('motor_rpm') / 60.0
        self.max_angular = self.max_linear / value('turning_radius')
        self.wheel_speed = value('wheel_speed') or self.max_linear

        self.create_subscription(Twist, '/cmd_vel', self.callback_cmd_vel, 1)
        self.pub_cmd_vel = self.create_publisher(Twist, '/sim/cmd_vel', 1)
        self.get_logger().info(
            'Firmware limits: linear %.3f m/s, angular %.2f rad/s, wheel %.3f m/s.'
            % (self.max_linear, self.max_angular, self.wheel_speed))

    def callback_cmd_vel(self, msg):
        # In hundredths, as the control table holds it.
        linear = round(msg.linear.x, 2)
        angular = round(msg.angular.z, 2)
        linear = max(-self.max_linear, min(self.max_linear, linear))
        angular = max(-self.max_angular, min(self.max_angular, angular))
        left = linear - angular * self.half_separation
        right = linear + angular * self.half_separation
        left = max(-self.wheel_speed, min(self.wheel_speed, left))
        right = max(-self.wheel_speed, min(self.wheel_speed, right))
        out = Twist()
        out.linear.x = (left + right) / 2.0
        out.angular.z = (right - left) / (2.0 * self.half_separation)
        self.pub_cmd_vel.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SimFirmware()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
