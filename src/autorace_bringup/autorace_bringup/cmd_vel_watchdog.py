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

"""Stop the wheels when the PC's commands stop coming: on the Pi, beside the OpenCR.

The OpenCR keeps to the last /cmd_vel it was given. With the driving done on
the PC, a Wi-Fi drop, a hung or killed PC node, or a closed lid would leave
the robot driving on blind; the PC's own safeguards (cmd_vel_mux, estop.py)
are on the far side of that same Wi-Fi. This node is on the near side.

It listens to /cmd_vel as the OpenCR does. While the last command it heard
moves the wheels, it expects the next within `timeout` (cmd_vel_mux sends at
50 Hz, teleop at 10 Hz); when none comes, it sends zero velocity itself, a
few times over `stop_for`, and says so. A zero command, from anyone, means the
wheels are stopped and nothing is expected: so its own zeros do not keep it
going, and a robot told to stand still is left alone. When commands come back
the PC drives again, unchanged.

It runs beside the PC's publishers, not between them and the OpenCR: if it
fails, /cmd_vel is as it was without it.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class CmdVelWatchdog(Node):

    def __init__(self):
        super().__init__('cmd_vel_watchdog')
        self.declare_parameter('timeout', 0.5)
        self.declare_parameter('stop_for', 1.0)
        self.declare_parameter('stop_rate', 20.0)
        self.timeout = self.get_parameter('timeout').value
        self.stop_for = self.get_parameter('stop_for').value

        self.moving = False         # the last command heard moves the wheels
        self.last_time = self.now()
        self.stop_until = None      # sending zeros until then

        # As cmd_vel_mux: reliable, and only the newest command queued.
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Twist, '/cmd_vel', self.callback_cmd_vel, qos)
        self.pub = self.create_publisher(Twist, '/cmd_vel', qos)
        self.create_timer(1.0 / self.get_parameter('stop_rate').value, self.update)
        self.get_logger().info(
            f'Watching /cmd_vel: zero velocity after {self.timeout:.2f} s without a command.')

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def callback_cmd_vel(self, msg):
        moving = any((msg.linear.x, msg.linear.y, msg.linear.z,
                      msg.angular.x, msg.angular.y, msg.angular.z))
        if moving:
            if self.stop_until is not None:
                self.get_logger().info('Commands back: the PC drives again.')
            self.stop_until = None
            self.last_time = self.now()
        self.moving = moving

    def update(self):
        now = self.now()
        if self.stop_until is not None:
            if now < self.stop_until:
                self.pub.publish(Twist())
            else:
                self.stop_until = None
            return
        if self.moving and now - self.last_time > self.timeout:
            self.get_logger().warn('No /cmd_vel for %.2f s while moving: stopping the wheels.'
                                   % (now - self.last_time))
            self.moving = False
            self.stop_until = now + self.stop_for
            self.pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
