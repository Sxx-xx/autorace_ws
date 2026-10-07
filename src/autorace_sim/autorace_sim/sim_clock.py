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

"""/clock at a sane rate.

Gazebo publishes its clock at every physics step, a thousand times a second,
and every node that runs on simulated time takes each one in a Python
callback: with twenty nodes that is half the machine gone to reading the
time. The bridge brings Gazebo's clock in as /sim/clock; this node passes
it on as /clock at `rate` hertz, which is as fine as any timer here needs.
"""

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


class SimClock(Node):

    def __init__(self):
        super().__init__('sim_clock')
        self.declare_parameter('rate', 100.0)
        rate = self.get_parameter('rate').value

        self.latest = None
        self.create_subscription(Clock, '/sim/clock', self.callback_clock, 10)
        self.pub_clock = self.create_publisher(Clock, '/clock', 10)
        self.create_timer(1.0 / rate, self.publish)

    def callback_clock(self, msg):
        self.latest = msg

    def publish(self):
        if self.latest is not None:
            self.pub_clock.publish(self.latest)


def main(args=None):
    rclpy.init(args=args)
    node = SimClock()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
