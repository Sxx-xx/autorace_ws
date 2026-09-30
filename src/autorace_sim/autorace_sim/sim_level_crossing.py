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

"""Drive the simulated level crossing bar.

The rules put sensor 1 somewhere on the approach: when the robot passes it the
bar comes down, and it opens again after a while.  Sensor 2 sits 6 cm in front
of the bar and failing the mission means touching it while the bar is down, so
the node reports crossing it on `/sim/level_crossing_violation` for scoring.
"""

import math

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import String


ANGLE_CLOSED = 0.0
ANGLE_OPEN = math.pi / 2.0


class SimLevelCrossing(Node):

    def __init__(self):
        super().__init__('sim_level_crossing')

        # Odometry starts at the spawn pose, so the offset turns it into world
        # coordinates without needing ground truth from the simulator.
        # The positions come from params/course.yaml, which is generated
        # together with the world file.
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw', 0.0)

        # Sensor 1: the trigger point on the approach.
        self.declare_parameter('sensor1_x', 0.0)
        self.declare_parameter('sensor1_y', 0.0)
        self.declare_parameter('sensor1_radius', 0.20)
        # Sensor 2: 6 cm in front of the bar, per the rules.
        self.declare_parameter('sensor2_x', 0.0)
        self.declare_parameter('sensor2_y', 0.0)
        self.declare_parameter('sensor2_radius', 0.08)
        self.declare_parameter('closed_duration', 10.0)

        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        self.start_yaw = self.get_parameter('start_yaw').value
        self.sensor1 = (
            self.get_parameter('sensor1_x').value,
            self.get_parameter('sensor1_y').value,
        )
        self.sensor1_radius = self.get_parameter('sensor1_radius').value
        self.sensor2_radius = self.get_parameter('sensor2_radius').value
        self.closed_duration = self.get_parameter('closed_duration').value
        self.sensor2 = (
            self.get_parameter('sensor2_x').value,
            self.get_parameter('sensor2_y').value,
        )

        self.pub_bar = self.create_publisher(Float64, '/level_bar/cmd', 1)
        self.pub_state = self.create_publisher(String, '/sim/level_crossing', 1)
        self.pub_violation = self.create_publisher(Bool, '/sim/level_crossing_violation', 1)

        self.create_subscription(Odometry, '/odom', self.callback_odom, 1)

        self.state = 'open'
        self.closed_at = None
        self.triggered = False
        self.violation = False
        self.position = None

        self.create_timer(0.1, self.update)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def callback_odom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        cos_yaw = math.cos(self.start_yaw)
        sin_yaw = math.sin(self.start_yaw)
        self.position = (
            self.start_x + cos_yaw * x - sin_yaw * y,
            self.start_y + sin_yaw * x + cos_yaw * y,
        )

    def near(self, point, radius):
        if self.position is None:
            return False
        dx = self.position[0] - point[0]
        dy = self.position[1] - point[1]
        return math.hypot(dx, dy) <= radius

    def set_bar(self, angle):
        self.pub_bar.publish(Float64(data=angle))

    def update(self):
        if self.state == 'open':
            self.set_bar(ANGLE_OPEN)
            if not self.triggered and self.near(self.sensor1, self.sensor1_radius):
                self.get_logger().info('Sensor 1 triggered, closing the bar.')
                self.triggered = True
                self.state = 'closed'
                self.closed_at = self.now()
        elif self.state == 'closed':
            self.set_bar(ANGLE_CLOSED)
            if not self.violation and self.near(self.sensor2, self.sensor2_radius):
                self.violation = True
                self.get_logger().error(
                    'Sensor 2 crossed while the bar is down: mission failed.'
                )
            if self.now() - self.closed_at > self.closed_duration:
                self.get_logger().info('Opening the bar.')
                self.state = 'open'

        self.pub_state.publish(String(data=self.state))
        self.pub_violation.publish(Bool(data=self.violation))


def main(args=None):
    rclpy.init(args=args)
    node = SimLevelCrossing()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
