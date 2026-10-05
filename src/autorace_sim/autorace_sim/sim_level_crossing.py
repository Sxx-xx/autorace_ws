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

Where sensor 1 stands is not known beforehand, so it is put somewhere new on
the approach for every lap, unless `sensor1_distance` (or a message on
`/sim/level_crossing/sensor1_distance`) says how far before the bar. Both
sensors are marked on the floor for whoever is watching the simulation; the
robot's cameras do not see the marks.
"""

import math
import random

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import String

try:
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.pose_pb2 import Pose as GzPose
    from gz.transport13 import Node as GzNode
except ImportError:
    GzNode = None


ANGLE_CLOSED = 0.0
ANGLE_OPEN = math.pi / 2.0


class SimLevelCrossing(Node):

    def __init__(self):
        super().__init__('sim_level_crossing')

        # The sensors stand on the course, so it is the robot's true position
        # they see: the simulator's own, published by the robot model's
        # PosePublisher and bridged to /sim/robot_pose. (Subscribing to
        # Gazebo's pose topic directly fell further and further behind.)
        # Until it arrives odometry is used instead: it starts at the spawn
        # pose, so the offset turns it into world coordinates, as long as the
        # robot has not been moved by hand.
        # The positions come from params/course.yaml, which is generated
        # together with the world file.
        self.declare_parameter('robot_name', 'autorace_burger')
        self.declare_parameter('world_name', 'autorace')
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw', 0.0)

        # Sensor 1: the trigger point on the approach.
        self.declare_parameter('sensor1_x', 0.0)
        self.declare_parameter('sensor1_y', 0.0)
        # Sensor 1 sees the robot this near it, coming up the road towards
        # the bar: the zigzag road passes close behind it, going the other way.
        self.declare_parameter('sensor1_radius', 0.12)
        # How far before the bar sensor 1 stands, along the line from the
        # bar back through sensor 2. Zero puts it somewhere between the
        # minimum and the maximum, anew for every lap.
        self.declare_parameter('sensor1_distance', 0.0)
        self.declare_parameter('sensor1_distance_min', 0.30)
        self.declare_parameter('sensor1_distance_max', 0.85)
        self.declare_parameter('bar_x', 0.0)
        self.declare_parameter('bar_y', 0.0)
        # Sensor 2: 6 cm in front of the bar, per the rules.
        self.declare_parameter('sensor2_x', 0.0)
        self.declare_parameter('sensor2_y', 0.0)
        self.declare_parameter('sensor2_radius', 0.08)
        self.declare_parameter('closed_duration', 10.0)
        # Sensor 1 closes the bar once, and again only after the robot has
        # been this far away from it.
        self.declare_parameter('rearm_distance', 1.5)

        self.robot_name = self.get_parameter('robot_name').value
        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        self.start_yaw = self.get_parameter('start_yaw').value
        self.sensor1 = (
            self.get_parameter('sensor1_x').value,
            self.get_parameter('sensor1_y').value,
        )
        self.sensor1_radius = self.get_parameter('sensor1_radius').value
        self.sensor1_distance = self.get_parameter('sensor1_distance').value
        self.sensor1_range = (self.get_parameter('sensor1_distance_min').value,
                              self.get_parameter('sensor1_distance_max').value)
        self.bar = (self.get_parameter('bar_x').value, self.get_parameter('bar_y').value)
        self.sensor2_radius = self.get_parameter('sensor2_radius').value
        self.closed_duration = self.get_parameter('closed_duration').value
        self.rearm_distance = self.get_parameter('rearm_distance').value
        self.sensor2 = (
            self.get_parameter('sensor2_x').value,
            self.get_parameter('sensor2_y').value,
        )

        self.pub_bar = self.create_publisher(Float64, '/level_bar/cmd', 1)
        self.pub_state = self.create_publisher(String, '/sim/level_crossing', 1)
        self.pub_violation = self.create_publisher(Bool, '/sim/level_crossing_violation', 1)

        self.create_subscription(Odometry, '/odom', self.callback_odom, 1)
        self.create_subscription(
            Float64, '/sim/level_crossing/sensor1_distance', self.callback_distance, 1)
        self.truth_seen = False
        self.create_subscription(Pose, '/sim/robot_pose', self.callback_truth, 1)
        # Gazebo's own transport is used only to move the mark of sensor 1.
        self.gz_node = GzNode() if GzNode is not None else None
        self.world = self.get_parameter('world_name').value

        self.state = 'open'
        self.closed_at = None
        self.triggered = False
        self.violation = False
        self.position = None
        self.yaw = None

        self.mark_placed = True
        self.place_sensor1()
        self.create_timer(0.1, self.update)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def callback_distance(self, msg):
        """Put sensor 1 this far before the bar; zero for anywhere."""
        self.sensor1_distance = msg.data
        self.triggered = False
        self.violation = False
        self.place_sensor1()

    def place_sensor1(self):
        """Stand sensor 1 on the approach and move its mark there."""
        to_sensor2 = (self.sensor2[0] - self.bar[0], self.sensor2[1] - self.bar[1])
        length = math.hypot(*to_sensor2)
        if length < 1e-6:
            # No bar position given: sensor 1 stays where the parameters put it.
            return
        distance = self.sensor1_distance
        if distance <= 0.0:
            distance = random.uniform(*self.sensor1_range)
        self.sensor1 = (self.bar[0] + to_sensor2[0] / length * distance,
                        self.bar[1] + to_sensor2[1] / length * distance)
        self.get_logger().info(f'Sensor 1 is {distance:.2f} m before the bar.')
        self.mark_placed = False
        self.move_mark()

    def move_mark(self):
        """Move the mark of sensor 1 to where the sensor is; retried until done."""
        if self.gz_node is None:
            self.mark_placed = True
            return
        mark = GzPose()
        mark.name = 'crossing_sensor_1'
        mark.position.x, mark.position.y, mark.position.z = *self.sensor1, 0.011
        mark.orientation.w = 1.0
        done, reply = self.gz_node.request(
            f'/world/{self.world}/set_pose', mark, GzPose, Boolean, 500)
        self.mark_placed = bool(done and reply.data)

    def callback_truth(self, msg):
        self.truth_seen = True
        self.position = (msg.position.x, msg.position.y)
        q = msg.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def callback_odom(self, msg):
        if self.truth_seen:
            return
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

    def towards_bar(self):
        """Whether the robot is heading up the road to the bar (true unless known otherwise)."""
        if self.yaw is None:
            return True
        approach = math.atan2(self.bar[1] - self.sensor1[1], self.bar[0] - self.sensor1[0])
        return abs(math.atan2(math.sin(self.yaw - approach), math.cos(self.yaw - approach))) < 1.0

    def set_bar(self, angle):
        self.pub_bar.publish(Float64(data=angle))

    def update(self):
        if not self.mark_placed:
            self.move_mark()
        if self.state == 'open':
            self.set_bar(ANGLE_OPEN)
            if self.triggered and not self.near(self.sensor1, self.rearm_distance):
                # The robot has left; the crossing is ready for the next lap.
                self.triggered = False
                self.violation = False
                self.place_sensor1()
            if (not self.triggered and self.near(self.sensor1, self.sensor1_radius)
                    and self.towards_bar()):
                self.get_logger().info(
                    'Sensor 1 triggered, closing the bar (robot at %.2f, %.2f, yaw %.2f; '
                    'sensor at %.2f, %.2f).' % (
                        self.position[0], self.position[1],
                        self.yaw if self.yaw is not None else 0.0, *self.sensor1))
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
