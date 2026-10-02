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
"""Traffic light mission: wait at the stop line for the green light.

Before the run has started the robot may drive up to the stop line, but not
over it. The mission manager starts the run once it has seen the green light;
until then this node watches for the line, and when the line is
`stop_distance` ahead it holds the robot there through the mission input of
the mux, which outranks the lane follower. Holding means keeping the line at
that distance, not just commanding a standstill: a robot left alone for a long
red light can creep, and the line is right there to measure against. It also
means facing down the lane. The light stands beside the road beyond the line,
and a robot that has stopped askew may have it outside the picture, where it
would wait for a green it cannot see. Down the lane is the way its lines run,
not the way to the point the lane follower steers for: a robot standing to
one side of the lane would turn towards that point and away from the lane.

If the light is green before the robot reaches the line the run has already
started, and the robot drives through without stopping.
"""

import math

from geometry_msgs.msg import Twist
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool
from std_msgs.msg import Float64


class TrafficLightMission(Node):

    def __init__(self):
        super().__init__('traffic_light_mission')

        # From the axle to the near edge of the line. The front of the robot
        # is about 7 cm ahead of the axle and it takes a few centimetres to
        # stop.
        self.declare_parameter('stop_distance', 0.12)
        # Holding the line at that distance: speed per metre of error, and
        # the most it may move at.
        self.declare_parameter('hold_gain', 1.0)
        self.declare_parameter('hold_speed', 0.03)
        self.declare_parameter('hold_tolerance', 0.005)
        # Turning to face down the lane: turn rate per radian off, the most
        # it may turn at, and how close is close enough.
        self.declare_parameter('face_gain', 1.5)
        self.declare_parameter('face_rate', 0.4)
        self.declare_parameter('face_tolerance', 0.03)
        self.stop_distance = self.get_parameter('stop_distance').value
        self.hold_gain = self.get_parameter('hold_gain').value
        self.hold_speed = self.get_parameter('hold_speed').value
        self.hold_tolerance = self.get_parameter('hold_tolerance').value
        self.face_gain = self.get_parameter('face_gain').value
        self.face_rate = self.get_parameter('face_rate').value
        self.face_tolerance = self.get_parameter('face_tolerance').value

        self.run_active = False
        self.holding = False
        self.line_distance = None
        self.line_time = 0.0
        self.lane_angle = 0.0       # the way the lane runs, rad to the left of the heading
        self.lane_time = 0.0

        self.create_subscription(Bool, '/autorace/run_active', self.callback_run_active, 1)
        self.create_subscription(Float64, '/detect/stop_line', self.callback_stop_line, 1)
        for colour, inner in (('yellow', 10), ('white', 90)):
            self.create_subscription(
                PointCloud2, f'/detect/lane_lines/{colour}',
                lambda msg, edge=inner: self.callback_line(msg, edge), 1)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel/mission', 1)

        self.create_timer(0.05, self.update)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def callback_run_active(self, msg):
        if msg.data and self.holding:
            self.get_logger().info('Traffic light: green, going.')
            self.holding = False
        self.run_active = msg.data

    def callback_line(self, msg, inner):
        """Take the way the lane runs from the edge of one of its lines.

        The edge towards the lane is followed rather than the whole line: the
        far side of a line is cut off by the edge of the picture, which
        slants. `inner` is the percentile of the points' offsets to the left
        that lies on that edge.
        """
        if self.run_active:
            return
        points = point_cloud2.read_points_numpy(msg, field_names=('x', 'y'))
        if len(points) < 100:
            return
        step = np.int_(points[:, 0] / 0.02)
        ahead, edge = [], []
        for index in np.unique(step):
            across = points[step == index, 1]
            if len(across) >= 5:
                ahead.append((index + 0.5) * 0.02)
                edge.append(np.percentile(across, inner))
        if len(ahead) < 6:
            return
        slope = np.polyfit(ahead, edge, 1)[0]
        self.lane_angle = math.atan(slope)
        self.lane_time = self.now()

    def callback_stop_line(self, msg):
        self.line_distance = msg.data
        self.line_time = self.now()
        if self.run_active or self.holding or msg.data > self.stop_distance:
            return
        self.get_logger().info('Traffic light: waiting at the stop line.')
        self.holding = True

    def update(self):
        if not self.holding:
            return
        twist = Twist()
        now = self.now()
        if now - self.lane_time < 0.5 and abs(self.lane_angle) > self.face_tolerance:
            rate = self.face_gain * self.lane_angle
            twist.angular.z = max(-self.face_rate, min(self.face_rate, rate))
        if now - self.line_time < 0.5:
            error = self.line_distance - self.stop_distance
            if abs(error) > self.hold_tolerance:
                speed = self.hold_gain * error
                twist.linear.x = max(-self.hold_speed, min(self.hold_speed, speed))
        self.pub_cmd_vel.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
