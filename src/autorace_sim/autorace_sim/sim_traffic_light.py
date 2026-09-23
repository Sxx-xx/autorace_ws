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

"""Cycle the simulated traffic light red -> yellow -> green.

gz-sim cannot swap a texture at runtime, so the three lit lamps exist as
separate models: the active one is teleported into its socket on the housing
and the others are parked below the course.

The node also publishes the ground truth colour on `/sim/traffic_light`, which
is only meant for debugging and scoring, never for the robot's own decision.
"""

import math

from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import UInt8


COLORS = ('red', 'yellow', 'green')

LIGHT_VALUE = {'red': 1, 'yellow': 2, 'green': 3}

# Socket heights in the housing frame, matching autorace_traffic_light.
SOCKET_Z = {'red': 0.075, 'yellow': 0.0, 'green': -0.075}

# Slightly in front of the housing face so the lamp is never z-fighting.
SOCKET_Y = -0.019

PARKED_Z = -5.0


class SimTrafficLight(Node):

    def __init__(self):
        super().__init__('sim_traffic_light')

        # Must match the housing pose in the world file.
        self.declare_parameter('housing_x', 1.3)
        self.declare_parameter('housing_y', -1.95)
        self.declare_parameter('housing_z', 0.13)
        self.declare_parameter('housing_yaw', -1.57)
        self.declare_parameter('duration_red', 5.0)
        self.declare_parameter('duration_yellow', 2.0)
        self.declare_parameter('duration_green', 5.0)
        self.declare_parameter('world', 'autorace')
        self.declare_parameter('start_delay', 5.0)

        self.housing = (
            self.get_parameter('housing_x').value,
            self.get_parameter('housing_y').value,
            self.get_parameter('housing_z').value,
        )
        self.yaw = self.get_parameter('housing_yaw').value
        self.durations = {
            'red': self.get_parameter('duration_red').value,
            'yellow': self.get_parameter('duration_yellow').value,
            'green': self.get_parameter('duration_green').value,
        }
        world = self.get_parameter('world').value

        self.pub_state = self.create_publisher(UInt8, '/sim/traffic_light', 1)
        self.client = self.create_client(SetEntityPose, f'/world/{world}/set_pose')

        self.index = 0
        self.placed = None
        self.next_switch = None
        self.start_delay = self.get_parameter('start_delay').value
        self.started_at = self.now()

        self.create_timer(0.1, self.update)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def socket_pose(self, color):
        """World pose of a lamp sitting in its socket."""
        x, y, z = self.housing
        offset_x = -SOCKET_Y * math.sin(self.yaw)
        offset_y = SOCKET_Y * math.cos(self.yaw)
        pose = Pose()
        pose.position.x = x + offset_x
        pose.position.y = y + offset_y
        pose.position.z = z + SOCKET_Z[color]
        pose.orientation.z = math.sin(self.yaw / 2.0)
        pose.orientation.w = math.cos(self.yaw / 2.0)
        return pose

    def parked_pose(self, color):
        pose = Pose()
        pose.position.x = self.housing[0]
        pose.position.y = self.housing[1]
        pose.position.z = PARKED_Z - SOCKET_Z[color]
        pose.orientation.w = 1.0
        return pose

    def move_lamp(self, color, pose):
        request = SetEntityPose.Request()
        request.entity = Entity(name=f'autorace_lamp_{color}', type=Entity.MODEL)
        request.pose = pose
        self.client.call_async(request)

    def show(self, color):
        for other in COLORS:
            if other == color:
                self.move_lamp(other, self.socket_pose(other))
            else:
                self.move_lamp(other, self.parked_pose(other))
        self.placed = color
        self.get_logger().info(f'Traffic light: {color}')

    def update(self):
        if not self.client.service_is_ready():
            # The bridge may not be up yet.
            return

        now = self.now()
        if now - self.started_at < self.start_delay:
            return

        if self.next_switch is None:
            self.show(COLORS[self.index])
            self.next_switch = now + self.durations[COLORS[self.index]]
        elif now >= self.next_switch:
            self.index = (self.index + 1) % len(COLORS)
            color = COLORS[self.index]
            self.show(color)
            self.next_switch = now + self.durations[color]

        self.pub_state.publish(UInt8(data=LIGHT_VALUE[self.placed]))


def main(args=None):
    rclpy.init(args=args)
    node = SimTrafficLight()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
