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

"""Priority based velocity command arbiter.

Every controller publishes on its own topic and the mux forwards the highest
priority input that is still fresh.  Inputs that stop publishing simply fall
away, so a mission node only has to publish while it wants control.

A stall watchdog guards the competition rule that a robot standing still for
30 s ends the run: after `stall_timeout` seconds without motion the mux drops
back to the lane controller, and if that does not help either it creeps
forward on its own.
"""

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


# Highest priority first.
INPUTS = (
    ('estop', '/cmd_vel/estop'),
    ('mission', '/cmd_vel/mission'),
    ('nav', '/cmd_vel/nav'),
    ('lane', '/cmd_vel/lane'),
)

FALLBACK_INPUT = 'lane'


class CmdVelMux(Node):

    def __init__(self):
        super().__init__('cmd_vel_mux')

        self.declare_parameter('input_timeout', 0.3)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('stall_timeout', 20.0)
        self.declare_parameter('creep_timeout', 25.0)
        self.declare_parameter('creep_speed', 0.05)
        self.declare_parameter('blocked_timeout', 3.0)
        self.declare_parameter('unstick_duration', 1.5)
        self.declare_parameter('unstick_speed', -0.07)

        self.input_timeout = self.get_parameter('input_timeout').value
        self.stall_timeout = self.get_parameter('stall_timeout').value
        self.creep_timeout = self.get_parameter('creep_timeout').value
        self.creep_speed = self.get_parameter('creep_speed').value
        self.blocked_timeout = self.get_parameter('blocked_timeout').value
        self.unstick_duration = self.get_parameter('unstick_duration').value
        self.unstick_speed = self.get_parameter('unstick_speed').value
        publish_rate = self.get_parameter('publish_rate').value

        self.commands = {name: None for name, _ in INPUTS}
        self.stamps = {name: 0.0 for name, _ in INPUTS}

        for name, topic in INPUTS:
            self.create_subscription(
                Twist,
                topic,
                lambda msg, key=name: self.callback_input(msg, key),
                1
            )

        # The watchdog only matters once the run is under way; waiting at a red
        # light before the start is not a stall.
        self.run_active = False
        self.create_subscription(Bool, '/autorace/run_active', self.callback_run_active, 1)
        self.create_subscription(Odometry, '/odom', self.callback_odom, 1)

        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 1)
        self.pub_active_input = self.create_publisher(Bool, '/autorace/mux_stalled', 1)

        self.last_motion_time = self.get_clock().now().nanoseconds / 1e9
        self.stall_override = False
        self.last_selected = None

        # Told to move but not moving: something is in the way.
        self.measured_speed = 0.0
        self.pushing_since = None
        self.unstick_until = None

        self.create_timer(1.0 / publish_rate, self.arbitrate)

    def callback_input(self, msg, key):
        self.commands[key] = msg
        self.stamps[key] = self.get_clock().now().nanoseconds / 1e9

    def callback_odom(self, msg):
        self.measured_speed = abs(msg.twist.twist.linear.x)

    def callback_run_active(self, msg):
        if msg.data and not self.run_active:
            self.last_motion_time = self.get_clock().now().nanoseconds / 1e9
        self.run_active = msg.data

    def select(self, now):
        """Return (name, twist) of the highest priority fresh input."""
        for name, _ in INPUTS:
            if self.stall_override and name != FALLBACK_INPUT:
                continue
            command = self.commands[name]
            if command is None:
                continue
            if now - self.stamps[name] > self.input_timeout:
                continue
            return name, command
        return None, None

    def arbitrate(self):
        now = self.get_clock().now().nanoseconds / 1e9
        name, command = self.select(now)

        if command is None:
            command = Twist()
            name = 'none'

        command, name = self.unstick(command, name, now)

        moving = abs(command.linear.x) > 1e-3 or abs(command.angular.z) > 1e-3
        if moving or not self.run_active:
            self.last_motion_time = now
            if self.stall_override:
                self.get_logger().info('Motion resumed, releasing stall override.')
                self.stall_override = False

        stalled_for = now - self.last_motion_time

        if self.run_active and stalled_for > self.stall_timeout and not self.stall_override:
            self.get_logger().error(
                f'No motion for {stalled_for:.1f} s, falling back to lane control.'
            )
            self.stall_override = True

        if self.run_active and stalled_for > self.creep_timeout:
            # Last resort: move, whatever the controllers say. Standing still
            # for 30 s ends the run.
            command = Twist()
            command.linear.x = self.creep_speed
            name = 'creep'

        if name != self.last_selected:
            self.get_logger().info(f'Active input: {self.last_selected} -> {name}')
            self.last_selected = name

        self.pub_cmd_vel.publish(command)
        self.pub_active_input.publish(Bool(data=self.stall_override))

    def unstick(self, command, name, now):
        """Back off when the robot is driving into something.

        Touching an obstacle costs nothing in this competition but pushing
        against it burns the clock, and the wheels turning is enough to hide
        the problem from the stall watchdog, which only sees the command.
        """
        if self.unstick_until is not None:
            if now < self.unstick_until:
                backing = Twist()
                backing.linear.x = self.unstick_speed
                return backing, 'unstick'
            self.unstick_until = None
            self.pushing_since = None
            return command, name

        wants_to_move = command.linear.x > 0.01
        if wants_to_move and self.measured_speed < 0.01:
            if self.pushing_since is None:
                self.pushing_since = now
            elif now - self.pushing_since > self.blocked_timeout:
                self.get_logger().error('Robot is blocked, backing off.')
                self.unstick_until = now + self.unstick_duration
        else:
            self.pushing_since = None

        return command, name

    def shut_down(self):
        self.pub_cmd_vel.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMux()
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
