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

"""AutoRace mission state machine.

The manager owns nothing but the decision of *which* behavior is allowed to
drive.  Lane following runs all the time; when a sign (or an explicit trigger
from a detector) says a mission section is coming up, the matching mission node
is activated over `/autorace/mission/<name>/active` and reports back on
`/autorace/mission/<name>/done`.

The course layout is only published on race day, so nothing here assumes an
order of missions.  Every mission is armed independently and fires once.
"""

from autorace_msgs.msg import MissionState
from autorace_msgs.msg import SignDetection
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import UInt8


# Traffic light colour published by autorace_perception/detect_traffic_light.
LIGHT_NONE = 0
LIGHT_RED = 1
LIGHT_YELLOW = 2
LIGHT_GREEN = 3

# Mission name -> MissionState constant. The name is also the namespace used
# for the activate/done topics.
MISSIONS = {
    'intersection': MissionState.INTERSECTION,
    'construction': MissionState.CONSTRUCTION,
    'parking': MissionState.PARKING,
    'level_crossing': MissionState.LEVEL_CROSSING,
    'tunnel': MissionState.TUNNEL,
}

# Sign name (from the aggregator) -> mission it triggers.
SIGN_TO_MISSION = {
    'intersection': 'intersection',
    # The direction signs stand at the fork itself; seeing one without having
    # seen the warning sign before it still means the fork is here.
    'left': 'intersection',
    'right': 'intersection',
    'construction': 'construction',
    'parking': 'parking',
    'stop': 'level_crossing',
    'tunnel': 'tunnel',
}

STATE_NAMES = {
    MissionState.STANDBY: 'STANDBY',
    MissionState.LANE_DRIVE: 'LANE_DRIVE',
    MissionState.INTERSECTION: 'INTERSECTION',
    MissionState.CONSTRUCTION: 'CONSTRUCTION',
    MissionState.PARKING: 'PARKING',
    MissionState.LEVEL_CROSSING: 'LEVEL_CROSSING',
    MissionState.TUNNEL: 'TUNNEL',
    MissionState.FINISH: 'FINISH',
}


class MissionManager(Node):

    def __init__(self):
        super().__init__('mission_manager')

        self.declare_parameter('auto_start', False)
        self.declare_parameter('green_streak', 5)
        self.declare_parameter('sign_streak', 3)
        self.declare_parameter('standby_timeout', 90.0)
        self.declare_parameter('mission_timeout.intersection', 45.0)
        self.declare_parameter('mission_timeout.construction', 60.0)
        self.declare_parameter('mission_timeout.parking', 60.0)
        self.declare_parameter('mission_timeout.level_crossing', 45.0)
        self.declare_parameter('mission_timeout.tunnel', 90.0)
        self.declare_parameter('enabled_missions', list(MISSIONS.keys()))

        self.green_streak = self.get_parameter('green_streak').value
        self.sign_streak = self.get_parameter('sign_streak').value
        self.standby_timeout = self.get_parameter('standby_timeout').value
        self.enabled = set(self.get_parameter('enabled_missions').value)
        self.timeouts = {
            name: self.get_parameter(f'mission_timeout.{name}').value for name in MISSIONS
        }

        self.state = MissionState.STANDBY
        self.detail = 'waiting for green light'
        self.completed = set()
        self.run_start = None
        self.state_start = self.now()
        self.green_count = 0

        self.pub_state = self.create_publisher(MissionState, '/autorace/mission_state', 1)
        self.pub_run_active = self.create_publisher(Bool, '/autorace/run_active', 1)
        self.pub_activate = {
            name: self.create_publisher(Bool, f'/autorace/mission/{name}/active', 1)
            for name in MISSIONS
        }

        self.create_subscription(SignDetection, '/autorace/sign', self.callback_sign, 5)
        self.create_subscription(UInt8, '/detect/traffic_light', self.callback_light, 1)
        self.create_subscription(String, '/autorace/trigger', self.callback_trigger, 5)
        self.create_subscription(Bool, '/autorace/manual_start', self.callback_manual_start, 1)
        for name in MISSIONS:
            self.create_subscription(
                Bool,
                f'/autorace/mission/{name}/done',
                lambda msg, key=name: self.callback_done(msg, key),
                1
            )

        if self.get_parameter('auto_start').value:
            self.get_logger().warn('auto_start is set, skipping the traffic light.')
            self.start_run('auto_start')

        self.create_timer(0.1, self.update)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    # --- callbacks -------------------------------------------------------

    def callback_light(self, msg):
        if self.state != MissionState.STANDBY:
            return
        if msg.data == LIGHT_GREEN:
            self.green_count += 1
            self.detail = f'green {self.green_count}/{self.green_streak}'
            if self.green_count >= self.green_streak:
                self.start_run('green light')
        else:
            # A single stray green frame must not start the run.
            self.green_count = 0

    def callback_manual_start(self, msg):
        if msg.data and self.state == MissionState.STANDBY:
            self.get_logger().warn('Manual start requested.')
            self.start_run('manual')

    def callback_sign(self, msg):
        if msg.streak < self.sign_streak:
            return
        mission = SIGN_TO_MISSION.get(msg.name)
        if mission is not None:
            self.request_mission(mission, f'sign:{msg.name}')

    def callback_trigger(self, msg):
        """Let a detector ask for a mission directly, e.g. the level crossing bar."""
        self.request_mission(msg.data, 'trigger')

    def callback_done(self, msg, mission):
        if not msg.data:
            return
        if self.state != MISSIONS.get(mission):
            return
        self.get_logger().info(f'Mission {mission} reported done.')
        self.finish_mission(mission)

    # --- state machine ---------------------------------------------------

    def start_run(self, reason):
        self.run_start = self.now()
        self.get_logger().info(f'Run started ({reason}).')
        self.transition(MissionState.LANE_DRIVE, 'lane following')

    def request_mission(self, mission, reason):
        if mission not in MISSIONS:
            self.get_logger().warn(f'Unknown mission requested: {mission}')
            return
        if mission not in self.enabled:
            return
        if self.state != MissionState.LANE_DRIVE:
            # Already busy, or not running yet.
            return
        if mission in self.completed:
            return
        self.get_logger().info(f'Entering mission {mission} ({reason}).')
        self.set_mission_active(mission, True)
        self.transition(MISSIONS[mission], f'{mission}:{reason}')

    def finish_mission(self, mission, timed_out=False):
        self.set_mission_active(mission, False)
        self.completed.add(mission)
        detail = f'{mission} timeout' if timed_out else f'{mission} done'
        self.transition(MissionState.LANE_DRIVE, detail)

    def transition(self, state, detail):
        if state != self.state:
            self.get_logger().info(
                f'{STATE_NAMES.get(self.state)} -> {STATE_NAMES.get(state)} ({detail})'
            )
        self.state = state
        self.detail = detail
        self.state_start = self.now()

    def set_mission_active(self, mission, active):
        self.pub_activate[mission].publish(Bool(data=active))

    def active_mission(self):
        for name, state in MISSIONS.items():
            if state == self.state:
                return name
        return None

    def update(self):
        now = self.now()
        if self.state_start == 0.0:
            # Simulated time had not started when the node came up; counting
            # from zero would put the standby timeout in the past already.
            self.state_start = now
        state_elapsed = now - self.state_start

        if self.state == MissionState.STANDBY and state_elapsed > self.standby_timeout:
            self.get_logger().error(
                'No green light seen within %.0f s, starting anyway.' % self.standby_timeout
            )
            self.start_run('standby timeout')

        mission = self.active_mission()
        if mission is not None:
            # Keep the activation latched for nodes that start late.
            self.set_mission_active(mission, True)
            if state_elapsed > self.timeouts[mission]:
                self.get_logger().error(f'Mission {mission} timed out, returning to lane.')
                self.finish_mission(mission, timed_out=True)

        self.publish_state(now)

    def publish_state(self, now):
        msg = MissionState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = self.state
        msg.detail = self.detail
        msg.state_elapsed = float(now - self.state_start)
        msg.run_elapsed = float(now - self.run_start) if self.run_start else 0.0
        mask = 0
        for name in self.completed:
            mask |= 1 << MISSIONS[name]
        msg.completed_mask = mask
        self.pub_state.publish(msg)
        self.pub_run_active.publish(Bool(data=self.run_start is not None))


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
