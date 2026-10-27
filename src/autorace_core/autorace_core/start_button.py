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

"""Start button: one window with the cameras, a key starts the run.

Left: /detect/image_sign, the forward camera with the signs the detector
found drawn on it. Right, top: the lane camera as it comes in
(/camera_lane/image_decoded). Right, bottom: /detect/image_lane, the lane
detector's map round the robot (yellow and white lines, the arc of
candidates in green, the target it steers for in red, its state). Along the
top: the mission manager's state, the last sign and its streak. The robot
stands still until a key is pressed:

    SPACE   start the run (/autorace/manual_start; the manager leaves
            STANDBY and lane following begins)
    p       ask for the mission now (/autorace/trigger), for a test
            without the sign: the first of the launch's mission list;
            1..5 ask for the first..fifth of the list
    s       stop: zero velocity on /cmd_vel/estop, which the mux puts
            above every mission, for as long as the stop is on; SPACE
            lets go again (the run itself goes on from where it was)
    q, ESC  close the window (the launch is still running; Ctrl+C there)

The keys only work with the window in front (OpenCV reads them from the
window, not the terminal). Run it from parking_test.launch.py, which keeps
the manager in STANDBY until SPACE.
"""

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import String

from autorace_core.mission_manager import STATE_NAMES
from autorace_msgs.msg import MissionState
from autorace_msgs.msg import SignDetection


WINDOW = 'AutoRace start  [SPACE start | p/1-5 mission | s stop | q quit]'


class StartButton(Node):

    def __init__(self):
        super().__init__('start_button')
        self.declare_parameter('image_topic', '/detect/image_sign')
        self.declare_parameter('lane_camera_topic', '/camera_lane/image_decoded')
        self.declare_parameter('lane_map_topic', '/detect/image_lane')
        self.declare_parameter('mission', 'parking')
        image_topic = self.get_parameter('image_topic').value
        self.missions = [m for m in self.get_parameter('mission').value.replace(' ', '').split(',') if m]

        self.bridge = CvBridge()
        # topic -> [picture, when]
        self.pictures = {}
        self.state = None
        self.state_time = 0.0
        self.sign = ''
        self.sign_time = 0.0
        self.started = False
        self.stopping = False
        self.notice = ''
        self.notice_time = 0.0

        self.panels = [image_topic, self.get_parameter('lane_camera_topic').value,
                       self.get_parameter('lane_map_topic').value]
        for topic in self.panels:
            self.pictures[topic] = [None, 0.0]
            self.create_subscription(
                Image, topic, lambda msg, key=topic: self.callback_image(msg, key), 1)
        self.create_subscription(MissionState, '/autorace/mission_state', self.callback_state, 1)
        self.create_subscription(SignDetection, '/autorace/sign', self.callback_sign, 5)
        self.pub_start = self.create_publisher(Bool, '/autorace/manual_start', 1)
        self.pub_trigger = self.create_publisher(String, '/autorace/trigger', 5)
        self.pub_estop = self.create_publisher(Twist, '/cmd_vel/estop', 1)

        cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
        self.create_timer(0.03, self.update)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def callback_image(self, msg, topic):
        self.pictures[topic] = [self.bridge.imgmsg_to_cv2(msg, 'bgr8'), self.now()]

    def panel(self, topic, height, width, now):
        """The topic's last picture scaled to fit height x width, or a note that it is missing."""
        picture, when = self.pictures[topic]
        out = np.zeros((height, width, 3), dtype=np.uint8)
        if picture is None or now - when > 2.0:
            cv2.putText(out, f'no picture: {topic}', (10, height // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
            return out
        h, w = picture.shape[:2]
        scale = min(height / h, width / w)
        small = cv2.resize(picture, (max(1, int(w * scale)), max(1, int(h * scale))))
        y = (height - small.shape[0]) // 2
        x = (width - small.shape[1]) // 2
        out[y:y + small.shape[0], x:x + small.shape[1]] = small
        cv2.putText(out, topic, (4, height - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        return out

    def callback_state(self, msg):
        self.state = msg
        self.state_time = self.now()
        if msg.state != MissionState.STANDBY:
            self.started = True

    def callback_sign(self, msg):
        self.sign = f'{msg.name} x{msg.streak}'
        self.sign_time = self.now()

    def say(self, text):
        self.notice = text
        self.notice_time = self.now()
        self.get_logger().info(text)

    def key(self, code):
        if code in (ord('q'), 27):
            raise KeyboardInterrupt
        if code == ord(' '):
            if self.stopping:
                self.stopping = False
                self.say('Stop released.')
            elif not self.started:
                for _ in range(3):
                    self.pub_start.publish(Bool(data=True))
                self.say('START: manual start sent.')
            else:
                self.say('Already running.')
        elif code == ord('p') or ord('1') <= code <= ord('5'):
            index = 0 if code == ord('p') else code - ord('1')
            if index >= len(self.missions):
                self.say(f'No mission {index + 1} in the list {self.missions}.')
                return
            self.pub_trigger.publish(String(data=self.missions[index]))
            self.say(f'Trigger sent: {self.missions[index]}'
                     + ('' if self.started else ' (start the run first: SPACE)'))
        elif code == ord('s'):
            self.stopping = True
            self.say('STOP: holding zero velocity. SPACE releases.')

    def update(self):
        now = self.now()
        if self.stopping:
            self.pub_estop.publish(Twist())

        sign, lane, lane_map = self.panels
        left = self.panel(sign, 480, 640, now)
        right = np.vstack([self.panel(lane, 240, 320, now), self.panel(lane_map, 240, 320, now)])
        frame = np.hstack([left, right])

        if self.state is not None and now - self.state_time < 1.0:
            name = STATE_NAMES.get(self.state.state, str(self.state.state))
            status = f'{name}  {self.state.detail}  {self.state.state_elapsed:.0f}s'
        else:
            status = 'mission_manager: no state (not running?)'
        seen = self.sign if now - self.sign_time < 1.0 else 'none'
        lines = [status, f'sign: {seen}']
        if self.stopping:
            lines.append('STOPPED (SPACE releases)')
        elif not self.started:
            lines.append('SPACE to start')
        if now - self.notice_time < 3.0:
            lines.append(self.notice)

        pad = np.zeros((16 * len(lines) + 8, frame.shape[1], 3), dtype=np.uint8)
        for i, text in enumerate(lines):
            colour = (0, 0, 255) if ('STOP' in text) else (0, 255, 0)
            cv2.putText(pad, text, (6, 16 * i + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, 1)
        cv2.imshow(WINDOW, np.vstack([pad, frame]))
        code = cv2.waitKey(1) & 0xFF
        if code != 0xFF:
            self.key(code)


def main(args=None):
    rclpy.init(args=args)
    node = StartButton()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
