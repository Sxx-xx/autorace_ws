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

"""USB webcam on the real robot: V4L2 MJPEG frames to image_raw + camera_info.

The webcams deliver MJPEG in hardware, which keeps the Pi's CPU free; OpenCV
decodes it.  The camera has not been calibrated yet, so the intrinsics come
from parameters: either fx/fy/cx/cy straight from a calibration, or, failing
that, a pinhole guess from the horizontal field of view.  The bird's eye view
only needs fx, fy, cx and cy, so distortion is left out.

The device must not be held by anything else (stop ustreamer first).
"""

import math
import subprocess
import threading

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image


class UsbCamera(Node):

    def __init__(self):
        super().__init__('usb_camera')

        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('width', 432)
        self.declare_parameter('height', 240)
        self.declare_parameter('fps', 15)
        self.declare_parameter('fourcc', 'MJPG')
        self.declare_parameter('frame_id', 'camera_lane')
        # Used for fx when no calibration is given (rad, across the image).
        self.declare_parameter('hfov', 1.20)
        self.declare_parameter('fx', 0.0)
        self.declare_parameter('fy', 0.0)
        self.declare_parameter('cx', 0.0)
        self.declare_parameter('cy', 0.0)
        # Some cameras are mounted upside down.
        self.declare_parameter('rotate_180', False)
        # V4L2 controls set after the device is opened, as 'name=value'
        # (see `v4l2-ctl -d <device> --list-ctrls`). Fixing exposure and white
        # balance keeps glare from swinging the whole picture's brightness.
        self.declare_parameter('v4l2_controls', [''])

        device = self.get_parameter('device').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        fourcc = self.get_parameter('fourcc').value
        self.frame_id = self.get_parameter('frame_id').value
        self.rotate = self.get_parameter('rotate_180').value

        self.capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.capture.isOpened():
            raise RuntimeError(f'Cannot open {device} (is ustreamer still running?)')
        self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.capture.set(cv2.CAP_PROP_FPS, fps)
        # Keep only the newest frame: a queue of old ones is pure latency.
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        width = int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.set_controls(device, [c for c in self.get_parameter('v4l2_controls').value if c])

        fx = self.get_parameter('fx').value
        if not fx:
            fx = 0.5 * width / math.tan(0.5 * self.get_parameter('hfov').value)
        fy = self.get_parameter('fy').value or fx
        cx = self.get_parameter('cx').value or 0.5 * (width - 1)
        cy = self.get_parameter('cy').value or 0.5 * (height - 1)

        self.info = CameraInfo()
        self.info.width, self.info.height = width, height
        self.info.distortion_model = 'plumb_bob'
        self.info.d = [0.0] * 5
        self.info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        self.info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        self.bridge = CvBridge()
        self.pub_image = self.create_publisher(Image, 'image_raw', 1)
        self.pub_info = self.create_publisher(CameraInfo, 'camera_info', 1)
        self.get_logger().info(
            f'{device}: {width}x{height} @ {self.capture.get(cv2.CAP_PROP_FPS):.0f} fps, '
            f'fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}'
        )

        # read() blocks until the next frame, so it gets a thread of its own.
        self.running = True
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def set_controls(self, device, controls):
        # One at a time and in order: a manual value is refused while its
        # automatic mode is still on, so the mode has to go first.
        for control in controls:
            result = subprocess.run(['v4l2-ctl', '-d', device, '-c', control],
                                    capture_output=True, text=True)
            if result.returncode != 0:
                self.get_logger().warn(f'{control}: {result.stderr.strip()}')
        if controls:
            self.get_logger().info('V4L2 controls: ' + ', '.join(controls))

    def loop(self):
        failures = 0
        while self.running and rclpy.ok():
            ok, frame = self.capture.read()
            if not ok:
                failures += 1
                self.get_logger().warn('Frame grab failed', throttle_duration_sec=2.0)
                if failures > 50:
                    self.get_logger().error('Camera lost.')
                    return
                continue
            failures = 0
            if self.rotate:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            self.info.header = msg.header
            self.pub_image.publish(msg)
            self.pub_info.publish(self.info)

    def shut_down(self):
        self.running = False
        self.thread.join(timeout=1.0)
        self.capture.release()


def main(args=None):
    rclpy.init(args=args)
    node = UsbCamera()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shut_down()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
