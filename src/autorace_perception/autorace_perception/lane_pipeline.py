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

"""The lane camera to the lane target in one process and one callback: JPEG -> BEV -> lane.

The separate nodes (republish, bev_projector, detect_lane) hand each picture on
as a raw image topic: 250 KB decoded and 720 KB bird's eye view, serialised and
copied between processes in Python. On the Pi camera demo that held the chain
at 23 -> 11 -> 7 Hz with the CPU to spare. Here the JPEG from the robot is
decoded, projected and searched in the callback that receives it:

  imdecode -> GaussianBlur -> remap (bev_projector's table, worked out once from
  camera_info and the camera's height and pitch) -> detect_lane.process (or
  detect_lane_sw's: --detector sliding_window)

The subscription is best effort, depth 1: a picture that arrives while the last
is still being worked on replaces any older one waiting, so the newest is
always next and nothing queues.

Two nodes live in this process, under the names of the ones they replace, so
perception_*.yaml applies unchanged and everything downstream sees the same
topics: 'bev_projector' (this class) and 'detect_lane' (the detector, which
publishes /detect/lane_target, /detect/lane_state, the line points and its
picture). The bird's eye view (/camera/image_output, for detect_stop_line) and
the decoded picture (<camera>/image_decoded, for tools/lane_view.py) go out only
while someone subscribes, the latter at most preview_rate Hz.

stats:=true logs once a second: pictures in, pictures done, and the time each
took (p50 / p95 / max ms) split into decode, BEV and detection.
"""

import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image

from autorace_perception.bev_projector import BevProjector
from autorace_perception.detect_lane import DetectLane
from autorace_perception.detect_lane_sw import DetectLaneSlidingWindow


class LanePipeline(BevProjector):

    def __init__(self, detector):
        super().__init__()
        self.declare_parameter('stats', False)
        self.declare_parameter('preview_rate', 10.0)
        self.stats = self.get_parameter('stats').value
        self.preview_rate = self.get_parameter('preview_rate').value
        self.detector = detector

        self.create_subscription(CompressedImage, '/camera/image_compressed',
                                 self.callback_compressed, qos_profile_sensor_data)
        self.pub_preview = self.create_publisher(Image, '/camera/image_decoded', 1)
        self.preview_time = 0.0
        self.received = 0
        self.timings = []           # (decode, bev, detect) seconds, this second
        self.window_start = time.monotonic()

    def callback_compressed(self, msg):
        self.received += 1
        if not self.fx:
            self.get_logger().warn('Waiting for camera_info', throttle_duration_sec=5.0)
            return
        t0 = time.perf_counter()
        image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            self.get_logger().warn('undecodable picture', throttle_duration_sec=5.0)
            return
        t1 = time.perf_counter()
        bev = self.project_image(image)
        t2 = time.perf_counter()
        self.detector.process(bev, msg.header)
        t3 = time.perf_counter()

        # For whoever watches; nothing is converted when no one does.
        if self.pub_bev.get_subscription_count() > 0:
            out = self.bridge.cv2_to_imgmsg(bev, 'bgr8')
            out.header = msg.header
            self.pub_bev.publish(out)
        if (self.pub_preview.get_subscription_count() > 0
                and time.monotonic() - self.preview_time >= 1.0 / self.preview_rate):
            self.preview_time = time.monotonic()
            out = self.bridge.cv2_to_imgmsg(image, 'bgr8')
            out.header = msg.header
            self.pub_preview.publish(out)

        if self.stats:
            self.timings.append((t1 - t0, t2 - t1, t3 - t2))
            if time.monotonic() - self.window_start >= 1.0:
                self.log_stats()

    def log_stats(self):
        span = time.monotonic() - self.window_start
        t = np.array(self.timings) * 1000.0
        total = t.sum(axis=1)
        self.get_logger().info(
            f'stats: in {self.received / span:.1f} Hz, done {len(t) / span:.1f} Hz, '
            f'ms p50 {np.median(total):.1f} p95 {np.percentile(total, 95):.1f} '
            f'max {total.max():.1f} (decode {np.median(t[:, 0]):.1f}, '
            f'bev {np.median(t[:, 1]):.1f}, '
            f'detect {np.median(t[:, 2]):.1f})')
        self.received = 0
        self.timings = []
        self.window_start = time.monotonic()


def main(args=None):
    argv = remove_ros_args(args if args is not None else sys.argv)
    detector_name = argv[argv.index('--detector') + 1] if '--detector' in argv else 'detect_lane'
    rclpy.init(args=args)
    detector = {'detect_lane': DetectLane,
                'sliding_window': DetectLaneSlidingWindow}[detector_name]()
    # The detector gets its pictures from here, not from a topic.
    pipeline = LanePipeline(detector)
    executor = SingleThreadedExecutor()
    executor.add_node(pipeline)
    executor.add_node(detector)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        pipeline.destroy_node()
        detector.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
