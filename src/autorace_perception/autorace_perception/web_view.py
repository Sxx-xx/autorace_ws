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

"""Image topics as MJPEG streams in a browser: http://<robot>:8090/

The robot runs Jazzy and the Windows PC has no matching ROS, so instead of
rqt over DDS the debug images are served over plain HTTP.  Each topic is
re-encoded only when a frame arrives and only while someone is watching.

On the bird's eye view a metric grid is drawn: the two lines where the lane
edges should sit when the robot is centred, and a tick every 10 cm ahead.  If
the camera height and pitch are right, the painted lines lie on the grid lines
and stay parallel.
"""

from http.server import BaseHTTPRequestHandler
from http.server import ThreadingHTTPServer
import threading
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image


GRID_COLOUR = (255, 0, 255)
TICK = 0.10


class WebView(Node):

    def __init__(self):
        super().__init__('web_view')

        self.declare_parameter('port', 8090)
        self.declare_parameter('topics', [
            '/camera_lane/image_raw', '/camera/image_projected', '/detect/image_lane'])
        # Captions under the streams, in the order of `topics` (topic name if missing).
        self.declare_parameter('labels', [''])
        self.declare_parameter('jpeg_quality', 70)
        self.declare_parameter('grid_topic', '/camera/image_projected')
        self.declare_parameter('bev.pixels_per_meter', 1200.0)
        self.declare_parameter('bev.near', 0.07)
        self.declare_parameter('lane.width_m', 0.25)

        self.topics = list(self.get_parameter('topics').value)
        labels = list(self.get_parameter('labels').value)
        self.labels = [(labels[i] if i < len(labels) and labels[i] else t)
                       for i, t in enumerate(self.topics)]
        self.quality = self.get_parameter('jpeg_quality').value
        self.grid_topic = self.get_parameter('grid_topic').value
        self.ppm = self.get_parameter('bev.pixels_per_meter').value
        self.near = self.get_parameter('bev.near').value
        self.lane_width = self.get_parameter('lane.width_m').value

        self.bridge = CvBridge()
        self.frames = [None] * len(self.topics)
        self.watchers = [0] * len(self.topics)
        self.cond = threading.Condition()
        for i, topic in enumerate(self.topics):
            self.create_subscription(Image, topic, lambda msg, i=i: self.callback(i, msg), 1)

        port = self.get_parameter('port').value
        self.server = ThreadingHTTPServer(('0.0.0.0', port), self.make_handler())
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.get_logger().info(f'Serving {len(self.topics)} streams on http://0.0.0.0:{port}/')

    def draw_grid(self, image):
        h, w = image.shape[:2]
        centre = w // 2
        half = int(0.5 * self.lane_width * self.ppm)
        for x in (centre - half, centre + half):
            cv2.line(image, (x, 0), (x, h - 1), GRID_COLOUR, 1)
        cv2.line(image, (centre, h - 1), (centre, h - 20), GRID_COLOUR, 1)
        distance = TICK * (int(self.near / TICK) + 1)
        while True:
            y = int(h - 1 - (distance - self.near) * self.ppm)
            if y < 0:
                break
            cv2.line(image, (0, y), (12, y), GRID_COLOUR, 1)
            cv2.putText(image, f'{distance:.1f}', (14, y + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, GRID_COLOUR, 1)
            distance += TICK

    def callback(self, i, msg):
        if not self.watchers[i]:
            return
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        if self.topics[i] == self.grid_topic:
            self.draw_grid(image)
        ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if ok:
            with self.cond:
                self.frames[i] = jpeg.tobytes()
                self.cond.notify_all()

    def make_handler(self):
        node = self

        class Handler(BaseHTTPRequestHandler):

            def log_message(self, *args):
                pass

            def do_GET(self):
                if self.path == '/':
                    self.index()
                elif self.path.startswith('/stream/'):
                    self.stream(int(self.path.rsplit('/', 1)[1]))
                else:
                    self.send_error(404)

            def index(self):
                cells = ''.join(
                    f'<figure><img src="/stream/{i}"><figcaption>{t}</figcaption></figure>'
                    for i, t in enumerate(node.labels))
                body = (
                    '<!doctype html><meta charset="utf-8"><title>AutoRace view</title>'
                    '<style>body{background:#111;color:#ddd;font:14px monospace;margin:8px}'
                    'figure{display:inline-block;margin:4px;vertical-align:top}'
                    'img{max-width:100%;image-rendering:pixelated}</style>' + cells
                ).encode()
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.end_headers()
                self.wfile.write(body)

            def stream(self, i):
                if not 0 <= i < len(node.topics):
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.end_headers()
                node.watchers[i] += 1
                last = None
                try:
                    while rclpy.ok():
                        with node.cond:
                            node.cond.wait_for(lambda: node.frames[i] is not last, timeout=1.0)
                            frame = node.frames[i]
                        if frame is None or frame is last:
                            time.sleep(0.05)
                            continue
                        last = frame
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n'
                                         + f'Content-Length: {len(frame)}\r\n\r\n'.encode()
                                         + frame + b'\r\n')
                except (BrokenPipeError, ConnectionResetError):
                    pass
                finally:
                    node.watchers[i] -= 1

        return Handler


def main(args=None):
    rclpy.init(args=args)
    node = WebView()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.server.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
