#!/usr/bin/env python3
"""record_run.py OUT.mp4 [seconds] : a video of a run, to see afterwards what the robot saw.

Left: the lane camera (/camera_lane/image_raw/compressed). Right: the lane detector's map
(/detect/image_lane: yellow and white lines, the arc of candidates, the target in red).
On top: time, lane state, target, the velocity command, the mission state and the last sign.
15 fps. Also writes OUT.csv with one row per frame (for finding a moment in the video).
"""
import csv
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import UInt8

try:
    from autorace_msgs.msg import MissionState, SignDetection
except ImportError:          # lane-only runs
    MissionState = SignDetection = None

STATES = {0: 'none', 1: 'yellow only', 2: 'both', 3: 'white only'}
H = 360


class Recorder(Node):

    def __init__(self, out):
        super().__init__('record_run')
        self.out = out
        self.writer = None
        self.lane = None
        self.s = {'state': '-', 'target': None, 'cmd': (0.0, 0.0), 'mission': '-', 'sign': '-', 'sign_t': 0.0}
        self.t0 = time.time()
        self.frames = 0
        self.rows = []
        self.create_subscription(CompressedImage, '/camera_lane/image_raw/compressed', self.on_camera, 5)
        self.create_subscription(Image, '/detect/image_lane', self.on_lane, 1)
        self.create_subscription(UInt8, '/detect/lane_state',
                                 lambda m: self.s.__setitem__('state', STATES.get(m.data, m.data)), 1)
        self.create_subscription(PointStamped, '/detect/lane_target',
                                 lambda m: self.s.__setitem__('target', (m.point.x, m.point.y)), 1)
        self.create_subscription(Twist, '/cmd_vel',
                                 lambda m: self.s.__setitem__('cmd', (m.linear.x, m.angular.z)), 1)
        if MissionState is not None:
            self.create_subscription(MissionState, '/autorace/mission_state',
                                     lambda m: self.s.__setitem__('mission', m.detail), 1)
            self.create_subscription(SignDetection, '/autorace/sign', self.on_sign, 5)

    def on_sign(self, m):
        self.s['sign'] = m.name
        self.s['sign_t'] = time.time()

    def on_lane(self, m):
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
        if m.encoding == 'rgb8':
            a = cv2.cvtColor(a, cv2.COLOR_RGB2BGR)
        self.lane = a

    def on_camera(self, m):
        self.frames += 1
        if self.frames % 2:                       # 30 -> 15 fps
            return
        cam = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
        cam = cv2.resize(cam, (int(cam.shape[1] * H / cam.shape[0]), H))
        lane = self.lane if self.lane is not None else np.zeros((H, H, 3), np.uint8)
        lane = cv2.resize(lane, (int(lane.shape[1] * H / lane.shape[0]), H))
        frame = np.hstack([cam, lane])
        frame = np.vstack([np.zeros((70, frame.shape[1], 3), np.uint8), frame])
        t = time.time() - self.t0
        tgt = self.s['target']
        v, w = self.s['cmd']
        sign = self.s['sign'] if time.time() - self.s['sign_t'] < 1.0 else '-'
        line1 = f"t {t:5.1f}s  lines: {self.s['state']}  target: " + (
            f"{tgt[0]:.2f} ahead {tgt[1]:+.3f} left" if tgt else '-')
        line2 = f"cmd v {v:+.2f} w {w:+.2f}   mission: {self.s['mission']}   sign: {sign}"
        cv2.putText(frame, line1, (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        cv2.putText(frame, line2, (8, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
        if self.writer is None:
            self.size = (frame.shape[1], frame.shape[0])
            self.writer = cv2.VideoWriter(self.out, cv2.VideoWriter_fourcc(*'mp4v'), 15.0, self.size)
        # VideoWriter silently drops frames of any other size (the lane map's
        # size changes during a run): keep every frame the first one's size.
        if (frame.shape[1], frame.shape[0]) != self.size:
            frame = cv2.resize(frame, self.size)
        self.writer.write(frame)
        self.rows.append([f'{t:.2f}', self.s['state'], f'{tgt[0]:.3f}' if tgt else '', f'{tgt[1]:.3f}' if tgt else '',
                          f'{v:.2f}', f'{w:.2f}', self.s['mission'], sign])

    def close(self):
        if self.writer is not None:
            self.writer.release()
        with open(self.out.rsplit('.', 1)[0] + '.csv', 'w', newline='') as f:
            csv.writer(f).writerows([['t', 'lines', 'target_ahead', 'target_left', 'v', 'w', 'mission', 'sign']] + self.rows)


def main():
    out = sys.argv[1]
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 1e9
    rclpy.init()
    node = Recorder(out)
    t0 = time.time()
    try:
        while rclpy.ok() and time.time() - t0 < seconds:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        print(f'wrote {out} ({len(node.rows)} frames)')


if __name__ == '__main__':
    main()
