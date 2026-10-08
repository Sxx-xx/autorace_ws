#!/usr/bin/env python3
"""Lane following on the camera picture itself: no bird's eye view, no camera geometry.

The lane camera's JPEG stream is decoded here. On a few rows across the lower part of the
picture the yellow line is looked for on the left and the white line on the right (yellow is
always the left edge of the lane). The lane centre on a row is half-way between them; with one
line only it is that line plus the lane's width in pixels on that row, learnt while both lines
were in view (the width shrinks towards the top of the picture by perspective).

Steering: the centre's horizontal offset on the look-ahead row, as a fraction of half the
picture, plus the slant of the centre between the bottom and the top rows (where the lane is
heading). Speed comes down with the offset. With no line in view the last turn goes on slowly
for a moment, then the robot creeps straight.

/camera_lane/image_raw/compressed in, /cmd_vel/lane out (through cmd_vel_mux), standing still
until /autorace/run_active. Debug picture on /detect/image_lane, state on /detect/lane_state
(0 none, 1 yellow only, 2 both, 3 white only), so the usual viewers and tools/record_run.py work.
"""
import math

import cv2
from geometry_msgs.msg import Twist
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, UInt8


class ImageLaneFollower(Node):

    def __init__(self):
        super().__init__('image_lane_follower')
        p = self.declare_parameter
        p('rows', [0.92, 0.80, 0.68, 0.56])   # scan rows, fraction of the height (bottom first)
        p('lookahead_row', 2)                 # index in rows used for the offset
        p('lane_width_px', [330.0, 270.0, 215.0, 165.0])   # first guess per row, learnt after
        p('max_speed', 0.15)
        p('min_speed', 0.06)
        p('offset_gain', 1.2)                 # rad/s for the centre at the picture's edge
        p('heading_gain', 1.0)                # rad/s per rad of slant
        p('max_angular', 1.2)
        p('lost_hold', 0.6)                   # s of the last turn after losing the lines
        p('creep_speed', 0.05)
        p('min_piece_px', 40)                 # specks smaller than this are not lines
        p('yellow_hue', [12, 35])
        p('yellow_sat_min', 70)
        p('yellow_light_min', 80)
        p('white_light_min', 155)
        p('white_sat_max', 120)
        g = lambda n: self.get_parameter(n).value
        self.rows_frac = list(g('rows'))
        self.look = g('lookahead_row')
        self.width_px = np.array(g('lane_width_px'), dtype=float)
        self.max_speed, self.min_speed = g('max_speed'), g('min_speed')
        self.k_off, self.k_head = g('offset_gain'), g('heading_gain')
        self.max_w = g('max_angular')
        self.lost_hold, self.creep = g('lost_hold'), g('creep_speed')
        self.min_piece = g('min_piece_px')
        self.y_hue = g('yellow_hue')
        self.y_sat, self.y_light = g('yellow_sat_min'), g('yellow_light_min')
        self.w_light, self.w_sat = g('white_light_min'), g('white_sat_max')

        self.run_active = False
        self.last_w = 0.0
        self.last_seen = -1e9
        self.create_subscription(Bool, '/autorace/run_active',
                                 lambda m: setattr(self, 'run_active', m.data), 1)
        self.create_subscription(CompressedImage, '/camera_lane/image_raw/compressed', self.on_image, 1)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel/lane', 1)
        self.pub_dbg = self.create_publisher(Image, '/detect/image_lane', 1)
        self.pub_state = self.create_publisher(UInt8, '/detect/lane_state', 1)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def clean(self, mask):
        n, lab, st, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        keep = st[:, cv2.CC_STAT_AREA] >= self.min_piece
        keep[0] = False
        return keep[lab]

    def on_image(self, msg):
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        h, w = img.shape[:2]
        hls = cv2.cvtColor(cv2.GaussianBlur(img, (5, 5), 0), cv2.COLOR_BGR2HLS)
        H, L, S = hls[..., 0], hls[..., 1], hls[..., 2]
        yellow = ((H >= self.y_hue[0]) & (H <= self.y_hue[1]) & (S >= self.y_sat) & (L >= self.y_light))
        white = (L >= self.w_light) & (S <= self.w_sat) & ~yellow
        yellow = self.clean(yellow.astype(np.uint8))
        white = self.clean(white.astype(np.uint8))

        centres, seen_y, seen_w = [], 0, 0
        dbg = img.copy()
        for i, f in enumerate(self.rows_frac):
            r = min(h - 1, int(f * h))
            ys = np.flatnonzero(yellow[r])
            ws = np.flatnonzero(white[r])
            # Yellow is the left edge: its right-most run. White is the right edge: the first
            # white run to the right of that yellow (or of the left part of the picture).
            yx = None
            if len(ys):
                runs = np.split(ys, np.flatnonzero(np.diff(ys) > 3) + 1)
                yx = float(runs[-1].mean())
            wx = None
            if len(ws):
                runs = np.split(ws, np.flatnonzero(np.diff(ws) > 3) + 1)
                right = [rr for rr in runs if yx is None or rr.mean() > yx + 20]
                if right:
                    wx = float(right[0].mean())
            if yx is not None and wx is not None and 40 < wx - yx < 1.6 * self.width_px[i]:
                self.width_px[i] = 0.9 * self.width_px[i] + 0.1 * (wx - yx)
                c = (yx + wx) / 2.0
                seen_y += 1
                seen_w += 1
            elif yx is not None:
                c = yx + self.width_px[i] / 2.0
                seen_y += 1
            elif wx is not None:
                c = wx - self.width_px[i] / 2.0
                seen_w += 1
            else:
                c = None
            centres.append((r, c))
            cv2.line(dbg, (0, r), (w - 1, r), (90, 90, 90), 1)
            if yx is not None:
                cv2.circle(dbg, (int(yx), r), 5, (0, 220, 255), -1)
            if wx is not None:
                cv2.circle(dbg, (int(wx), r), 5, (255, 255, 255), -1)
            if c is not None:
                cv2.circle(dbg, (int(c), r), 6, (0, 0, 255), 2)

        state = 2 if seen_y and seen_w else (1 if seen_y else (3 if seen_w else 0))
        self.pub_state.publish(UInt8(data=state))
        now = self.now()
        twist = Twist()
        known = [(r, c) for r, c in centres if c is not None]
        if known:
            self.last_seen = now
            look = centres[self.look] if centres[self.look][1] is not None else known[min(len(known) - 1, self.look)]
            offset = (look[1] - w / 2.0) / (w / 2.0)            # +: lane centre right of the robot
            heading = 0.0
            if len(known) >= 2:
                (r0, c0), (r1, c1) = known[0], known[-1]
                heading = math.atan2(c1 - c0, r0 - r1)          # +: lane heading to the right
            ang = -(self.k_off * offset + self.k_head * heading)
            ang = max(-self.max_w, min(self.max_w, ang))
            speed = self.max_speed * max(0.0, 1.0 - 1.5 * abs(offset) - 0.8 * abs(heading))
            twist.linear.x = max(self.min_speed, speed)
            twist.angular.z = ang
            self.last_w = ang
            cv2.line(dbg, (w // 2, h - 1), (int(look[1]), look[0]), (0, 0, 255), 2)
        elif now - self.last_seen < self.lost_hold:
            twist.linear.x = self.creep
            twist.angular.z = self.last_w
        else:
            twist.linear.x = self.creep
        if not self.run_active:
            twist = Twist()
        self.pub_cmd.publish(twist)

        cv2.putText(dbg, f'state {state}  v {twist.linear.x:.2f} w {twist.angular.z:+.2f}',
                    (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        out = Image()
        out.header = msg.header
        out.height, out.width, out.encoding, out.step = h, w, 'bgr8', 3 * w
        out.data = dbg.tobytes()
        self.pub_dbg.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImageLaneFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_cmd.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
