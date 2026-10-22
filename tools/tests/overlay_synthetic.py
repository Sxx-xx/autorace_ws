"""The camera overlay of lane_pipeline on a synthetic bent lane: python3 ... <yaml> out.jpg"""
import sys, math
import cv2, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, CompressedImage
from autorace_perception.detect_lane import DetectLane
from autorace_perception.lane_pipeline import LanePipeline
rclpy.init(args=['--ros-args', '--params-file', sys.argv[1], '-p', 'pursuit_gain:=1.17'])
det = DetectLane(); pipe = LanePipeline(det)
W, H = 432, 240; fx = W / (2 * math.tan(math.radians(70.4) / 2))
pipe.callback_info(CameraInfo(width=W, height=H, k=[fx, 0., W/2, 0., fx, H/2, 0., 0., 1.]))
pipe.build_homography()
half = 0.165
bev = np.full((pipe.out_h, pipe.out_w, 3), 70, np.uint8)
for base, col in ((half, (0, 220, 255)), (-half, (235, 235, 235))):
    for y in range(pipe.out_h):
        a = pipe.near + (pipe.out_h - 1 - y) / pipe.ppm
        x = pipe.out_w / 2 - (base + 2.5 * (a - pipe.near) ** 2) * pipe.ppm      # bending left
        cv2.line(bev, (int(x - 18), y), (int(x + 18), y), col, 1)
cam = cv2.warpPerspective(bev, pipe.homography, (W, H), flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP, borderValue=(70, 70, 70))
msg = CompressedImage(format='jpeg', data=cv2.imencode('.jpg', cam)[1].tobytes())
msg.header.stamp = pipe.get_clock().now().to_msg()
for _ in range(3): pipe.callback_compressed(msg)
out = pipe.draw_overlay(cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR))
print('target', det.overlay['target'], 'state', det.overlay['state'], 'points yellow', len(det.overlay['yellow']), 'white', len(det.overlay['white']), 'path', len(det.overlay['path']))
cv2.imwrite(sys.argv[2], cv2.resize(out, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST))
