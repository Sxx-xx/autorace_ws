#!/usr/bin/env python3
"""Robot camera to ROS: a V4L2 device (the CSI camera on the legacy stack, or a UVC
webcam such as the C920) or GStreamer's libcamerasrc.

camera_ros (the ROS libcamera) could not run on this Pi: the vendored IPA
module's signature is invalid, libcamera then runs it isolated, and the
isolated worker dies at start(). Ubuntu's own libcamera works through
GStreamer, which OpenCV can read, so this node does that and publishes
<ns>/image_raw/compressed (JPEG) and <ns>/camera_info, like usb_cam with
the compressed transport. The raw image is not published: nothing on the
PC wants it, and it would cross the Wi-Fi for nothing.

Parameters: width 320, height 240, fps 30, jpeg_quality 80, frame_id,
camera_info_url (a camera_calibration yaml; without it a lens of fov_deg,
62 by default, is assumed), 'v4l2_device' or 'v4l2_name' to pick the V4L2
device (by node or by the name in /sys/class/video4linux, since the numbers
move when a webcam is plugged in), 'passthrough' to send the camera's own
MJPEG frames untouched (a UVC webcam compresses in hardware, so the Pi then
does no image work at all), and 'pipeline' to replace the GStreamer pipeline.

Fast capture (2026-10-22). On the IMX219 (legacy stack) the GPU already
encodes MJPEG, so 'passthrough' works for the CSI camera too: the Pi then
neither decodes nor re-encodes (90 fps at 2 % of a core, against 23 % for
decoding alone). Its MJPEG is rate controlled, not quality controlled
(compression_quality has no effect on it; the JPEG format would honour it but
gives 6.7 fps through OpenCV): 'bitrate' (bits/s, 0 leaves the driver's 10 M,
13 KB a frame at 90 fps) sets video_bitrate; 2 M is visibly blocky. Ask for a 16:9 size for 90 fps (384x216): the
firmware picks the sensor mode from size and rate, and 4:3 at 90 fps is a
cropped, 2x zoomed mode. 'exposure_us' > 0 fixes the exposure (at most
1e6 / fps), 'controls' sets any other V4L2 control ("name=value,..."; run
`v4l2-ctl -d <dev> -L` for the list; a USB camera has its own names).
'stamp': 'capture' stamps each frame with the time the driver took it
(V4L2 buffer time, monotonic, moved onto the ROS clock), 'publish' (default,
as before) with the time it was read. 'stats' logs once a second: frames,
size, Mbps, and capture-to-publish delay.

Both topics go out best effort, depth 1: a lost JPEG over the Wi-Fi is not
worth resending, the next one is 33 ms behind. Subscribers must be best effort
too (a reliable reader does not match a best effort writer); launch_helpers'
republish is. If the device stops giving frames (seen once mid-run), it is
closed and opened again instead of warning forever.
"""
import glob
import os
import subprocess
import time

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage
import yaml

# Frames that may fail in a row before the device is opened again (2 s at 30 fps).
REOPEN_AFTER = 60
SENSOR_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


class CsiCamera(Node):

    def __init__(self):
        super().__init__('csi_camera')
        self.declare_parameter('width', 320)
        self.declare_parameter('height', 240)
        self.declare_parameter('fps', 30)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('frame_id', 'camera_lane_link')
        self.declare_parameter('camera_info_url', '')
        self.declare_parameter('pipeline', '')
        # With the legacy camera stack (start_x=1, bcm2835-v4l2) the camera is
        # a plain V4L2 device and needs no libcamera at all: set this instead.
        self.declare_parameter('v4l2_device', '')
        self.declare_parameter('v4l2_name', '')
        self.declare_parameter('passthrough', False)
        self.declare_parameter('fov_deg', 62.0)
        self.declare_parameter('exposure_us', 0)       # 0: automatic
        self.declare_parameter('bitrate', 0)           # passthrough MJPEG, bits/s; 0: driver's
        self.declare_parameter('controls', '')         # 'name=value,...' for v4l2-ctl
        self.declare_parameter('stamp', 'publish')     # or 'capture'
        self.declare_parameter('stats', False)
        p = self.get_parameter
        self.quality = p('jpeg_quality').value
        self.frame_id = p('frame_id').value
        self.passthrough = p('passthrough').value
        self.exposure_us = p('exposure_us').value
        self.bitrate = p('bitrate').value
        self.controls = p('controls').value
        self.capture_stamp = p('stamp').value == 'capture'
        self.stats = p('stats').value
        self.size = (p('width').value, p('height').value, p('fps').value)
        device = p('v4l2_device').value or self.device_named(p('v4l2_name').value)
        if device:
            self.source = device
        else:
            w, h, fps = self.size
            self.source = p('pipeline').value or (
                f'libcamerasrc ! video/x-raw,width={w},height={h},framerate={fps}/1,format=RGBx '
                f'! videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1')
        self.is_v4l2 = bool(device)
        self.open()
        if not self.cap.isOpened():
            raise RuntimeError(f'cannot open: {self.source} (busy? fuser -v {self.source})'
                               if self.is_v4l2 else f'cannot open: {self.source}')
        self.pub_image = self.create_publisher(CompressedImage, 'image_raw/compressed', SENSOR_QOS)
        self.pub_info = self.create_publisher(CameraInfo, 'camera_info', SENSOR_QOS)
        w, h, fps = self.size
        self.info = self.camera_info(p('camera_info_url').value, w, h, p('fov_deg').value)
        self.get_logger().info(
            f'{w}x{h} at {fps} fps: {self.source}' + (' (MJPEG passthrough)' if self.passthrough else ''))
        self.count = 0
        self.failures = 0
        self.started = time.time()
        self.window = []            # (bytes, capture to publish s) over the current second
        self.window_start = time.time()

    def open(self):
        w, h, fps = self.size
        if not self.is_v4l2:
            self.cap = cv2.VideoCapture(self.source, cv2.CAP_GSTREAMER)
            return
        self.cap = cv2.VideoCapture(self.source, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if self.passthrough:
            # Hand over the MJPEG buffer as it comes off the USB bus (or the GPU's encoder).
            self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        self.set_controls()

    def set_controls(self):
        """Exposure, the encoder's quality and any other controls, through v4l2-ctl."""
        controls = [c for c in self.controls.split(',') if c.strip()]
        if self.passthrough and self.bitrate > 0 and self.has_control('video_bitrate'):
            controls.insert(0, f'video_bitrate={self.bitrate}')
        if self.exposure_us > 0:
            # bcm2835-v4l2 and UVC: auto_exposure 1 is manual, exposure_time_absolute in 100 us.
            controls += ['auto_exposure=1',
                         f'exposure_time_absolute={max(1, round(self.exposure_us / 100))}']
        if not controls:
            return
        result = subprocess.run(['v4l2-ctl', '-d', self.source, '-c', ','.join(controls)],
                                capture_output=True, text=True)
        if result.returncode != 0:
            self.get_logger().warn(f'v4l2-ctl -c {",".join(controls)}: {result.stderr.strip()}')
        else:
            self.get_logger().info(f'controls: {",".join(controls)}')

    def has_control(self, name):
        listed = subprocess.run(['v4l2-ctl', '-d', self.source, '-l'],
                                capture_output=True, text=True)
        return f' {name} ' in listed.stdout

    @staticmethod
    def device_named(name):
        """/dev/videoN whose V4L2 name is `name` ('camera0' is the CSI camera)."""
        if not name:
            return ''
        names = []
        for d in sorted(glob.glob('/sys/class/video4linux/video*')):
            try:
                found = open(f'{d}/name').read().strip()
            except OSError:
                continue
            if found == name:
                return '/dev/' + os.path.basename(d)
            names.append(f'{os.path.basename(d)}={found!r}')
        # The CSI camera is missing (start_x, the ribbon, bcm2835-v4l2): say what is there.
        raise RuntimeError(f'no V4L2 device named {name!r}; found: {", ".join(names) or "none"}')

    def camera_info(self, url, w, h, fov_deg=62.0):
        info = CameraInfo()
        info.header.frame_id = self.frame_id
        info.width, info.height = w, h
        if url:
            d = yaml.safe_load(open(url.replace('file://', '')))
            info.distortion_model = d.get('distortion_model', 'plumb_bob')
            info.d = list(map(float, d['distortion_coefficients']['data']))
            info.k = list(map(float, d['camera_matrix']['data']))
            info.r = list(map(float, d['rectification_matrix']['data']))
            info.p = list(map(float, d['projection_matrix']['data']))
        else:
            fx = w / (2.0 * np.tan(np.radians(fov_deg) / 2.0))   # horizontal field of view
            info.distortion_model = 'plumb_bob'
            info.d = [0.0] * 5
            info.k = [fx, 0.0, w / 2.0, 0.0, fx, h / 2.0, 0.0, 0.0, 1.0]
            info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info.p = [fx, 0.0, w / 2.0, 0.0, 0.0, fx, h / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def spin(self):
        while rclpy.ok():
            ok, frame = self.cap.read()
            if not ok:
                self.get_logger().warn('no frame', throttle_duration_sec=5.0)
                self.failures += 1
                if self.failures >= REOPEN_AFTER:
                    self.get_logger().warn(f'no frame for {self.failures} reads: reopening {self.source}')
                    self.cap.release()
                    time.sleep(1.0)
                    self.open()
                    self.failures = 0
                time.sleep(0.05)
                continue
            self.failures = 0
            now = self.get_clock().now()
            stamp = now.to_msg()
            age = 0.0
            if self.capture_stamp and self.is_v4l2:
                # The driver's buffer time is on the monotonic clock: how long ago, then.
                age = time.monotonic() - self.cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
                if 0.0 <= age < 1.0:
                    stamp = (now - Duration(seconds=age)).to_msg()
                else:
                    age = 0.0
            if self.passthrough and frame.ndim <= 2:
                data = frame.tobytes()
                if data[:2] != b'\xff\xd8':
                    self.get_logger().warn('not a JPEG frame', throttle_duration_sec=5.0)
                    continue
            else:
                ok, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
                if not ok:
                    continue
                data = jpg.tobytes()
            msg = CompressedImage()
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            msg.format = 'jpeg'
            msg.data = data
            self.pub_image.publish(msg)
            self.info.header.stamp = stamp
            self.pub_info.publish(self.info)
            self.count += 1
            if self.stats:
                held = (self.get_clock().now() - now).nanoseconds / 1e9
                self.window.append((len(data), age + held))
                if time.time() - self.window_start >= 1.0:
                    self.log_stats()
            elif self.count % 300 == 0:
                self.get_logger().info(f'{self.count / (time.time() - self.started):.1f} fps')

    def log_stats(self):
        """One second: frames, mean size, Mbps, capture (or read) to publish delay."""
        span = time.time() - self.window_start
        sizes = np.array([s for s, _ in self.window])
        delays = np.array([d for _, d in self.window]) * 1000.0
        if len(sizes):
            self.get_logger().info(
                f'stats: {len(sizes) / span:.1f} fps, {sizes.mean() / 1024:.1f} KB, '
                f'{sizes.sum() * 8 / span / 1e6:.1f} Mbps, to publish ms p50 '
                f'{np.median(delays):.1f} max {delays.max():.1f}')
        self.window = []
        self.window_start = time.time()


def main(args=None):
    rclpy.init(args=args)
    node = CsiCamera()
    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
