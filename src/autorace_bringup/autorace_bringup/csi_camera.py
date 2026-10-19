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

Both topics go out best effort, depth 1: a lost JPEG over the Wi-Fi is not
worth resending, the next one is 33 ms behind. Subscribers must be best effort
too (a reliable reader does not match a best effort writer); launch_helpers'
republish is. If the device stops giving frames (seen once mid-run), it is
closed and opened again instead of warning forever.
"""
import glob
import os
import time

import cv2
import numpy as np
import rclpy
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
        p = self.get_parameter
        self.quality = p('jpeg_quality').value
        self.frame_id = p('frame_id').value
        self.passthrough = p('passthrough').value
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
            # Hand over the MJPEG buffer as it comes off the USB bus.
            self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)

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
            stamp = self.get_clock().now().to_msg()
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
            if self.count % 300 == 0:
                self.get_logger().info(f'{self.count / (time.time() - self.started):.1f} fps')


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
