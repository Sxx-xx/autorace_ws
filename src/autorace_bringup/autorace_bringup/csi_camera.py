#!/usr/bin/env python3
"""Raspberry Pi CSI camera to ROS: a V4L2 device (legacy stack) or GStreamer's libcamerasrc.

camera_ros (the ROS libcamera) could not run on this Pi: the vendored IPA
module's signature is invalid, libcamera then runs it isolated, and the
isolated worker dies at start(). Ubuntu's own libcamera works through
GStreamer, which OpenCV can read, so this node does that and publishes
<ns>/image_raw/compressed (JPEG) and <ns>/camera_info, like usb_cam with
the compressed transport. The raw image is not published: nothing on the
PC wants it, and it would cross the Wi-Fi for nothing.

Parameters: width 320, height 240, fps 30, jpeg_quality 80, frame_id,
camera_info_url (a camera_calibration yaml; without it a 62 deg lens is
assumed), and 'pipeline' to replace the GStreamer pipeline entirely.
"""
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage
import yaml


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
        p = self.get_parameter
        w, h, fps = p('width').value, p('height').value, p('fps').value
        self.quality = p('jpeg_quality').value
        self.frame_id = p('frame_id').value
        device = p('v4l2_device').value
        if device:
            pipeline = device
            self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
            self.cap.set(cv2.CAP_PROP_FPS, fps)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        else:
            pipeline = p('pipeline').value or (
                f'libcamerasrc ! video/x-raw,width={w},height={h},framerate={fps}/1,format=RGBx '
                f'! videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1')
            self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise RuntimeError(f'cannot open: {pipeline}')
        self.pub_image = self.create_publisher(CompressedImage, 'image_raw/compressed', 1)
        self.pub_info = self.create_publisher(CameraInfo, 'camera_info', 1)
        self.info = self.camera_info(p('camera_info_url').value, w, h)
        self.get_logger().info(f'{w}x{h} at {fps} fps: {pipeline}')
        self.count = 0
        self.started = time.time()

    def camera_info(self, url, w, h):
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
            fx = w / (2.0 * np.tan(np.radians(62.0) / 2.0))   # a 62 deg lens, as the sim's
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
                time.sleep(0.05)
                continue
            stamp = self.get_clock().now().to_msg()
            ok, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
            if not ok:
                continue
            msg = CompressedImage()
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            msg.format = 'jpeg'
            msg.data = jpg.tobytes()
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
