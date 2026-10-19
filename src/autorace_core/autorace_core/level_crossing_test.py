#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry
import cv2
import numpy as np
import math

class LevelCrossingTiltNode(Node):
    def __init__(self):
        super().__init__('level_crossing_tilt_node')

        # 1. 퍼블리셔 및 서브스크라이버 설정
        # 로봇(turtlebot3_node, tb3_burger.yaml)은 /cmd_vel 을 Twist 로 받는다 (TwistStamped 는 연결이 안 됨).
        # Reliable + depth 1: 잃은 명령은 다시 보내되 최신 것 하나만. race.launch.py 의 cmd_vel_mux 와
        # 같은 토픽이므로 레이스와 동시에 띄우지 않는다.
        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        # Pi 카메라는 JPEG(/camera/image_raw/compressed)만, Best Effort 로 낸다: raw 토픽은 없다.
        self.image_sub = self.create_subscription(
            CompressedImage, '/camera/image_raw/compressed', self.image_callback,
            qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # 2. 상태 변수
        self.current_yaw_speed = 0.0
        self.current_linear_speed = 0.0
        self.is_bar_down = False
        
        # 3. 판단 기준 (각도: 0도는 수평, 90도는 수직)
        self.tilt_threshold_down = 35.0  # 기울기가 35도 이하로 줄어들면 차단바가 내려온 것으로 판단
        self.tilt_threshold_up = 55.0    # 기울기가 55도 이상으로 커지면 차단바가 올라간 것으로 판단
        
        self.get_logger().info("카메라 기울기 감지 기반 차단바 테스트 노드 시작!")

    def odom_callback(self, msg):
        # 카메라가 움직이는 동적 상황을 인지하기 위해 현재 로봇의 속도를 저장
        self.current_linear_speed = msg.twist.twist.linear.x
        self.current_yaw_speed = abs(msg.twist.twist.angular.z)

    def image_callback(self, msg):
        # JPEG 를 OpenCV 이미지로 변환
        cv_image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if cv_image is None:
            self.get_logger().error("Image decode error", throttle_duration_sec=5.0)
            return

        # 1. 빨간색 패턴 필터링 (HSV 변환)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # 빨간색은 HSV 스케일에서 양끝(0~10, 170~180)에 분포함
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        # 2. 윤곽선 및 직선 검출 (Canny + HoughLines)
        edges = cv2.Canny(red_mask, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50, minLineLength=50, maxLineGap=10)

        tilt_angle = 90.0  # 기본값 수직(90도)으로 설정 (차단바 없음)
        
        if lines is not None:
            angles = []
            for line in lines:
                x1, y1, x2, y2 = line[0]
                # 두 점 사이의 기울기(각도) 계산 (-90 ~ 90)
                angle = math.degrees(math.atan2(y2 - y1, x2 - x1))
                # 절대값을 취해 0도(수평) ~ 90도(수직) 형태로 변환
                angles.append(abs(angle))
            
            # 검출된 직선들의 평균 기울기 산출
            tilt_angle = np.median(angles)
        
        # 3. 로봇의 흔들림(Odometry)을 반영한 임계값 보정
        # 로봇이 회전 중이거나 빠르게 이동 중일 때는 화면이 흔들리므로 판단 기준을 보수적으로 잡음
        dynamic_down_thresh = self.tilt_threshold_down
        if self.current_yaw_speed > 0.1:
            dynamic_down_thresh -= 10.0  # 회전 중일 땐 기울기가 더 확실하게 누워야 멈춤

        # 4. 차단바 상태 판단 로직
        if tilt_angle < dynamic_down_thresh:
            self.is_bar_down = True
            self.get_logger().info(f"차단바 닫힘 감지! (기울기: {tilt_angle:.1f}도)")
        elif tilt_angle > self.tilt_threshold_up:
            self.is_bar_down = False

        self.drive_robot()

    def drive_robot(self):
        twist = Twist()

        if self.is_bar_down:
            # 멈춤
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            # 직진 (안전한 저속 0.05 m/s)
            twist.linear.x = 0.05
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = LevelCrossingTiltNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
