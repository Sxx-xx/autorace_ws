#!/bin/bash
# [로봇] 모터(OpenCR, /cmd_vel 은 Twist) + LDS-02 라이다(/scan) 를 띄운다.
# 결과: /odom /scan /imu 발행, /cmd_vel 수신. 이게 없으면 바퀴가 안 돈다.
# Ctrl+C 로 둘 다 종료 (OpenCR 정지).
source "$(dirname "$0")/../_env.sh"
for f in "$HOME/tb3_burger_twist.yaml" "$HOME/ld_lidar.py"; do
  [ -f "$f" ] || { echo "없음: $f"; exit 1; }
done
# 같은 포트를 두 드라이버가 읽지 않도록, 이미 떠 있는 bringup/라이다를 먼저 끈다.
pkill -INT -f "turtlebot3_bringup robot.launch.py" 2>/dev/null
pkill -f "hlds_laser_publisher|ld_lidar.py|turtlebot3_ros" 2>/dev/null
sleep 2
trap 'kill 0' INT TERM HUP EXIT
ros2 run turtlebot3_node turtlebot3_ros -i /dev/ttyACM0 --ros-args --params-file "$HOME/tb3_burger_twist.yaml" -p 'namespace:=""' &
python3 "$HOME/ld_lidar.py" --ros-args -p port:=/dev/ttyUSB0 &
wait
