# Raspberry Pi 4 (4 GB) 설치 — Ubuntu Server 24.04 + ROS 2 Jazzy

TurtleBot3 Burger의 Pi 를 처음부터 세팅하는 순서. 모니터·키보드 없이 PC 에서 SD 카드를
굽고 SSH 로 끝낸다. 전체 1~2 시간 (패키지 다운로드 시간 포함).

준비물: Pi 4 (4 GB), microSD 32 GB 이상(Class 10/A1), SD 리더, USB-C 전원 케이블(OpenCR
5 V 출력 → Pi, 또는 세팅 중엔 5 V 3 A 어댑터), USB 웹캠 2개, PC 와 같은 Wi-Fi.

## 1. SD 카드 굽기 (PC)

1. Raspberry Pi Imager 설치 (`sudo snap install rpi-imager` 또는 raspberrypi.com/software).
2. 실행 → **기기 선택**: Raspberry Pi 4.
3. **운영체제 선택**: Other general-purpose OS → Ubuntu → **Ubuntu Server 24.04.x LTS (64-bit)**.
   Desktop 은 느리고 쓸 일이 없다. 22.04 는 Jazzy 가 안 올라간다.
4. **저장소 선택**: SD 카드.
5. 다음 → **설정 편집** (OS customisation). 여기서 넣어 두면 첫 부팅부터 Wi-Fi·SSH 가 된다.
   - 일반 탭: 호스트 이름 `tb3`, 사용자 `ubuntu` / 비밀번호, Wi-Fi SSID·비밀번호,
     무선 LAN 국가 `KR`, 시간대 `Asia/Seoul`, 키보드 `us`.
   - 서비스 탭: **SSH 사용**, 비밀번호 인증.
   - 저장 → 예(설정 적용) → 예(쓰기). 5~10 분.
6. 카드를 Pi 에 꽂고 전원. 첫 부팅은 cloud-init 이 돌아 2~3 분 걸린다. 녹색 LED 가 잠잠해질 때까지.

## 2. 접속

```bash
ssh dd@172.20.10.3   # 실제 설치: 사용자 dd, 호스트 turtlebot1
```
- `tb3.local` 이 안 풀리면 공유기 관리 페이지의 접속 기기 목록에서 IP 를 찾거나
  PC 에서 `sudo nmap -sn 192.168.0.0/24` (nmap 설치 필요).
- Wi-Fi 가 아예 안 붙었으면 카드를 PC 에 다시 꽂아 `system-boot` 파티션의
  `network-config` 에 SSID 를 적는다:
  ```yaml
  wifis:
    wlan0:
      dhcp4: true
      optional: true
      access-points:
        "SSID":
          password: "비밀번호"
  ```
- 공유기에서 Pi 의 MAC 에 **고정 IP 를 예약**해 두면 chrony 와 접속이 편하다.

## 3. 기본 설정

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y chrony v4l-utils net-tools git curl
# 경기 중에 자동 업그레이드가 돌면 CPU 와 네트워크를 먹는다
sudo systemctl disable --now unattended-upgrades
# 부팅 때 네트워크를 2 분까지 기다리는 서비스
sudo systemctl disable systemd-networkd-wait-online.service
# 방화벽은 끈다 (DDS 멀티캐스트)
sudo ufw disable 2>/dev/null; true
# 로케일
sudo locale-gen en_US en_US.UTF-8 && sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
sudo reboot
```

## 4. ROS 2 Jazzy

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
sudo apt install -y ros-jazzy-ros-base ros-dev-tools python3-colcon-common-extensions python3-rosdep
sudo rosdep init; rosdep update
```
`ros-base` 는 GUI 없는 묶음(약 1 GB). 확인:
```bash
source /opt/ros/jazzy/setup.bash && ros2 --help | head -3
```

## 5. TurtleBot3·카메라 패키지

```bash
sudo apt install -y ros-jazzy-turtlebot3-bringup ros-jazzy-turtlebot3-node \
    ros-jazzy-hls-lfcd-lds-driver ros-jazzy-ld08-driver ros-jazzy-dynamixel-sdk \
    ros-jazzy-usb-cam ros-jazzy-image-transport-plugins ros-jazzy-camera-calibration \
    ros-jazzy-turtlebot3-teleop
```
apt 에 turtlebot3 가 없다고 하면 소스로 빌드한다 (10~15 분):
```bash
mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/ld08_driver.git
rm -rf turtlebot3/turtlebot3_cartographer turtlebot3/turtlebot3_navigation2   # Pi 에서 불필요
cd ~/turtlebot3_ws && rosdep install -i --from-path src --rosdistro jazzy -y
colcon build --symlink-install --parallel-workers 2
echo "source ~/turtlebot3_ws/install/setup.bash" >> ~/.bashrc
```

장치 권한과 udev 규칙 (OpenCR → `/dev/ttyACM0`, 라이다 → `/dev/ttyUSB0`):
```bash
sudo cp $(ros2 pkg prefix turtlebot3_bringup)/share/turtlebot3_bringup/script/99-turtlebot3-cdc.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG dialout,video ubuntu
```

`~/.bashrc` 끝에 (ROS_DOMAIN_ID 는 PC 와 같은 값, LDS 는 라이다 라벨대로):
```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-01          # 라벨이 LDS-02 면 LDS-02
export ROS_DOMAIN_ID=30
```
로그아웃했다 다시 들어온다 (그룹 반영). `ls -l /dev/ttyACM0 /dev/ttyUSB0` 가 보여야 한다.

## 6. OpenCR 펌웨어 (ROS 2 용, Pi 에서 USB 로)

OpenCR 이 ROS 1 펌웨어거나 처음이면 한 번 올린다. Pi 4 는 arm64 라 32 비트 라이브러리가 필요하다.
```bash
sudo dpkg --add-architecture armhf && sudo apt update && sudo apt install -y libc6:armhf
export OPENCR_PORT=/dev/ttyACM0 OPENCR_MODEL=burger
cd ~ && rm -rf opencr_update*
wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS2/latest/opencr_update.tar.bz2
tar -xvf opencr_update.tar.bz2 && cd opencr_update
./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr
```
끝에 `Update Complete` 가 나오면 된다. 안 되면 OpenCR 의 전원 스위치, USB 케이블(충전 전용 아닌지),
`dmesg | tail` 로 ttyACM0 인식 여부.

바퀴 확인: OpenCR 의 PUSH SW 1 을 누르면 앞으로 30 cm, SW 2 는 제자리 180°.

## 7. 시계 동기 (chrony)

PC 가 서버, Pi 가 클라이언트. 영상·스캔 stamp 를 odom 과 맞추는 코드가 있어 두 시계가
0.05 s 안으로 맞아야 한다.

PC (`/etc/chrony/chrony.conf` 끝에):
```
allow 192.168.0.0/16
local stratum 10
```
```bash
sudo systemctl restart chrony
```
Pi (`/etc/chrony/chrony.conf`): 기존 `pool`/`server` 줄은 주석 처리하고
```
server <PC IP> iburst prefer
makestep 1 -1
```
```bash
sudo systemctl restart chrony && sleep 5 && chronyc tracking
```
`System time : 0.000xxx seconds fast/slow` 가 ms 단위면 된다. `chronyc sources` 에서 PC 가 `^*` 여야 한다.

## 8. 카메라

```bash
v4l2-ctl --list-devices                       # 웹캠마다 /dev/videoN 두 개(캡처+메타) 가 나온다
v4l2-ctl -d /dev/video0 --list-formats-ext | head -30   # 320x240 에 YUYV 가 있는지
```
- YUYV 320x240 30 fps 가 있으면 기본값 그대로. MJPEG 만 있으면 런치에 `pixel_format:=mjpeg2rgb`.
- 꽂는 순서에 따라 번호가 바뀌니 **udev 로 이름을 고정**한다. 각 카메라의 ID 확인:
  ```bash
  udevadm info -a -n /dev/video0 | grep -E 'idVendor|idProduct|serial|KERNELS' | head -8
  ```
  `/etc/udev/rules.d/99-autorace-cams.rules` (같은 모델 두 개면 serial 대신 꽂은 포트 `KERNELS=="1-1.2"` 로 구분):
  ```
  SUBSYSTEM=="video4linux", ATTR{index}=="0", ATTRS{serial}=="AAAA", SYMLINK+="cam_forward"
  SUBSYSTEM=="video4linux", ATTR{index}=="0", ATTRS{serial}=="BBBB", SYMLINK+="cam_lane"
  ```
  ```bash
  sudo udevadm control --reload-rules && sudo udevadm trigger && ls -l /dev/cam_*
  ```
  그 뒤 런치는 `forward_device:=/dev/cam_forward lane_device:=/dev/cam_lane`.
- 장착 위치는 시뮬과 같게: 전방 카메라 바퀴 축 앞 0.045 m·높이 0.12 m·0.10 rad 숙임,
  차선 카메라 바퀴 축 앞 0.030 m·높이 0.22 m·1.05 rad(60°) 숙임. 자로 재서 적어 둔다.

## 9. 우리 워크스페이스

Pi 에서는 런치·보정 파일만 쓰므로 두 패키지만 빌드한다. PC 에서 복사:
```bash
# PC 에서
rsync -a --exclude build --exclude install --exclude log ~/Desktop/autorace_real_ws/src ubuntu@tb3.local:~/autorace_real_ws/
```
```bash
# Pi 에서
cd ~/autorace_real_ws
colcon build --symlink-install --packages-select autorace_msgs autorace_bringup
echo "source ~/autorace_real_ws/install/setup.bash" >> ~/.bashrc && source ~/.bashrc
```

## 10. 확인

Pi:
```bash
ros2 launch autorace_bringup robot.launch.py forward_device:=/dev/cam_forward lane_device:=/dev/cam_lane
```
PC (같은 ROS_DOMAIN_ID):
```bash
tools/check_robot.sh
```
`/scan` 5 Hz, `/odom` 30 Hz, 카메라 2개 `/compressed` 30 Hz 근처, clock offset 0.05 s 이내면 된다.
텔레옵으로 바퀴 확인:
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

## 11. 부팅하면 자동으로 뜨게 (선택, 경기 당일용)

`/etc/systemd/system/autorace-robot.service`:
```ini
[Unit]
Description=AutoRace robot drivers
After=network-online.target
[Service]
User=ubuntu
Environment=TURTLEBOT3_MODEL=burger LDS_MODEL=LDS-01 ROS_DOMAIN_ID=30
ExecStart=/bin/bash -lc 'source /opt/ros/jazzy/setup.bash && source /home/ubuntu/autorace_real_ws/install/setup.bash && ros2 launch autorace_bringup robot.launch.py forward_device:=/dev/cam_forward lane_device:=/dev/cam_lane'
Restart=on-failure
RestartSec=3
[Install]
WantedBy=multi-user.target
```
```bash
sudo systemctl daemon-reload && sudo systemctl enable --now autorace-robot
journalctl -u autorace-robot -f     # 로그
```

## 막히기 쉬운 곳

| 증상 | 확인 |
|---|---|
| PC 에서 `ros2 topic list` 에 아무것도 없음 | 두 쪽 `ROS_DOMAIN_ID` 같은지, 같은 공유기인지(게스트 Wi-Fi 는 기기 간 통신 차단), `ufw` 꺼졌는지 |
| `/dev/ttyACM0` 없음 | OpenCR 전원 스위치, 데이터용 USB 케이블, `dmesg \| tail` |
| `/dev/ttyUSB0` 없음 | 라이다 USB 어댑터, `LDS_MODEL` |
| 카메라가 `Device not available` | 장치 번호(`v4l2-ctl --list-devices`), `video` 그룹, 다른 프로세스가 잡고 있는지 `fuser /dev/video0` |
| 영상이 15 fps 이하 | USB 허브 대역폭(카메라를 다른 포트로), MJPEG 디코드 부하(`top`), 5 GHz Wi-Fi |
| chrony 오프셋이 큼 | PC 의 `allow`, Pi 의 `server` IP, `chronyc sources` |
| Pi 가 느려짐·재부팅 | 전원 부족(5 V 3 A, 굵은 케이블), 발열(`vcgencmd measure_temp`, 80 °C 넘으면 방열판/팬) |
