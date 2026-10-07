# 실차 운용 (TurtleBot3 Burger)

이 워크스페이스(`autorace_real_ws`, 브랜치 `real`)는 실차용이다. 시뮬은
`autorace_ws`(브랜치 `master`)에 그대로 있다. 시뮬에서 고친 것을 가져오려면
여기서 `git pull origin master`.

## 구성

```
 로봇 (Raspberry Pi)                      PC
 ───────────────────                      ──────────────────────────────
 turtlebot3_bringup robot.launch.py       race.launch.py
   OpenCR  → /odom, /imu, ← /cmd_vel        republish ×2 (JPEG → raw, PC 에서 디코드)
   LDS     → /scan                          bev_projector, detect_lane, lane_controller
 usb_cam /camera       (전방: 표지판·신호등·차단바)   detect_sign/traffic_light/stop_line/level_crossing
 usb_cam /camera_lane  (아래: 차선·정지선)            mission_manager + 미션 6개, cmd_vel_mux
 = autorace_bringup robot.launch.py
```

- 카메라는 시뮬과 같이 **2대**다. 토픽·장착 위치·각도를 시뮬 모델과 같게 한다.

| 카메라 | 바퀴 축 앞 | 노면 위 | 숙임 | 토픽 |
|---|---|---|---|---|
| 전방 `/camera` | 0.045 m | 0.12 m | 0.10 rad | `/camera/image_raw[/compressed]`, `/camera/camera_info` |
| 차선 `/camera_lane` | 0.030 m | 0.22 m | 1.05 rad (60°) | `/camera_lane/image_raw[/compressed]`, `/camera_lane/camera_info` |

  장착이 다르면 `param/perception_real.yaml` 의 `camera.height / pitch / forward_offset` 을
  실측값으로 바꾼다. 차선 파이프라인은 이 세 값과 `camera_info` 만으로 BEV 를 만든다.
- 영상은 Pi 에서 JPEG(`/compressed`)로 보내고 PC 에서 푼다 (`compressed:=true` 기본).
  Pi 의 raw 토픽은 아무도 구독하지 않으므로 Wi-Fi 를 타지 않는다.
- 해상도 320×240, 30 fps. 모든 픽셀 임계값이 이 해상도 기준이다.

## 처음 한 번

**Pi**
1. Ubuntu 24.04 + ROS 2 Jazzy, `turtlebot3_bringup`, `usb_cam`, `image_transport_plugins`(compressed) 설치.
   OpenCR 펌웨어는 ROBOTIS 매뉴얼대로 (burger).
2. 이 저장소를 Pi 에도 두고 `autorace_bringup` 만 빌드한다
   (`colcon build --packages-select autorace_msgs autorace_bringup`; 런치와 보정 파일만 쓴다).
3. `~/.bashrc`: `export TURTLEBOT3_MODEL=burger LDS_MODEL=LDS-01 ROS_DOMAIN_ID=<PC 와 같은 값>`
4. 카메라 장치 번호 확인: `v4l2-ctl --list-devices`. 전방이 `/dev/video0`, 차선이 `/dev/video2`
   가 아니면 `robot.launch.py forward_device:=... lane_device:=...`. udev 규칙으로 고정하면 편하다.
   `v4l2-ctl -d /dev/video0 --list-formats-ext` 에 320×240 YUYV 가 없으면 `pixel_format:=mjpeg2rgb`.
5. **시계 동기**: PC 와 Pi 둘 다 `chrony` 를 켜고 같은 서버(또는 PC 를 서버로)를 본다.
   `detect_lane`·공사·주차 미션이 영상/스캔 stamp 를 `/odom` stamp 와 맞추므로 두 시계가
   0.05 s 이상 어긋나면 안 된다. `tools/check_robot.sh` 가 오프셋을 찍는다.

**PC**
1. `cd ~/Desktop/autorace_real_ws && colcon build --symlink-install && source install/setup.bash`
2. `~/.bashrc` 에 같은 `ROS_DOMAIN_ID`. 같은 Wi-Fi.

**카메라 보정** (한 번, 카메라마다)
```bash
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 \
    --ros-args -r image:=/camera_lane/image_raw -p camera:=/camera_lane
```
결과 yaml 을 `src/autorace_bringup/param/camera_lane.yaml`(전방은 `camera_forward.yaml`)에
덮어쓴다. 지금 들어 있는 것은 62° 렌즈 근사값(fx 265)이다. 보정 없이도 돌아가지만
BEV 의 축척이 몇 % 틀어져 차선 폭 판정이 흔들린다.

## 매번

```bash
# Pi
ros2 launch autorace_bringup robot.launch.py
# PC
tools/check_robot.sh                       # 토픽 속도, 시계 오프셋, 라이다 노이즈
ros2 launch autorace_bringup race.launch.py
```

- 미션 일부만: `missions:=parking,tunnel`. 빈 값이면 yaml 의 `enabled_missions`.
- 신호등 없이 바로 출발: `auto_start:=true`.
- 차선만: `ros2 launch autorace_bringup lane_drive.launch.py`.
- **비상 정지**: `tools/stop_robot.sh` (제어 노드를 죽이고 0 속도를 2 초 보낸다. OpenCR 은
  마지막 명령을 유지하므로 노드만 죽이면 계속 간다).
- 시뮬 설정으로 돌리려면 `profile:=sim use_sim_time:=true compressed:=false`.

## 실차 맞추기 순서

시뮬에서 맞춘 것을 **한 번에 하나씩** 확인한다. 각 단계의 확인 수단은 디버그 영상이다
(`rqt_image_view`).

1. **바퀴·오도메트리**: `ros2 run turtlebot3_teleop teleop_keyboard` 로 1 m 직진, 제자리
   한 바퀴. `/odom` 이 실제와 맞는지. 1.0 rad/s 회전 뒤 방향 오차가 0.25 rad 를 넘으면
   `parking_mission.turn_rate` 를 더 낮춘다.
2. **라이다 노이즈**: 정지 상태에서 `tools/check_robot.sh` 의 median |delta| 가
   `cmd_vel_mux.scan_change_threshold`(0.05) 의 절반 아래여야 한다. 아니면 임계값을 올린다.
3. **차선 카메라 기하**: 직선 구간에 로봇을 두고 `/camera/image_projected` 를 본다. 두 선이
   평행하고 간격이 0.25 m × 1200 = 300 px 이어야 한다. 아니면 `camera.pitch` 를 0.02 씩,
   `camera.height` 를 5 mm 씩 조정. 선이 수렴하면 pitch, 간격이 틀리면 height.
4. **차선 색**: `/detect/image_lane` 에서 흰선·노란선이 마스크에 잡히고 바닥(검정 포맥스,
   광택)의 반사가 안 잡혀야 한다. `detect.lane.white/yellow` 의 `lightness_l`, `saturation`
   조정. 천장 조명 반사가 흰선으로 잡히면 `white.lightness_l` 을 올린다.
5. **차선 주행**: `lane_drive.launch.py`, `max_speed` 0.12 부터 시작해 0.18 까지.
   코너에서 바깥 바퀴가 포화되면(언더스티어) `cmd_vel_mux.wheel_speed` 를 낮춘다.
6. **전방 검출기** (`race.launch.py auto_start:=true missions:=`로 미션 없이):
   `/detect/image_sign`, `/detect/image_traffic_light`, `/detect/image_level_crossing`.
   `param/mission_real.yaml` 의 HSV 를 현장 조명에서 맞춘다. 카메라 자동 노출을 끄고
   (`robot.launch.py autoexposure:=false exposure:=<값>`) 고정해야 한 번 맞춘 값이 유지된다.
   OpenCV hue 는 0~179 (빨강 0/179 근처, 노랑 ~25, 녹 ~60, 파랑 ~110).
7. **미션 하나씩**: `missions:=intersection` 식으로 해당 구간에 로봇을 두고 확인.
   코스 치수(터널 `inside/exit_x`, 주차 `bay_station/bay_depth`, 갈림길 `branch_distance`)는
   연습장을 줄자로 재서 `mission_real.yaml` 에 넣는다.
8. **전체 주행** 반복. 시간은 `/autorace/mission_state` 로그로.

## 시뮬과 다른 것 (코드 밖)

| 항목 | 시뮬 | 실차 | 대응 |
|---|---|---|---|
| 시간 | `/clock` (use_sim_time) | 벽시계 | 런치 기본 `use_sim_time:=false` |
| 카메라 전송 | raw | JPEG → PC 디코드 | `compressed:=true`, `image_transport republish` |
| 카메라 노출 | 고정 | 자동 → 색이 변함 | 노출·화이트밸런스 고정 |
| 바퀴 한계 | sim_firmware 가 흉내 | 모터 0.211 m/s, 부하 시 그 이하 | `max_speed` 0.18, `wheel_speed` 0.20 |
| 미끄러짐 | 거의 없음 | 포맥스에서 실측 필요 | `turn_rate`, 급회전 회피 |
| 라이다 | 5 Hz, σ 1 cm | LDS-01 5 Hz(실측) | `scan_change_threshold` 재확인 |
| 차단바 | 빨강/흰색 | 출력물 C4 를 **빨강/흰색**으로 | `hardware/venue_props` |
| 신호등 | 램프 모델 이동 | LED + 메가2560 | `hardware/venue_props/firmware` |

## 변경 이력 (시뮬 대비)

- `race.launch.py`, `lane_drive.launch.py`: `profile:=real|sim`, `use_sim_time` 기본 false,
  `compressed` 옵션, mux 에 파라미터 파일, `missions:=''` 가 yaml 값을 덮어쓰던 버그 수정.
- `robot.launch.py`: Pi 쪽 (turtlebot3_bringup + usb_cam ×2).
- `param/perception_real.yaml`, `param/mission_real.yaml`: 실차 값. 코드 기본값으로만 있던
  mux·검출기 HSV 파라미터를 yaml 로 꺼냄.
- `param/camera_forward.yaml`, `param/camera_lane.yaml`: 보정 자리표시자.
- `tools/stop_robot.sh`, `tools/check_robot.sh`.
- `autorace_bringup/package.xml`: `autorace_sim` 의존 제거.
