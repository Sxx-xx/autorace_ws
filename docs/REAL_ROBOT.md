# 실기체 실행 가이드 (TurtleBot3 Burger, Jazzy)

이 문서의 노드는 모두 **로봇(Pi)에서 실행**합니다. PC는 SSH 터미널과 브라우저로만 씁니다.
PC(Windows/WSL Humble)와 로봇(Jazzy)의 배포판이 달라 DDS로 직접 연결하지 않습니다.

## 0. 실행 파일 정리

> 실행은 `실행파일/` 폴더를 쓰세요 (`실행파일/README.md`). PC에서 `PC_더블클릭_카메라만_보기.bat` / `PC_더블클릭_차선주행.bat` 더블클릭이 가장 쉽습니다.
> 로봇 기동은 `robot.launch.py` 대신 `실행파일/개별실행/1_로봇기동_모터_라이다.sh`(차선주행_한번에.sh 가 자동 호출)를 씁니다: 기본 launch는 `TwistStamped`를 받고 LDS-01 드라이버를 띄워 이 로봇에서는 바퀴와 라이다가 동작하지 않습니다.

### 실제로 실행하는 것
| 어디서 | 명령 | 결과 |
|---|---|---|
| 로봇 | `ros2 launch turtlebot3_bringup robot.launch.py` | OpenCR + LiDAR 실행. `/odom` `/scan` `/imu` 발행, `/cmd_vel` 수신. 이게 없으면 바퀴가 돌지 않음 |
| 로봇 | `ros2 launch autorace_bringup real_lane.launch.py` | 카메라 → BEV → 차선 검출 → 웹 뷰어. **바퀴는 움직이지 않음** |
| 로봇 | `ros2 launch autorace_bringup real_lane.launch.py drive:=true` | 위에 더해 차선 제어기와 mux 실행. `run_active`가 true가 될 때까지 **정지 상태로 대기** |
| 로봇 | `ros2 topic pub --once /autorace/run_active std_msgs/msg/Bool "{data: true}"` | **주행 시작** (기본 0.10 m/s) |
| 로봇 | `ros2 topic pub --once /autorace/run_active std_msgs/msg/Bool "{data: false}"` | 차선이 보이면 정지. 차선이 안 보이면 mux 타임아웃(0.3초) 뒤 정지 |
| PC 브라우저 | `http://<로봇IP>:8090/` | 원본 / BEV(격자 표시) / 차선 검출 디버그 화면 3개 |

### 새로 추가한 파일
| 파일 | 역할 |
|---|---|
| `src/autorace_perception/autorace_perception/usb_camera.py` | `/dev/video0` MJPEG → `/camera_lane/image_raw` + `camera_info` |
| `src/autorace_perception/autorace_perception/web_view.py` | ROS 이미지를 HTTP MJPEG로 송출 (8090). BEV에 차선 위치 격자와 10cm 눈금 표시 |
| `src/autorace_bringup/launch/real_lane.launch.py` | 위 노드 + `bev_projector` + `detect_lane` (+ `drive:=true`일 때 `lane_controller`, `cmd_vel_mux`) |
| `src/autorace_bringup/param/perception_real.yaml` | 실기체 파라미터. `MEASURE` 표시 값은 아직 시뮬 값 |

### 시뮬 전용 (실기체에서 실행하지 않음)
| 명령 | 결과 |
|---|---|
| `ros2 launch autorace_sim autorace_world.launch.py` | Gazebo 코스 + 로봇 + 신호등/차단바 시뮬 |
| `ros2 launch autorace_bringup lane_drive.launch.py` | 시뮬 차선 주행만 |
| `ros2 launch autorace_bringup race.launch.py` | 시뮬 전체 미션 (신호등 대기 → 6미션) |

## 1. 최초 1회: 코드 배포와 빌드 (로봇)

PC(Git Bash)에서 로컬 작업본을 그대로 로봇에 복사하고 빌드합니다 (git push 없음).
```bash
bash 실행파일/PC_코드배포_빌드.sh     # autorace_ws 에서. 기본 dd@10.137.92.195
```
로봇에서 1회:
```bash
echo 'source ~/autorace_ws/install/setup.bash' >> ~/.bashrc
```
수동으로 할 때:
```bash
cd ~/autorace_ws && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --packages-select autorace_msgs autorace_perception autorace_core autorace_bringup
```
- `autorace_sim`은 Gazebo가 필요하므로 로봇에서는 빌드하지 않습니다.
- 필요한 apt 패키지: `ros-jazzy-cv-bridge python3-opencv ros-jazzy-sensor-msgs-py`.
- `~/.bashrc` 확인: `export TURTLEBOT3_MODEL=burger`, `export LDS_MODEL=LDS-02`, `export ROS_DOMAIN_ID=<팀 번호>`.

## 2. 버드아이뷰 확인 (바퀴 안 움직임)

```bash
# 터미널 1: video0을 점유한 ustreamer 종료 (안 끄면 usb_camera가 "Cannot open"으로 실패)
sudo pkill ustreamer
ros2 launch autorace_bringup real_lane.launch.py
```
PC 브라우저에서 `http://<로봇IP>:8090/` 접속 후 로봇을 직선 차로 가운데에 놓습니다.

| 화면 상태 | 의미 | 조치 (`perception_real.yaml`) |
|---|---|---|
| 흰선·노란선이 자홍색 세로선 위에 평행하게 놓임 | 정상 | — |
| 위로 갈수록 두 선이 벌어짐 | pitch가 실제보다 작음 | `camera.pitch` 증가 |
| 위로 갈수록 두 선이 모임 | pitch가 실제보다 큼 | `camera.pitch` 감소 |
| 평행하지만 격자보다 넓거나 좁음 | 높이나 fx가 틀림 | `camera.height` 또는 `usb_camera.hfov`/`fx` 조정 |
| 아래쪽에 로봇 몸체가 보임 | near가 너무 가까움 | `bev.near` 증가 (`detect_lane`, `web_view`, `detect_stop_line`도 같은 값으로) |
| 화면이 뒤집힘 | 카메라가 거꾸로 장착됨 | `usb_camera.rotate_180: true` |

- `--symlink-install`로 빌드했다면 yaml만 고친 뒤 런치를 재시작하면 됩니다(재빌드 불필요).
- 처음 값: 높이(렌즈 중심~바닥)는 자로, pitch는 휴대폰 각도계 앱으로 잽니다. 수평이 0, 수직 아래가 1.57 rad입니다.
- 원본 화면에 차선이 아예 보이지 않으면 카메라 각도부터 조정합니다.

차선 검출 확인:
```bash
ros2 topic echo /detect/lane_state     # 0 없음, 1 노란선만, 3 흰선만, 2 양쪽
ros2 topic echo /detect/lane_offset    # 차로 중심 대비 로봇 위치 (m)
ros2 topic hz /camera_lane/image_raw   # 약 15 Hz
```
- 흰선과 노란선이 잡히지 않으면 `detect_lane.detect.lane.*`의 HSV 범위를 조정합니다(조명 영향이 큼).

## 3. 차선 추종 주행

**첫 주행 전 바퀴를 바닥에서 띄워 방향부터 확인하세요.**

```bash
# 터미널 1
ros2 launch turtlebot3_bringup robot.launch.py
# 터미널 2
ros2 launch autorace_bringup real_lane.launch.py drive:=true
# 터미널 3: 시작 / 정지
ros2 topic pub --once /autorace/run_active std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /autorace/run_active std_msgs/msg/Bool "{data: false}"
```

| 동작 | 원인 |
|---|---|
| 시작 전 그대로 서 있음 | 정상 (`standby_speed 0`) |
| 차선을 잃으면 0.05 m/s로 마지막 조향 방향을 따라 탐색, 8초 뒤 포기 | `lane_controller` 복구 동작 |
| 25초 이상 정지하면 mux가 0.05 m/s로 강제 전진 | 대회 30초 규정 방어 (`run_active` true일 때만) |
| 전진 명령 중 3초 동안 odom 속도가 0이거나 라이다 스캔이 변하지 않으면 0.07 m/s로 1.5초 후진 | mux 끼임 감지 — **바퀴를 띄운 시험에서도 발동 가능** |

### 비상 정지
1. `Ctrl+C` (터미널 2): 제어기가 0 속도를 보내고 종료합니다.
2. 그래도 움직이면: `ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"`
3. 최후 수단: 터미널 1(`robot.launch.py`)까지 `Ctrl+C` → OpenCR이 정지합니다.
- 시뮬에서는 마지막 명령이 무한히 유지됩니다. 실기체에서도 같은지 확인 전까지 위 순서를 지킵니다.

### 속도 올리기
`lane_controller.max_speed`를 0.10 → 0.15 → 0.20 순서로 올립니다(시뮬 최종값 0.20). 커브에서 바깥으로 나가면 `cornering_rate`를 낮춥니다.

## 4. 카메라 장착 (2026-10-08 보정 완료)
- C920, 높이 15.5 cm, 축 앞 6 cm, **pitch 0.128 rad (7.3° 아래)**.
  "정면"으로 달았지만 차선 두 선의 소실점이 화면 중심보다 39 px 위(row 80)에 있어 7.3°로 계산했다.
- 검증: BEV에서 선 중심 간격 0.332~0.342 m (앞 0.36~0.58 m 구간, 실제 0.33) → 평행, 축척 맞음.
- 화면 맨 아래가 바닥 약 0.34 m 앞 → `bev.near 0.35`(4곳), BEV 0.35~0.60 m.
- LAD = 0.20 + 1.5·v, 0.37~0.55 m (v 0.12 → 0.38, v 0.22 → 0.53). 직선·커브 차이를 크게 (이전 0.30 + 0.6v: 0.37 / 0.43).
- 카메라 각도를 바꾸면: 직선 차로 가운데에 놓고 원본에서 두 선을 연장한 교점(소실점) 행 v0를 읽어
  `pitch = atan((119.5 - v0) / 305.8)`. 그 뒤 `near`·LAD 범위를 다시 맞춘다.
- 화각(fx)은 앞뒤 거리 축척에만 영향 (좌우 간격은 높이와 pitch로 정해짐). 아직 실측 안 함.
- 직선 0.22 m/s, 커브 0.12 m/s (`lane_controller.curve_*`). 웹 화면 3·4번 왼쪽 위에 STRAIGHT/CURVE, 속도, LAD 표시.

## 4-1. 차선 색 규칙 (실기체: 꺼짐)
- `detect_lane.lane.yellow_left: false` — 노란/흰 색과 상관없이 목표 방향 왼쪽·오른쪽에서 가장 가까운 선의 가운데를 달린다.
  선마다(연결된 덩어리) 따로 판단하므로 같은 색 선이 양쪽에 있어도 된다. 한쪽만 보이면 그 선에서 반 차로.
- 시뮬은 `true`(ROBOTIS 규칙: 노란선 왼쪽, 흰선 오른쪽) 그대로.
- 미션(갈림길·주차)은 노란선=왼쪽 가지를 전제로 짜여 있다 — 실기체 미션 이식 때 재검토.

## 5. 알려진 미확인 사항
- 카메라 내부 파라미터: 보정 전이라 `hfov`로 fx를 추정합니다. 광각 렌즈 왜곡은 보정하지 않으므로 BEV 가장자리가 휠 수 있습니다.
- 표지판 카메라(`/dev/video2`)와 미션 노드는 아직 실기체에 연결하지 않았습니다.
- 실제 바닥(검정 포맥스)에서 흰선 대비(`white_contrast_min`)는 검증하지 않았습니다.
