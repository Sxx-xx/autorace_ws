# 시뮬레이션 운영 메모 (Gazebo Harmonic)

실측으로 확인한 사실만 적는다. 추측은 넣지 말 것.

## 좌표계

- 경기장: 4m × 4m, 월드 원점이 코스 중심.
- 상공 카메라(`autorace_topcam`, 원점 위 5m, 800×800, 5m 시야):
  - **이미지 위쪽 = 월드 +X, 이미지 왼쪽 = 월드 +Y**
  - `px = 400 - Y/0.00625`, `py = 400 - X/0.00625` (1px = 6.25mm)
- 코스 텍스처(`course.png`, 520×496)는 월드 기준 180° 회전되어 배치된다.
  텍스처 좌표로 위치를 추정하지 말고 **상공 카메라로 확인**할 것.

## 검증된 차선 좌표 (상공 카메라 실측)

| 구간 | 노란선 Y | 흰선 Y | 폭 |
|---|---|---|---|
| X = 0.4 ~ 1.1 직선 | 0.131 | 0.381 | 25.0 cm |

- 이 직선의 주행 방향은 **yaw = π (−X 방향)**. 그래야 노란선이 왼쪽, 흰선이 오른쪽이 된다.
- 테스트 시작 포즈로 `(1.0, 0.256, yaw=π)`를 쓴다.
- ROBOTIS 기본 스폰 `(0.8, −1.747, yaw=0)`은 폭이 다른 구간이라 차선 추종 튜닝에는 부적합.

## 카메라 기하 (실측 역산으로 검증됨)

`autorace_sim/models/autorace_waffle_pi` 기준:

| 항목 | 값 |
|---|---|
| 해상도 | 320 × 240 |
| fx = fy | 265.23 (`/camera/camera_info`) |
| 노면 위 높이 | **0.20 m** |
| 하향 피치 | **0.30 rad** (실측 역산 0.292 rad) |
| 보이는 최근접 노면 | **0.226 m** (이미지 최하단 행) |
| 지평선 | 이미지 38행 |

지면 점 → 픽셀 변환:
```
x_c = d·cosθ + h·sinθ        z_c = d·sinθ − h·cosθ
u   = 160 − fx·Y/x_c          v   = 120 − fx·z_c/x_c
```

## BEV(원근변환) — `autorace_perception/bev_projector`

파라미터는 **미터 단위**다. 카메라 높이/피치와 `camera_info`만 주면 호모그래피를
직접 계산한다.

| 파라미터 | 값 | 의미 |
|---|---|---|
| `camera.height` | 0.20 | 노면 위 높이 (m) |
| `camera.pitch` | 0.30 | 하향 피치 (rad) |
| `camera.forward_offset` | 0.073 | 로봇 중심에서 카메라까지 (m) |
| `bev.pixels_per_meter` | 1200 | 출력 스케일 |
| `bev.near` | 0.24 | 출력 최하단이 보는 전방 거리 (m) |

출력 1000×600 → 가로 **±0.417 m**, 전방 **0.24~0.74 m**를 담는다.
`bev.near`는 0.226 m(카메라가 볼 수 있는 최근접 노면)보다 작게 잡지 말 것.

### 원본(`turtlebot3_autorace_camera/image_projection`)을 왜 버렸나

원본은 캘리브레이션이 **이미지 픽셀 좌표 4개**이고 범위 제한까지 있다
(`top_x`,`top_y` ≤ 120 / `bottom_x`,`bottom_y` ≤ 320). 게다가 원본 `detect_lane`이
BEV 스케일을 하드코딩한다 — 차선 중심 x=500, 한쪽 차선만 보일 때 ±**280 px**.
즉 0.125 m ↔ 280 px(**2240 px/m**)로 고정되어, 목적 영역이 노면 **±0.134 m**,
전체 폭으로도 ±0.223 m 밖에 못 담는다. 코스의 급커브에서는 양쪽 차선이 모두
이 범위를 벗어나 주행이 멈춘다. 그래서 두 노드를 포크했다.

## 차선 검출 — `autorace_perception/detect_lane`

- 차선 반폭 = `lane.width_m / 2 × pixels_per_meter` (하드코딩 제거)
- 제어점 = `lane.control_lookahead` (전방 m), 행 번호가 아님
- `/detect/lane_offset` (Float64, **m**) 발행 → 제어 게인이 캘리브레이션과 무관
- **블롭 제거**: 가로·세로 양쪽으로 두꺼운 영역은 차선이 아니다.
  이게 없으면 코스 밖 회색 바닥을 흰 차선으로 오인해 트랙을 이탈한다
  (실제로 발생했음: 코스 경계를 따라 (3.0, 3.7)까지 주행).
- **좌우 검증**: 흰선은 BEV 중심 오른쪽, 노란선은 왼쪽에 있어야 한다.
  한쪽 차선만 보일 때 이 검사가 없으면 차선 바깥으로 반 차선 더 나가는 조향을 한다.

## 시뮬 액추에이터

| 대상 | 인터페이스 | 값 |
|---|---|---|
| 차단바 | ROS `/level_bar/cmd` (`std_msgs/Float64`) → gz `JointPositionController` | 0.0 = 하강(차단), 1.5708 = 개방 |
| 신호등 | `sim_traffic_light`이 `/world/autorace/set_pose`로 점등 램프 모델을 소켓에 이동 | `/sim/traffic_light`: 1=적, 2=황, 3=녹 |
| 로봇 이동 | `autorace_sim/scripts/teleport.sh X Y [YAW]` | — |

gz에는 런타임 텍스처 교체 수단이 없어 신호등은 "밝은 램프 모델 3개를 옮기는" 방식으로 구현했다.

## 디버깅 도구

```bash
# 상공 카메라 띄우기 (트랙 전체 확인)
ros2 run ros_gz_sim create -world autorace -file $(ros2 pkg prefix --share autorace_sim)/models/autorace_topcam/model.sdf -name autorace_topcam
ros2 run ros_gz_image image_bridge /topcam/image_raw

# 로봇 위치를 월드 좌표로 확인
gz model -m autorace_waffle_pi | head -6

# 특정 위치로 이동
ros2 run autorace_sim ... (scripts/teleport.sh 1.0 0.256 3.14159)
```

## 알려진 미해결 문제

1. **급커브에서 차선 유실** — 하피너 구간(월드 약 (0.6, 1.4) 부근)에서 양쪽 차선이
   모두 BEV ±0.223 m 밖으로 나가 `lane_state`가 0이 되고 로봇이 정지한다.
   → `detect_lane` 포크(차선 반폭 파라미터화) + 차선 재탐색 동작 필요.
2. `detect_traffic_light`에 판정 결과 발행부가 없다 (디버그 이미지만 발행).
3. 터널 구조물에 천장이 없어 "암전" 조건이 재현되지 않는다.
4. 공사 구간 배리어 3개의 배치가 실제 대회 규격과 일치하는지 미확인.

## ⚠️ DiffDrive는 마지막 명령을 계속 유지한다

gz-sim의 `DiffDrive`는 **마지막으로 받은 cmd_vel을 무한히 적용한다**. 제어 노드를
죽여도 로봇은 그 속도로 계속 달린다. 측정이 오염되는 주 원인이므로, 노드를 내린
뒤에는 반드시 정지 명령을 보낼 것.

```bash
bash src/autorace_sim/scripts/stop_robot.sh
```

경기 중에는 `cmd_vel_mux`가 20Hz로 계속 발행하므로 문제되지 않지만,
**실기체에서도 같은 전제**(명령이 끊기면 정지가 아니라 유지)를 확인해야 한다.

## ⚠️ 시뮬레이터 종료 시 주의

`ros2 launch`를 죽여도 **gz 서버는 살아남는다**. 이 상태에서 다시 런치하면
서버가 2개가 되고, gz 토픽 이름이 같아서 **ROS 브리지가 두 시뮬레이션의 카메라
프레임을 번갈아 받는다**. 영상이 엉뚱한 장소를 비추거나 텔레포트가 안 먹는 것처럼
보이면 먼저 이것부터 의심할 것.

```bash
# 완전 종료
pkill -f 'gz[ ]sim'; pkill -f 'ruby.*gz[ ]sim'
# 확인 (0이어야 함)
ps -eo pid,args | grep -c '[g]z sim'
```
