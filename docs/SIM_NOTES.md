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

## BEV(원근변환) 파라미터 제약 ⚠️

`image_projection`의 네 파라미터는 **이미지 좌표**이며 범위 제한이 있다
(`top_x`, `top_y` ≤ 120 / `bottom_x`, `bottom_y` ≤ 320).

그리고 `detect_lane`이 **BEV 스케일을 하드코딩**하고 있다:
- 차선 중심 = x 500
- 한쪽 차선만 보일 때 중심 추정 = ±**280 px**

즉 0.125 m(차선 반폭) ↔ 280 px, **2240 px/m** 로 스케일이 고정된다.
따라서 목적 영역(x=200~800, 600px)은 노면 **±0.134 m**만 담을 수 있고,
1000px 전체로도 **±0.223 m**가 한계다. 이 스케일을 바꾸려면 `detect_lane`을
포크해서 반폭을 파라미터화해야 한다.

현재 값 (`autorace_bringup/param/projection_sim.yaml`): 전방 0.24~0.45 m 구간
- `top_x: 73, top_y: 29, bottom_x: 123, bottom_y: 110`

**주의**: `bottom_y`가 120을 넘으면 소스 사다리꼴 아랫변이 이미지(240행) 밖으로
나가 BEV 하단이 비어버린다.

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
