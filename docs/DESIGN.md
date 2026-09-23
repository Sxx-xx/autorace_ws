# AutoRace 2023 자율주행 시스템 설계서

- 대상 대회: 오토레이스(Autorace) v.2023.1
- 플랫폼: TurtleBot3 Waffle Pi (ROS 2 **Jazzy** / Ubuntu 24.04)
- 1차 목표: **Gazebo Harmonic(gz-sim 8) 시뮬레이션에서 6개 미션 완주**, 이후 실기체 이식
- 워크스페이스: `~/Desktop/autorace_ws`

---

## 1. 대회 요구사항 요약

| 미션 | 배점 | 성공 조건 | 실패 조건 |
|---|---|---|---|
| 신호등 | 20 | 녹색 점등 중 출발선 통과 | 적/황색에 출발 |
| 갈림길 | 20 | 랜덤 표시되는 좌/우 이정표 방향으로 진행 | 반대 방향 진행 |
| 공사 구간 | 20 | 고정 장애물을 피해 주행로 통과 | 통과 실패 (접촉은 무관) |
| 주차 | 20 | 비어있는 구역에 완전 진입 후 탈출 | 하우스 로봇이 있는 칸 진입 |
| 차단바 | 20 | 1번 센서 감지 → 정지, 개방 후 통과 | 차단바 하강 중 2번 센서(차단바 6cm 전) 감지 |
| 터널 | 20 | 암전 + 랜덤 장애물 구간(1.8m×1.8m, 높이 24cm) 탈출 | 터널 내 탈출 실패 |
| 시간 | 20 | 미션 제한시간 5분 (녹색불 최초 점등 시 자동 시작) | — |

추가 제약
- 로봇/PC 접촉 시 **회당 -5점**, 30초 이상 정지 시 **경기 종료** → *멈추는 것보다 계속 움직이는 것이 항상 유리*
- 주행 1회만 주어짐 → **재현성 > 최고 성능**
- 주행로 폭 30cm(차선 포함), 경기장 4m×4m, 검정 포맥스 재질(반사 주의)
- 미션 순서와 주행로 형태는 **대회 당일 공개** → 미션 순서를 하드코딩하지 말고 **표지판 인식 기반 전이**로 설계

> 설계 원칙 #1: 순서 하드코딩 금지. 상태머신은 `표지판/신호 인식`으로 전이하고, 순서 정보는 파라미터(YAML)로만 힌트를 준다.
> 설계 원칙 #2: 어떤 미션이 실패해도 **차선 주행으로 복귀**해서 나머지 미션을 계속 수행한다 (부분 점수 확보).
> 설계 원칙 #3: 30초 정지 = 경기 종료. 모든 정지 동작에는 **타임아웃과 탈출 행동**을 반드시 넣는다.

---

## 2. 기존 자산 분석 (ROBOTIS `turtlebot3_autorace`, jazzy 브랜치)

클론 위치: `src/turtlebot3_autorace`

### 그대로 쓸 수 있는 것 ✅
| 파일 | 역할 | 출력 토픽 |
|---|---|---|
| `camera/image_projection.py` | 원근변환(BEV, 1000×600) | `/camera/image_projected` |
| `camera/image_compensation.py` | 밝기/대비 보정(CLAHE 계열) | `/camera/image_compensated` |
| `detect/detect_lane.py` | 흰색/노란색 차선 검출 + 슬라이딩 윈도우 | `/detect/lane`(Float64), `/detect/lane_state`(UInt8) |
| `detect/detect_level_crossing.py` | 차단바(빨간 막대) 검출 | `/detect/level_crossing_order` |
| `detect/detect_*_sign.py` | SIFT 기반 표지판 매칭 5종 | `/detect/traffic_sign`(UInt8) |
| `mission/control_lane.py` | 차선 PD 추종 (→ `autorace_core/lane_controller.py`로 대체) | `/control/cmd_vel` |
| `mission/avoid_construction.py` | LiDAR 기반 공사구간 회피 | `/avoid_control`, `/avoid_active` |
| `mission/mission_tunnel.py` | Nav2 goal 전송형 터널 주행 | `/goal_pose` |

### 포크한 것 🔧 (`autorace_perception`)

| 원본 | 우리 것 | 이유 |
|---|---|---|
| `camera/image_projection.py` | `bev_projector.py` | 원본은 의미 없는 픽셀 좌표 4개로 캘리브레이션하고 파라미터 범위 제한(`top_x≤120` 등)까지 걸려 있어 시야를 넓힐 수 없다. 포크는 **카메라 높이·피치 + `camera_info`** 로 호모그래피를 직접 계산하고 출력 스케일을 `pixels_per_meter`로 명시한다. 실기체 이식 시 높이·피치만 재측정하면 된다. |
| `detect/detect_lane.py` | `detect_lane.py` | 원본은 BEV 스케일을 하드코딩(차선 반폭 **280px**, 제어점 **350행**)해 시야 확대가 불가능했다. 포크는 `lane.width_m`와 `pixels_per_meter`로 반폭을 계산하고, 제어점은 **전방 주시거리(m)** 로 지정한다. 신뢰도 판정 기준(원본: 600행 중 500행 이상 커버)도 파라미터화. |

포크에서 함께 고친 것:
- 한 프레임에서 양쪽 차선을 모두 놓치면 원본은 중심값을 **아예 발행하지 않아** 제어가 끊긴다 → 짧은 유지(`hold_last_center_sec`) 추가
- 초기 프레임에서 아직 피팅되지 않은 곡선을 참조해 예외가 나는 경로가 있었다 → 상태 검사 추가
- 차선 중심 오프셋을 **미터 단위**(`/detect/lane_offset`)로도 발행 → 제어 게인이 캘리브레이션과 무관해짐

### 없어서 직접 만들어야 하는 것 ❌
1. **미션 총괄 상태머신** — 2020 버전의 `core_node_mode/mission`이 jazzy 브랜치에서 통째로 빠짐. 지금 상태로는 미션들이 서로 연결되지 않음.
2. **신호등 판정 출력** — `detect_traffic_light.py`가 **디버그 이미지만 발행하고 판정 결과 토픽이 없음**. 적/황/녹 판정 결과 발행부를 추가해야 함.
3. **갈림길(교차로) 주행 로직** — 좌/우 표지판 인식 노드는 있으나, 인식 후 *어느 차선을 따라갈지* 결정하는 주행 로직이 없음.
4. **주차 미션 로직** — 전무. 표지판 검출만 존재.
5. **차단바 미션 로직** — 검출만 존재. 정지/재출발 제어 없음.
6. **cmd_vel 중재기** — 지금은 `avoid_active` Bool 하나로만 전환. 미션 6개를 붙이려면 우선순위 기반 mux 필요.
7. **시뮬레이션 트랙** — jazzy 브랜치의 `turtlebot3_autorace_2020.world`는 **Gazebo Classic 유물**이며 참조 모델(`course`, `traffic_light` 등)이 브랜치에 존재하지 않음. humble 브랜치에서 가져와 gz-sim용으로 포팅 필요.

### 🚨 발견된 구조적 결함: 표지판 토픽 충돌
5개 표지판 검출 노드가 **모두 `/detect/traffic_sign`에 발행**하는데, 각자 enum이 1부터 시작한다.

| 노드 | enum |
|---|---|
| `detect_intersection_sign` | intersection=1, left=2, right=3 |
| `detect_parking_sign` | parking=**1** |
| `detect_construction_sign` | construction=**1** |
| `detect_level_crossing_sign` | stop=**1** |
| `detect_tunnel_sign` | tunnel=**1** |

→ 동시에 켜면 구분이 불가능하다. **대책**: 런치에서 검출기마다 개별 토픽으로 리맵한다.
```
/detect/sign/intersection, /detect/sign/parking, /detect/sign/construction,
/detect/sign/level_crossing, /detect/sign/tunnel
```
그리고 `sign_aggregator` 노드가 이를 모아 `/autorace/sign`(문자열+신뢰도)로 정규화한다.
이 방식이면 **모든 검출기를 상시 구동**할 수 있어, 당일 미션 순서가 바뀌어도 대응된다.

---

## 3. 패키지 구조

```
~/Desktop/autorace_ws/
├── docs/                          # 설계/운영 문서
└── src/
    ├── turtlebot3_autorace/       # (외부) ROBOTIS 원본 - 수정 최소화
    ├── turtlebot3_simulations/    # (외부) 시뮬 모델 원본
    ├── autorace_msgs/             # 커스텀 메시지/서비스
    ├── autorace_core/             # 상태머신 + 미션 로직 + cmd_vel mux   ← 핵심
    ├── autorace_perception/       # 신호등 판정 보강, 표지판 집계
    ├── autorace_sim/              # gz-sim 트랙 포팅, 심판 노드
    └── autorace_bringup/          # 통합 런치, 파라미터
```

원본 저장소는 **포크하지 않고 그대로 두고**, 부족한 부분만 우리 패키지에서 런치 리맵/추가 노드로 덮는다. (업스트림 갱신 수용 용이)

---

## 4. 데이터 흐름

```
 [gz-sim camera]  or  [Pi Camera]
        │ /camera/image_raw
        ▼
 image_proc (rectify)  ──► /camera/image_rect_color
        │
        ├──► image_compensation ──► /camera/image_compensated ─┬─► detect_*_sign ──► /detect/sign/*
        │                                                      ├─► detect_traffic_light ──► /detect/traffic_light *(신규 출력)*
        │                                                      └─► detect_level_crossing ──► /detect/level_crossing_order
        │
        └──► image_projection(BEV) ──► /camera/image_projected ──► detect_lane ──┬─► /detect/lane (Float64, 목표 중심 px)
                                                                                 └─► /detect/lane_state (UInt8)
 [LiDAR] /scan ──┐
 [odom]  /odom ──┤
                 ▼
        ┌──────────────────────────────────────────┐
        │  mission_manager (상태머신)               │
        │  - /autorace/sign, /detect/traffic_light │
        │  - 활성 미션 결정, 미션 노드 enable/disable│
        └───────────┬──────────────────────────────┘
                    │ /autorace/mission_state
     ┌──────────────┴───────────────┬──────────────────┐
     ▼                              ▼                  ▼
 control_lane              mission_* 노드들        (Nav2, 터널만)
 /cmd_vel/lane             /cmd_vel/mission        /cmd_vel/nav
     └──────────────┬───────────────┴──────────────────┘
                    ▼
             cmd_vel_mux (우선순위 중재 + 워치독)
                    │ /cmd_vel
                    ▼
              [gz bridge] or [turtlebot3_node]
```

### cmd_vel 우선순위
| 우선순위 | 소스 | 설명 |
|---|---|---|
| 0 (최상) | `/cmd_vel/estop` | 안전 정지 (충돌 임박, 수동 개입) |
| 1 | `/cmd_vel/mission` | 활성 미션 노드의 직접 제어 |
| 2 | `/cmd_vel/nav` | Nav2 (터널) |
| 3 (최하) | `/cmd_vel/lane` | 기본 차선 주행 |

- 각 입력은 **타임아웃 0.3s**. 상위 소스가 끊기면 자동으로 하위로 폴백.
- **정지 워치독**: 총 정지 시간이 25초를 넘기면 강제로 `/cmd_vel/lane`으로 폴백 + 경고 (대회 30초 룰 방어).

---

## 5. 상태머신

```
                    ┌──────────┐
                    │ STANDBY  │  출발선 대기, 신호등 주시
                    └────┬─────┘
                  녹색 인식 │ (적/황에서는 절대 출발 금지)
                    ┌────▼─────┐
              ┌────►│LANE_DRIVE│◄───────────┐  기본 상태: 차선 주행
              │     └────┬─────┘            │
              │  표지판/검출 트리거          │ 미션 완료 or 타임아웃
              │          ▼                  │
              │   ┌──────────────┐          │
              └───┤ MISSION_<X>  ├──────────┘
                  └──────────────┘
   X ∈ {INTERSECTION, CONSTRUCTION, PARKING, LEVEL_CROSSING, TUNNEL}
```

- 각 미션은 **1회만 트리거**(완료 플래그). 오인식 재진입 방지.
- 모든 미션에 **하드 타임아웃**(기본 45s, 터널 90s). 초과 시 무조건 `LANE_DRIVE` 복귀.
- 상태 전이는 전부 `/autorace/mission_state`로 발행 → rqt/로그로 디버깅.

---

## 6. 미션별 전략

### 6.1 신호등 (STANDBY)
- HSV 색공간에서 적/황/녹 마스크 + **원형 Hough**로 램프 위치 검증(배경 빨간 물체 오검출 방지).
- **연속 N프레임(기본 5) 녹색**일 때만 출발 → 황색→녹색 전이 순간의 오판 방지.
- 녹색은 5초간만 점등되므로 출발 지연 예산은 ~1초 이내.
- 실패 시 보험: 60초간 녹색 미검출이면 경기 진행 자체가 불가하므로, 로그 경고 후 수동 시작 토픽(`/autorace/manual_start`) 허용.

### 6.2 갈림길 (INTERSECTION)
- `intersection` 표지판 인식 → 감속하며 `left`/`right` 표지판 대기.
- 좌/우 판정 후 **`detect_lane`의 추종 차선을 한쪽으로 고정**한다.
  - 좌회전: 노란선(좌측) 단독 추종 / 우회전: 흰선(우측) 단독 추종
  - `detect_lane`에 차선 선택 파라미터를 주입 (원본은 양쪽 중앙 추종)
- 분기 통과 후 일정 주행거리(odom 기준 ~0.5m) 뒤 양측 추종으로 복귀.

### 6.3 공사 구간 (CONSTRUCTION)
- `avoid_construction.py` 재사용. LiDAR로 장애물 감지 → 차선 변경식 회피.
- 접촉은 감점 없음 → **속도 유지 우선**, 과도한 회피 마진으로 코스 이탈하지 않게 튜닝.
- 장애물이 바닥 고정이므로 동적 회피 불필요.

### 6.4 주차 (PARKING)
- `parking` 표지판 인식 → 감속.
- LiDAR로 좌/우 주차 공간 점유 판별 (거리 히스토그램: 비어있으면 벽까지 거리 큼).
- **빈 칸 쪽으로 진입 → 정지 1초 → 후진 탈출 → 차선 복귀**.
- 하우스 로봇 위치가 바뀌므로 좌/우 판정은 반드시 실시간. 판정 실패 시 진입하지 않고 통과(0점이지만 실패 페널티 회피).

### 6.5 차단바 (LEVEL_CROSSING)
- `stop` 표지판 + `detect_level_crossing`(빨간 막대 검출) 병행.
- 차단바 **하강 감지 즉시 정지**. 2번 센서(차단바 6cm 전)를 넘지 않도록 **여유 있게 앞에서 정지**.
- 개방 감지 후 출발. **정지 상태 25초 초과 시 강제 진행**(30초 룰 방어).

### 6.6 터널 (TUNNEL)
- 암전 구간이라 카메라 무력화 → **LiDAR 전용 주행**으로 전환.
- 1차 구현: `mission_tunnel.py`의 Nav2 방식(사전 제작 맵 + goal). 단, 대회 당일 배치 변경 위험.
- 2차 구현(권장): **맵 없는 반응형 주행** — LiDAR 갭 팔로잉(follow-the-gap) + 벽 추종으로 출구 탐색. 랜덤 장애물/배치 변경에 강함.
- 탈출 판정: 밝기 회복 + 차선 재검출.

---

## 7. 시뮬레이션 포팅 계획 (Gazebo Classic → Harmonic)

| 항목 | 현재 | 조치 |
|---|---|---|
| 트랙 모델 | humble 브랜치에만 존재 | `autorace_sim/models/`로 복사 |
| 재질 | Ogre `.material` 스크립트 | SDF `<material><pbr><albedo_map>`으로 변환 |
| 월드 플러그인 | 없음 (Classic 형식) | `Physics`, `Sensors`, `SceneBroadcaster`, `UserCommands`, `Contact` 추가 |
| 로봇 | `turtlebot3_waffle_pi` (gz 대응 완료) | 카메라 센서 확인 후 그대로 사용 |
| 토픽 | gz 내부 토픽 | `ros_gz_bridge`로 `/camera/image_raw`, `/scan`, `/cmd_vel`, `/odom`, `/imu` 연결 |
| 신호등 | 정적 텍스처 3종 | **신규**: 적→황→녹 순환 제어 노드 (녹색 5초) |
| 차단바 | 정적 모델 | **신규**: 1번 센서 통과 시 하강하는 제어 노드 |
| 주차 로봇 | — | **신규**: 좌/우 랜덤 배치 스포너 |

→ 시뮬레이터가 대회 룰을 재현해야 연습이 의미 있으므로, `autorace_sim`에 **심판 노드(referee)**를 둬서 미션 성공/실패와 시간을 자동 채점한다.

---

## 8. 마일스톤

| # | 내용 | 산출물 |
|---|---|---|
| M0 | OpenCV 복구, 워크스페이스/패키지 골격 | 빌드 성공 |
| M1 | gz-sim 트랙 부활 + 로봇 스폰 + 브리지 | `ros2 launch autorace_sim autorace_world.launch.py` |
| M2 | **차선 주행** (BEV 캘리브레이션 + PD 튜닝) | 트랙 무정지 1바퀴 |
| M3 | cmd_vel mux + 상태머신 뼈대 | 상태 전이 로그 |
| M4 | 신호등 + 차단바 (정지/출발 계열) | 미션 2종 |
| M5 | 갈림길 + 공사 (조향 계열) | 미션 4종 |
| M6 | 주차 + 터널 | 미션 6종 |
| M7 | 심판 노드로 전체 주행 반복 검증, 파라미터 고정 | 재현율 측정 |
| M8 | 실기체 이식 (카메라 캘리브레이션, 조도 대응) | — |
