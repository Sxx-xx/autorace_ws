# integration_2 — 카메라 지연·화면 문제 수정 (2026-11-13)

`integration_1` (`51b9565`) 을 그대로 복사하고 **카메라 QoS, 카메라 압축, 통신(DDS) 설정**만 바꾼 브랜치다.
미션, 검출기, 파라미터 값은 `integration_1` 과 같다. 바꾼 이유와 내용, 확인한 것과 안 한 것,
로봇에 올리는 순서를 여기 모았다.

## 최종 목표

1. **대회**: 2026 STEAM Cup 오토레이스 코스를 실차(TurtleBot3 Burger)로 완주한다.
   차선 → 신호등 → 갈림길 → 공사 → 주차 → 차단바 → 터널, 미션 순서와 구성은 `integration_1` 그대로.
2. **이 브랜치**: 대회장에서 생긴 "카메라 지연이 크고 화면이 안 뜸"을 없앤다. 목표 상태는
   - 두 카메라가 PC 에 제 속도로 들어온다: 전방 `/camera` 15 Hz, 차선 `/camera_lane` 30 Hz
   - 화면(rqt_image_view, 시작 창, RViz)이 뜨고, 영상이 몇 초씩 밀리지 않는다
   - `tools/check_robot.sh` 에서 PC·로봇 DDS 가 같고 카메라 publisher 가 `BEST_EFFORT`
3. 실차로 위를 확인한 뒤 integration 계열에 합친다.

## 무엇이 문제였나 (integration_1 코드 기준)

| # | integration_1 | 결과 |
|---|---|---|
| 1 | 카메라 노드(`csi_camera`)가 영상과 camera_info 를 **Reliable** 로 보냄. PC 의 디코더(`republish`)도 Reliable | Wi-Fi 에서 잃은 조각을 다시 보내느라 지연이 쌓인다. 큰 영상(640×480)일수록 심함 |
| 2 | 한쪽만 Best Effort 로 바꾸면(카메라 Best Effort, 받는 쪽 Reliable) **연결 자체가 안 됨** | 화면이 아예 안 뜬다 (대회장에서 한쪽만 고쳤다면 이것) |
| 3 | 전방 Pi 카메라 640×480@15 를 **Pi CPU 가 디코드 후 JPEG 로 재압축** | Pi CPU 와 시간 소모, 586 KB/s |
| 4 | DDS 설정이 어디에도 없음 (PC·Pi 모두 Jazzy 기본 Fast DDS, 멀티캐스트 탐색만) | 핫스팟·노트북 AP 에서 탐색이 불안정하면 토픽은 보이는데 데이터가 끊기거나 늦음 |
| 5 | Pi 의 시작 스크립트가 **홈의 복사본**(`~/csi_camera.py`, `~/ld_lidar.py` 등)을 실행 | 저장소를 고쳐도 로봇에는 반영되지 않음 |

## 바뀐 것 (integration_1 → integration_2)

| 파일 | 바뀐 내용 |
|---|---|
| `src/autorace_bringup/autorace_bringup/csi_camera.py` | 다른 로봇(turtlebot2, `fast-lane` 브랜치)에서 실차 검증한 버전으로 교체. 파라미터 이름은 그대로이고 기능만 추가: 영상·camera_info **Best Effort, depth 1**; Pi 카메라 GPU MJPEG **passthrough** 와 `bitrate`; `exposure_us`, `rotate`(기본 0), `stamp`(publish/capture), `stats`; 2 s 동안 프레임이 없으면 장치 다시 열기 |
| `src/autorace_bringup/autorace_bringup/launch_helpers.py` | `republish` 가 압축 영상을 **Best Effort, depth 1** 로 받음 (`qos_overrides`). 선택 인자 `out_topic` 추가 (기존 호출은 그대로) |
| `src/autorace_perception/autorace_perception/bev_projector.py` | `/camera/camera_info` 구독을 Best Effort 로 (나머지는 integration_1 그대로, `camera.lateral_offset` 유지) |
| `src/autorace_core/autorace_core/image_lane_follower.py` | 압축 영상 구독 Best Effort |
| `tools/record_run.py`, `tools/grab_views.py`, `tools/lane_overlay.py` | 압축 영상·camera_info 구독 Best Effort (안 바꾸면 Best Effort 카메라에서 아무것도 못 받음) |
| `tools/pi/start_camera.sh` | 전방 카메라: **passthrough + bitrate 5 Mbit/s** (640×480@15 그대로). 저장소의 `csi_camera.py` 를 먼저 쓰고, 없으면 `~/csi_camera.py` (경고 출력) |
| `tools/pi/start_camera_lane.sh` | 저장소 `csi_camera.py` 우선 (C920 설정은 그대로, 원래 passthrough) |
| `tools/dds.sh` (새 파일) | PC 와 로봇이 같은 DDS 를 쓰게 하는 설정. CycloneDDS 가 깔려 있으면 Cyclone + `DDS_PEERS`(유니캐스트로 찾을 상대) + `DDS_IFACE`(인터페이스), 없거나 `DDS=fastdds` 면 Fast DDS. 해석 안 되는 호스트 이름은 경고 후 빠짐 (노드가 멈추지 않게). `ROS_DOMAIN_ID` 기본 30 |
| `tools/pi/start_robot.sh`, `start_lidar.sh`, `start_camera*.sh` | `export ROS_DOMAIN_ID=30` 대신 `tools/dds.sh` 를 source (없으면 `~/dds.sh`, 그것도 없으면 Fast DDS + 경고) |
| `tools/check_robot.sh` | 추가: PC·로봇의 DDS 비교 (로봇에서 실행 중인 노드의 환경변수를 읽음), 카메라 publisher QoS 와 구독자 수 |
| `tools/pi/check_sync.sh` (새 파일) | **Pi 에서** 실행: 홈의 복사본이 저장소와 같은지, CycloneDDS 설치, Wi-Fi 절전 설정. 다르면 맞추는 명령을 출력 (스스로는 안 바꿈) |
| `체크리스트.md` | 0번(준비): `source tools/dds.sh`, 처음 한 번 할 일, check_robot 에서 볼 것 |
| `CLAUDE.md`, `docs/INTEGRATION_2.md` (새 파일) | 이 문서들 |

바뀌지 않은 것: 미션·검출기 코드와 파라미터, C920 설정(432×240@30 passthrough), 라이다(`~/ld_lidar.py`, 180°), `start_all.sh`.

## 확인한 것 (로봇 없이, 2026-11-13)

- **빌드**: 5 패키지 모두 성공.
- **카메라 → 미션 전체 경로**: Best Effort 로 보내는 가짜 카메라 2 대(640×480@15, 432×240@30)에 `race.launch.py` 를 붙여
  노드 하나로 모든 토픽을 동시에 셈. 압축 영상·디코드·BEV·차선(15~28.6 Hz)·신호등·차단바·미션까지 모두 들어옴,
  카메라 publisher Best Effort ↔ 디코더·`bev_projector` 구독 Best Effort, QoS 불일치 경고·오류 0.
- **`csi_camera.py`**: `fast-lane` 과 바이트 단위로 같은 파일. 그 브랜치의 로봇(turtlebot2)에서 Best Effort + passthrough 로 60 fps 수신 확인(2026-10-22~27).
- **Pi 스크립트**: `python3`/`ros2` 를 가짜로 바꿔 실행. 저장소 안이면 저장소 `csi_camera.py` + passthrough + bitrate,
  홈 복사본이면 `~/csi_camera.py` 와 경고, `dds.sh` 가 없으면 Fast DDS 와 경고.
- **`dds.sh`**: Cyclone(피어 지정), 해석 안 되는 피어 섞임, Fast DDS 세 경우 모두 실제로 메시지가 오감.
- **`check_robot.sh` / `check_sync.sh`**: 가짜 카메라·가짜 Pi 홈으로 출력 확인.

## 확인 안 한 것 (로봇 필요)

- 이 로봇(turtlebot1)의 Pi 카메라가 640×480@15 에서 GPU MJPEG passthrough 를 내는지, 5 Mbit/s 화질이 표지판에 충분한지
- 대회장 Wi-Fi 에서 실제 지연이 줄었는지
- 5 GHz 망에서 달려 보기 (아래 'Wi-Fi: 5 GHz 로 옮기기'). 기록상 아직 한 번도 없음
- 로봇 Pi 와 PC 에 CycloneDDS 가 깔려 있는지 (없으면 둘 다 Fast DDS 로 돌고, 그래도 QoS·압축 개선은 그대로 적용된다)

## 로봇에 올리는 순서

```bash
# PC 와 Pi 둘 다 (처음 한 번)
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
# Pi: 저장소를 integration_2 로 받은 뒤
bash <workspace>/tools/pi/check_sync.sh      # 다른 것이 있으면 출력된 명령으로 맞춘다 (아래 '저장소 ↔ Pi')
~/start_all.sh
# PC: 모든 터미널
export DDS_PEERS=turtlebot1.local; source tools/dds.sh
tools/check_robot.sh                          # middleware 두 줄이 같고, 카메라 publisher BEST_EFFORT
ros2 topic hz /camera/image_raw/compressed    # 15 Hz 근처
ros2 topic hz /camera_lane/image_raw/compressed   # 30 Hz 근처
```

되돌리기: 통신만 원래대로 → PC·Pi 모두 `DDS=fastdds`. 전방 카메라 재압축으로 → `start_camera.sh` 의
`passthrough:=true -p bitrate:=5000000` 을 빼면 된다.

## Wi-Fi: 5 GHz 로 옮기기 (코드로는 못 고치는 지연의 바탕)

QoS·압축을 고쳐도 무선 자체가 재전송투성이면 지연과 끊김은 남는다. 기록상 이 로봇은 **5 GHz 로 실험한 적이 없다**.

| 날짜 | 기록 (`docs/worklog_2026-10-27.txt`, `README.md`) |
|---|---|
| 10-27 | 핫스팟 `1954` 가 **2.4 GHz 6 번 채널**, 주변 AP 가 5·9·13 번이라 겹침 → Pi 재시도 카운터 **1953** |
| 10-27 | 캠퍼스 Guest(5 GHz, 채널 52/100/132 전부 **DFS**)에는 Pi 가 **접속 못 함**. "핫스팟을 5 GHz 로 바꿔 보기로" 는 계획으로만 남음 |
| 이후 | README 남은 일 `[ ] 핫스팟 5 GHz 비교` 그대로. 11-13 worklog 에는 Wi-Fi 대역 기록 없음 (그날 쓴 핫스팟의 대역은 모름) |
| 11-13 | `tools/robot_add_navigator.sh` 추가: 노트북 AP(`Navigator`) 나 ipTIME(`iptime5G`) 을 Pi 에 등록하는 도구. 실제로 그 망으로 달린 기록은 없음 |

참고로 다른 로봇(turtlebot2, 같은 Pi 카메라)에서 아이폰 핫스팟을 2.4 → 5 GHz 로 바꿨을 때
(2026-10-22): 90 fps·10 Mbit/s 영상 수신 83~88 → **90.2 Hz**, 최대 공백 0.85 s → **37 ms**, ROS 왕복 지연 p90 22.8 → **11.2 ms**.

### 무엇을 쓸까 (좋은 순서)

1. **ipTIME 공유기 `iptime5G`**: 채널을 **36·40·44·48 중 하나로 고정** (자동이면 DFS 채널로 옮겨 갈 수 있음),
   대역폭 20~40 MHz (붐비는 곳에서는 좁을수록 안정적), 로봇 가까이. 등록: `PROFILE=robot-router tools/robot_add_navigator.sh`.
2. **아이폰 핫스팟**: 설정 → 개인용 핫스팟 → **"호환성 최대화" 끄기** = 5 GHz. 켜져 있으면 2.4 GHz.
3. **노트북 AP `Navigator`**: 노트북 와이파이를 5 GHz 비DFS 채널(36~48)로 띄운다 (`nmcli con modify robot-ap 802-11-wireless.band a 802-11-wireless.channel 36`).
4. 어쩔 수 없이 2.4 GHz 면 채널 **1 이나 11** (6 은 주변과 겹쳤다), 대역폭 20 MHz.

**DFS 채널(52~64, 100~144) 은 피한다**: 레이더를 감지하면 공유기가 채널을 옮기고, 기기는 들어 본 뒤에야 붙는다.
한국 비DFS 5 GHz 채널은 36~48, 149~161.

### Pi 국가 코드 (추정 원인, 확인 필요)

Pi 의 무선 국가 코드가 비어 있으면(`country 00`) 5 GHz 일부 채널을 못 쓰거나 수동으로만 찾는다. 10-27 에 DFS 망에 못 붙은 것이
이 때문일 수 있다. 확인과 설정 (Pi):

```bash
iw reg get                     # country KR 이 아니면
# /etc/netplan/ 의 wifis.wlan0 아래에  regulatory-domain: "KR"  추가 후
sudo netplan apply             # (ssh 가 끊길 수 있음)
```

### 대회장에서 잴 것 (Pi)

```bash
iw dev wlan0 link | grep -E "SSID|freq|signal|tx bitrate"   # freq 2412~2472 = 2.4 GHz, 5180 이상 = 5 GHz
iw dev wlan0 station dump | grep -E "tx (retries|failed)"   # 몇 초 간격으로 두 번: 빠르게 늘면 무선 문제
sudo iw dev wlan0 scan | grep -E "freq|SSID|signal" | paste - - -   # 주변 AP 와 채널
```
PC: `ping -c 100 turtlebot1.local` (최대·손실). **어느 망·대역·채널로 달렸는지 worklog 에 꼭 적는다** (지금까지 없어서 비교가 안 된다).

## 저장소 ↔ Pi 설정 맞추기 (이 브랜치를 이어받는 사람·Claude Code 에게)

Pi 는 저장소가 아니라 **홈의 파일**을 실행한다. 저장소만 고치면 로봇은 바뀌지 않으므로, 로봇에 접근할 수 있으면
**먼저 Pi 에서 `tools/pi/check_sync.sh` 를 돌려 저장소와 Pi 내부 설정이 다른 곳을 찾고, 다르면 맞춰라.**

- 대상: `~/csi_camera.py`, `~/ld_lidar.py`, `~/tb3_burger_twist.yaml`, `~/dds.sh`, `~/start_*.sh`, CycloneDDS 설치,
  Wi-Fi 절전(`/etc/sudoers.d/iw-powersave`). 그 밖에 Pi 에만 있는 설정(netplan 의 Wi-Fi, chrony 등)도 저장소 문서와 비교.
- 맞추는 방향: 저장소가 기준. 단 **Pi 에만 있고 저장소에 없는 변경**(대회장에서 Pi 에서 바로 고친 것)은 지우지 말고
  먼저 `diff` 로 확인해 저장소에 옮긴 다음 맞춘다.
- 방법: 복사(`cp`)보다 링크(`ln -sf <저장소 파일> ~/<파일>`). 복사는 다음 수정 때 다시 어긋난다.
- 맞춘 뒤 `~/start_all.sh` → PC 에서 `tools/check_robot.sh` 로 확인하고, 무엇을 맞췄는지 이 문서나 worklog 에 적는다.

## 주의

- QoS 는 **보내는 쪽과 받는 쪽을 같이** 바꿔야 한다. 카메라 Best Effort + 받는 쪽 Reliable = 영상 0.
  새 도구에서 카메라 압축 영상을 받을 때는 `qos_profile_sensor_data` 를 쓸 것.
- DDS 도 **양쪽이 같아야** 한다. PC Cyclone + 로봇 Fast DDS 이면 아무 토픽도 오가지 않는다 (`check_robot.sh` 의 middleware 두 줄).
- 압축 영상을 받는 프로그램 하나마다 Wi-Fi 로 사본이 하나 더 나간다. 화면 창은 필요할 때만.
- 시계가 몇 초 어긋나면 RViz 표시와 `ros2 topic delay` 가 틀어진다: `python3 tools/ntp_now.py --check` (체크리스트 0번).
