# 이 저장소에서 일하는 Claude Code 에게

2026 STEAM Cup 오토레이스 실차(TurtleBot3 Burger, ROS 2 Jazzy) 저장소. 현재 브랜치 `integration_2` 는
`integration_1` (`51b9565`) 에 **카메라 QoS·카메라 압축·통신(DDS)** 수정만 더한 것이다.

**먼저 읽을 것**: [docs/INTEGRATION_2.md](docs/INTEGRATION_2.md) — 최종 목표, integration_1 과 달라진 파일과 이유,
확인한 것/안 한 것, 로봇에 올리는 순서. 대회장 절차는 `체크리스트.md`, 지난 기록은 `README.md` 와 `docs/worklog_*`.

## 지켜야 할 것

- **저장소와 Pi 내부 설정을 맞춰라.** Pi 의 시작 스크립트는 홈의 복사본(`~/csi_camera.py`, `~/ld_lidar.py`,
  `~/tb3_burger_twist.yaml`, `~/dds.sh`, `~/start_*.sh`)을 실행한다. 로봇에 접근할 수 있으면 Pi 에서
  `tools/pi/check_sync.sh` 를 먼저 돌리고, 다른 것이 있으면 맞춘다(저장소 기준, Pi 에만 있는 수정은 저장소로 먼저 옮김,
  복사 대신 `ln -sf`). 맞춘 내용은 문서에 남긴다.
- 카메라 영상 QoS 는 **Best Effort** 로 통일되어 있다. 새로 카메라 토픽을 받는 코드는 `qos_profile_sensor_data` 를 쓴다.
  한쪽만 Reliable 이면 영상이 아예 안 온다.
- PC 와 로봇은 **같은 DDS** 여야 한다: PC 터미널마다 `export DDS_PEERS=<로봇 호스트>; source tools/dds.sh`,
  Pi 스크립트는 자동. 확인은 `tools/check_robot.sh`.
- `ROS_DOMAIN_ID` 는 30. 같은 Wi-Fi 에서 로봇 두 대를 동시에 쓰지 말 것.
- Wi-Fi 는 5 GHz. 채널은 `tools/wifi_channel.py` 로 고른다 (붐비지 않는 비DFS > 붐비지 않는 DFS > 덜 붐비는 비DFS > 덜 붐비는 DFS).
  기록상 이 로봇은 2.4 GHz(재전송 많음)로만 달렸다.
  실험할 때 Pi 의 `iw dev wlan0 link` 로 대역을 확인하고 worklog 에 적는다 (docs/INTEGRATION_2.md 'Wi-Fi').
- 다른 사람의 브랜치(`integration_1`, `parking_finish` 등)는 직접 고치지 말고 새 브랜치로.
