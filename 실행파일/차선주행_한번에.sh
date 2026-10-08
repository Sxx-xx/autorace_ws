#!/bin/bash
# [로봇] 차선 주행에 필요한 것 전부: 모터+라이다, 카메라, BEV, 차선 인식, 제어기, mux, 웹 뷰어.
# 시작하면 정지 상태로 대기한다.
#   Enter  : 주행 시작 / 정지 (번갈아)
#   Ctrl+C : 정지 후 전부 종료
# 결과: 브라우저 http://<로봇IP>:8090 에 원본 / BEV+격자 / BEV+인식선 / 차선 지도.
DIR="$(cd "$(dirname "$0")" && pwd)"
source "$DIR/_env.sh"
"$DIR/전체종료.sh" quiet

stop_all() {
  trap - INT TERM HUP EXIT
  echo; echo "정지 후 종료 중..."
  "$DIR/전체종료.sh" quiet
  kill 0
}
trap stop_all INT TERM HUP EXIT

"$DIR/개별실행/1_로봇기동_모터_라이다.sh" > "$WS/logs/base.log" 2>&1 &
sleep 5
ros2 launch autorace_bringup real_lane.launch.py drive:=true > "$WS/logs/lane.log" 2>&1 &
sleep 8
echo
echo "브라우저: http://$(hostname -I | awk '{print $1}'):8090"
echo "로그: $WS/logs/base.log, lane.log"
# Enter toggles the run at once (run_switch.py stays up; see its docstring).
python3 "$DIR/run_switch.py"
stop_all
