# 모든 실행파일이 공통으로 불러오는 환경 설정 (직접 실행하지 않음)
source /opt/ros/jazzy/setup.bash
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=30
export TURTLEBOT3_MODEL=burger
