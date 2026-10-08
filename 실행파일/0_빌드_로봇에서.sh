#!/bin/bash
# [로봇] 실기체에 필요한 4개 패키지를 빌드한다. 코드를 바꾼 뒤 한 번 (yaml만 바꿨으면 불필요).
source "$(dirname "$0")/_env.sh"
cd "$WS" && colcon build --symlink-install \
  --packages-select autorace_msgs autorace_perception autorace_core autorace_bringup
