#!/usr/bin/env bash
# [PC, Git Bash] 로컬 코드를 로봇 ~/autorace_ws 로 복사하고 빌드한다 (git push 없음).
#   bash 실행파일/PC_코드배포_빌드.sh [user@host]
set -euo pipefail
ROBOT="${1:-dd@10.137.92.195}"
PKGS="autorace_msgs autorace_perception autorace_core autorace_bringup"
cd "$(dirname "$0")/.."
tar czf - --exclude=__pycache__ $(for p in $PKGS; do echo "src/$p"; done) docs/REAL_ROBOT.md 실행파일 \
  | ssh "$ROBOT" 'mkdir -p ~/autorace_ws && tar xzf - -C ~/autorace_ws && chmod +x ~/autorace_ws/실행파일/*.sh'
ssh "$ROBOT" "bash ~/autorace_ws/실행파일/0_빌드_로봇에서.sh 2>&1 | tail -4"
