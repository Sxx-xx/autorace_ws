@echo off
chcp 65001 >nul
rem [PC] 더블클릭: 로봇에서 돌고 있는 것 전부 정지·종료.
ssh dd@10.137.92.195 "bash ~/autorace_ws/실행파일/전체종료.sh"
pause
