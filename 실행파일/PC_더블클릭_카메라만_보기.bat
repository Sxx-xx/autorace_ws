@echo off
chcp 65001 >nul
rem [PC] 더블클릭: 로봇의 차선 카메라 원본만 띄우고 브라우저를 연다. 창을 닫거나 Ctrl+C 로 종료.
start "" cmd /c "timeout /t 7 >nul & start http://10.137.92.195:8090"
ssh -t dd@10.137.92.195 "bash ~/autorace_ws/실행파일/카메라만_보기.sh"
pause
