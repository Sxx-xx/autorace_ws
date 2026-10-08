@echo off
chcp 65001 >nul
rem [PC] 더블클릭: 로봇에서 차선 주행 전체를 띄우고 브라우저를 연다. Enter 로 출발/정지, Ctrl+C 로 전부 종료.
start "" cmd /c "timeout /t 15 >nul & start http://10.137.92.195:8090"
ssh -t dd@10.137.92.195 "bash ~/autorace_ws/실행파일/차선주행_한번에.sh"
pause
