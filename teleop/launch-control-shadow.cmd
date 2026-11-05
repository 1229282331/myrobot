@echo off
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
    echo Please run bootstrap.ps1 first.
    pause
    exit /b 1
)
echo Shadow validation at http://127.0.0.1:8765
echo Physical input is locked. No enable, motion or CAN commands in shadow.
".venv\Scripts\python.exe" -m e6_teleop serve --backend hardware-control --control-stage shadow --controller-role harness --ssh-host 192.168.1.13 --remote-root /root/e6-control-20261005 --media-hz 20
if errorlevel 1 pause
