@echo off
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
    echo Please run bootstrap.ps1 first.
    pause
    exit /b 1
)
echo Read-only hardware observations at http://127.0.0.1:8765
echo Ctrl+C releases this observation session. It does not shut down the robot or X5.
".venv\Scripts\python.exe" -m e6_teleop serve --backend hardware-observe --ssh-host 192.168.1.13 --remote-root /root/e6-observer-20261004 --media-hz 20
if errorlevel 1 pause
