@echo off
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
    echo Run bootstrap.ps1 first.
    pause
    exit /b 1
)
echo E6 operator workbench: http://127.0.0.1:8765
echo Gamepad connects to this PC. Keep the hardware emergency stop available.
echo Startup connects only; Enable and Prepare are separate explicit operations.
echo Ctrl+C stops control and releases this session. It keeps the robot enabled.
".venv\Scripts\python.exe" -m e6_teleop.operator_launcher
if errorlevel 1 pause
