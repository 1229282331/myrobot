@echo off
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
    echo Please run bootstrap.ps1 first.
    pause
    exit /b 1
)
echo Open http://127.0.0.1:8765 in your browser. Ctrl+C exits both simulation processes.
".venv\Scripts\python.exe" -m e6_teleop serve
if errorlevel 1 pause
