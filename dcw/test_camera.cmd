@echo off
setlocal
cd /d "%~dp0"
".runtime\python.exe" "tools\camera.py" --mode both --seconds 60 --snapshots
pause
