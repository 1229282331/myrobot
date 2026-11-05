@echo off
setlocal
cd /d "%~dp0"
if not exist ".runtime\python.exe" (
  echo Missing dcw Python environment. See README.md.
  pause
  exit /b 1
)
".runtime\python.exe" "tools\camera.py" --mode both --preview --seconds 0
if errorlevel 1 pause
