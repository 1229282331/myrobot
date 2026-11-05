@echo off
cd /d "%~dp0"
upstream\phosphobot\phosphobot\.venv\Scripts\python.exe -B scripts\launch_pc.py --backend upstream-simulation %*
