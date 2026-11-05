"""Reproducible PC setup. Uses upstream npm lock and our resolved uv lock."""
from pathlib import Path
import shutil
import subprocess
from prepare_upstream import prepare, build_dashboard, ROOT, UPSTREAM

prepare()
backend=UPSTREAM/'phosphobot'
shutil.copy2(ROOT/'locks/phosphobot.uv.lock',backend/'uv.lock')
subprocess.run(['uv','sync','--frozen','--no-dev','--python','3.10'],cwd=backend,check=True)
npm=shutil.which('npm.cmd') or shutil.which('npm')
subprocess.run([npm,'ci','--no-audit','--no-fund'],cwd=UPSTREAM/'dashboard',check=True)
build_dashboard()
