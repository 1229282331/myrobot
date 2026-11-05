"""Ordinary app with parent lifecycle enabled after application startup."""
import runpy
import sys
from pathlib import Path


sys.argv.append('--managed-stdin')
runpy.run_path(str(Path(__file__).with_name('launch_pc.py')), run_name='__main__')
