"""Restore the focused gripper source bundle after a normal session exit.

No robot, CAN, remote-shell or process control. Restart with the usual launcher
after switching; it deploys and verifies these sources through the normal path.
"""
from pathlib import Path
import argparse
import hashlib
import json
import shutil
import socket

ROOT = Path(__file__).resolve().parents[2]
BUNDLE = ROOT / 'output/gripper-smooth-v272'
SOURCE = ROOT / 'teleop-platform/src/e6_phosphobot'


def switch(rollback=False, check=False):
    manifest = json.loads((BUNDLE / 'source-manifest.json').read_text())
    selected = 'baseline' if rollback else 'release'
    for name in ('gripper.py', 'gateway.py'):
        for kind in ('baseline', 'release'):
            digest = hashlib.sha256((BUNDLE / kind / name).read_bytes()).hexdigest()
            if digest != manifest[kind][name]:
                raise RuntimeError('Bundle hash mismatch: ' + kind + '/' + name)
        current = hashlib.sha256((SOURCE / name).read_bytes()).hexdigest()
        if current not in (manifest['baseline'][name], manifest['release'][name]):
            raise RuntimeError('Source changed since v272; review before switching: ' + name)
    if check:
        print('Verified gripper v272 source bundle; selected=' + selected)
        return
    for port in (8765, 8776, 8777, 8889):
        with socket.socket() as sock:
            sock.settimeout(.2)
            if sock.connect_ex(('127.0.0.1', port)) == 0:
                raise RuntimeError('Release the owned session normally before switching')
    sessions = ROOT / 'teleop-platform/runtime/sessions'
    deployments = list(sessions.glob('*/deployment.json'))
    if deployments:
        latest = max(deployments, key=lambda p: p.stat().st_mtime_ns).parent
        result = latest / 'result.json'
        if not result.is_file() or json.loads(result.read_text()).get('clean') is not True:
            raise RuntimeError('Latest session has no verified clean exit: ' + latest.name)
    for name in ('gripper.py', 'gateway.py'):
        shutil.copy2(BUNDLE / selected / name, SOURCE / name)
    print('Selected ' + selected + '; start the normal launcher to deploy')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--rollback', action='store_true')
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    switch(args.rollback, args.check)
