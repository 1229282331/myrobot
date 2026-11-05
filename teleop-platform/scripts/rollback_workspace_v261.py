"""Restore the saved pre-v261 UI/config after the owned session exits.

No hardware access, process termination or file deletion. Run --check first.
"""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import socket

ROOT = Path(__file__).resolve().parents[2]
BASELINE = ROOT/'output/workspace-v261/baseline'
DIST = ROOT/'teleop-platform/upstream/phosphobot/phosphobot/resources/dist'
CONFIG = ROOT/'teleop-platform/config/workstation.json'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='Verify saved files only; make no changes')
    args = parser.parse_args()
    manifest = json.loads((BASELINE/'rollback-sha256.json').read_text(encoding='utf8'))
    for name, expected in manifest.items():
        path = (BASELINE/name).resolve()
        if not path.is_relative_to(BASELINE.resolve()) or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise RuntimeError('Baseline missing or changed: '+name)
    if args.check:
        print('Rollback baseline verified; no changes made.')
        return
    for port in (8765, 8776, 8777, 8889):
        with socket.socket() as sock:
            sock.settimeout(.2)
            if sock.connect_ex(('127.0.0.1', port)) == 0:
                raise RuntimeError(f'Port {port} still active. Stop input, finish recording and normally exit the owning launcher first.')
    saved = ROOT/'output/workspace-v261'/('before-rollback-'+datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
    saved.mkdir(parents=True, exist_ok=False)
    shutil.copytree(DIST, saved/'dist')
    shutil.copy2(CONFIG, saved/'workstation.json')
    # Keep unused hashed assets so rollback never deletes another build's files.
    shutil.copytree(BASELINE/'dist', DIST, dirs_exist_ok=True)
    shutil.copy2(BASELINE/'workstation.json', CONFIG)
    print('Restored old UI and local 50 mm configuration. Saved current build/config:', saved)
    print('Start with Start-E6.ps1 -Stage live -MotionProfile normal -MoveItApi cpp -TcpProfile omnipicker-r4 -PreviewCodec vp8')


if __name__ == '__main__':
    main()
