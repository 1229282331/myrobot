"""Install confirmed mount B or roll back, after normal owned-session exit.

Only swaps versioned model assets. No robot IO, process killing or deletion.
"""
import argparse
from datetime import datetime
from pathlib import Path
import shutil
import socket
from switch_collision_model_v262 import verify

ROOT = Path(__file__).resolve().parents[2]
BUNDLE = ROOT/'output/gripper-mount-v268'
ASSETS = ROOT/'teleop-platform/src/e6_phosphobot/assets/workspace'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--rollback', action='store_true')
    args = parser.parse_args()
    source = BUNDLE/('baseline/workspace' if args.rollback else 'publish')
    info = verify(source)
    if args.check:
        print('Verified', info['model_id']); return
    for port in (8765,8776,8777,8889):
        with socket.socket() as sock:
            sock.settimeout(.2)
            if sock.connect_ex(('127.0.0.1',port)) == 0:
                raise RuntimeError(f'Owned service still listening on {port}; exit normally first')
    stamp = datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    pending = ASSETS.with_name('workspace-v268-pending-'+stamp)
    saved = ASSETS.with_name('workspace-before-v268-'+stamp)
    for path in (source,ASSETS,pending,saved):
        assert path.resolve().is_relative_to(ROOT.resolve())
    shutil.copytree(source,pending);verify(pending)
    ASSETS.rename(saved);pending.rename(ASSETS)
    print('Installed',info['model_id'],'Previous model retained:',saved)


if __name__ == '__main__': main()
