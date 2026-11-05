"""Switch the verified component model AND dashboard with all owned ports closed.

Retains previous directories. No robot connection, process termination or
configuration changes. Exit the owning launcher normally before using it.
"""
import argparse
import hashlib
import json
import shutil
import socket
from datetime import datetime
from pathlib import Path
from switch_collision_model_v262 import verify

ROOT=Path(__file__).resolve().parents[2]
BUNDLE=ROOT/'output/collision-boxes-v263'
ASSETS=ROOT/'teleop-platform/src/e6_phosphobot/assets/workspace'
DIST=ROOT/'teleop-platform/upstream/phosphobot/phosphobot/resources/dist'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rollback',action='store_true')
    parser.add_argument('--check',action='store_true')
    args=parser.parse_args()
    model=BUNDLE/'baseline/workspace' if args.rollback else BUNDLE/'release'
    dashboard=BUNDLE/'baseline/dist' if args.rollback else BUNDLE/'dashboard'
    info=verify(model)
    build=json.loads((dashboard/'e6-build.json').read_text(encoding='utf-8'))
    for name,digest in build['artifacts'].items():
        path=(dashboard/name).resolve()
        assert path.is_relative_to(dashboard.resolve())
        assert hashlib.sha256(path.read_bytes()).hexdigest()==digest,name
    if args.check:
        print('Verified model and dashboard',info['model_id']);return
    for port in (8765,8776,8777,8889):
        with socket.socket() as sock:
            sock.settimeout(.2)
            if sock.connect_ex(('127.0.0.1',port))==0:
                raise RuntimeError(f'Owned session still active on {port}; exit normally first')
    stamp=datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    prepared=[]
    for destination,source in [(ASSETS,model),(DIST,dashboard)]:
        pending=destination.with_name(destination.name+'-v263-pending-'+stamp)
        saved=destination.with_name(destination.name+'-before-v263-'+stamp)
        for path in (destination,pending,saved):
            assert path.resolve().is_relative_to(ROOT.resolve())
        shutil.copytree(source,pending)
        for file in source.rglob('*'):
            if file.is_file():assert file.read_bytes()==(pending/file.relative_to(source)).read_bytes()
        prepared.append((destination,pending,saved))
    for destination,pending,saved in prepared:
        destination.rename(saved);pending.rename(destination)
    print('Installed',info['model_id'],'previous files retained with suffix',stamp)


if __name__=='__main__':main()
