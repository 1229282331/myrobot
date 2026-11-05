"""Install verified staged v262 geometry or restore v261, only with ports closed.

This command never terminates a process, connects to a robot or changes its
parameters. Normally exit the owning launcher first. Both versions are kept.
"""
import argparse,hashlib,json,shutil,socket
from datetime import datetime
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
ASSETS=ROOT/'teleop-platform/src/e6_phosphobot/assets/workspace'
BASELINE=ROOT/'output/collision-v262/baseline'


def verify(directory):
    info=json.loads((directory/'manifest.json').read_text())
    content={k:v for k,v in info.items() if k!='model_id'}
    assert hashlib.sha256(json.dumps(content,sort_keys=True).encode()).hexdigest()==info['model_id']
    for name,digest in info['files'].items():
        path=(directory/name).resolve()
        assert path.is_relative_to(directory.resolve()) and hashlib.sha256(path.read_bytes()).hexdigest()==digest
    assert json.loads((directory/'reachability.json').read_text())['model_id']==info['model_id']
    return info


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    group=parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--staged',type=Path);group.add_argument('--rollback',action='store_true')
    parser.add_argument('--check',action='store_true');args=parser.parse_args()
    source=BASELINE/'workspace' if args.rollback else args.staged.resolve()
    info=verify(source)
    if args.check:
        print('Verified',info['model_id']);return
    for port in (8765,8776,8777,8889):
        with socket.socket() as sock:
            sock.settimeout(.2)
            if sock.connect_ex(('127.0.0.1',port))==0:raise RuntimeError(f'Owned session still active on {port}; exit normally first')
    archive=ROOT/'output/collision-v262'/('before-switch-'+datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
    shutil.copytree(ASSETS,archive)
    # Move within the same parent, preserving the whole prior directory. No
    # recursive delete and no partly-overwritten set visible to a new session.
    pending=ASSETS.with_name('workspace-v262-pending-'+datetime.now().strftime('%H%M%S-%f'))
    shutil.copytree(source,pending);verify(pending)
    saved=ASSETS.with_name('workspace-before-v262-'+datetime.now().strftime('%H%M%S-%f'))
    for path in (ASSETS,pending,saved,archive):
        if not path.resolve().is_relative_to(ROOT.resolve()):raise RuntimeError('Switch path escaped workspace')
    ASSETS.rename(saved);pending.rename(ASSETS)
    print('Installed',info['model_id'],'prior assets retained at',archive)


if __name__=='__main__':main()
