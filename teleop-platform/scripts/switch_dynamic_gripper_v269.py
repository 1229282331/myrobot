"""Switch the qualified model, dashboard and pinned solver; retain rollback.

No enable, error clearing, gripper initialization or motion commands. The
owning launcher must have exited normally before the switch is allowed.
"""
import argparse,hashlib,json,shutil,socket,sys
from datetime import datetime
from pathlib import Path
from switch_collision_model_v262 import verify
ROOT=Path(__file__).resolve().parents[2];BUNDLE=ROOT/'output/dynamic-gripper-v269'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def switch(connect=None,rollback=False,check=False):
    package=BUNDLE/'baseline' if rollback else BUNDLE
    model=package/'workspace' if rollback else package/'publish'
    dashboard=package/'dist' if rollback else package/'dashboard'
    info=verify(model);pin=json.loads((package/'cpp-adapter.json').read_text());config=json.loads((package/'workstation.json').read_text())
    assert sha(package/'servo_cpp_adapter')==pin['sha256']
    build=json.loads((dashboard/'e6-build.json').read_text())
    for name,digest in build['artifacts'].items():
        file=(dashboard/name).resolve();assert file.is_relative_to(dashboard.resolve()) and sha(file)==digest
    if not rollback:
        evidence=ROOT/'evidence/dynamic-gripper-v269'
        qualified=json.loads((evidence/'qualification-summary.json').read_text())
        assert qualified['passed'] and qualified['model_id']==info['model_id'] and qualified['binary_sha256']==pin['sha256']
        for name in ('geometry-coverage.json','witness-verification.json'):
            assert json.loads((evidence/'final-model'/name).read_text())['model_id']==info['model_id']
        for name,digest in json.loads((package/'python-hashes.json').read_text()).items():assert sha(ROOT/'teleop-platform/src/e6_phosphobot'/name)==digest
    if check:print('Verified',info['model_id'],pin['sha256']);return
    for port in (8765,8776,8777,8889):
        with socket.socket() as sock:
            sock.settimeout(.2)
            if sock.connect_ex(('127.0.0.1',port))==0:raise RuntimeError(f'Owned session active on {port}; exit normally first')
    assert connect is not None
    stamp=datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    prepared=[]
    for destination,source in [(ROOT/'teleop-platform/src/e6_phosphobot/assets/workspace',model),(ROOT/'teleop-platform/upstream/phosphobot/phosphobot/resources/dist',dashboard)]:
        pending=destination.with_name(destination.name+'-pending-v269-'+stamp);saved=destination.with_name(destination.name+'-before-v269-'+stamp)
        for p in (source,destination,pending,saved):assert p.resolve().is_relative_to(ROOT.resolve())
        shutil.copytree(source,pending);prepared.append((destination,pending,saved))
    with connect() as c,c.open_sftp() as s:
        rfs=config['rootfs'];remote_binary=rfs+'/qualification/'+pin['binary']
        assert remote_binary.startswith(rfs+'/qualification/') and '..' not in pin['binary']
        # Both old and new binaries have immutable names. Never overwrite one.
        try:
            with s.open(remote_binary,'rb') as f:assert hashlib.sha256(f.read()).hexdigest()==pin['sha256']
        except FileNotFoundError:
            temp=remote_binary+'.pending-'+stamp;s.put(str(package/'servo_cpp_adapter'),temp);s.chmod(temp,0o755);s.rename(temp,remote_binary)
        target=rfs+'/qualification/cpp-adapter.json';temp=target+'.pending-'+stamp
        s.put(str(package/'cpp-adapter.json'),temp)
        with s.open(temp,'rb') as f:assert hashlib.sha256(f.read()).hexdigest()==sha(package/'cpp-adapter.json')
        s.posix_rename(temp,target)
    for destination,pending,saved in prepared:destination.rename(saved);pending.rename(destination)
    if rollback:
        for p in (package/'source/e6_phosphobot').glob('*.py'):shutil.copy2(p,ROOT/'teleop-platform/src/e6_phosphobot'/p.name)
    shutil.copy2(package/'workstation.json',ROOT/'teleop-platform/config/workstation.json')
    print('Installed',info['model_id'],'rollback retained',BUNDLE/'baseline')

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--check',action='store_true');p.add_argument('--rollback',action='store_true');p.add_argument('--password',action='store_true');a=p.parse_args()
    sys.path.insert(0,str(ROOT/'teleop-platform/src'))
    from e6_phosphobot.launcher import ssh_factory
    config=json.loads((ROOT/'teleop-platform/config/workstation.json').read_text())
    switch(None if a.check else ssh_factory(config,a.password),a.rollback,a.check)
