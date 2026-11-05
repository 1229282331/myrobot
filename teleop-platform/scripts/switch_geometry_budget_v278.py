"""Switch qualified geometry runtime, preserving device state. No device IO.

First release the owned operator normally while unprepared, stationary and
recording idle. --check is read-only; --rollback restores the exact v271 pin.
"""
from pathlib import Path
import argparse,hashlib,json,socket,sys,shutil
ROOT=Path(__file__).resolve().parents[2];BUNDLE=ROOT/'output/geometry-budget-v278'
SOURCES=('servo_cpp_adapter.cpp','dynamic_gripper.hpp','sweep_distance.hpp','dynamic_gripper_probe.cpp','probe_sweep_certificate.cpp')

def switch(connect=None,rollback=False,check=False):
    package=BUNDLE/'baseline' if rollback else BUNDLE
    pin=json.loads((package/'cpp-adapter.json').read_text());binary=package/'servo_cpp_adapter'
    assert hashlib.sha256(binary.read_bytes()).hexdigest()==pin['sha256']
    expected={'qualification/cpp-adapter.json':hashlib.sha256((package/'cpp-adapter.json').read_bytes()).hexdigest(),'qualification/'+pin['binary']:pin['sha256']}
    sys.path.insert(0,str(ROOT/'teleop-platform/src'))
    from e6_phosphobot.session_runner import runtime_artifacts
    runtime_artifacts(dict(moveit_api='cpp',cpp_runtime_manifest=expected))
    if not rollback:
        evidence=ROOT/'evidence/geometry-budget-v278'
        summary=json.loads((evidence/'rotation-summary.json').read_text());assert summary['passed'] and summary['binary_sha256']==pin['sha256']
        assert json.loads((evidence/'dense-regression-summary.json').read_text())['passed']
        assert json.loads((evidence/'solver-regression-summary.json').read_text())['passed']
        assert summary['model_id']==json.loads((ROOT/'teleop-platform/src/e6_phosphobot/assets/workspace/manifest.json').read_text())['model_id']
    for name in SOURCES:
        source=package/name
        assert source.is_file()
        if name in pin['sources']:assert hashlib.sha256(source.read_bytes()).hexdigest()==pin['sources'][name]
    if check:print('Verified',pin['sha256'],'rollback',rollback);return
    for port in (8765,8776,8777,8889):
        with socket.socket() as sock:
            sock.settimeout(.2)
            if sock.connect_ex(('127.0.0.1',port))==0:raise RuntimeError('Release owned session normally first')
    config=json.loads((ROOT/'teleop-platform/config/workstation.json').read_text());rfs=config['rootfs']
    assert '..' not in pin['binary'] and not pin['binary'].startswith('/')
    with connect() as c,c.open_sftp() as s:
        target=rfs+'/qualification/'+pin['binary']
        try:
            with s.open(target,'rb') as f:assert hashlib.sha256(f.read()).hexdigest()==pin['sha256']
        except FileNotFoundError:
            s.put(str(binary),target+'.pending');s.chmod(target+'.pending',0o755);s.rename(target+'.pending',target)
        manifest=rfs+'/qualification/cpp-adapter.json';temporary=manifest+'.v278-pending'
        s.put(str(package/'cpp-adapter.json'),temporary)
        with s.open(temporary,'rb') as f:assert json.loads(f.read())==pin
        s.posix_rename(temporary,manifest)
    config['cpp_runtime_manifest']=expected
    for name in SOURCES:shutil.copy2(package/name,ROOT/'teleop-platform/scripts'/name)
    path=ROOT/'teleop-platform/config/workstation.json';temp=path.with_suffix('.v278-pending');temp.write_text(json.dumps(config,indent=2)+'\n',encoding='utf8');temp.replace(path)
    print('Pinned',pin['sha256'],'rollback',rollback)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--check',action='store_true');p.add_argument('--rollback',action='store_true');p.add_argument('--password',action='store_true');a=p.parse_args()
    sys.path.insert(0,str(ROOT/'teleop-platform/src'));from e6_phosphobot.launcher import ssh_factory
    config=json.loads((ROOT/'teleop-platform/config/workstation.json').read_text());switch(None if a.check else ssh_factory(config,a.password),a.rollback,a.check)
