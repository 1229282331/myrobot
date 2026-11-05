"""Switch qualified runtime after normal owner release; never sends robot commands."""
from pathlib import Path
import argparse,hashlib,json,socket,sys,shutil
ROOT=Path(__file__).resolve().parents[2];BUNDLE=ROOT/'output/collision-retreat-v285'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def switch(connect=None,rollback=False,check=False):
    package=BUNDLE/'baseline' if rollback else BUNDLE
    pin=json.loads((package/'cpp-adapter.json').read_text());binary=package/'servo_cpp_adapter'
    assert sha(binary)==pin['sha256']
    expected={'qualification/cpp-adapter.json':sha(package/'cpp-adapter.json'),'qualification/'+pin['binary']:pin['sha256']}
    sys.path.insert(0,str(ROOT/'teleop-platform/src'))
    from e6_phosphobot.session_runner import runtime_artifacts
    runtime_artifacts(dict(moveit_api='cpp',cpp_runtime_manifest=expected))
    if not rollback:
        ev=ROOT/'evidence/collision-retreat-v285'
        for name in ('policy-summary.json','policy-pairs-summary.json','dense-regression-summary.json','regression-summary.json','solver-regression-summary.json'):
            summary=json.loads((ev/name).read_text());assert summary['passed'],name
            if 'binary_sha256' in summary:assert summary['binary_sha256']==pin['sha256'],name
        assert pin['model_id']==json.loads((ROOT/'teleop-platform/src/e6_phosphobot/assets/workspace/manifest.json').read_text())['model_id']
        for name,digest in pin['sources'].items():assert sha(package/name)==digest,name
    for name in ('servo_cpp_adapter.cpp','dynamic_gripper.hpp','sweep_distance.hpp','cpp_cycle.py'):assert (package/name).is_file()
    if check:print('Verified',pin['sha256'],'rollback',rollback);return
    # PC8765 is a shared web server. Only the owned control/media ports must close.
    for port in (8776,8777,8889):
        with socket.socket() as s:
            s.settimeout(.2)
            if s.connect_ex(('127.0.0.1',port))==0:raise RuntimeError('Release the owned session normally first')
    sessions=list((ROOT/'teleop-platform/runtime/sessions').glob('*/deployment.json'))
    if sessions:
        latest=max(sessions,key=lambda p:p.stat().st_mtime_ns).parent
        result=latest/'result.json'
        if not result.exists() or json.loads(result.read_text()).get('clean') is not True:raise RuntimeError('Latest owner has no verified clean exit')
    config=json.loads((ROOT/'teleop-platform/config/workstation.json').read_text());rfs=config['rootfs']
    assert '..' not in pin['binary'] and not pin['binary'].startswith('/')
    with connect() as c,c.open_sftp() as s:
        target=rfs+'/qualification/'+pin['binary']
        try:
            with s.open(target,'rb') as f:assert hashlib.sha256(f.read()).hexdigest()==pin['sha256']
        except FileNotFoundError:
            s.put(str(binary),target+'.pending');s.chmod(target+'.pending',0o755);s.posix_rename(target+'.pending',target)
        manifest=rfs+'/qualification/cpp-adapter.json';temp=manifest+'.v285-pending'
        s.put(str(package/'cpp-adapter.json'),temp)
        with s.open(temp,'rb') as f:assert json.loads(f.read())==pin
        # Hold the same exclusive lock as the session runner while changing pin.
        command=f"flock -n /run/e6-phosphobot-session.lock mv '{temp}' '{manifest}'"
        _,stdout,stderr=c.exec_command(command);code=stdout.channel.recv_exit_status()
        if code:raise RuntimeError('Runtime switch refused: '+stderr.read().decode())
    for name in ('servo_cpp_adapter.cpp','dynamic_gripper.hpp','sweep_distance.hpp'):
        shutil.copy2(package/name,ROOT/'teleop-platform/scripts'/name)
    shutil.copy2(package/'cpp_cycle.py',ROOT/'teleop-platform/src/e6_phosphobot/cpp_cycle.py')
    config['cpp_runtime_manifest']=expected
    path=ROOT/'teleop-platform/config/workstation.json';temp=path.with_suffix('.v285-pending')
    temp.write_text(json.dumps(config,indent=2)+'\n',encoding='utf8');temp.replace(path)
    print('Pinned',pin['sha256'],'rollback',rollback)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--check',action='store_true');p.add_argument('--rollback',action='store_true');p.add_argument('--password',action='store_true');a=p.parse_args()
    sys.path.insert(0,str(ROOT/'teleop-platform/src'));from e6_phosphobot.launcher import ssh_factory
    config=json.loads((ROOT/'teleop-platform/config/workstation.json').read_text());switch(None if a.check else ssh_factory(config,a.password),a.rollback,a.check)
