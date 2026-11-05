"""Switch a hashed solver/source/model/UI bundle after normal owner release.

No robot commands. --check validates only; --rollback restores the saved bundle.
All replaced directories are retained. The launcher remains the single owner.
"""
from pathlib import Path
import argparse,hashlib,json,os,shutil,socket,sys,re
from datetime import datetime
ROOT=Path(__file__).resolve().parents[2];BUNDLE=ROOT/'output/slow-v294'
ASSETS=ROOT/'teleop-platform/src/e6_phosphobot/assets/workspace'
DIST=ROOT/'teleop-platform/upstream/phosphobot/phosphobot/resources/dist'
digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def switch(connect=None,rollback=False,check=False):
    manifest=json.loads((BUNDLE/'bundle-manifest.json').read_text())
    variant='baseline' if rollback else 'release';package=BUNDLE/variant
    for label,files in manifest['files'].items():
        for name,sha in files.items():
            path=(BUNDLE/label/name).resolve();assert path.is_relative_to((BUNDLE/label).resolve())
            assert digest(path)==sha,(label,name)
    pin=json.loads((package/'cpp-adapter.json').read_text())
    assert digest(package/'servo_cpp_adapter')==pin['sha256']
    assert re.fullmatch(r'[A-Za-z0-9_./-]+',pin['binary']) and not pin['binary'].startswith('/') and '..' not in pin['binary']
    assert Path(pin['binary']).name=='servo_cpp_adapter'
    config_path=ROOT/'teleop-platform/config/workstation.json';config=json.loads(config_path.read_text())
    from e6_phosphobot.session_runner import runtime_artifacts
    selected_runtime={'qualification/cpp-adapter.json':digest(package/'cpp-adapter.json'),
                      'qualification/'+pin['binary']:pin['sha256']}
    runtime_artifacts(dict(config,moveit_api='cpp',cpp_runtime_manifest=selected_runtime))
    from switch_collision_model_v262 import verify
    model=verify(package/'model');assert pin['model_id']==model['model_id']
    if not rollback:
        validation=json.loads((ROOT/'evidence/slow-v294/offline-summary.json').read_text())
        assert validation['passed'] and validation['binary_sha256']==pin['sha256'] and validation['model_id']==model['model_id']
    for name in manifest['sources']:
        path=ROOT/'teleop-platform/src/e6_phosphobot'/name
        assert digest(path) in {manifest['files'][v]['source/'+name] for v in ('baseline','release')},name
    if check:
        print(json.dumps(dict(checked=True,variant=variant,binary=pin['sha256'],model=model['model_id'])));return
    for port in (8765,8776,8777,8889):
        with socket.socket() as sock:
            sock.settimeout(.2);assert sock.connect_ex(('127.0.0.1',port))!=0,'Normal owner release required'
    sessions=list((ROOT/'teleop-platform/runtime/sessions').glob('*/deployment.json'))
    last=max(sessions,key=lambda p:p.stat().st_mtime_ns).parent
    assert json.loads((last/'result.json').read_text())['clean'] is True
    stamp=datetime.now().strftime('%Y%m%d-%H%M%S-%f');prepared=[]
    for destination,source in ((ASSETS,package/'model'),(DIST,package/'dist')):
        pending=destination.with_name(destination.name+'-v294-pending-'+stamp)
        saved=destination.with_name(destination.name+'-before-v294-'+stamp)
        for path in (destination,pending,saved):assert path.resolve().is_relative_to(ROOT.resolve())
        shutil.copytree(source,pending)
        assert all(digest(p)==digest(pending/p.relative_to(source)) for p in source.rglob('*') if p.is_file())
        prepared.append((destination,pending,saved))
    rfs=config['rootfs']
    with connect() as c,c.open_sftp() as s:
        with s.open(rfs+'/qualification/cpp-adapter.json','rb') as current:
            assert hashlib.sha256(current.read()).hexdigest() in {
                manifest['files'][v]['cpp-adapter.json'] for v in ('baseline','release')
            },'Unrecognized deployed runtime; inspect before switching'
        target=rfs+'/qualification/'+pin['binary']
        parent=rfs+'/qualification'
        for part in Path(pin['binary']).parts[:-1]:
            parent+='/'+part
            try:s.stat(parent)
            except FileNotFoundError:s.mkdir(parent)
        s.put(str(package/'servo_cpp_adapter'),target+'.v294-pending');s.chmod(target+'.v294-pending',0o755)
        with s.open(target+'.v294-pending','rb') as f:assert hashlib.sha256(f.read()).hexdigest()==pin['sha256']
        temp=rfs+'/qualification/cpp-adapter.json.v294-pending';s.put(str(package/'cpp-adapter.json'),temp)
        with s.open(temp,'rb') as f:assert json.loads(f.read())==pin
        command=f"flock -n /run/e6-phosphobot-session.lock sh -c 'mv {target}.v294-pending {target} && mv {temp} {rfs}/qualification/cpp-adapter.json'"
        _,stdout,stderr=c.exec_command(command);assert stdout.channel.recv_exit_status()==0,stderr.read().decode()
    for destination,pending,saved in prepared:destination.rename(saved);pending.rename(destination)
    for name in manifest['sources']:shutil.copy2(package/'source'/name,ROOT/'teleop-platform/src/e6_phosphobot'/name)
    config['cpp_runtime_manifest']=selected_runtime
    temp=config_path.with_suffix('.v294-pending');temp.write_text(json.dumps(config,indent=2)+'\n');os.replace(temp,config_path)
    print(json.dumps(dict(switched=True,variant=variant,binary=pin['sha256'],model=model['model_id'],saved_suffix=stamp)))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--rollback',action='store_true');p.add_argument('--check',action='store_true');p.add_argument('--password',action='store_true');a=p.parse_args()
    sys.path.insert(0,str(ROOT/'teleop-platform/src'))
    from e6_phosphobot.launcher import ssh_factory
    cfg=json.loads((ROOT/'teleop-platform/config/workstation.json').read_text())
    switch(None if a.check else ssh_factory(cfg,a.password),a.rollback,a.check)
