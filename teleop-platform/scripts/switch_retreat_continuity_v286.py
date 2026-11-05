"""Switch pinned solver, gateway and built UI after normal owner release.

No actuator commands. --check is read-only. Retains both hashed variants.
"""
from pathlib import Path
import argparse,hashlib,json,os,shutil,socket,sys

ROOT=Path(__file__).resolve().parents[2]
BUNDLE=ROOT/'output/retreat-continuity-v286'
digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def switch(connect=None,rollback=False,check=False):
    manifest=json.loads((BUNDLE/'bundle-manifest.json').read_text())
    variant='baseline' if rollback else 'release';package=BUNDLE/variant
    for kind,files in manifest['files'].items():
        for name,sha in files.items():
            assert digest(BUNDLE/kind/name)==sha,(kind,name)
    for name,sha in manifest['files']['release'].items():
        if not name.startswith('teleop-platform/') or name.endswith('/workstation.json'):continue
        path=ROOT/name
        assert (not path.exists() and name in manifest['baseline_absent']) or digest(path) in (
            sha,manifest['files']['baseline'].get(name)),('Source changed; review before switch',name)
    pin=json.loads((package/'cpp-adapter.json').read_text());assert digest(package/'servo_cpp_adapter')==pin['sha256']
    expected={'qualification/cpp-adapter.json':digest(package/'cpp-adapter.json'),
              'qualification/'+pin['binary']:pin['sha256']}
    assert not pin['binary'].startswith('/') and '..' not in pin['binary']
    if not rollback:
        evidence=ROOT/'evidence/retreat-continuity-v286'
        validation=json.loads((evidence/'offline-summary.json').read_text())
        assert validation['passed'] and validation['binary_sha256']==pin['sha256']
        assert pin['model_id']==json.loads((ROOT/'teleop-platform/src/e6_phosphobot/assets/workspace/manifest.json').read_text())['model_id']
    if check:
        print(json.dumps(dict(verified=True,variant=variant,binary=pin['sha256'])));return
    for port in (8776,8777,8889):
        with socket.socket() as s:
            s.settimeout(.2)
            assert s.connect_ex(('127.0.0.1',port))!=0,'Release owned session normally first'
    deployments=list((ROOT/'teleop-platform/runtime/sessions').glob('*/deployment.json'))
    if deployments:
        latest=max(deployments,key=lambda p:p.stat().st_mtime_ns).parent
        assert json.loads((latest/'result.json').read_text())['clean'] is True,'Latest owner has not exited cleanly'
    config_path=ROOT/'teleop-platform/config/workstation.json'
    config=json.loads(config_path.read_text());rfs=config['rootfs']
    with connect() as c,c.open_sftp() as s:
        target=rfs+'/qualification/'+pin['binary']
        try:
            with s.open(target,'rb') as f:assert hashlib.sha256(f.read()).hexdigest()==pin['sha256']
        except FileNotFoundError:
            s.put(str(package/'servo_cpp_adapter'),target+'.pending');s.chmod(target+'.pending',0o755);s.posix_rename(target+'.pending',target)
        destination=rfs+'/qualification/cpp-adapter.json';temp=destination+'.v286-pending'
        s.put(str(package/'cpp-adapter.json'),temp)
        with s.open(temp,'rb') as f:assert json.loads(f.read())==pin
        _,stdout,stderr=c.exec_command(f"flock -n /run/e6-phosphobot-session.lock mv '{temp}' '{destination}'")
        assert stdout.channel.recv_exit_status()==0,stderr.read().decode()
    dist=ROOT/'teleop-platform/upstream/phosphobot/phosphobot/resources/dist'
    for name in manifest['files'][variant]:
        if name in ('cpp-adapter.json','servo_cpp_adapter','dist/index.html') or name.endswith('/workstation.json'):continue
        source=package/name;target=(dist/name[5:]) if name.startswith('dist/') else ROOT/name
        target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target)
        if name.startswith('teleop-platform/frontend/'):
            shutil.copy2(source,ROOT/'teleop-platform/upstream/phosphobot/dashboard/src/e6'/source.name)
    if rollback:
        for name in manifest['baseline_absent']:
            path=ROOT/name
            if path.exists():os.replace(path,path.with_suffix(path.suffix+'.disabled-v286'))
    temp=dist/'index-v286.tmp';shutil.copy2(package/'dist/index.html',temp);os.replace(temp,dist/'index.html')
    config['cpp_runtime_manifest']=expected;temp=config_path.with_suffix('.v286-pending')
    temp.write_text(json.dumps(config,indent=2)+'\n',encoding='utf8');temp.replace(config_path)
    print(json.dumps(dict(switched=True,variant=variant,binary=pin['sha256'])))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--check',action='store_true');p.add_argument('--rollback',action='store_true');p.add_argument('--password',action='store_true');a=p.parse_args()
    sys.path.insert(0,str(ROOT/'teleop-platform/src'));from e6_phosphobot.launcher import ssh_factory
    config=json.loads((ROOT/'teleop-platform/config/workstation.json').read_text())
    switch(None if a.check else ssh_factory(config,a.password),a.rollback,a.check)
