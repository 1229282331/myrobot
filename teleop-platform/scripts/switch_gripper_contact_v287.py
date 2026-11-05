"""Publish/rollback contact adapter and static UI after normal owner release."""
from pathlib import Path
import argparse,hashlib,json,os,shutil,socket
r=Path(__file__).resolve().parents[2];b=r/'output/gripper-contact-v287'
p=argparse.ArgumentParser();p.add_argument('--rollback',action='store_true');p.add_argument('--check',action='store_true');a=p.parse_args()
m=json.loads((b/'bundle-manifest.json').read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
for variant,files in m['files'].items():
    for name,digest in files.items():assert sha(b/variant/name)==digest,(variant,name)
variant='baseline' if a.rollback else 'release'
for name in m['files']['release']:
    if name.startswith('teleop-platform/'):
        assert sha(r/name) in (m['files']['release'][name],m['files']['baseline'][name]),name
if a.check:print(json.dumps(dict(verified=True,variant=variant)));raise SystemExit
for port in (8776,8777,8889):
    with socket.socket() as s:
        s.settimeout(.2);assert s.connect_ex(('127.0.0.1',port))!=0,'Release the owned session first'
latest=max((r/'teleop-platform/runtime/sessions').glob('*/deployment.json'),key=lambda p:p.stat().st_mtime_ns).parent
assert json.loads((latest/'result.json').read_text())['clean'] is True
dist=r/'teleop-platform/upstream/phosphobot/phosphobot/resources/dist'
for name in m['files'][variant]:
    if name=='dist/index.html':continue
    source=b/variant/name;target=dist/name[5:] if name.startswith('dist/') else r/name
    target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target)
    if name.startswith('teleop-platform/frontend/'):
        shutil.copy2(source,r/'teleop-platform/upstream/phosphobot/dashboard/src/e6'/source.name)
temp=dist/'index-v287.tmp';shutil.copy2(b/variant/'dist/index.html',temp);os.replace(temp,dist/'index.html')
print(json.dumps(dict(switched=True,variant=variant,solver_configuration_unchanged=True)))
