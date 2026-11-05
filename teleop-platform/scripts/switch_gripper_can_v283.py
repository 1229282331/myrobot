"""Switch reviewed v283 code/UI after normal owner release; no hardware IO.

--check verifies bundle and unchanged current source only. --rollback selects
the pre-v283 source and built UI (manual initialization and 15–60% range).
The shared PC UI may stay running on8765; the Control page must be released.
"""
from pathlib import Path
import argparse,hashlib,json,os,shutil,socket

root=Path(__file__).resolve().parents[2];bundle=root/'output/gripper-can-v283'
p=argparse.ArgumentParser();p.add_argument('--rollback',action='store_true');p.add_argument('--check',action='store_true');a=p.parse_args()
m=json.loads((bundle/'bundle-manifest.json').read_text());variant='baseline' if a.rollback else 'release'
digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
for kind,files in m.items():
 for path,sha in files.items():
  if digest(bundle/kind/path)!=sha:raise RuntimeError('Bundle hash differs: '+kind+'/'+path)
for path,sha in m['release'].items():
 if path.startswith('dist/'):continue
 if digest(root/path) not in (m['baseline'].get(path),sha):raise RuntimeError('Current source changed; review first: '+path)
if not a.check:
 for port in (8776,8777,8889):
  with socket.socket() as s:
   s.settimeout(.2)
   if s.connect_ex(('127.0.0.1',port))==0:raise RuntimeError('Release the owned session normally first')
 deployed=list((root/'teleop-platform/runtime/sessions').glob('*/deployment.json'))
 if deployed:
  latest=max(deployed,key=lambda p:p.stat().st_mtime_ns).parent
  result=latest/'result.json'
  if not result.exists() or json.loads(result.read_text()).get('clean') is not True:raise RuntimeError('Latest owner has no verified clean exit')
 dist=root/'teleop-platform/upstream/phosphobot/phosphobot/resources/dist'
 for path in m[variant]:
  source=bundle/variant/path
  if path.startswith('dist/'):
   if path=='dist/index.html':continue
   target=dist/path[5:]
  else:target=root/path
  target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target)
  if path.startswith('teleop-platform/frontend/'):
   ext=root/'teleop-platform/upstream/phosphobot/dashboard/src/e6'/source.name
   shutil.copy2(source,ext)
 temp=dist/'index-v283.tmp';shutil.copy2(bundle/variant/'dist/index.html',temp);os.replace(temp,dist/'index.html')
 # On rollback the new pure mapping module remains unused; no deletion needed.
print(json.dumps(dict(verified=True,selected=variant,switched=not a.check)))
