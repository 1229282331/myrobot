"""Reversible Python/UI release; only after normal owner release, never motion."""
from pathlib import Path
import argparse,hashlib,json,os,shutil,socket,sys
from datetime import datetime

ROOT=Path(__file__).resolve().parents[2]
BUNDLE=ROOT/'output/solver-stream-v297'
NAMES=('intent.py','gateway.py','cpp_cycle.py','jazzy_cpp_worker.py')
FRONTEND=('ControlStatus.tsx','useE6Control.ts','EpisodeBrowser.tsx')
digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def switch(connect=None,rollback=False,check=False):
 manifest=json.loads((BUNDLE/'release.json').read_text())
 for variant,files in manifest['files'].items():
  for name,sha in files.items():
   path=(BUNDLE/variant/name).resolve()
   assert path.is_relative_to((BUNDLE/variant).resolve()) and digest(path)==sha
 variant='baseline' if rollback else 'release'
 source=ROOT/'teleop-platform/src/e6_phosphobot'
 for name in NAMES:
  assert digest(source/name) in {manifest['files'][v]['source/'+name] for v in ('baseline','release')}
 proof=json.loads((ROOT/manifest['validation']).read_text())
 assert proof['passed'] and proof['binary_sha256']==manifest['binary_sha256']
 assert proof['source_sha256']=={n:manifest['files']['release']['source/'+n] for n in NAMES}
 assert proof['frontend_sha256']=={n:manifest['files']['release'][n] for n in FRONTEND}
 for name in FRONTEND:
  assert digest(ROOT/'teleop-platform/frontend'/name) in {manifest['files'][v][name] for v in ('baseline','release')}
 if check:
  print(json.dumps({'checked':True,'variant':variant,'binary_unchanged':manifest['binary_sha256']}));return
 for port in (8765,8776,8777,8889):
  with socket.socket() as sock:
   sock.settimeout(.2);assert sock.connect_ex(('127.0.0.1',port))!=0,'Normal owner release required'
 last=max((ROOT/'teleop-platform/runtime/sessions').glob('*/deployment.json'),key=lambda p:p.stat().st_mtime_ns).parent
 assert json.loads((last/'result.json').read_text())['clean'] is True
 cfg=json.loads((ROOT/'teleop-platform/config/workstation.json').read_text())
 assert cfg==json.loads((BUNDLE/'baseline/workstation.json').read_text())
 with connect() as c,c.open_sftp() as s:
  with s.open(cfg['rootfs']+'/qualification/cpp-adapter.json','rb') as f:pin=json.loads(f.read())
  assert pin==json.loads((BUNDLE/'baseline/cpp-adapter.json').read_text())
  _,out,err=c.exec_command('flock -n /run/e6-phosphobot-session.lock true')
  assert out.channel.recv_exit_status()==0,err.read().decode()
 saved=BUNDLE/('before-switch-'+datetime.now().strftime('%Y%m%d-%H%M%S-%f'));saved.mkdir()
 for name in NAMES:
  shutil.copy2(source/name,saved/name)
  temp=source/(name+'.v297-pending');shutil.copy2(BUNDLE/variant/'source'/name,temp)
  os.replace(temp,source/name)
 for name in FRONTEND:
  shutil.copy2(ROOT/'teleop-platform/frontend'/name,saved/name)
  for folder in ('teleop-platform/frontend','teleop-platform/upstream/phosphobot/dashboard/src/e6'):
   shutil.copy2(BUNDLE/variant/name,ROOT/folder/name)
 dist=ROOT/'teleop-platform/upstream/phosphobot/phosphobot/resources/dist'
 # Content-addressed assets first, index last. Keep older assets for rollback.
 for name in manifest['files'][variant]:
  if name.startswith('dist/') and name!='dist/index.html':
   dest=dist/name[5:];dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(BUNDLE/variant/name,dest)
 temp=dist/'index.v297-pending';shutil.copy2(BUNDLE/variant/'dist/index.html',temp);os.replace(temp,dist/'index.html')
 print(json.dumps({'switched':True,'variant':variant,'saved':str(saved),'binary_unchanged':pin['sha256']}))


if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--rollback',action='store_true');p.add_argument('--check',action='store_true');p.add_argument('--password',action='store_true');a=p.parse_args()
 sys.path.insert(0,str(ROOT/'teleop-platform/src'))
 from e6_phosphobot.launcher import ssh_factory
 cfg=json.loads((ROOT/'teleop-platform/config/workstation.json').read_text())
 switch(None if a.check else ssh_factory(cfg,a.password),a.rollback,a.check)
