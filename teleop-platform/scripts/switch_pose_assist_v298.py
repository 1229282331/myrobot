"""Switch the qualified orientation-assist package after normal owner release.

Read-only --check verifies the package. --rollback restores the recorded actual
runtime, Python/UI sources and index. No device or motion command is issued.
"""
from pathlib import Path
import argparse,hashlib,json,os,shutil,socket,sys,re
from datetime import datetime
ROOT=Path(__file__).resolve().parents[2];BUNDLE=ROOT/'output/pose-assist-v298'
NAMES=('intent.py','intent_ramp.py','cpp_cycle.py','jazzy_cpp_worker.py','gateway.py')
FRONTEND=('ControlStatus.tsx','useE6Control.ts')
digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def switch(connect=None,rollback=False,check=False):
 manifest=json.loads((BUNDLE/'release.json').read_text())
 for variant,files in manifest['files'].items():
  for name,sha in files.items():
   path=(BUNDLE/variant/name).resolve()
   assert path.is_relative_to((BUNDLE/variant).resolve()) and digest(path)==sha,(variant,name)
 variant='baseline' if rollback else 'release';package=BUNDLE/variant
 pin=json.loads((package/'runtime/cpp-adapter.json').read_text())
 assert digest(package/'servo_cpp_adapter')==pin['sha256']
 assert re.fullmatch(r'[A-Za-z0-9_./-]+',pin['binary']) and not pin['binary'].startswith('/') and '..' not in pin['binary']
 proof=json.loads((ROOT/manifest['validation']).read_text())
 assert proof['passed'] and proof['binary_sha256']==manifest['binary_sha256']
 assert proof['source_sha256']=={n:manifest['files']['release']['src/e6_phosphobot/'+n] for n in NAMES}
 assert proof['frontend_sha256']=={n:manifest['files']['release']['frontend/'+n] for n in FRONTEND}
 for folder,names in [('src/e6_phosphobot',NAMES),('frontend',FRONTEND)]:
  for name in names:
   assert digest(ROOT/'teleop-platform'/folder/name) in {manifest['files'][v][folder+'/'+name] for v in ('baseline','release')},name
 config_path=ROOT/'teleop-platform/config/workstation.json';cfg=json.loads(config_path.read_text())
 selected_runtime={'qualification/cpp-adapter.json':digest(package/'runtime/cpp-adapter.json'),'qualification/'+pin['binary']:pin['sha256']}
 from e6_phosphobot.session_runner import runtime_artifacts
 runtime_artifacts(dict(cfg,moveit_api='cpp',cpp_runtime_manifest=selected_runtime))
 baseline_cfg=json.loads((BUNDLE/'baseline/config/workstation.json').read_text())
 assert {k:v for k,v in cfg.items() if k!='cpp_runtime_manifest'}=={k:v for k,v in baseline_cfg.items() if k!='cpp_runtime_manifest'}
 if check:
  print(json.dumps(dict(checked=True,variant=variant,binary=pin['sha256'])));return
 for port in (8765,8776,8777,8889):
  with socket.socket() as sock:
   sock.settimeout(.2);assert sock.connect_ex(('127.0.0.1',port))!=0,'Normal owner release required'
 last=max((ROOT/'teleop-platform/runtime/sessions').glob('*/deployment.json'),key=lambda p:p.stat().st_mtime_ns).parent
 assert json.loads((last/'result.json').read_text())['clean'] is True
 stamp=datetime.now().strftime('%Y%m%d-%H%M%S-%f');saved=BUNDLE/('before-switch-'+stamp);saved.mkdir()
 with connect() as c,c.open_sftp() as s:
  root=cfg['rootfs']+'/qualification'
  with s.open(root+'/cpp-adapter.json','rb') as f:current=f.read()
  assert hashlib.sha256(current).hexdigest() in {manifest['files'][v]['runtime/cpp-adapter.json'] for v in ('baseline','release')}
  target=root+'/'+pin['binary'];parent=root
  for part in Path(pin['binary']).parts[:-1]:
   parent+='/'+part
   try:s.stat(parent)
   except FileNotFoundError:s.mkdir(parent)
  s.put(str(package/'servo_cpp_adapter'),target+'.v298-pending');s.chmod(target+'.v298-pending',0o755)
  with s.open(target+'.v298-pending','rb') as f:assert hashlib.sha256(f.read()).hexdigest()==pin['sha256']
  temp=root+'/cpp-adapter.json.v298-pending';s.put(str(package/'runtime/cpp-adapter.json'),temp)
  with s.open(temp,'rb') as f:assert hashlib.sha256(f.read()).hexdigest()==selected_runtime['qualification/cpp-adapter.json']
  _,out,err=c.exec_command(f"flock -n /run/e6-phosphobot-session.lock sh -c 'mv {target}.v298-pending {target} && mv {temp} {root}/cpp-adapter.json'")
  assert out.channel.recv_exit_status()==0,err.read().decode()
 for folder,names in [('src/e6_phosphobot',NAMES),('frontend',FRONTEND)]:
  for name in names:
   destination=ROOT/'teleop-platform'/folder/name
   shutil.copy2(destination,saved/name)
   temp=destination.with_suffix(destination.suffix+'.v298-pending');shutil.copy2(package/folder/name,temp);os.replace(temp,destination)
   if folder=='frontend':shutil.copy2(destination,ROOT/'teleop-platform/upstream/phosphobot/dashboard/src/e6'/name)
 dist=ROOT/'teleop-platform/upstream/phosphobot/phosphobot/resources/dist'
 for name in manifest['files'][variant]:
  if name.startswith('dist/') and name!='dist/index.html':
   dest=dist/name[5:];dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(package/name,dest)
 temp=dist/'index.v298-pending';shutil.copy2(package/'dist/index.html',temp);os.replace(temp,dist/'index.html')
 shutil.copy2(config_path,saved/'workstation.json');cfg['cpp_runtime_manifest']=selected_runtime
 temp=config_path.with_suffix('.v298-pending');temp.write_text(json.dumps(cfg,indent=2)+'\n');os.replace(temp,config_path)
 print(json.dumps(dict(switched=True,variant=variant,binary=pin['sha256'],saved=str(saved))))

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--rollback',action='store_true');p.add_argument('--check',action='store_true');p.add_argument('--password',action='store_true');a=p.parse_args()
 sys.path.insert(0,str(ROOT/'teleop-platform/src'))
 from e6_phosphobot.launcher import ssh_factory
 cfg=json.loads((ROOT/'teleop-platform/config/workstation.json').read_text())
 switch(None if a.check else ssh_factory(cfg,a.password),a.rollback,a.check)
