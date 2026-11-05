"""Switch a qualified geometry package after normal owner release; no device IO."""
from pathlib import Path
import argparse,hashlib,json,os,re,shutil,socket,sys
from datetime import datetime
ROOT=Path(__file__).resolve().parents[2];BUNDLE=ROOT/'output/retreat-held-v300'
HEADERS=('servo_cpp_adapter.cpp','collision_retreat.hpp','dynamic_gripper.hpp','sweep_distance.hpp','held_retreat.cpp','jazzy_cpp_CMakeLists.txt')
digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def switch(connect=None,rollback=False,check=False):
 manifest=json.loads((BUNDLE/'release.json').read_text())
 for variant,files in manifest['files'].items():
  for name,sha in files.items():
   path=(BUNDLE/variant/name).resolve()
   assert path.is_relative_to((BUNDLE/variant).resolve()) and digest(path)==sha,(variant,name)
 variant='baseline' if rollback else 'release';package=BUNDLE/variant
 proof=json.loads((ROOT/manifest['validation']).read_text());assert proof['passed'] and proof['binary_sha256']==manifest['binary_sha256']
 pin=json.loads((package/'runtime/cpp-adapter.json').read_text());assert digest(package/'servo_cpp_adapter')==pin['sha256']
 assert re.fullmatch(r'[A-Za-z0-9_./-]+',pin['binary']) and not pin['binary'].startswith('/') and '..' not in pin['binary']
 pairs=[('src/gateway.py','src/e6_phosphobot/gateway.py')]+[('scripts/'+n,'scripts/'+n) for n in HEADERS]
 for archived,current in pairs:
  assert digest(ROOT/'teleop-platform'/current) in {files[archived] for files in manifest['files'].values()},current
 config_path=ROOT/'teleop-platform/config/workstation.json';cfg=json.loads(config_path.read_text())
 baseline=json.loads((BUNDLE/'baseline/workstation.json').read_text())
 assert {k:v for k,v in cfg.items() if k!='cpp_runtime_manifest'}=={k:v for k,v in baseline.items() if k!='cpp_runtime_manifest'}
 runtime={'qualification/cpp-adapter.json':digest(package/'runtime/cpp-adapter.json'),'qualification/'+pin['binary']:pin['sha256']}
 from e6_phosphobot.session_runner import runtime_artifacts
 runtime_artifacts(dict(cfg,moveit_api='cpp',cpp_runtime_manifest=runtime))
 if check:print(json.dumps(dict(checked=True,variant=variant,binary=pin['sha256'])));return
 for port in (8765,8776,8777,8889):
  with socket.socket() as sock:
   sock.settimeout(.2);assert sock.connect_ex(('127.0.0.1',port))!=0,'Normal owner release required'
 last=max((ROOT/'teleop-platform/runtime/sessions').glob('*/deployment.json'),key=lambda p:p.stat().st_mtime_ns).parent
 assert json.loads((last/'result.json').read_text())['clean']
 saved=BUNDLE/('before-switch-'+datetime.now().strftime('%Y%m%d-%H%M%S-%f'));saved.mkdir()
 with connect() as c,c.open_sftp() as s:
  root=cfg['rootfs']+'/qualification'
  with s.open(root+'/cpp-adapter.json','rb') as f:assert hashlib.sha256(f.read()).hexdigest() in {files['runtime/cpp-adapter.json'] for files in manifest['files'].values()}
  target=root+'/'+pin['binary'];parent=root
  for part in Path(pin['binary']).parts[:-1]:
   parent+='/'+part
   try:s.stat(parent)
   except FileNotFoundError:s.mkdir(parent)
  s.put(str(package/'servo_cpp_adapter'),target+'.v300-pending');s.chmod(target+'.v300-pending',0o755)
  with s.open(target+'.v300-pending','rb') as f:assert hashlib.sha256(f.read()).hexdigest()==pin['sha256']
  temp=root+'/cpp-adapter.json.v300-pending';s.put(str(package/'runtime/cpp-adapter.json'),temp)
  with s.open(temp,'rb') as f:assert hashlib.sha256(f.read()).hexdigest()==runtime['qualification/cpp-adapter.json']
  _,out,err=c.exec_command(f"flock -n /run/e6-phosphobot-session.lock sh -c 'mv {target}.v300-pending {target} && mv {temp} {root}/cpp-adapter.json'")
  assert out.channel.recv_exit_status()==0,err.read().decode()
 for archived,current in pairs:
  destination=ROOT/'teleop-platform'/current;shutil.copy2(destination,saved/destination.name)
  temp=destination.with_suffix(destination.suffix+'.v300-pending');shutil.copy2(package/archived,temp);os.replace(temp,destination)
 shutil.copy2(config_path,saved/'workstation.json');cfg['cpp_runtime_manifest']=runtime
 temp=config_path.with_suffix('.v300-pending');temp.write_text(json.dumps(cfg,indent=2)+'\n');os.replace(temp,config_path)
 print(json.dumps(dict(switched=True,variant=variant,binary=pin['sha256'],saved=str(saved))))

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--rollback',action='store_true');p.add_argument('--check',action='store_true');p.add_argument('--password',action='store_true');a=p.parse_args()
 sys.path.insert(0,str(ROOT/'teleop-platform/src'))
 from e6_phosphobot.launcher import ssh_factory
 cfg=json.loads((ROOT/'teleop-platform/config/workstation.json').read_text())
 switch(None if a.check else ssh_factory(cfg,a.password),a.rollback,a.check)
