"""Switch the qualified Servo pin and its small matching source/UI bundle. No device commands."""
from pathlib import Path
import argparse,hashlib,json,socket,sys,shutil
ROOT=Path(__file__).resolve().parents[2];BUNDLE=ROOT/'output/arm-proximity-v271'
def switch(connect=None,rollback=False,check=False):
 package=BUNDLE/'baseline' if rollback else BUNDLE
 pin=json.loads((package/'cpp-adapter.json').read_text());binary=package/'servo_cpp_adapter'
 assert hashlib.sha256(binary.read_bytes()).hexdigest()==pin['sha256']
 expected_manifest={'qualification/cpp-adapter.json':hashlib.sha256((package/'cpp-adapter.json').read_bytes()).hexdigest(),
                    'qualification/'+pin['binary']:pin['sha256']}
 sys.path.insert(0,str(ROOT/'teleop-platform/src'))
 from e6_phosphobot.session_runner import runtime_artifacts
 runtime_artifacts(dict(moveit_api='cpp',cpp_runtime_manifest=expected_manifest))
 guard=package/'source/e6_phosphobot/gripper_geometry.py' if rollback else package/'gripper_geometry.py'
 gateway=package/'source/e6_phosphobot/gateway.py' if rollback else package/'gateway.py'
 front=package/'ControlStatus.tsx'
 input_hook=package/'useE6Control.ts'
 if not guard.is_file() or not gateway.is_file() or not front.is_file() or not input_hook.is_file():raise RuntimeError('Matching source bundle is missing')
 build=json.loads((package/'dist/e6-build.json').read_text())
 for name,digest in build['extensions'].items():
  source={'ControlStatus.tsx':front,'useE6Control.ts':input_hook}.get(name,ROOT/'teleop-platform/frontend'/name)
  assert hashlib.sha256(source.read_bytes()).hexdigest()==digest, 'Unrelated frontend changes: '+name
 if not rollback:
  evidence=ROOT/'evidence/arm-proximity-v271'
  qualification=json.loads((evidence/'solver-summary.json').read_text());assert qualification['passed'] and qualification['binary_sha256']==pin['sha256']
  geometry=json.loads((evidence/'geometry-summary.json').read_text());assert geometry['passed']
  certificates=json.loads((evidence/'sweep-certificate-summary.json').read_text());assert certificates['passed']
  combined=json.loads((evidence/'combined-e-summary.json').read_text())
  assert all(v['arm_denied']==v['proposal_denied']==v['budget_exhausted']==0 for v in combined.values())
  assert geometry['model_id']==json.loads((ROOT/'teleop-platform/src/e6_phosphobot/assets/workspace/manifest.json').read_text())['model_id']
  for name,digest in pin['sources'].items():assert hashlib.sha256((ROOT/'teleop-platform/scripts'/name).read_bytes()).hexdigest()==digest
 if check:print('Verified',pin['sha256']);return
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
  manifest=rfs+'/qualification/cpp-adapter.json';temporary=manifest+'.v271-pending'
  s.put(str(package/'cpp-adapter.json'),temporary)
  with s.open(temporary,'rb') as f:assert json.loads(f.read())==pin
  s.posix_rename(temporary,manifest)
 config['cpp_runtime_manifest']=expected_manifest
 shutil.copy2(guard,ROOT/'teleop-platform/src/e6_phosphobot/gripper_geometry.py')
 shutil.copy2(gateway,ROOT/'teleop-platform/src/e6_phosphobot/gateway.py')
 shutil.copy2(front,ROOT/'teleop-platform/frontend/ControlStatus.tsx')
 shutil.copy2(input_hook,ROOT/'teleop-platform/frontend/useE6Control.ts')
 shutil.copytree(package/'dist',ROOT/'teleop-platform/upstream/phosphobot/phosphobot/resources/dist',dirs_exist_ok=True)
 (ROOT/'teleop-platform/config/workstation.json').write_text(json.dumps(config,indent=2)+'\n',encoding='utf8')
 print('Pinned',pin['sha256'],'rollback',rollback)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--check',action='store_true');p.add_argument('--rollback',action='store_true');p.add_argument('--password',action='store_true');a=p.parse_args()
 sys.path.insert(0,str(ROOT/'teleop-platform/src'));from e6_phosphobot.launcher import ssh_factory
 config=json.loads((ROOT/'teleop-platform/config/workstation.json').read_text());switch(None if a.check else ssh_factory(config,a.password),a.rollback,a.check)
