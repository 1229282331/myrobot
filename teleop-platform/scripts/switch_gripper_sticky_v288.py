"""Switch only the contact adapter/log labels after a normal owned shutdown."""
from pathlib import Path
import argparse,hashlib,json,shutil,socket
r=Path(__file__).resolve().parents[2];bundle=r/'output/gripper-blocked-v288'
ap=argparse.ArgumentParser();ap.add_argument('--rollback',action='store_true');ap.add_argument('--check',action='store_true');a=ap.parse_args()
m=json.loads((bundle/'manifest.json').read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
for variant,files in m.items():
 for name,digest in files.items():
  assert name in ('gripper.py','operator_logs.py') and sha(bundle/variant/name)==digest
for name in m['release']:
 assert sha(r/'teleop-platform/src/e6_phosphobot'/name) in (m['release'][name],m['baseline'][name])
variant='baseline' if a.rollback else 'release'
if not a.check:
 for port in (8776,8777,8889):
  with socket.socket() as s:
   s.settimeout(.2);assert s.connect_ex(('127.0.0.1',port))!=0,'Release the owned session first'
 latest=max((r/'teleop-platform/runtime/sessions').glob('*/deployment.json'),key=lambda p:p.stat().st_mtime_ns).parent
 assert json.loads((latest/'result.json').read_text())['clean'] is True
 for name in m[variant]:shutil.copy2(bundle/variant/name,r/'teleop-platform/src/e6_phosphobot'/name)
print(json.dumps(dict(verified=True,variant=variant,switched=not a.check,frontend_solver_configuration_unchanged=True)))
