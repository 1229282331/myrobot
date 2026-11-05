"""Reversible audit/observation-clock switch; no robot commands or new solver."""
from pathlib import Path
import argparse,hashlib,json,os,shutil,socket,sys
from datetime import datetime
ROOT=Path(__file__).resolve().parents[2];BUNDLE=ROOT/'output/cup-repair-v296'
digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def switch(connect=None,rollback=False,check=False):
 manifest=json.loads((BUNDLE/'audit-release.json').read_text());variant='baseline' if rollback else 'release'
 source=ROOT/'teleop-platform/src/e6_phosphobot'
 for label,files in manifest['files'].items():
  for name,sha in files.items():
   assert name in ('audit.py','audit_writer.py','moveit_bridge.py')
   path=BUNDLE/label/'source'/name
   assert (not path.exists()) if sha is None else digest(path)==sha
 for name in manifest['files']['release']:
  current=source/name
  valid={v[name] for v in manifest['files'].values()}
  assert (digest(current) if current.exists() else None) in valid,name
 pin=json.loads((BUNDLE/'baseline/cpp-adapter.json').read_text())
 assert pin['sha256']==manifest['binary_sha256']
 if not rollback:
  proof=json.loads((ROOT/manifest['validation']).read_text())
  assert proof['passed'] and proof['binary_sha256']==pin['sha256'] and proof['audit_sha256']==manifest['files']['release']['audit.py']
  assert proof['bridge_sha256']==manifest['files']['release']['moveit_bridge.py']
 if check:
  print(json.dumps(dict(checked=True,variant=variant,binary_unchanged=pin['sha256'])));return
 for port in (8765,8776,8777,8889):
  with socket.socket() as sock:
   sock.settimeout(.2);assert sock.connect_ex(('127.0.0.1',port))!=0,'Normal owner release required'
 last=max((ROOT/'teleop-platform/runtime/sessions').glob('*/deployment.json'),key=lambda p:p.stat().st_mtime_ns).parent
 assert json.loads((last/'result.json').read_text())['clean'] is True
 config=json.loads((ROOT/'teleop-platform/config/workstation.json').read_text());rootfs=config['rootfs']
 with connect() as c,c.open_sftp() as s:
  with s.open(rootfs+'/qualification/cpp-adapter.json','rb') as f:assert json.loads(f.read())==pin
  # Test the same session lock used by every hardware owner; qualification
  # jobs must also be finished. This acquires/releases without device IO.
  _,out,err=c.exec_command('flock -n /run/e6-phosphobot-session.lock true');assert out.channel.recv_exit_status()==0,err.read().decode()
 stamp=datetime.now().strftime('%Y%m%d-%H%M%S-%f');saved=BUNDLE/('source-before-'+stamp);saved.mkdir()
 for name,sha in manifest['files'][variant].items():
  current=source/name
  if current.exists():shutil.copy2(current,saved/name)
  if sha is None:
   if current.exists():current.unlink()  # Saved above; rollback is reversible.
  else:
   pending=current.with_suffix('.v296-pending');shutil.copy2(BUNDLE/variant/'source'/name,pending)
   assert digest(pending)==sha;os.replace(pending,current)
 print(json.dumps(dict(switched=True,variant=variant,saved=str(saved),binary_unchanged=pin['sha256'])))


if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--rollback',action='store_true');p.add_argument('--check',action='store_true');p.add_argument('--password',action='store_true');a=p.parse_args()
 sys.path.insert(0,str(ROOT/'teleop-platform/src'))
 from e6_phosphobot.launcher import ssh_factory
 cfg=json.loads((ROOT/'teleop-platform/config/workstation.json').read_text())
 switch(None if a.check else ssh_factory(cfg,a.password),a.rollback,a.check)
