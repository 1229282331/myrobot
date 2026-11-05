"""Reversible one-file release; requires normal owner cleanup."""
from pathlib import Path
import argparse,hashlib,json,os,socket,sys
r=Path(__file__).resolve().parents[2];bundle=r/'output/tracking-v311'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def switch(connect=None,rollback=False,check=False):
 proof=json.loads((bundle/'release.json').read_text());assert proof['passed']
 for v in ('baseline','release'):assert sha(bundle/v/'joint_tracking.py')==proof['files'][v]
 source=r/'teleop-platform/src/e6_phosphobot/joint_tracking.py';assert sha(source) in proof['files'].values()
 assert sha(r/'teleop-platform/config/workstation.json')==proof['config_sha256']
 assert sha(r/'evidence/tracking-v311/replay-proof.json')==proof['replay_sha256']
 assert sha(r/'evidence/tracking-v311/tests-full.txt')==proof['tests']['log_sha256']
 if check:print(json.dumps(dict(checked=True,rollback=rollback)));return
 for port in (8765,8776,8777,8889):
  with socket.socket() as s:s.settimeout(.2);assert s.connect_ex(('127.0.0.1',port))!=0,'Release active owner normally first'
 last=max((r/'teleop-platform/runtime/sessions').glob('*/deployment.json'),key=lambda p:p.stat().st_mtime_ns).parent
 assert json.loads((last/'result.json').read_text())['clean'] is True
 cfg=json.loads((r/'teleop-platform/config/workstation.json').read_text())
 with connect() as c,c.open_sftp() as sf:
  _,stdout,stderr=c.exec_command('flock -n /run/e6-phosphobot-session.lock true')
  assert stdout.channel.recv_exit_status()==0,stderr.read().decode()
  with sf.open(cfg['rootfs']+'/qualification/cpp-adapter.json','rb') as f:pin=f.read()
  assert hashlib.sha256(pin).hexdigest()==proof['cpp_runtime_manifest']['qualification/cpp-adapter.json']
 variant='baseline' if rollback else 'release';temp=source.with_suffix('.v311-pending')
 temp.write_bytes((bundle/variant/'joint_tracking.py').read_bytes());os.replace(temp,source)
 print(json.dumps(dict(switched=True,variant=variant,sha256=sha(source))))
if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--check',action='store_true');p.add_argument('--rollback',action='store_true');a=p.parse_args()
 h=r/'output/control_delivery_session_v59.py';ctx={'__file__':str(h)}
 sys.argv=[str(h),'context'];exec(compile(h.read_text(encoding='utf8').split("if mode=='deploy':",1)[0],str(h),'exec'),ctx)
 switch(ctx['client'],a.rollback,a.check)
