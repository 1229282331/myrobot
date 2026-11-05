"""Restore either two-file Python revision after normal owner shutdown."""
from pathlib import Path
import argparse,hashlib,json,os,socket,sys
r=Path(__file__).resolve().parents[2];b=r/'output/j5-speed-v318'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def switch(connect,rollback=False,check=False):
 proof=json.loads((b/'release.json').read_text());assert proof['software_qualified']
 config=r/'teleop-platform/config/workstation.json';assert sha(config)==proof['config_sha256']
 assert sha(b/'full-tests-final.txt')==proof['tests_sha256']
 for rel in proof['changed']:
  name=Path(rel).name
  assert sha(b/'release'/name)==proof['source_manifest'][rel]
  assert sha(b/'baseline'/name)==proof['baseline_source_manifest'][rel]
  assert sha(r/'teleop-platform/src'/rel) in (proof['source_manifest'][rel],proof['baseline_source_manifest'][rel])
 if check:print(json.dumps(dict(checked=True,rollback=rollback)));return
 for port in (8765,8776,8777,8889):
  with socket.socket() as s:s.settimeout(.2);assert s.connect_ex(('127.0.0.1',port))!=0,'Release active owner normally first'
 last=max((r/'teleop-platform/runtime/sessions').glob('*/deployment.json'),key=lambda p:p.stat().st_mtime_ns).parent
 assert json.loads((last/'result.json').read_text())['clean']
 cfg=json.loads(config.read_text())
 with connect() as c,c.open_sftp() as sf:
  _,o,e=c.exec_command('flock -n /run/e6-phosphobot-session.lock true');assert o.channel.recv_exit_status()==0,e.read().decode()
  with sf.open(cfg['rootfs']+'/qualification/cpp-adapter.json','rb') as f:pin=f.read()
  assert hashlib.sha256(pin).hexdigest()==proof['cpp_runtime_manifest']['qualification/cpp-adapter.json']
 variant='baseline' if rollback else 'release'
 for rel in proof['changed']:
  dest=r/'teleop-platform/src'/rel;temp=dest.with_suffix('.v318-pending');temp.write_bytes((b/variant/dest.name).read_bytes());os.replace(temp,dest)
 print(json.dumps(dict(restored=variant,next='Launch the matching pinned owner; no device commands sent.')))
if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--check',action='store_true');p.add_argument('--rollback',action='store_true');a=p.parse_args()
 h=r/'output/control_delivery_session_v59.py';ctx={'__file__':str(h)};sys.argv=[str(h),'context']
 exec(compile(h.read_text(encoding='utf8').split("if mode=='deploy':",1)[0],str(h),'exec'),ctx)
 switch(ctx['client'],a.rollback,a.check)
