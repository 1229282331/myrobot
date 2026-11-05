"""Switch a verified native package after normal owner release; no device IO."""
from pathlib import Path
import argparse,hashlib,json,os,socket,sys
ROOT=Path(__file__).resolve().parents[2];B=ROOT/'output/collision-snapshot-v321'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def switch(connect,rollback=False,check=False):
 proof=json.loads((B/'release.json').read_text());assert proof['software_qualified']
 for variant,files in proof['files'].items():
  for name,digest in files.items():
   p=(B/variant/name).resolve();assert p.is_relative_to((B/variant).resolve()) and sha(p)==digest,(variant,name)
 for name,digest in proof['native_source_manifest'].items():assert sha(ROOT/'teleop-platform/native/collision-snapshot-v321'/name)==digest
 for name,digest in proof['evidence'].items():assert sha(ROOT/'evidence/collision-snapshot-v321'/name)==digest
 from e6_phosphobot.launcher import make_payload
 assert make_payload()[1]==proof['source_manifest']
 config=ROOT/'teleop-platform/config/workstation.json'
 assert sha(config) in (proof['baseline_config_sha256'],proof['config_sha256'])
 variant='baseline' if rollback else 'release';package=B/variant
 pinpath=package/('qualification/cpp-adapter.json' if rollback else 'cpp-adapter.json')
 pin=json.loads(pinpath.read_text());assert '..' not in Path(pin['binary']).parts and not pin['binary'].startswith('/')
 binary=package/('qualification/'+pin['binary'] if rollback else 'servo_cpp_adapter');assert sha(binary)==pin['sha256']
 assert 'orientation_assist_version' not in pin
 if check:print(json.dumps(dict(checked=True,variant=variant,binary=pin['sha256'])));return
 for port in (8765,8776,8777,8889):
  with socket.socket() as s:s.settimeout(.2);assert s.connect_ex(('127.0.0.1',port))!=0,'Release active owner normally first'
 last=max((ROOT/'teleop-platform/runtime/sessions').glob('*/deployment.json'),key=lambda p:p.stat().st_mtime_ns).parent
 assert json.loads((last/'result.json').read_text())['clean']
 cfg=json.loads(config.read_text());remote=cfg['rootfs']+'/qualification'
 expected=[proof['files']['baseline']['qualification/cpp-adapter.json'],proof['files']['release']['cpp-adapter.json']]
 with connect() as c,c.open_sftp() as sf:
  _,o,e=c.exec_command('flock -n /run/e6-phosphobot-session.lock true');assert o.channel.recv_exit_status()==0,e.read().decode()
  target=remote+'/'+pin['binary'];parent=remote
  for part in Path(pin['binary']).parts[:-1]:
   parent+='/'+part
   try:sf.stat(parent)
   except FileNotFoundError:sf.mkdir(parent)
  sf.put(str(binary),target+'.v321-pending');sf.chmod(target+'.v321-pending',0o755)
  sf.put(str(pinpath),remote+'/cpp-adapter.json.v321-pending')
  program=f'''from pathlib import Path
import fcntl,hashlib,json,os
lock=open('/run/e6-phosphobot-session.lock','a+');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
root=Path({remote!r}).resolve();target=Path({target!r}).resolve();assert target.is_relative_to(root)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
assert sha(root/'cpp-adapter.json') in {expected!r}
assert sha(Path(str(target)+'.v321-pending'))=={pin['sha256']!r}
assert sha(root/'cpp-adapter.json.v321-pending')=={sha(pinpath)!r}
os.replace(str(target)+'.v321-pending',target);os.replace(root/'cpp-adapter.json.v321-pending',root/'cpp-adapter.json')
print(json.dumps(dict(remote_switched=True,binary=sha(target))))
'''
  i,o,e=c.exec_command('python3 -',timeout=20);i.write(program);i.channel.shutdown_write();data=o.read();assert o.channel.recv_exit_status()==0,e.read().decode()
 temp=config.with_suffix('.v321-pending');temp.write_bytes((package/'workstation.json').read_bytes());os.replace(temp,config)
 print(json.dumps(dict(switched=variant,config_sha256=sha(config),remote=json.loads(data),physical_qualified=proof['physical_platform_qualified'])))

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--check',action='store_true');p.add_argument('--rollback',action='store_true');a=p.parse_args()
 sys.path.insert(0,str(ROOT/'teleop-platform/src'))
 h=ROOT/'output/control_delivery_session_v59.py';ctx={'__file__':str(h)};sys.argv=[str(h),'context']
 exec(compile(h.read_text(encoding='utf8').split("if mode=='deploy':",1)[0],str(h),'exec'),ctx)
 switch(ctx['client'],a.rollback,a.check)
