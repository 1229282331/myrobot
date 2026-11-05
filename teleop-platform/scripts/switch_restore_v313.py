"""Switch v313 restored runtime or return to pre-restoration runtime, while idle.

No robot commands. Requires normal owner exit and an exclusive RDK lock.
"""
from pathlib import Path
import argparse,hashlib,json,os,socket,sys,zipfile
r=Path(__file__).resolve().parents[2];b=r/'output/restore-v313';sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def switch(connect,check=False,rollback=False):
 proof=json.loads((b/'release.json').read_text());assert proof['passed']
 for name,digest in proof['files'].items():assert sha(b/'release'/name)==digest,name
 for name,digest in proof['baseline_files'].items():assert sha(b/'baseline'/name)==digest,name
 for name,digest in proof['evidence'].items():assert sha(r/'evidence/restore-v313'/name)==digest,name
 sys.path.insert(0,str(r/'teleop-platform/src'))
 from e6_phosphobot.launcher import make_payload
 assert make_payload()[1]==proof['source_manifest']
 cfgpath=r/'teleop-platform/config/workstation.json'
 allowed={proof['files']['workstation.json'],proof['baseline_files']['config/workstation.json']}
 assert sha(cfgpath) in allowed
 cfg=json.loads(cfgpath.read_text());variant='baseline' if rollback else 'release'
 data=(b/variant/('config/workstation.json' if rollback else 'workstation.json')).read_bytes()
 pinbytes=(b/variant/'cpp-adapter.json').read_bytes();pin=json.loads(pinbytes)
 pending=cfgpath.with_suffix('.v313-pending');assert not pending.exists()
 if check:
  print(json.dumps(dict(checked=True,rollback=rollback,binary=pin['sha256'])));return
 for port in (8765,8776,8777,8889):
  with socket.socket() as sock:sock.settimeout(.2);assert sock.connect_ex(('127.0.0.1',port))!=0,'Normal owner release required'
 last=max((r/'teleop-platform/runtime/sessions').glob('*/deployment.json'),key=lambda p:p.stat().st_mtime_ns).parent
 assert json.loads((last/'result.json').read_text())['clean'] is True
 pending.write_bytes(data)
 try:
  with connect() as c,c.open_sftp() as sf:
   stage=cfg['rootfs']+'/qualification/restore-v313-switch-'+('baseline' if rollback else 'release')
   sf.mkdir(stage)
   with sf.open(stage+'/cpp-adapter.json','wb') as f:f.write(pinbytes)
   binary=(r/'output/rollback-review-v312/current/rdk-runtime/qualification/retreat-held-v300/servo_cpp_adapter'
           if rollback else b/'release/servo_cpp_adapter')
   assert sha(binary)==pin['sha256'];sf.put(str(binary),stage+'/servo_cpp_adapter')
   args=dict(root=cfg['rootfs'],stage=stage,binary=pin['binary'],binary_sha=pin['sha256'],
      pin_sha=hashlib.sha256(pinbytes).hexdigest(),allowed=[proof['files']['cpp-adapter.json'],proof['baseline_files']['cpp-adapter.json']])
   script='''from pathlib import Path
import fcntl,hashlib,json,os,shutil
a=ARGS
root=Path(a['root']);stage=Path(a['stage']);pin=root/'qualification/cpp-adapter.json'
target=(root/'qualification'/a['binary']).resolve();assert target.is_relative_to(root/'qualification')
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
lock=open('/run/e6-phosphobot-session.lock','a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
assert sha(pin) in a['allowed']
assert sha(stage/'cpp-adapter.json')==a['pin_sha'] and sha(stage/'servo_cpp_adapter')==a['binary_sha']
shutil.copy2(pin,stage/'previous-cpp-adapter.json')
if target.exists():assert sha(target)==a['binary_sha']
else:
 target.parent.mkdir(parents=True,exist_ok=True);os.replace(stage/'servo_cpp_adapter',target);target.chmod(0o755)
os.replace(stage/'cpp-adapter.json',pin)
print(json.dumps(dict(switched=True,binary=sha(target),pin=sha(pin),saved=str(stage))))
'''.replace('ARGS',repr(args))
   _,out,err=c.exec_command('python3 -',timeout=20);out.channel.sendall(script.encode());out.channel.shutdown_write()
   result=out.read().decode();error=err.read().decode();assert out.channel.recv_exit_status()==0,error
   remote=json.loads(result)
  os.replace(pending,cfgpath)
  if rollback:
   # Restore archived source/build bytes; retain newer unreferenced assets.
   with zipfile.ZipFile(r/'output/rollback-review-v312/pc-current.zip') as z:
    for name in z.namelist():
     if name.startswith('teleop-platform/upstream/phosphobot/phosphobot/resources/dist/'):
      target=r/name;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(z.read(name))
   for name in ('teleop-platform/frontend/ControlStatus.tsx','teleop-platform/upstream/phosphobot/dashboard/src/e6/ControlStatus.tsx'):
    (r/name).write_bytes((b/'baseline/frontend/ControlStatus.tsx').read_bytes())
  record=dict(switched=True,rollback=rollback,remote=remote,pc_config_sha256=sha(cfgpath),robot_commands=0)
  name='rollback-switch.json' if rollback else 'switch.json';path=r/'evidence/restore-v313'/name;assert not path.exists()
  path.write_text(json.dumps(record,indent=2));print(json.dumps(record))
 except Exception:
  # Never start a control owner on an unknown partial switch.
  raise

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--check',action='store_true');p.add_argument('--rollback',action='store_true');a=p.parse_args()
 h=r/'output/control_delivery_session_v59.py';ctx={'__file__':str(h)};sys.argv=[str(h),'context']
 exec(compile(h.read_text(encoding='utf8').split("if mode=='deploy':",1)[0],str(h),'exec'),ctx)
 switch(ctx['client'],a.check,a.rollback)
