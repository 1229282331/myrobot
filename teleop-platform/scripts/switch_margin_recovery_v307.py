"""Switch a qualified package only after normal control-owner release."""
from pathlib import Path
import json,hashlib,shutil,os,socket,sys,argparse
ROOT=Path(__file__).resolve().parents[2];BUNDLE=ROOT/'output/escape-v307'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def switch(connect=None,rollback=False,check=False):
    m=json.loads((BUNDLE/'release.json').read_text());variant='baseline' if rollback else 'release';p=BUNDLE/variant
    for v,files in m['files'].items():
        for n,h in files.items():assert sha(BUNDLE/v/n)==h,(v,n)
    proof=json.loads((ROOT/m['validation']).read_text());assert proof['passed'] and proof['binary_sha256']==m['binary_sha256']
    pin=json.loads((p/'runtime/cpp-adapter.json').read_text());assert sha(p/'servo_cpp_adapter')==pin['sha256']
    assert '..' not in pin['binary'] and not pin['binary'].startswith('/')
    cfgpath=ROOT/'teleop-platform/config/workstation.json';cfg=json.loads(cfgpath.read_text());base=json.loads((BUNDLE/'baseline/teleop-platform/config/workstation.json').read_text())
    assert {k:v for k,v in cfg.items() if k!='cpp_runtime_manifest'}=={k:v for k,v in base.items() if k!='cpp_runtime_manifest'}
    for n in m['changed']:
        current=ROOT/'teleop-platform'/n
        expected={files.get('teleop-platform/'+n) for files in m['files'].values()}
        assert sha(current) in expected,n
    if check:print(json.dumps(dict(checked=True,variant=variant,binary=pin['sha256'])));return
    for port in (8765,8776,8777,8889):
        with socket.socket() as sock:sock.settimeout(.2);assert sock.connect_ex(('127.0.0.1',port))!=0,'Normal owner release required'
    last=max((ROOT/'teleop-platform/runtime/sessions').glob('*/deployment.json'),key=lambda p:p.stat().st_mtime_ns).parent
    assert json.loads((last/'result.json').read_text())['clean']
    with connect() as c,c.open_sftp() as s:
        root=cfg['rootfs']+'/qualification';target=root+'/'+pin['binary']
        parent=target.rsplit('/',1)[0]
        try:s.stat(parent)
        except FileNotFoundError:s.mkdir(parent)
        with s.open(root+'/cpp-adapter.json','rb') as f:assert hashlib.sha256(f.read()).hexdigest() in {x['runtime/cpp-adapter.json'] for x in m['files'].values()}
        s.put(str(p/'servo_cpp_adapter'),target+'.v307-pending');s.chmod(target+'.v307-pending',0o755)
        with s.open(target+'.v307-pending','rb') as f:assert hashlib.sha256(f.read()).hexdigest()==pin['sha256']
        s.put(str(p/'runtime/cpp-adapter.json'),root+'/cpp-adapter.json.v307-pending')
        _,out,err=c.exec_command(f"flock -n /run/e6-phosphobot-session.lock sh -c 'mv {target}.v307-pending {target} && mv {root}/cpp-adapter.json.v307-pending {root}/cpp-adapter.json'")
        assert out.channel.recv_exit_status()==0,err.read().decode()
    for n in m['changed']:
        source=p/'teleop-platform'/n
        if not source.exists():continue # New inert modules are unused by rollback.
        destination=ROOT/'teleop-platform'/n;temp=destination.with_suffix(destination.suffix+'.v307-pending');shutil.copy2(source,temp);os.replace(temp,destination)
    cfg['cpp_runtime_manifest']={'qualification/cpp-adapter.json':sha(p/'runtime/cpp-adapter.json'),'qualification/'+pin['binary']:pin['sha256']}
    temp=cfgpath.with_suffix('.v307-pending');temp.write_text(json.dumps(cfg,indent=2)+'\n');os.replace(temp,cfgpath)
    print(json.dumps(dict(switched=True,variant=variant,binary=pin['sha256'])))

if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('--check',action='store_true');a.add_argument('--rollback',action='store_true');a=a.parse_args()
    sys.path.insert(0,str(ROOT/'teleop-platform/src'))
    from e6_phosphobot.launcher import ssh_factory
    switch(None if a.check else ssh_factory(json.loads((ROOT/'teleop-platform/config/workstation.json').read_text()),False),a.rollback,a.check)
