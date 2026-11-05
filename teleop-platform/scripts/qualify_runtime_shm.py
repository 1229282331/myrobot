"""Linux-only lifecycle qualification of the owned private shared-memory mount.

Runs a harmless child in a temporary root, never chroots or connects devices.
Requires CAP_SYS_ADMIN for the same unshare/mount operations as the RDK host.
"""
from pathlib import Path
import argparse
import hashlib
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
from e6_phosphobot.jazzy_host import finish_owned_group, isolated_runtime_argv


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    out=parser.parse_args().output;out.mkdir(parents=True,exist_ok=False)
    results=[]
    with tempfile.TemporaryDirectory(prefix='e6-shm-lifecycle-') as temporary:
        root=Path(temporary)/'root with spaces';shm=root/'dev/shm';shm.mkdir(parents=True)
        sentinel=shm/'original';sentinel.write_text('Keep the underlying directory intact.')
        expected=hashlib.sha256(sentinel.read_bytes()).hexdigest()
        before=subprocess.check_output(['findmnt','-J','-T',str(shm)],text=True)
        for kind in ('normal','nonzero','terminated'):
            runtime=out/kind;runtime.mkdir();ready=runtime/'inside.json'
            code='''from pathlib import Path
import json,sys,time,subprocess
shm,ready,kind=Path(sys.argv[1]),Path(sys.argv[2]),sys.argv[3]
assert not (shm/'original').exists()
(shm/'ram-only').write_text('temporary')
ready.write_text(json.dumps(dict(tmpfs=subprocess.check_output(['stat','-f','-c','%T',str(shm)],text=True).strip(),ram=True)))
if kind=='terminated':time.sleep(20)
raise SystemExit(7 if kind=='nonzero' else 0)
'''
            command=isolated_runtime_argv(root,runtime,[sys.executable,'-c',code,str(shm),str(ready),kind])
            with (runtime/'child.log').open('w') as log:
                child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                try:
                    deadline=time.monotonic()+10
                    while not ready.exists() and child.poll() is None and time.monotonic()<deadline:time.sleep(.02)
                    assert ready.exists(),f'{kind}: private mount child failed, see child.log'
                    assert json.loads(ready.read_text())['tmpfs']=='tmpfs'
                    assert sentinel.exists() and not (shm/'ram-only').exists()
                    if kind=='terminated':os.killpg(child.pid,signal.SIGTERM)
                    exit_code=child.wait(timeout=5)
                    finish_owned_group(child.pid)
                    assert exit_code=={'normal':0,'nonzero':7,'terminated':-signal.SIGTERM}[kind]
                    assert hashlib.sha256(sentinel.read_bytes()).hexdigest()==expected
                    assert not (shm/'ram-only').exists()
                    assert subprocess.check_output(['findmnt','-J','-T',str(shm)],text=True)==before
                    mount=json.loads((runtime/'shm-mount.json').read_text())['filesystems'][0]
                    assert mount['fstype']=='tmpfs'
                    assert all(x in mount['options'] for x in ('nosuid','nodev','noexec','size=131072k'))
                    results.append(dict(case=kind,exit=exit_code,underlying_preserved=True,parent_mount_unchanged=True,private_mount=mount))
                finally:
                    if child.poll() is None:os.killpg(child.pid,signal.SIGKILL);child.wait(timeout=5)
                    finish_owned_group(child.pid)
    result=dict(passed=True,cases=results,physical_io=False)
    (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))


if __name__=='__main__':main()
